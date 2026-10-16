"""Where exactly to put something, and whether it's there yet. Pure geometry, no hardware.

A destination is a short string the planner produces and code checks:
    "tray"                 a named zone (taught position)
    "left_of:cup"          beside an object, from the PERSON's point of view
    "right_of:cup"
    "in_front_of:cup"      on the person's side of it
    "behind:cup"
    "next_to:cup"          any free side
    "on:blue_cube"         stack on top (only objects marked stackable)

Positions are (x, y) in world mm. A scene maps object id -> (x, y), or None if unseen.
Footprints are treated as circles (radius = half the larger side): simple and safe.
"""
import math

import config

RELATIONS = ("left_of", "right_of", "in_front_of", "behind", "next_to", "on")


class PlaceError(Exception):
    """A destination that can't be used right now. The message is said to the person."""


# ---------------- directions, from the person's point of view ----------------

def _unit(axis):
    sign = -1.0 if axis[0] == "-" else 1.0
    return (sign, 0.0) if axis[1] == "x" else (0.0, sign)


def directions():
    """Unit vectors in world x, y for each relation, as the person sees the table."""
    toward = _unit(config.USER_DIRECTION)          # from table toward the person
    facing = (-toward[0], -toward[1])               # the person looks at the robot
    left = (-facing[1], facing[0])                  # 90 degrees counter-clockwise
    return {
        "in_front_of": toward,
        "behind": facing,
        "left_of": left,
        "right_of": (-left[0], -left[1]),
    }


# ---------------- small helpers ----------------

def radius(obj):
    w, d, _h = config.OBJECTS[obj]["size"]
    return max(w, d) / 2.0


def height(obj):
    return config.OBJECTS[obj]["size"][2]


def parse(dest):
    """'left_of:cup' -> ('left_of', 'cup'); 'tray' -> ('zone', 'tray'). Raises PlaceError."""
    if not isinstance(dest, str) or not dest:
        raise PlaceError("I didn't get where to put it.")
    if ":" in dest:
        rel, anchor = dest.split(":", 1)
        if rel not in RELATIONS:
            raise PlaceError(f"I don't know how to put something '{rel.replace('_', ' ')}'.")
        if anchor not in config.OBJECTS:
            raise PlaceError(f"I don't know '{anchor}'.")
        return rel, anchor
    if dest in config.ZONES:
        return "zone", dest
    raise PlaceError(f"'{dest}' isn't a place I can put things.")


def anchor_of(dest):
    try:
        kind, ref = parse(dest)
    except PlaceError:
        return None
    return None if kind == "zone" else ref


def reachable(x, y):
    (x0, x1), (y0, y1) = config.WORKSPACE["x"], config.WORKSPACE["y"]
    r = math.hypot(x, y)
    if any(a <= x <= b and c <= y <= d for a, b, c, d in getattr(config, "NO_GO", ())):
        return False
    return x0 <= x <= x1 and y0 <= y <= y1 and config.MIN_REACH_MM <= r <= config.MAX_REACH_MM


def blocker(obj, x, y, scene, ignore=()):
    """The first object that would overlap `obj` placed at (x, y), or None."""
    for other, pos in scene.items():
        if other == obj or other in ignore or pos is None:
            continue
        need = radius(obj) + radius(other) + config.MIN_CLEARANCE_MM
        if math.dist((x, y), pos) < need:
            return other
    return None


def _pretty(obj):
    return obj.replace("_", " ")


# ---------------- the two questions the executor asks ----------------

def resolve(obj, dest, scene):
    """Where to put `obj` for `dest` right now: (x, y, lift_mm). lift is the height of
    whatever it lands on (0 for the table). Raises PlaceError with a reason."""
    kind, ref = parse(dest)

    if kind == "zone":
        cx, cy = config.ZONES[ref]
        # The zone center first, then rings around it, staying inside the zone.
        step = 15.0
        rings = int((config.ZONE_RADIUS_MM - radius(obj) * 0.5) // step)
        candidates = [(cx, cy)] + [
            (cx + r * step * math.cos(a * math.pi / 4), cy + r * step * math.sin(a * math.pi / 4))
            for r in range(1, max(rings, 0) + 1) for a in range(8)
        ]
        for x, y in candidates:
            if reachable(x, y) and blocker(obj, x, y, scene) is None:
                return x, y, float(getattr(config, "ZONE_LIFT", {}).get(ref, 0.0))
        raise PlaceError(f"The {ref.replace('_', ' ')} is full.")

    anchor = ref
    if anchor == obj:
        raise PlaceError("I can't put something next to itself.")
    apos = scene.get(anchor)
    if apos is None:
        raise PlaceError(f"I can't see the {_pretty(anchor)}.")

    if kind == "on":
        if not config.OBJECTS[anchor].get("stackable"):
            raise PlaceError(f"I can't stack things on the {_pretty(anchor)}.")
        if blocker(obj, apos[0], apos[1], scene, ignore=(anchor,)) is not None:
            raise PlaceError(f"Something is on the {_pretty(anchor)} already.")
        return apos[0], apos[1], float(height(anchor))

    dist = radius(anchor) + radius(obj) + config.NEXT_TO_GAP_MM
    dirs = directions()
    sides = ["left_of", "right_of", "behind", "in_front_of"] if kind == "next_to" else [kind]
    reasons, options = [], []
    for side in sides:
        ux, uy = dirs[side]
        x, y = apos[0] + ux * dist, apos[1] + uy * dist
        if not reachable(x, y):
            reasons.append("out of my reach")
            continue
        b = blocker(obj, x, y, scene)
        if b is not None:
            reasons.append(f"the {_pretty(b)} is in the way")
            continue
        options.append((x, y))
    if options:
        # "next to" may use any side: take the one nearest the middle of the table,
        # furthest from edges and reach limits.
        mid = (sum(config.WORKSPACE["x"]) / 2, sum(config.WORKSPACE["y"]) / 2)
        x, y = min(options, key=lambda p: math.dist(p, mid))
        return x, y, 0.0
    where = "next to" if kind == "next_to" else kind.replace("_", " ")
    raise PlaceError(f"There's no room {where} the {_pretty(anchor)}: {reasons[0]}.")


def error_mm(obj, dest, scene):
    """How far `obj` is from where `dest` wants it, in mm (None if it can't tell)."""
    pos = scene.get(obj)
    if pos is None:
        return None
    try:
        kind, ref = parse(dest)
    except PlaceError:
        return None
    if kind == "zone":
        return math.dist(pos, config.ZONES[ref])
    apos = scene.get(ref)
    if apos is None:
        return None
    if kind == "on":
        return math.dist(pos, apos)
    want = radius(ref) + radius(obj) + config.NEXT_TO_GAP_MM
    if kind == "next_to":
        # any side is fine: how far from the right distance to the anchor
        return abs(math.dist(pos, apos) - want)
    ux, uy = directions()[kind]
    return math.dist(pos, (apos[0] + ux * want, apos[1] + uy * want))


def done(obj, dest, scene):
    """Is `obj` where `dest` wants it?"""
    err = error_mm(obj, dest, scene)
    if err is None:
        return False
    kind, _ = parse(dest)
    limit = config.ZONE_RADIUS_MM if kind == "zone" else config.PLACE_TOLERANCE_MM
    return err <= limit
