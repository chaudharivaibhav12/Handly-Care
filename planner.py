"""Understanding: turn what the person said into a checked plan.

The AI model only ever outputs a small JSON plan:
    {"say": "...", "actions": [{"object": "marker", "destination": "to_me"},
                               {"object": "red_cube", "destination": "on:blue_cube"}]}
Destinations use the small grammar in places.py (zones, left_of:, next_to:, on:, ...).
Our own code then checks every action before the robot moves.
If no AI key is set (ANTHROPIC_API_KEY or OPENROUTER_API_KEY) (or the AI call fails), a keyword planner is used
instead, so the demo never depends on the network.
"""
import json
import os
import re

import config
import perception
import places

SYSTEM = """You plan tasks for a robot arm that moves small objects on a desk.
You may ONLY use the objects listed. The robot can do exactly one kind of action:
move one object to one destination. A destination is one of:
- a zone name from the list (a marked spot on the table)
- "left_of:<object>", "right_of:<object>", "in_front_of:<object>", "behind:<object>":
  beside another object, as seen by the PERSON (in front of = closer to the person)
- "next_to:<object>": beside another object, any free side
- "on:<object>": stacked on top of another object (only objects marked stackable)

Rules:
- Read ALL the person's messages in order. Later messages override earlier ones
  ("actually, leave the cup" removes it from the plan).
- Return the full set of moves the person wants, based on where things are NOW.
  Include moves that are already done if they are still wanted; the robot skips them.
- Order moves so an object is moved before something is placed relative to it.
- "Everything" or "the rest" means objects not already in a zone, unless the person
  clearly says otherwise.
- People don't know the object ids and describe things loosely ("the red one", "the cup",
  "the block"). The names were guessed from a photo taken from above and may be wrong
  ("red block" may be what they call a red cup). If exactly ONE listed object clearly
  matches by color or kind, use it and name it in "say". If several match, or none,
  return no actions and ask which one.
- If they ask for something not in the object list, or anything unsafe, return no
  actions and explain briefly in "say".
- Keep "say" to one short friendly sentence.

Reply with JSON only, no other text:
{"say": "<sentence>", "actions": [{"object": "<object id>", "destination": "<destination>"}]}"""


def describe(scene):
    lines = ["Objects:"]
    for obj, info in config.OBJECTS.items():
        pos = scene.get(obj)
        if pos is None:
            where = "not visible"
        else:
            zone = perception.zone_of(pos)
            where = f"at x={pos[0]:.0f} y={pos[1]:.0f} mm" + (f", in zone {zone}" if zone else "")
        stack = ", stackable" if info.get("stackable") else ""
        lines.append(f"- {obj}: {info['description']}{stack} ({where})")
    lines.append("Zones: " + ", ".join(config.ZONES))
    return "\n".join(lines)


def parse_reply(text):
    """Pull the JSON object out of the model's reply. Returns None if there isn't one."""
    match = re.search(r"\{.*\}", text or "", re.S)
    if not match:
        return None
    try:
        data = json.loads(match.group(0))
    except json.JSONDecodeError:
        return None
    return data if isinstance(data, dict) else None


def _model(name):
    import ai
    return ai.model(name)


def plan_with_ai(requests, scene, client=None):
    if client is None:
        import ai
        # A hung call would freeze the robot mid-demo: short timeout, no silent
        # retries. On failure make_plan falls back to the keyword planner.
        client = ai.client(config.PLANNER_TIMEOUT_S)
    convo = "\n".join(f"{i+1}. {r}" for i, r in enumerate(requests))
    msg = client.messages.create(
        model=_model(config.PLANNER_MODEL),
        max_tokens=600,
        # No `temperature`: current models (Sonnet 5.5) reject sampling params with a 400,
        # which silently sent every plan to the keyword fallback. Low effort = fast, short JSON.
        output_config={"effort": "low"},
        system=SYSTEM,
        messages=[{"role": "user", "content": f"{describe(scene)}\n\nThe person said:\n{convo}"}],
    )
    text = "".join(getattr(b, "text", "") for b in msg.content)
    return parse_reply(text)


# ---- Offline fallback: enough keywords for the demo script. Whole words only.
_ALL_WORDS = {"red": "red_cube", "blue": "blue_cube", "green": "green_cube",
              "cup": "cup", "mug": "cup", "marker": "marker", "yellow": "marker", "pen": "marker"}
ZONE_WORDS = {"tray": "tray", "left": "left", "right": "right", "me": "to_me"}
REL_PATTERNS = [
    (r"\bon top of\b|\bonto\b|\bon\b", "on"),
    (r"\bnext to\b|\bbeside\b", "next_to"),
    (r"\bleft of\b", "left_of"),
    (r"\bright of\b", "right_of"),
    (r"\bin front of\b", "in_front_of"),
    (r"\bbehind\b", "behind"),
]


_STOP = {"the", "and", "for", "with", "object", "small", "big", "large", "thing"}


def words():
    found = {w: o for w, o in _ALL_WORDS.items() if o in config.OBJECTS}
    if config.MODE == "anything":     # discovered objects: every word of their name ("blue mug")
        for o, info in config.OBJECTS.items():
            for lab in info.get("labels", []) + [o.replace("_", " ")]:
                for w in lab.lower().split():
                    if len(w) > 2 and w not in _STOP:
                        found.setdefault(w, o)
    return found


def _writer():
    """The object to hand over for "something to write with", or None."""
    if "marker" in config.OBJECTS:
        return "marker"
    pattern = r"writ|\bpens?\b|pencil|marker|highlighter|crayon"
    return next((o for o, i in config.OBJECTS.items()
                 if re.search(pattern, f"{o} {i.get('description', '')}") and i.get("graspable", True)), None)


def _has(word, text):
    return re.search(rf"\b{re.escape(word)}\b", text) is not None


def _objects_in(text):
    return {o for w, o in words().items() if _has(w, text)}


def _mentions(text):
    """Objects mentioned in `text`, in order of first mention: [(index, obj), ...]."""
    first = {}
    for w, o in words().items():
        m = re.search(rf"\b{re.escape(w)}\b", text)
        if m and (o not in first or m.start() < first[o]):
            first[o] = m.start()
    return sorted((i, o) for o, i in first.items())


def _relation(clause):
    """'red cube on top of the blue cube' -> ('red_cube', 'on:blue_cube'), else None."""
    ms = _mentions(clause)
    if len(ms) < 2:
        return None
    (i1, mover), (i2, anchor) = ms[0], ms[1]
    between = clause[i1:i2]
    for pattern, rel in REL_PATTERNS:
        if re.search(pattern, between):
            return mover, f"{rel}:{anchor}"
    return None


def plan_with_rules(requests, scene):
    actions, skip, say = {}, set(), "On it."
    for r in requests:
        t = r.lower()
        if re.search(r"\b(leave|skip|don'?t|except)\b", t):
            skip |= _objects_in(t)
            say = "Okay, I'll leave that one."
            continue
        if re.search(r"\bwrit(e|ing)\b", t) and _writer():
            actions[_writer()] = "to_me"
            skip.discard(_writer())
            say = "Here's something to write with."
            continue
        found = False
        # e.g. "cup in the tray, the red cube on top of the blue one, everything else to the left"
        for clause in re.split(r"[,.;]|\band\b|\bthen\b", t):
            rel = _relation(clause)
            if rel:
                mover, dest = rel
                actions[mover] = dest
                skip.discard(mover)
                found = True
                continue
            zone = next((z for w, z in ZONE_WORDS.items() if _has(w, clause)), None)
            if not zone:
                continue
            if _has("everything", clause) or _has("rest", clause):
                for o in config.OBJECTS:
                    if scene.get(o) is not None and perception.zone_of(scene[o]) is None:
                        actions.setdefault(o, zone)
                        found = True
            for o in _objects_in(clause):
                actions[o] = zone
                skip.discard(o)
                found = True
        if not found:
            # e.g. "pick up my laptop": say the measured reason if we know it
            bad = [o for o in _objects_in(t) if config.OBJECTS[o].get("graspable") is False]
            if bad:
                info = config.OBJECTS[bad[0]]
                return {"say": f"I can't pick up the {bad[0].replace('_', ' ')}: {info['why_not']}.",
                        "actions": []}
            return {"say": "Sorry, I can't do that safely.", "actions": []}
    return {"say": say,
            "actions": [{"object": o, "destination": d} for o, d in actions.items() if o not in skip]}


def make_plan(requests, scene, client=None):
    raw = None
    note = None
    import ai
    if client is not None or ai.available():
        try:
            raw = plan_with_ai(requests, scene, client)
            if raw is None:
                note = "AI reply wasn't valid JSON; used keyword planner"
        except Exception as e:  # noqa: BLE001
            note = f"AI planner unavailable ({type(e).__name__}); used keyword planner"
    if raw is None:
        raw = plan_with_rules(requests, scene)
    plan = validate(raw, scene)
    if note:
        plan["problems"].insert(0, note)
    return plan


def validate(raw, scene):
    """Never trust the plan blindly. Drop anything the robot can't or shouldn't do.

    Returns:
      actions:  moves still to do, in a safe order (anchors before what goes next to them)
      already:  wanted moves that are already true (kept so we can notice if
                someone undoes them later)
      problems: human-readable reasons for anything dropped
    """
    ok, already, problems = [], [], []
    acts = raw.get("actions", []) if isinstance(raw, dict) else []
    if not isinstance(acts, list):
        acts = []
    seen = set()
    movers = {a.get("object") for a in acts if isinstance(a, dict)}
    for a in acts:
        if not isinstance(a, dict):
            problems.append("ignored a malformed step")
            continue
        obj, dst = a.get("object"), a.get("destination")
        if obj not in config.OBJECTS:
            problems.append(f"I don't know '{obj}'.")
            continue
        if obj in seen:
            continue  # first instruction for an object wins
        if config.OBJECTS[obj].get("graspable") is False:
            problems.append(f"I can't pick up the {obj.replace('_', ' ')}: {config.OBJECTS[obj]['why_not']}.")
            continue
        try:
            kind, ref = places.parse(dst)
        except places.PlaceError as e:
            problems.append(str(e))
            continue
        if config.MODE == "slots" and kind != "zone":
            problems.append("Right now I can only put things in the marked zones.")
            continue
        if scene.get(obj) is None:
            problems.append(f"I can't see the {obj.replace('_', ' ')}.")
            continue
        if kind != "zone" and ref == obj:
            problems.append("I can't put something next to itself.")
            continue
        if places.done(obj, dst, scene):
            already.append({"object": obj, "destination": dst})
            seen.add(obj)
            continue
        # Check it's possible now, unless the anchor is about to move first.
        if kind == "zone" or ref not in movers:
            try:
                places.resolve(obj, dst, scene)
            except places.PlaceError as e:
                problems.append(str(e))
                continue
        ok.append({"object": obj, "destination": dst})
        seen.add(obj)
    say = raw.get("say", "") if isinstance(raw, dict) else ""
    return {"say": str(say), "actions": order(ok), "already": already, "problems": problems}


def order(actions):
    """Move anchors before the things placed relative to them (stable otherwise)."""
    pending = list(actions)
    out = []
    while pending:
        movers = {a["object"] for a in pending}
        nxt = next((a for a in pending if places.anchor_of(a["destination"]) not in movers), pending[0])
        out.append(nxt)
        pending.remove(nxt)
    return out
