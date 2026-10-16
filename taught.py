"""Joint poses taught by hand, by name (drawer_joints.json). Fri 5:10 PM: the motion planner would not plan
the tilted straight lines the drawer needs (EXP-007), so the drawer is replayed in joint space instead:
exactly the arm configuration Vaibhav jogged through, at a capped speed, no planner.

Names for the drawer (record in any order, re-recording a name replaces it):
  high      tilted, well above the box (safe place to turn the wrist)
  approach  fingers open, just in front of the knob
  grip      fingers around the knob, drawer shut
  pull1..N  points along the pull (every ~3 cm), still gripping
  open      drawer open far enough for the medicine box
  retreat   fingers backed off the knob after letting go

Names for taking the medicine out of the open drawer (Fri 5:58 PM; red cylinder stands in for the pill box,
always placed on the same marked spot inside the drawer):
  pill_above  tool pointing down, fingers open, well above the item and clear of the drawer walls
  pill_grip   fingers around the item (not squeezed)
  pill_lift   (optional) still holding, lifted straight up until the item clears the drawer walls
  handoff     where the patient takes it
"""
import json
import math
import os
import time

import config

REQUIRED = ("high", "approach", "grip", "open", "retreat")
PILLS_REQUIRED = ("high", "pill_above", "pill_grip", "handoff")


def load():
    try:
        with open(config.DRAWER_JOINTS_PATH) as f:
            return json.load(f)
    except (OSError, json.JSONDecodeError):
        return {}


def save(name, joints, pose):
    data = load()
    data[name] = {"joints": [round(v, 3) for v in joints], "pose": pose, "time": time.strftime("%a %H:%M:%S")}
    tmp = config.DRAWER_JOINTS_PATH + ".tmp"
    with open(tmp, "w") as f:
        json.dump(data, f, indent=1)
    os.replace(tmp, config.DRAWER_JOINTS_PATH)
    return data


def pulls(data):
    """pull1, pull2, ... in number order."""
    return sorted((k for k in data if k.startswith("pull") and k[4:].isdigit()), key=lambda k: int(k[4:]))


def _xyz(v):
    return tuple(v["pose"][k] for k in ("x", "y", "z"))


def add_close_stop(data, short_mm):
    """'close_stop': short_mm before 'grip' along grip -> first pull point, interpolated in joints (and pose).
    Fri 5:35 PM: closing all the way to 'grip' pushed the drawer ~1 cm too far, nudging the box back."""
    nxt = (pulls(data) or ["open"])[0]
    g, n = data["grip"], data[nxt]
    d = math.dist(_xyz(g), _xyz(n))
    f = min(short_mm / d, 0.9) if d > 0 else 0.0
    mix = lambda a, b: [x + (y - x) * f for x, y in zip(a, b)]
    pose = {k: g["pose"][k] + (n["pose"][k] - g["pose"][k]) * f for k in ("x", "y", "z")}
    data["close_stop"] = {"joints": mix(g["joints"], n["joints"]), "pose": pose}
    return data


def add_open_far(data, extra_mm):
    """'open_far': extra_mm past 'open', continuing the last pull segment (last pull point -> open), extrapolated
    in joints (and pose). Fri 6:31 PM: Vaibhav wanted the drawer ~3 cm further open than taught."""
    if extra_mm <= 0:
        return data
    prev = (pulls(data) or ["grip"])[-1]
    a, b = data[prev], data["open"]
    d = math.dist(_xyz(a), _xyz(b))
    if d <= 0:
        return data
    f = extra_mm / d
    data["open_far"] = {"joints": [y + (y - x) * f for x, y in zip(a["joints"], b["joints"])],
                        "pose": {k: b["pose"][k] + (b["pose"][k] - a["pose"][k]) * f for k in ("x", "y", "z")}}
    return data


def sequence(data, action="open"):
    """[(step, [joint pose names])]: steps are 'move' (one joint-space move through the names) or 'grab'/'release'."""
    missing = [n for n in REQUIRED if n not in data]
    if missing:
        raise KeyError(f"teach these first with `python app.py joints <name>`: {', '.join(missing)}")
    slide = ["grip"] + pulls(data) + ["open"] + (["open_far"] if "open_far" in data else [])
    if action != "open":
        slide.reverse()
        if "close_stop" in data:
            slide[-1] = "close_stop"
    start, end = slide[0], slide[-1]
    near = (lambda n: "approach" if n in ("grip", "close_stop") else "retreat")   # backed-off pose next to each end
    return [("release", []),
            ("move", ["high", near(start)]),
            ("move", [start]),
            ("grab", []),
            ("move", slide[1:]),
            ("release", []),
            ("move", [near(end), "high"])]


def pills_sequence(data, hold_s=3.0):
    """Drawer already open, arm at 'high': take the item out and hand it over, then back to 'high' the way it came.
    'grab?' = grab and stop if the gripper says it holds nothing."""
    missing = [n for n in PILLS_REQUIRED if n not in data]
    if missing:
        raise KeyError(f"teach these first with `python app.py joints <name>`: {', '.join(missing)}")
    return [("release", []),
            ("move", ["pill_above"]),
            ("move", ["pill_grip"]),
            ("grab?", []),
            ("move", (["pill_lift"] if "pill_lift" in data else []) + ["pill_above"]),
            ("move", ["handoff"]),
            ("wait", [hold_s]),
            ("release", []),
            ("move", ["pill_above", "high"])]
