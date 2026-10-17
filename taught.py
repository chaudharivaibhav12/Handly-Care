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
# Thermometer (Sat): therm_above = fingers open over the thermometer on its taped spot; therm_grip = fingers
# around it (button between the fingers); therm_aim = held, pointing at the ball (never at a person).
THERM_REQUIRED = ("high", "therm_above", "therm_grip", "therm_aim")


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


def therm_sequence(data, mode="press", show_s=2.0, hold_pos=None, press_pos=None, click_s=0.5):
    """Thermometer from its taped spot to pointing at the ball and back. Its button is one click, and the light
    goes off ~2 s later if the button stays pressed (Vaibhav, Sat 12 AM), so:
      press: gripper to hold_pos (holds the body, button NOT pressed) -> normal speed to therm_aim ->
             click: press_pos for click_s, then back to hold_pos (Sat 11 AM: held down it doesn't measure)
             -> show -> put it back
      fast:  grab (the grip clicks the button) -> ONE fast move grip -> aim -> show -> put it back"""
    missing = [n for n in THERM_REQUIRED if n not in data]
    if missing:
        raise KeyError(f"teach these first with `python app.py joints <name>`: {', '.join(missing)}")
    back = [("move", ["therm_grip"]), ("release", []), ("move", ["therm_above", "high"])]
    if mode == "fast":
        return [("release", []), ("move", ["therm_above"]), ("move", ["therm_grip"]), ("grab", []),
                ("fast", ["therm_aim"]), ("wait", [show_s])] + back
    if mode != "press":
        raise ValueError(f"THERM_MODE must be 'press' or 'fast', not {mode!r}")
    if hold_pos is None or press_pos is None:
        raise KeyError("measure THERM_HOLD_POS and THERM_PRESS_POS first with tools/gripper_pos.py "
                       "(or set THERM_MODE = 'fast')")
    return [("release", []), ("move", ["therm_above"]), ("move", ["therm_grip"]), ("set", [hold_pos]),
            ("move", ["therm_aim"]), ("click", [press_pos, hold_pos, click_s]), ("wait", [show_s])] + back


# Bottle cap (Sat 1:35 PM): a person holds the bottle upright on its spot.
#   cap_above  fingers open, tool pointing down, a few cm straight above the cap, centered on it
#   cap_grip   straight down from cap_above, fingers around the cap (not squeezed)
CAP_REQUIRED = ("high", "cap_above", "cap_grip")


def cap_sequence(data, twists=3, deg=-120.0):
    """Unscrew like a hand does: grip, twist, let go, turn back, grip again. On the last twist keep hold and lift
    straight up with the wrist still turned (turning back while holding would screw it on again)."""
    missing = [n for n in CAP_REQUIRED if n not in data]
    if missing:
        raise KeyError(f"teach these first with `python app.py joints <name>`: {', '.join(missing)}")
    steps = [("release", []), ("move", ["cap_above"]), ("move", ["cap_grip"])]
    for i in range(twists):
        steps += [("grab", []), ("turned", ["cap_grip", deg])]
        if i < twists - 1:
            steps += [("release", []), ("turned", ["cap_grip", 0.0])]
    return steps + [("turned", ["cap_above", deg])]


# Pouring (Sat 2 PM): the uncapped bottle stands on slot 2; the cup to fill is on slot 1.
#   bottle_approach  fingers open beside the bottle (side grip), not touching it
#   bottle_grip      fingers around the bottle, about halfway up
#   bottle_lift      same grip, lifted ~10 cm straight up
#   pour_above       bottle upright, its opening just above and beside the cup
#   pour             tilted so the water flows into the cup
POUR_REQUIRED = ("high", "bottle_approach", "bottle_grip", "bottle_lift", "pour_above", "pour")


def pour_sequence(data, hold_s=3.0):
    """Pick the bottle up, tilt it SLOWLY into the cup, hold, upright, and put it back where it was."""
    missing = [n for n in POUR_REQUIRED if n not in data]
    if missing:
        raise KeyError(f"teach these first with `python app.py joints <name>`: {', '.join(missing)}")
    return [("release", []), ("move", ["bottle_approach"]), ("move", ["bottle_grip"]), ("grab", []),
            ("move", ["bottle_lift", "pour_above"]),
            ("slow", ["pour"]), ("wait", [hold_s]), ("slow", ["pour_above"]),
            ("move", ["bottle_lift", "bottle_grip"]), ("release", []), ("move", ["bottle_approach", "high"])]
