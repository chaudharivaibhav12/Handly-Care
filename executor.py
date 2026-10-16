"""The loop that makes Sidekick adaptive.

    observe -> plan goals -> move ONE object -> look again -> new message? -> re-plan ...

The plan is a set of GOALS ("blue_cube should be in the tray"), not a script.
After every object the robot looks at the whole table and recomputes what's
left, so it automatically:
  - retries an object that didn't land (up to config.MAX_RETRIES),
  - notices if something it already finished got moved, and fixes it,
  - follows the object if someone moved it before it was picked.
It only changes its mind at a safe checkpoint: after finishing an object and
returning to the overview pose. Never mid-motion.
"""
import json
import time

import math

import config
import perception
import places
import planner
from robot import MoveError


class Status:
    """Everything worth showing on screen. Also written to a JSON file for a dashboard."""

    def __init__(self, path=None, echo=True):
        self.path = path if path is not None else config.STATUS_PATH
        self.echo = echo
        self.data = {"requests": [], "scene": {}, "goals": {}, "remaining": [],
                     "done": [], "current": None, "state": "idle", "log": []}

    def log(self, text, kind="info"):
        self.data["log"].append({"t": time.strftime("%H:%M:%S"), "kind": kind, "text": text})
        if self.echo:
            icon = {"say": "🤖", "replan": "🔄", "ok": "✅", "fail": "⚠️ ", "info": "  "}.get(kind, "  ")
            print(f"{icon} {text}")
        self.save()

    def set(self, **kw):
        self.data.update(kw)
        self.save()

    def save(self):
        if not self.path:
            return
        try:
            with open(self.path, "w") as f:
                json.dump(self.data, f, indent=1)
        except OSError:
            pass  # a dashboard file must never stop the robot

    def kinds(self):
        return [e["kind"] for e in self.data["log"]]

    def texts(self):
        return [e["text"] for e in self.data["log"]]


def _pretty(obj):
    return obj.replace("_", " ")


def _next(goals, remaining):
    """First remaining object whose anchor isn't itself still waiting to move."""
    waiting = set(remaining)
    for o in remaining:
        if places.anchor_of(goals[o]) not in waiting:
            return o
    return remaining[0]


def _fmt(pos):
    return "nowhere" if pos is None else f"({pos[0]:.0f}, {pos[1]:.0f})"


async def run_task(robot, first_request, inbox, status, say=lambda text: None):
    """Carry out one request (plus any follow-ups typed while it runs).

    Goals are object -> destination ("tray", "left_of:cup", "on:blue_cube").
    After every object the robot looks at the whole table and re-checks EVERY goal,
    so it retries placements that landed off, and repairs arrangements that someone
    (or something) disturbed, including when an anchor object is moved.

    inbox: an asyncio.Queue; messages are read only at checkpoints.
    say:   hook for text-to-speech; the status log already prints.
    Returns the final scene.
    """
    requests = [first_request]
    attempts = {}        # object -> failed attempts
    gave_up = set()      # objects we stopped trying
    reached = {}         # object -> goal it reached at least once
    status.set(requests=list(requests), done=[], errors_mm=[], state="planning")

    def speak(text, kind="say"):
        if text:
            status.log(text, kind)
            if kind == "say":
                say(text)

    def adopt(plan):
        speak(plan["say"])
        for p in plan["problems"]:
            speak(p, "fail")
        return {a["object"]: a["destination"] for a in plan["already"] + plan["actions"]}

    scene = await perception.find(robot)
    goals = adopt(planner.make_plan(requests, scene))

    # Safety net: never loop forever (e.g. someone keeps moving things).
    budget = 3 * max(len(config.OBJECTS), 1) + 5

    while budget > 0:
        remaining = [o for o, d in goals.items() if o not in gave_up and not places.done(o, d, scene)]
        status.set(scene=scene, goals=goals, remaining=remaining, state="working")
        if not remaining:
            break

        obj = _next(goals, remaining)
        dst, src = goals[obj], scene.get(obj)
        if src is None:
            gave_up.add(obj)
            speak(f"I can't see the {_pretty(obj)} anymore, so I'm skipping it.", "fail")
            continue
        if not places.reachable(*src):
            gave_up.add(obj)
            speak(f"The {_pretty(obj)} is out of my reach.", "fail")
            continue
        try:
            tx, ty, lift = places.resolve(obj, dst, scene)
        except places.PlaceError as e:
            gave_up.add(obj)
            speak(str(e), "fail")
            continue
        if reached.get(obj) == dst:
            status.log(f"the {_pretty(obj)} is no longer where it should be; fixing it", "replan")

        budget -= 1
        status.set(current={"object": obj, "from": src, "destination": dst, "target": (tx, ty)},
                   state="moving")
        status.log(f"moving {obj} from {_fmt(src)} to {dst} at {_fmt((tx, ty))}")
        try:
            grabbed = await robot.move_object(obj, src, (tx, ty), lift, others=scene)
        except MoveError as e:
            # Fri 3:39: an unreachable carry crashed the app mid-task. Say why and go on with the rest.
            gave_up.add(obj)
            if e.holding:
                speak(f"I couldn't find a safe path and I'm still holding the {_pretty(obj)}. "
                      f"Please take it from me.", "fail")
                break
            speak(f"I couldn't find a safe path for the {_pretty(obj)} ({e}).", "fail")
            scene = await perception.find(robot)
            continue
        if grabbed:
            perception.moved(obj, (tx, ty))

        # ---- Checkpoint: look again at the whole table.
        scene = await perception.find(robot)
        status.set(scene=scene, current=None, state="checking")
        landed = scene.get(obj)
        if grabbed and places.done(obj, dst, scene):
            err = math.dist(landed, (tx, ty))
            reached[obj] = dst
            attempts.pop(obj, None)
            status.data["done"].append({"object": obj, "destination": dst})
            status.data["errors_mm"].append(round(err, 1))
            status.set(last_error_mm=round(err, 1))
            status.log(f"{obj} placed ({dst}), {err:.0f} mm from the target", "ok")
        else:
            attempts[obj] = attempts.get(obj, 0) + 1
            if attempts[obj] > config.MAX_RETRIES:
                gave_up.add(obj)
                speak(f"I couldn't place the {_pretty(obj)} well enough. Could someone help?", "fail")
            elif landed is not None and grabbed:
                status.log(f"{obj} landed {math.dist(landed, (tx, ty)):.0f} mm off; adjusting", "fail")
            else:
                status.log(f"check failed for {obj} (it's at {_fmt(landed)}); trying again", "fail")

        # ---- Anything new from the person? Only now, at a safe checkpoint.
        new = []
        while not inbox.empty():
            msg = inbox.get_nowait()
            if msg is None:          # end-of-input marker: put it back for the caller
                inbox.put_nowait(None)
                break
            new.append(msg)
        if new:
            requests.extend(new)
            status.log("new instruction; re-planning the remaining work", "replan")
            status.set(requests=list(requests), state="planning")
            goals = adopt(planner.make_plan(requests, scene))
    else:
        speak("I've stopped: too many attempts. Please check the table.", "fail")

    status.set(current=None, remaining=[], state="idle", scene=scene)
    status.log("All done.", "ok")
    return scene
