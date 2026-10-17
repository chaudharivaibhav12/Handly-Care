"""Sidekick: type what you want; the arm does it, checks its work, and adapts.

    python app.py --sim                 # no robot needed
    python app.py --sim --flaky 0.3     # sim where 30% of grabs miss (tests retries)
    python app.py --sim --feed demo.txt # scripted demo, incl. a mid-task interruption
    python app.py                       # real robot (needs .env, see README)
    python app.py check                 # sanity-check config.py (run after every edit)
    python app.py where                 # print gripper pose  -> teach slots/zones/overview
    python app.py resources             # list what the machine exposes
    python app.py overview              # move to the overview pose
    python app.py snapshot              # save overview image + detection pixels ("slots" mode)
    python app.py test red_cube slot1 tray   # one move between taught spots (Milestone 1)
    python app.py joints <name>|list        # record a hand-taught joint pose (drawer: high approach grip pull1.. open retreat)
    python app.py drawer open|close         # the medicine drawer: replays the taught joint poses (taught.py)
    python app.py pills | medicine          # take the pill box out of the open drawer and hand it over (+ open/close)
    python app.py tissue [name]             # look, pinch one tissue from the pop-up box, hand it over, end at 'high'
    python app.py to_high                   # safely to the drawer's 'high' pose, one joint at a time
    python app.py thermometer               # from 'high': pick up the thermometer, point it at the ball, put it back
    python app.py cap | pour | water        # from 'high': unscrew the cap -> slot 3 cup | fill the slot 1 cup | both
    python app.py objects               # what the camera sees, in world mm (Milestone 2)
    python app.py calibrate red_cube=slot1 blue_cube=slot2 cup=slot3
                                        # objects placed on taught spots -> camera correction
    python app.py pick red_cube tray    # find it anywhere, put it in a zone (Milestone 3)
    python app.py stats                 # success rate + mm error per object from all test/pick runs

While a task runs, type a new message (e.g. "actually, leave the cup").
It's picked up at the next safe checkpoint.
"""
import argparse
import asyncio
import sys
import threading

import math
import os
import random
import signal

import calibration
import checks
import config
import executor
import perception
import places
import trials
import vision
from robot import MoveError, RealRobot, SimRobot

# Scattered on the table, not on taught spots: the sim exercises "pick from anywhere".
SIM_LAYOUT = {"red_cube": (480, -120), "blue_cube": (500, 90), "cup": (420, 200), "marker": (380, -40)}  # tests' table


def sim_layout():
    """`--sim` starts each object on a real taught slot (red_cube on slot1, ...), slightly off-center."""
    return {o: (x + 6, y - 4) for o, (x, y) in zip(config.OBJECTS, config.SLOTS.values())}


def describe_table(robot):
    return ", ".join(
        f"{o} {'gone' if p is None else f'({p[0]:.0f},{p[1]:.0f})' + (f' [{perception.zone_of(p)}]' if perception.zone_of(p) else '')}"
        for o, p in robot.seen().items())


def load_env():
    try:
        from dotenv import load_dotenv
        load_dotenv()
    except ImportError:
        pass


def start_stdin_reader(loop, queue):
    def reader():
        for line in sys.stdin:
            if line.strip():
                loop.call_soon_threadsafe(queue.put_nowait, line.strip())
        loop.call_soon_threadsafe(queue.put_nowait, None)
    threading.Thread(target=reader, daemon=True).start()


def read_feed(path):
    """Lines 'seconds_to_wait|message'; blank lines and # comments ignored."""
    items = []
    for raw in open(path):
        if not raw.strip() or raw.lstrip().startswith("#"):
            continue
        delay, text = raw.split("|", 1)
        items.append((float(delay), text.strip()))
    return items


def start_feed(loop, queue, items):
    async def feed():
        try:
            for delay, text in items:
                await asyncio.sleep(delay)
                queue.put_nowait(text)
                try:
                    print(f"\n👤 {text}")
                except Exception:  # noqa: BLE001  (a console that can't print must not stop the demo)
                    pass
        finally:
            queue.put_nowait(None)   # always end, so the app never waits forever
    return loop.create_task(feed())


def safe_console():
    """Windows consoles and pipes often can't print emoji (cp1252). Never crash on output."""
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except (AttributeError, ValueError):
            pass


async def interactive(robot, feed=None, status=None):
    queue = asyncio.Queue()
    loop = asyncio.get_running_loop()
    if feed:
        start_feed(loop, queue, read_feed(feed) if isinstance(feed, str) else feed)
    else:
        start_stdin_reader(loop, queue)
        print("Sidekick ready. What would you like? (Ctrl+D to quit)")
    status = status or executor.Status()
    while True:
        request = await queue.get()
        if request is None:
            return status
        # Messages typed while the task runs land in the same queue: the inbox.
        await executor.run_task(robot, request, queue, status)
        if hasattr(robot, "pos"):
            print("   table now:", describe_table(robot))


async def run_command(robot, cmd):
    if cmd[0] == "where":
        print(await robot.where())
    elif cmd[0] == "resources":
        for n in sorted(r.name for r in robot.machine.resource_names):
            print(" ", n)
    elif cmd[0] == "snapshot":
        await vision.snapshot(robot)
    elif cmd[0] == "overview":
        await robot.go_overview()
    elif cmd[0] == "test" and len(cmd) == 4:
        obj, src, dst = cmd[1:]
        if obj not in config.OBJECTS or src not in config.ALL_SPOTS or dst not in config.ALL_SPOTS:
            print(f"usage: test <object> <from> <to>\n objects: {list(config.OBJECTS)}\n"
                  f" spots: {list(config.ALL_SPOTS)}")
            return False
        ok = await robot.move_object(obj, config.ALL_SPOTS[src], config.ALL_SPOTS[dst])
        print("done" if ok else "grab missed")
        note = ask_note("did it land right? (y/n/comment, Enter to skip): ")
        trials.log("test", obj, src, dst, ok, note=note)
        return ok
    elif cmd[0] == "objects":
        scene = await perception.find(robot)
        for obj, pos in scene.items():
            if pos is None:
                print(f"  {obj:10s} not seen")
                continue
            zone = perception.zone_of(pos)
            reach = "" if places.reachable(*pos) else "   <- OUT OF REACH"
            info = config.OBJECTS.get(obj, {})
            extra = ""
            if info.get("found"):           # "anything" mode: what was measured
                w, d, h = info["size"]
                extra = f"  {w:.0f}x{d:.0f}x{h:.0f} mm, grip at {info['grasp_height']:.0f}" + \
                    (" (near the top)" if info.get("grip") == "top" else "")
                if not info.get("graspable", True):
                    extra += f"  <- CAN'T GRAB: {info['why_not']}"
            print(f"  {obj:10s} x={pos[0]:6.0f} y={pos[1]:6.0f}" + (f"  in {zone}" if zone else "") + reach + extra)
        return any(p is not None for p in scene.values())
    elif cmd[0] == "calibrate":
        return await calibrate(robot, cmd[1:])
    elif cmd[0] == "pick" and len(cmd) == 3:
        obj, dst = cmd[1:]
        scene = await perception.find(robot)        # first: in "anything" mode this is what names things
        if obj in config.ALL_SPOTS and obj not in config.OBJECTS:   # `pick slot1 tray`: whatever sits on slot1
            obj = object_on(scene, config.ALL_SPOTS[obj]) or obj
            print(f"  on that spot: {obj}")
        if obj not in config.OBJECTS or dst not in config.ZONES:
            print(f"usage: pick <object> <zone>\n objects: {list(config.OBJECTS)}\n zones: {list(config.ZONES)}")
            return False
        if config.OBJECTS[obj].get("graspable") is False:
            print(f"I can't pick up the {obj}: {config.OBJECTS[obj]['why_not']}")
            return False
        if scene.get(obj) is None:
            print(f"can't see the {obj}")
            return False
        if not places.reachable(*scene[obj]):
            print(f"the {obj} is out of reach at {scene[obj]}")
            return False
        tx, ty, lift = places.resolve(obj, dst, scene)
        try:
            ok = await robot.move_object(obj, scene[obj], (tx, ty), lift, others=scene)
        except MoveError as e:
            print(f"couldn't move it: {e}" + (" -- STILL HOLDING IT, take it from the gripper" if e.holding else ""))
            trials.log("pick", obj, f"{scene[obj][0]:.0f},{scene[obj][1]:.0f}", dst, False, None, note=f"move error: {e}")
            return False
        if ok:
            perception.moved(obj, (tx, ty))
        after = await perception.find(robot)
        err = None if after.get(obj) is None else math.dist(after[obj], (tx, ty))
        print(("placed" if ok else "grab missed") + (f", {err:.0f} mm from target" if err is not None else ", not seen after"))
        trials.log("pick", obj, f"{scene[obj][0]:.0f},{scene[obj][1]:.0f}", dst, ok, err,
                   note="" if after.get(obj) is not None else "not seen after")
        return ok
    elif cmd[0] == "joints" and len(cmd) == 2:
        import taught
        if cmd[1] == "list":
            for n, v in taught.load().items():
                p = v["pose"]
                print(f"  {n:9s} {v['time']}  x {p['x']:.0f} y {p['y']:.0f} z {p['z']:.0f}  joints {v['joints']}")
            return True
        j = await robot.arm.get_joint_positions()
        p = await robot.where()
        pose = {k: round(getattr(p, k), 2) for k in ("x", "y", "z", "o_x", "o_y", "o_z", "theta")}
        taught.save(cmd[1], list(j.values), pose)
        print(f"saved '{cmd[1]}': joints {[round(v, 1) for v in j.values]}  at x {p.x:.0f} y {p.y:.0f} z {p.z:.0f}")
        return True
    elif cmd[0] == "drawer" and len(cmd) == 2 and cmd[1] in ("open", "close"):
        try:
            moved = await robot.drawer(cmd[1])
        except KeyError as e:
            print(f"can't run the drawer yet: {e.args[0]}")
            return False
        except MoveError as e:
            print(f"couldn't {cmd[1]} the drawer: {e}")
            trials.log("drawer", "middle_drawer", "", cmd[1], False, None, note=f"move error: {e}")
            return False
        print(f"drawer {cmd[1]}: slid {moved:.0f} mm")
        note = ask_note("did it open/close fully? (y/n/comment, Enter to skip): ")
        trials.log("drawer", "middle_drawer", "", cmd[1], True, None, note=note)
        return True
    elif cmd[0] == "to_high" and len(cmd) == 1:
        try:
            await robot.to_high()
        except KeyError:
            print("teach 'high' first with `python app.py joints high`")
            return False
        except MoveError as e:
            print(e)
            return False
        print("at 'high'")
        return True
    elif cmd[0] == "tissue" and len(cmd) <= 2:
        # Look, find the tissue box (Claude names it; or give the name), pull one tissue, hand it over.
        scene = await perception.find(robot)
        want = cmd[1] if len(cmd) == 2 else None
        names = [o for o, p in scene.items() if p is not None and (o == want or (want is None and "tissue" in o))]
        if not names:
            print(f"I can't see a tissue box. I see: {[o for o, p in scene.items() if p is not None]}")
            await back_to_high(robot)
            return False
        box = names[0]
        w, d, h = config.OBJECTS[box]["size"]
        print(f"  {box} at ({scene[box][0]:.0f}, {scene[box][1]:.0f}), {w:.0f}x{d:.0f} mm, top {h:.0f} mm")
        try:
            await robot.tissue(scene[box], h, others={o: p for o, p in scene.items() if o != box},
                               hold_s=config.HANDOFF_HOLD_S)
        except KeyError as e:
            print(f"can't do that yet: {e.args[0]}")
            return False
        except MoveError as e:
            print(f"stopped: {e}" + (" -- STILL HOLDING IT" if e.holding else ""))
            trials.log("tissue", box, f"{scene[box][0]:.0f},{scene[box][1]:.0f}", "handoff", False, None, note=f"move error: {e}")
            if not e.holding:
                await back_to_high(robot)
            return False
        print("tissue: done")
        note = ask_note("one tissue handed over? (y/n/comment, Enter to skip): ")
        trials.log("tissue", box, f"{scene[box][0]:.0f},{scene[box][1]:.0f}", "handoff", True, None,
                   note=note or "ran (outcome not checked)")
        return True
    elif cmd[0] in ("pills", "medicine") and len(cmd) == 1:
        # pills: drawer already open, arm at 'high'. medicine: open the drawer, hand the pills over, close it.
        try:
            if cmd[0] == "medicine":
                await robot.drawer("open")
            await robot.pills(config.HANDOFF_HOLD_S)
            if cmd[0] == "medicine":
                await robot.drawer("close")
        except KeyError as e:
            print(f"can't do that yet: {e.args[0]}")
            return False
        except MoveError as e:
            print(f"stopped: {e}")
            trials.log(cmd[0], "pill_box", "drawer", "handoff", False, None, note=f"move error: {e}")
            return False
        print(f"{cmd[0]}: done")
        note = ask_note("did the patient get it? (y/n/comment, Enter to skip): ")
        trials.log(cmd[0], "pill_box", "drawer", "handoff", True, None, note=note or "ran (outcome not checked)")
        return True
    elif cmd[0] in ("cap", "pour", "water") and len(cmd) == 1:
        # From 'high'. cap: a person holds the bottle on slot 2; unscrew the cap, drop it in the slot 3 cup,
        # back to 'high'. pour: pick the open bottle up, fill the slot 1 cup, put it back. water: both (Sat 2 PM).
        step = "start"
        try:
            if cmd[0] in ("cap", "water"):
                step = "unscrewing the cap"
                await robot.cap()
                step = "dropping the cap"
                import taught
                spot = taught.load().get("cap_drop")
                if spot:      # Sat 2:30: the calibrated slot 3 missed the cup; a hand-shown spot over its centre wins
                    p = spot["pose"]
                    await robot.drop_cap((p["x"], p["y"]), config.CAP_CUP_HEIGHT_MM, drop_z=p["z"])
                else:
                    await robot.drop_cap(config.ALL_SPOTS[config.CAP_CUP_SPOT], config.CAP_CUP_HEIGHT_MM)
                step = "going back to 'high'"
                await robot.to_high()
            if cmd[0] in ("pour", "water"):
                step = "pouring"
                await robot.pour()
        except KeyError as e:
            print(f"can't do that yet: {e.args[0]}")
            return False
        except Exception as e:  # noqa: BLE001 -- MoveError or a planner refusal: stop where we are, say what
            print(f"stopped while {step}: {e}")
            trials.log(cmd[0], "bottle", "slot2", "slot1", False, None, note=f"stopped while {step}: {e}")
            return False
        print(f"{cmd[0]}: done")
        note = ask_note("did it work? (y/n/comment, Enter to skip): ")
        trials.log(cmd[0], "bottle", "slot2", "slot1", True, None, note=note or "ran (outcome not checked)")
        return True
    elif cmd[0] == "thermometer" and len(cmd) == 1:
        # From 'high': pick up the IR thermometer, point it at the ball (light visible), put it back.
        try:
            await robot.thermometer()
        except KeyError as e:
            print(f"can't do that yet: {e.args[0]}")
            return False
        except MoveError as e:
            print(f"stopped: {e}")
            trials.log("thermometer", "thermometer", "therm_spot", "ball", False, None, note=f"move error: {e}")
            return False
        print("thermometer: done")
        note = ask_note("light seen on the ball? (y/n/comment, Enter to skip): ")
        trials.log("thermometer", "thermometer", "therm_spot", "ball", True, None,
                   note=note or f"ran, mode {config.THERM_MODE} (outcome not checked)")
        return True
    elif cmd[0] == "stats":
        print(trials.report())
        return True
    else:
        print(__doc__)
        return False
    return True


async def back_to_high(robot):
    """After a failed tissue (nothing held): back to 'high' so the next routine can start (Sat 12 PM: the demo's
    medicine was refused because the arm was left at bird's-eye)."""
    try:
        await robot.to_high()
        print("back at 'high'")
    except Exception as e:  # noqa: BLE001
        print(f"couldn't get back to 'high': {e}")


def ask_note(prompt):
    """Ask the operator for a trial note; never crash without a keyboard (Fri 5:25: EOFError lost a drawer trial)."""
    if not sys.stdin.isatty() or os.environ.get("SIDEKICK_NO_PROMPT"):   # bridge.py: NUL looks like a TTY on Windows
        return ""
    try:
        return input(prompt).strip()
    except (EOFError, OSError):
        return ""


def object_on(scene, spot_xy, radius=60.0):
    """The object seen nearest a spot (within radius mm), or None."""
    near = sorted((math.dist(p, spot_xy), o) for o, p in scene.items() if p is not None)
    return near[0][1] if near and near[0][0] <= radius else None


def auto_pairs(raw, spots, radius=None):
    """`calibrate auto`: each taught slot paired with the one object seen nearest to it (within radius).
    Objects can be anything; their names don't matter. -> [(seen, true), ...]"""
    radius = config.AUTO_CALIBRATE_RADIUS_MM if radius is None else radius
    pairs, used = [], set()
    for spot, true in spots.items():
        near = sorted((math.dist(p, true), o) for o, p in raw.items() if p is not None and o not in used)
        if near and near[0][0] <= radius:
            o = near[0][1]
            used.add(o)
            pairs.append((raw[o], tuple(true)))
            print(f"  {spot}: {o} seen at ({raw[o][0]:.0f}, {raw[o][1]:.0f}), truth ({true[0]:.0f}, {true[1]:.0f}), "
                  f"off by {near[0][0]:.0f} mm")
        else:
            print(f"  {spot}: nothing within {radius:.0f} mm, skipped")
    return pairs


async def calibrate(robot, assignments):
    """`calibrate red_cube=slot1 blue_cube=slot2 cup=slot3`: objects sit on taught spots.
    `calibrate auto`: put any objects centered on slots/zones (spread out!); each spot takes the nearest one."""
    if assignments == ["auto"]:
        raw = await perception.find(robot, cal=dict(calibration.IDENTITY))
        # Every taught spot, zones too: points spread over the WHOLE table (slots alone left the tray,
        # 40 cm away, extrapolated -> a missed grab, Fri 2:24 PM).
        pairs = auto_pairs(raw, config.ALL_SPOTS)
        if not pairs:
            print("nothing near any slot: put objects centered on the taped X's, then check `python app.py objects`")
            return False
        return _fit_and_save(pairs)
    pairs_wanted = {}
    for a in assignments:
        obj, _, spot = a.partition("=")
        if obj not in config.OBJECTS or spot not in config.ALL_SPOTS:
            print(f"bad '{a}': use object=spot. objects: {list(config.OBJECTS)} spots: {list(config.ALL_SPOTS)}")
            return False
        pairs_wanted[obj] = config.ALL_SPOTS[spot]
    if not pairs_wanted:
        print("usage: calibrate red_cube=slot1 blue_cube=slot2 cup=slot3  (3+ spread-out spots is best)")
        return False
    raw = await perception.find(robot, cal=dict(calibration.IDENTITY))
    pairs = []
    for obj, true in pairs_wanted.items():
        seen = raw.get(obj)
        if seen is None:
            print(f"  {obj}: NOT SEEN, skipped")
            continue
        print(f"  {obj}: camera says ({seen[0]:.0f}, {seen[1]:.0f}), truth ({true[0]:.0f}, {true[1]:.0f}), "
              f"off by {math.dist(seen, true):.0f} mm")
        pairs.append((seen, true))
    if not pairs:
        print("nothing seen: check the segmenter and labels with `python app.py objects`")
        return False
    return _fit_and_save(pairs)


def _fit_and_save(pairs):
    cal = calibration.fit(pairs)
    calibration.save(cal)
    print(f"saved {config.CALIBRATION_PATH}: {cal['kind']} fit from {cal['points']} point(s); "
          f"average error {cal['before_mm']} mm -> {cal['after_mm']} mm")
    if cal["after_mm"] > 10:
        print("  still over 10 mm: re-check the taught spots, or that each object really is centered on its spot")
    if cal["kind"] != "affine":
        print("  tip: 3+ objects on spread-out spots (not in a line) also fixes rotation and scale")
    return True


def stop_on_break():
    """bridge.py's Stop sends Ctrl+Break (Windows): cancel this run so main() stops the arm like Ctrl+C does."""
    if not hasattr(signal, "SIGBREAK"):
        return
    loop, task = asyncio.get_running_loop(), asyncio.current_task()
    signal.signal(signal.SIGBREAK, lambda *_: loop.call_soon_threadsafe(task.cancel))


async def main(argv=None):
    safe_console()
    p = argparse.ArgumentParser()
    p.add_argument("command", nargs="*", default=[])
    p.add_argument("--sim", action="store_true")
    p.add_argument("--flaky", type=float, default=0.0)
    p.add_argument("--seed", type=int)
    p.add_argument("--feed")
    p.add_argument("--place-noise", type=float, default=6.0,
                   help="sim only: random placement error, mm (std dev); 0 = perfect")
    args = p.parse_args(argv)
    load_env()

    if args.command[:1] == ["check"]:
        problems = checks.config_problems(config)
        print("config looks good" if not problems else "\n".join("✗ " + x for x in problems))
        return not problems

    if args.sim:
        config.TRIALS_PATH = "trials-sim.csv"   # sim runs (e.g. bridge.py --sim) never mix into the real trials
        robot = await SimRobot.connect(sim_layout(), args.flaky, args.seed, move_seconds=0.5)
        if args.place_noise > 0:
            rng = random.Random(args.seed)
            robot.place_error = lambda obj, n: (rng.gauss(0, args.place_noise), rng.gauss(0, args.place_noise))
    elif args.command[:1] == ["resources"]:
        machine = await RealRobot.dial()
        try:
            for r in sorted(machine.resource_names, key=lambda r: (r.type, r.subtype, r.name)):
                if r.type in ("component", "service"):
                    print(f"  {r.type:9s} {r.subtype:16s} {r.name}")
        finally:
            await machine.close()
        return True
    else:
        robot = await RealRobot.connect()
    stop_on_break()
    try:
        if args.command:
            return await run_command(robot, args.command)
        await interactive(robot, args.feed)
        return True
    except (KeyboardInterrupt, asyncio.CancelledError):
        print("\nstopping the arm")
        await robot.stop()
        raise
    finally:
        await robot.close()


if __name__ == "__main__":
    try:
        sys.exit(0 if asyncio.run(main()) else 1)
    except (KeyboardInterrupt, asyncio.CancelledError):
        sys.exit(130)
