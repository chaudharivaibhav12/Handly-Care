"""Sidekick's care routines with the camera in the loop. Run from this folder; Sidekick (Handly-Care) is the parent folder.

    python care.py look [--execute]            # where is the person? forehead + hand-over point
    python care.py thermometer [--execute] [--aim]
    python care.py tissue [--execute]          # pull one tissue, hand it over in front of the person
    python care.py medicine [--execute]        # drawer check -> open -> pill box to the person -> close
    python care.py remember-drawer [--execute] # at sidekick's overview: remember where the yellow knob is
    python care.py check-drawer [--execute]    # has the drawer box moved since?
    python care.py go <pose>                   # go overview | go high | go look_person

Add --sim to any of them to run on the simulator (no robot), e.g.
    python care.py remember-drawer --sim --execute
    python care.py medicine --sim --execute --yes

Once, teach where the camera looks at the person (in the sidekick folder):
    python app.py joints look_person           # camera facing the chair, the face 60-80 cm away

What changed against the hackathon:
- thermometer: aims at the forehead it finds instead of a taught pose (thermometer.py).
- tissue / medicine: the arm goes to the taught 'handoff' first (the path that worked), then on to the
  person: in front of their face, wherever they sit (person.py), planned around their head and body,
  slowly. Too far from the taught spot, or no safe path: it hands over at the taught spot as before.
- medicine: before sidekick replays the drawer poses (blind), the camera checks that the drawer box is
  where those poses were taught (knob.py), and refuses if it moved.

Only `go` and --execute move the arm, and they ask before every stage (unless --yes).
It uses sidekick's .env, its taught poses (drawer_joints.json) and its robot code; it changes none of them.
"""
from __future__ import annotations

import argparse
import asyncio
import os
import sys
import traceback
from types import SimpleNamespace

import cv2
import numpy as np

import knob
import person
import robot_io
import settings
import thermometer
from face import FaceFinder
from geometry import reach_problems

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = "out"
AT_POSE_DEG = 2.0       # every joint this close to a taught pose = the arm is at that pose


class Refused(Exception):
    """A check failed, so the arm did not (or no longer) move."""


def _fmt(v) -> str:
    return "(" + ", ".join(f"{x:.0f}" for x in v) + ")"


# ---------------------------------------------------------------------------
# Sidekick, loaded from the parent folder (Handly-Care)
# ---------------------------------------------------------------------------


def sidekick_dir() -> str:
    path = os.path.normpath(os.path.join(HERE, settings.SIDEKICK_DIR))
    if not os.path.exists(os.path.join(path, "robot.py")):
        raise SystemExit(f"can't find sidekick at {path}: set SIDEKICK_DIR in settings.py")
    return path


def load_sidekick() -> SimpleNamespace:
    """sidekick's config, robot, taught and perception modules, with its files found from here."""
    path = sidekick_dir()
    if path not in sys.path:         # right after this folder, ahead of site-packages, as when sidekick runs
        sys.path.insert(sys.path.index(HERE) + 1 if HERE in sys.path else 0, path)   # (no names are shared)
    import config as sk_config
    for name in ("DRAWER_JOINTS_PATH", "CALIBRATION_PATH", "TRIALS_PATH", "STATUS_PATH"):
        value = getattr(sk_config, name, None)
        if isinstance(value, str) and value and not os.path.isabs(value):
            setattr(sk_config, name, os.path.join(path, value))
    import perception as sk_perception
    import robot as sk_robot
    import taught as sk_taught
    robot_io.load_env(os.path.join(path, ".env"))
    return SimpleNamespace(config=sk_config, robot=sk_robot, taught=sk_taught, perception=sk_perception, dir=path)


def rig_for(robot, sk) -> robot_io.Rig:
    """Our camera and geometry layer on sidekick's own connection (no second login)."""
    return robot_io.Rig(robot.machine, robot.camera, robot.motion, robot.arm, robot.gripper,
                        camera_name=sk.config.CAMERA, gripper_name=sk.config.GRIPPER,
                        correction=robot_io.load_camera_correction())


# ---------------------------------------------------------------------------
# Moving between sidekick's taught poses
# ---------------------------------------------------------------------------


def need(data, *names):
    missing = [n for n in names if n not in data]
    if missing:
        raise Refused("teach " + ", ".join(f"'{n}'" for n in missing) + " first. In the sidekick folder: "
                      + "  ".join(f"python app.py joints {n}" for n in missing))


def taught_xyz(pose) -> np.ndarray:
    return np.array([pose["x"], pose["y"], pose["z"]], dtype=np.float64)


def taught_ov(pose) -> tuple:
    return (pose["o_x"], pose["o_y"], pose["o_z"], pose["theta"])


async def at_pose(robot, data, name, tol=AT_POSE_DEG) -> bool:
    if name not in data:
        return False
    now = [float(v) for v in (await robot.arm.get_joint_positions()).values]
    return max(abs(a - b) for a, b in zip(now, data[name]["joints"])) <= tol


async def joint_path(robot, sk, data, names):
    """One joint-space move through taught poses at sidekick's drawer speed cap: exactly what its replay
    does for a 'move' step (minus the start-near-'high' check, so only for paths we came along)."""
    from viam.proto.component.arm import JointPositions, MoveOptions
    opts = MoveOptions(max_vel_degs_per_sec=sk.config.DRAWER_MAX_VEL_DEGS,
                       max_acc_degs_per_sec2=sk.config.DRAWER_MAX_ACC_DEGS)
    await sk.robot.retry("joint move", lambda: robot.arm.move_through_joint_positions(
        [JointPositions(values=[float(v) for v in data[n]["joints"]]) for n in names], options=opts))


def spokes() -> set:
    """Taught poses reached from 'high' by one joint move, and left the same way back."""
    return {settings.LOOK_POSE_NAME} | ({settings.KNOB_VIEW} - {"overview", "high"})


async def back_to_high(robot, sk, data):
    """To 'high': from a spoke pose along the joint path it was reached by (reversed), from anywhere else
    with sidekick's to_high (one joint at a time, height checked)."""
    for name in spokes():
        if await at_pose(robot, data, name):
            await joint_path(robot, sk, data, ["high"])
            return
    if not await at_pose(robot, data, "high"):
        await robot.to_high()


async def go(robot, sk, data, name, yes):
    """To sidekick's overview, to 'high', or to a spoke pose (look_person, KNOB_VIEW) via 'high'.
    The drawer and pill poses are only reached by sidekick's own routines, never directly."""
    if name not in {"overview", "high"} | spokes():
        raise Refused(f"go knows: overview, high, {', '.join(sorted(spokes()))}. The drawer and pill poses are "
                      f"reached only by sidekick's routines (python app.py drawer open | pills).")
    if name == "overview":
        await robot_io.ask("Press Enter to go to sidekick's overview", yes)
        if any([await at_pose(robot, data, s) for s in spokes()]):
            await back_to_high(robot, sk, data)
        await robot.go_overview()
        return
    need(data, "high", name)
    if await at_pose(robot, data, name):
        return
    await robot_io.ask(f"Press Enter to go to '{name}'" + ("" if name == "high" else " (via 'high')"), yes)
    await back_to_high(robot, sk, data)
    if name != "high":
        await joint_path(robot, sk, data, [name])


async def go_look(robot, sk, data, yes):
    await go(robot, sk, data, settings.LOOK_POSE_NAME, yes)


async def leave_look(robot, sk, data):
    """Back to 'high' along the joint path go_look came (reversed). From anywhere else: sidekick's to_high."""
    await back_to_high(robot, sk, data)


async def at_overview(rig, sk) -> bool:
    xyz, _ = await rig.gripper_pose()
    return float(np.linalg.norm(xyz - np.asarray(sk.config.OVERVIEW[:3], dtype=np.float64))) <= settings.OVERVIEW_MATCH_MM


async def at_knob_view(robot, rig, sk, data) -> bool:
    """Is the arm where the drawer check looks from (settings.KNOB_VIEW)?"""
    if settings.KNOB_VIEW == "overview":
        return await at_overview(rig, sk)
    return await at_pose(robot, data, settings.KNOB_VIEW)


# ---------------------------------------------------------------------------
# The person and the hand-over
# ---------------------------------------------------------------------------


async def find_person(rig, finder):
    """Two looks at the person: forehead, where the camera was, and the hand-over point."""
    fix, cap, gap = await thermometer.two_looks(rig, finder)
    hand = person.handover_point(fix.point_w, cap.camera_position)
    print(f"Person: {thermometer.describe(fix)}; two looks agreed within {gap:.1f} mm")
    print(f"Hand-over point, in front of them: {_fmt(hand)} mm (world)")
    view = thermometer.save_view(cap, fix, [thermometer.describe(fix)[:60], f"hand-over {_fmt(hand)}"], tag="person")
    print(f"Picture: {view}")
    return SimpleNamespace(fix=fix, cap=cap, hand=hand, camera=cap.camera_position)


def handover_plan(data, who):
    """(gripper frame target or None, orientation, mm from the taught 'handoff', why not).
    The orientation is the taught 'handoff' one: the item hangs the way it did at the hackathon."""
    h = data["handoff"]["pose"]
    ov = taught_ov(h)
    target = robot_io.frame_for_fingertips(who.hand, ov)
    shift = float(np.linalg.norm(target - taught_xyz(h)))
    if shift > settings.MAX_HANDOVER_SHIFT_MM:
        return None, ov, shift, (f"you are {shift:.0f} mm from the taught 'handoff' spot "
                                 f"(max {settings.MAX_HANDOVER_SHIFT_MM:.0f})")
    problems = reach_problems(target, settings.ARM_BASE_XY, settings.REACH_MM, settings.Z_LIMITS_MM)
    if problems:
        return None, ov, shift, "the arm can't go there: " + "; ".join(problems)
    return target, ov, shift, ""


def print_handover_plan(data, who):
    if "handoff" not in data:
        print("No 'handoff' taught yet (sidekick: python app.py joints handoff).")
        return
    target, _, shift, why = handover_plan(data, who)
    if target is None:
        print(f"Hand-over: at the taught 'handoff' spot, because {why}.")
    else:
        print(f"Hand-over: taught 'handoff' spot first, then {shift:.0f} mm on to in front of the person.")


def person_world_state(who, others=None):
    """The person (head sphere + body box) for the motion planner, plus any WorldState already in use."""
    from viam.proto.common import WorldState
    obstacles = list(others.obstacles) if others is not None else []
    if settings.USE_PERSON_OBSTACLES:
        obstacles += person.obstacles(who.fix.point_w, who.camera)
    return WorldState(obstacles=obstacles) if obstacles else None


async def hand_over(robot, sk, data, who, hold_s) -> bool:
    """At the taught 'handoff', holding the item: move it to in front of the person (around them, slowly),
    hold, let go, and come back to the taught 'handoff'. If that isn't possible, hand it over right here
    (what the hackathon did). True if it went to the person."""
    target, ov, shift, why = handover_plan(data, who)
    avoid = person_world_state(who)
    went = False
    if target is None:
        print(f"Handing over at the taught spot: {why}.")
    else:
        try:
            await robot.move_to(float(target[0]), float(target[1]), float(target[2]), theta=ov[3],
                                o=tuple(ov[:3]), avoid=avoid, max_vel=settings.HANDOVER_MAX_VEL_DEGS)
            went = True
            print(f"In front of the person ({shift:.0f} mm on from the taught spot).")
        except Exception as e:  # noqa: BLE001  the item is still held: hand it over here instead
            if sk.robot.arm_halted(e):
                raise
            print(f"No safe path to the person ({str(e)[:160]}); handing over at the taught spot.")
    await asyncio.sleep(hold_s)
    await robot.gripper.open()
    if went:
        h = data["handoff"]["pose"]
        await robot.move_to(h["x"], h["y"], h["z"], theta=h["theta"], o=(h["o_x"], h["o_y"], h["o_z"]), avoid=avoid)
    return went


async def drawer_check(rig, saved=None) -> float:
    cap = await rig.capture()
    found = knob.find_knobs(cap)
    os.makedirs(OUT, exist_ok=True)
    cv2.imwrite(os.path.join(OUT, "drawer_check.png"), knob.draw(cap, found))
    moved, _ = knob.check(cap, saved if saved is not None else knob.load())
    return moved


def moved_message(moved: float) -> str:
    return (f"the drawer box is {moved:.0f} mm from where its poses were taught (max "
            f"{settings.DRAWER_MOVED_TOLERANCE_MM:.0f}). Put it back on its marks and close the drawer, "
            f"or re-teach the drawer poses and run remember-drawer again. Picture: {OUT}/drawer_check.png")


# ---------------------------------------------------------------------------
# The routines
# ---------------------------------------------------------------------------


async def back_to_look(robot, data, log):
    """After measuring, the thermometer is at the pre-approach point in front of the face. Plan back to the
    look pose around the person, so the way to 'high' is the taught joint path again. If the planner
    refuses, stop there (no unplanned move next to a face)."""
    from viam.proto.common import WorldState
    name = settings.LOOK_POSE_NAME
    if "forehead" not in log or await at_pose(robot, data, name):
        return
    look = data[name]["pose"]
    avoid = (WorldState(obstacles=person.obstacles(np.asarray(log["forehead"], dtype=np.float64), taught_xyz(look)))
             if settings.USE_PERSON_OBSTACLES else None)
    await robot.move_to(look["x"], look["y"], look["z"], theta=look["theta"],
                        o=(look["o_x"], look["o_y"], look["o_z"]), avoid=avoid)


async def thermometer_flow(robot, rig, finder, data, sk, execute=False, yes=False, aim=False):
    settings.LOOK_AT_PERSON_JOINTS = None          # care.py moves to and from the look pose itself
    if execute:
        await go_look(robot, sk, data, yes)
    log = await thermometer.run(rig, finder, execute=execute, yes=yes, aim=aim)
    if execute:
        await back_to_look(robot, data, log)
        await leave_look(robot, sk, data)
    return log


async def tissue_flow(robot, rig, finder, data, sk, execute=False, yes=False):
    cfg, move_error, halted = sk.config, sk.robot.MoveError, sk.robot.arm_halted
    if execute:
        need(data, "high", "handoff", settings.LOOK_POSE_NAME)
        await go_look(robot, sk, data, yes)
    who = await find_person(rig, finder)
    print_handover_plan(data, who)
    if not execute:
        print("Dry run: nothing moved. Add --execute to fetch a tissue.")
        return "dry run"
    await leave_look(robot, sk, data)

    await robot_io.ask("Press Enter to look for the tissue box from the overview", yes)
    scene = await sk.perception.find(robot)        # sidekick: overview, height map, names the objects
    names = [o for o, p in scene.items() if p is not None and "tissue" in o]
    if not names:
        raise Refused(f"I can't see a tissue box. I see: {[o for o, p in scene.items() if p is not None]}")
    box = names[0]
    (x, y), (w, d, h) = scene[box], cfg.OBJECTS[box]["size"]
    print(f"  {box} at ({x:.0f}, {y:.0f}), {w:.0f}x{d:.0f} mm, top {h:.0f} mm")
    others = {o: p for o, p in scene.items() if o != box and p is not None}
    avoid = sk.robot.world_state(others) if others else None
    grip_z = cfg.TABLE_Z + h - cfg.TISSUE_GRIP_BELOW_TOP_MM
    await robot_io.ask("Press Enter to pull one tissue and bring it to the person", yes)

    # sidekick's own tissue routine (robot.py RealRobot.tissue); only the hand-over is new.
    await robot.gripper.open()
    try:
        await robot.move_to(x, y, cfg.TABLE_Z + h + cfg.HOVER, avoid=avoid)           # above the tissue
    except Exception as e:  # noqa: BLE001
        if halted(e):
            raise
        raise move_error("no safe path to the tissues", holding=False) from e
    await robot.move_to(x, y, grip_z, straight=True)                                  # down into the puff
    await robot.gripper.grab()
    await robot.move_to(x, y, grip_z + cfg.TISSUE_PULL_MM, straight=True)            # pull one out
    hp = data["handoff"]["pose"]
    try:
        await robot.move_to(hp["x"], hp["y"], hp["z"], theta=hp["theta"], o=(hp["o_x"], hp["o_y"], hp["o_z"]),
                            avoid=person_world_state(who, avoid))                     # the taught hand-over spot
    except Exception as e:  # noqa: BLE001
        if halted(e):
            raise
        raise move_error("no safe path to the hand-over spot; I'm holding the tissue up high", holding=True) from e
    await hand_over(robot, sk, data, who, cfg.HANDOFF_HOLD_S)
    await robot.to_high()
    return "handed over"


def pills_steps(sk, data, hold_s):
    """sidekick's pills sequence, split where it reaches 'handoff': (steps up to there, the way back)."""
    seq = sk.taught.pills_sequence(data, hold_s)
    ends = [i for i, (kind, names) in enumerate(seq) if kind == "move" and names and names[-1] == "handoff"]
    if not ends:
        raise Refused("sidekick's pills sequence never reaches 'handoff'")
    back = [names for kind, names in seq[ends[0] + 1:] if kind == "move"]
    return seq[:ends[0] + 1], (back[-1] if back else ["high"])


async def medicine_flow(robot, rig, finder, data, sk, execute=False, yes=False):
    cfg = sk.config
    if not execute:
        who = await find_person(rig, finder)
        print_handover_plan(data, who)
        print("Dry run: nothing moved. The drawer check runs from the overview: python care.py check-drawer --execute."
              " Add --execute to bring the medicine.")
        return "dry run"
    need(data, "high", "handoff", "pill_above", "pill_grip", settings.LOOK_POSE_NAME)
    saved = knob.load()                            # before anything moves: was the drawer remembered?
    to_handoff, way_back = pills_steps(sk, data, cfg.HANDOFF_HOLD_S)

    # 1. Is the drawer box where its poses were taught? sidekick replays them with no camera.
    await go(robot, sk, data, settings.KNOB_VIEW, yes)
    moved = await drawer_check(rig, saved)
    if moved > settings.DRAWER_MOVED_TOLERANCE_MM:
        raise Refused(moved_message(moved))
    print(f"Drawer box: {moved:.0f} mm from where it was taught. OK.")

    # 2. Where is the person?
    await go_look(robot, sk, data, yes)
    who = await find_person(rig, finder)
    print_handover_plan(data, who)
    await leave_look(robot, sk, data)

    # 3. sidekick's drawer and pills routines, with the hand-over in front of the person.
    await robot_io.ask("Press Enter to open the drawer and take the medicine out", yes)
    await robot.drawer("open")
    try:
        await robot.replay(data, to_handoff)
    except sk.robot.MoveError:
        print("The drawer is still open. Close it with:  python app.py drawer close  (in the sidekick folder)")
        raise
    await hand_over(robot, sk, data, who, cfg.HANDOFF_HOLD_S)
    if await at_pose(robot, data, "handoff", tol=3.0):
        await joint_path(robot, sk, data, way_back)      # back the way sidekick's pills routine goes
    else:
        await robot.to_high()
    await robot.drawer("close")
    return "handed over"


async def drawer_command(robot, rig, data, sk, step, execute, yes):
    view = settings.KNOB_VIEW
    if not await at_knob_view(robot, rig, sk, data):
        if not execute:
            raise Refused(f"the arm isn't at '{view}', where the medicine routine checks the drawer "
                          f"(KNOB_VIEW in settings.py). Add --execute to go there first (or: python care.py go {view}).")
        await go(robot, sk, data, view, yes)
    if step == "remember-drawer":
        cap = await rig.capture()
        found = knob.find_knobs(cap)
        os.makedirs(OUT, exist_ok=True)
        cv2.imwrite(os.path.join(OUT, "drawer_remember.png"), knob.draw(cap, found))
        saved = knob.remember(cap)
        extra = (f" I see {len(found)} yellow things: check that {OUT}/drawer_remember.png circles the knob."
                 if len(found) > 1 else "")
        print(f"Remembered the drawer knob at {_fmt(saved['knobs'][0])} mm (world), saved {settings.KNOB_FILE}.{extra}")
        print("Do this again whenever you re-teach the drawer poses.")
    else:
        moved = await drawer_check(rig)
        ok = moved <= settings.DRAWER_MOVED_TOLERANCE_MM
        print(f"Drawer box moved {moved:.0f} mm since it was remembered: "
              + ("OK." if ok else f"too far (max {settings.DRAWER_MOVED_TOLERANCE_MM:.0f}).")
              + f" Picture: {OUT}/drawer_check.png")


# ---------------------------------------------------------------------------
# Command line
# ---------------------------------------------------------------------------


async def connect(args):
    """(robot, rig, face finder, taught poses, sidekick modules): the real ones, or the simulator's."""
    if args.sim:
        import sim
        return sim.make_care()
    sk = load_sidekick()
    data = sk.taught.load()
    robot = await sk.robot.RealRobot.connect()
    finder = FaceFinder(score_threshold=settings.FACE_MIN_SCORE, up_ratio=settings.FOREHEAD_UP_RATIO)
    return robot, rig_for(robot, sk), finder, data, sk


async def run_step(args, robot, rig, finder, data, sk) -> int:
    if args.step == "go":
        await go(robot, sk, data, args.pose, args.yes)
        print(f"at '{args.pose}'")
    elif args.step == "look":
        if args.execute:
            await go_look(robot, sk, data, args.yes)
        who = await find_person(rig, finder)
        print_handover_plan(data, who)
    elif args.step in ("remember-drawer", "check-drawer"):
        await drawer_command(robot, rig, data, sk, args.step, args.execute, args.yes)
    elif args.step == "thermometer":
        log = await thermometer_flow(robot, rig, finder, data, sk, args.execute, args.yes, args.aim)
        print(f"thermometer: {log.get('result')}")
        return 0 if log.get("result") in ("measured", "dry run") else 1
    elif args.step == "tissue":
        print(f"tissue: {await tissue_flow(robot, rig, finder, data, sk, args.execute, args.yes)}")
    elif args.step == "medicine":
        print(f"medicine: {await medicine_flow(robot, rig, finder, data, sk, args.execute, args.yes)}")
    return 0


async def _main(args) -> int:
    try:
        robot, rig, finder, data, sk = await connect(args)
    except KeyError as e:
        print(f"STOPPED: {e.args[0]} is not set. Put VIAM_ADDRESS, VIAM_API_KEY_ID and VIAM_API_KEY in sidekick's .env")
        return 1
    try:
        return await run_step(args, robot, rig, finder, data, sk)
    except thermometer.NoFace as e:
        print(f"STOPPED: {e}\n(Is the arm at the look pose? python care.py go {settings.LOOK_POSE_NAME})")
        return 1
    except (Refused, knob.KnobNotFound, robot_io.UnsafeTarget) as e:
        print(f"STOPPED: {e}")
        return 1
    except sk.robot.MoveError as e:
        print(f"STOPPED: {e}" + (" -- STILL HOLDING IT: take it from the gripper" if getattr(e, "holding", False) else ""))
        return 1
    except asyncio.CancelledError:
        await robot.stop()
        raise
    except Exception as e:  # noqa: BLE001  a motion or connection error: say so plainly, keep the details
        traceback.print_exc()
        print(f"\nSTOPPED ({type(e).__name__}): {e}\nCheck the arm in the Viam app before trying again.")
        return 1
    finally:
        await robot.close()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("step", choices=["look", "thermometer", "tissue", "medicine", "remember-drawer", "check-drawer", "go"])
    ap.add_argument("pose", nargs="?", help="with go: overview, high or look_person")
    ap.add_argument("--execute", action="store_true", help="actually move (asks before each stage)")
    ap.add_argument("--yes", action="store_true", help="don't ask before moves")
    ap.add_argument("--aim", action="store_true", help="thermometer: turn to point straight at the forehead")
    ap.add_argument("--sim", action="store_true", help="use the simulator instead of the robot")
    args = ap.parse_args(argv)
    if args.step == "go" and not args.pose:
        ap.error(f"go needs a pose name, e.g.  python care.py go {settings.LOOK_POSE_NAME}")
    return robot_io.run_main(_main(args))


if __name__ == "__main__":
    raise SystemExit(main())
