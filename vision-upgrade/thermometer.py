"""Point the thermometer at the person's forehead, wherever they are sitting.

Before: the arm went to one taught spot (it worked on a wall).
Now:    look -> find the face -> forehead pixel + depth = a 3D point -> stop the tip
        STANDOFF_MM in front of it, approaching slowly in a straight line.

    python thermometer.py --watch            # live: where is the forehead? (nothing moves)
    python thermometer.py                    # one look and the full plan (nothing moves)
    python thermometer.py --execute          # do it, asking before every move
    python thermometer.py --execute --yes    # no questions (only after it worked 5/5 on a paper face)
    python thermometer.py --sim --execute --yes   # the whole thing on the simulator

Before a real person: tape a printed face to a box at head height and run on that first.
Keep a hand on the E-stop. The person sits still with their hands down.
"""
from __future__ import annotations

import argparse
import asyncio
import json
import math
import os
import time
import traceback
from dataclasses import dataclass, field

import cv2
import numpy as np

import robot_io
import settings
from depth_tools import robust_depth, surface_normal
from face import Face, FaceFinder, draw_faces
from geometry import ForeheadApproach, angle_deg, deproject, in_box, matrix_to_ov, plan_forehead_approach, reach_problems, unit

OUT = "out"


class NoFace(Exception):
    pass


@dataclass
class ForeheadFix:
    point_w: np.ndarray      # forehead, world mm
    point_cam: np.ndarray
    depth_mm: float          # camera to forehead (along the lens axis)
    face: Face
    depth_from: str          # "forehead" or "mid-face"
    normal_w: np.ndarray | None = None   # which way the forehead faces (world), from the depth around it
    faces: list = field(default_factory=list)
    ignored: list = field(default_factory=list)


def locate_forehead(cap: robot_io.Capture, finder) -> ForeheadFix:
    """Forehead in world coordinates from one picture. Raises NoFace with the reasons."""
    faces = [f for f in finder.detect(cap.color) if f.score >= settings.FACE_MIN_SCORE]
    if not faces:
        raise NoFace("no face in the picture (is the person in view and facing the camera?)")
    lo, hi = settings.FACE_DISTANCE_MM
    fixes, ignored = [], []
    for f in faces:
        d = robust_depth(cap.depth, *f.forehead, f.patch_radius, settings.DEPTH_VALID_MM)
        d_mid = robust_depth(cap.depth, *f.mid_face, f.patch_radius, settings.DEPTH_VALID_MM)
        source = "forehead"
        if d is None or (d_mid is not None and abs(d - d_mid) > 60):
            if d_mid is None:
                ignored.append(f"face at pixel {f.forehead[0]:.0f},{f.forehead[1]:.0f}: no depth on it (too close?)")
                continue
            d, source = d_mid + 10.0, "mid-face"   # hair or a hole on the forehead: use the cheeks/nose
        if not lo <= d <= hi:
            ignored.append(f"face {d:.0f} mm away (allowed {lo:.0f}-{hi:.0f})")
            continue
        p_cam = deproject(f.forehead[0], f.forehead[1], d, cap.intr)
        p_w = cap.to_world(p_cam)
        if not in_box(p_w, settings.PERSON_ZONE):
            ignored.append(f"face at world {np.round(p_w).tolist()} is outside PERSON_ZONE")
            continue
        n_cam = surface_normal(cap.depth, cap.intr, *f.forehead, int(2.5 * f.patch_radius), settings.DEPTH_VALID_MM)
        n_w = None if n_cam is None else cap.T_world_cam[:3, :3] @ n_cam
        fixes.append(ForeheadFix(p_w, p_cam, d, f, source, n_w))
    if not fixes:
        raise NoFace("; ".join(ignored))
    best = min(fixes, key=lambda x: x.depth_mm)   # the nearest face is the person in the chair
    best.faces, best.ignored = faces, ignored
    return best


async def two_looks(rig: robot_io.Rig, finder):
    """Two pictures must agree, otherwise the person is moving: refuse."""
    cap1 = await rig.capture()
    fix1 = locate_forehead(cap1, finder)
    cap2 = await rig.capture()
    fix2 = locate_forehead(cap2, finder)
    gap = float(np.linalg.norm(fix1.point_w - fix2.point_w))
    if gap > settings.TWO_LOOK_TOLERANCE_MM:
        raise NoFace(f"the forehead moved {gap:.0f} mm between two looks; ask the person to sit still")
    fix2.point_w = (fix1.point_w + fix2.point_w) / 2
    return fix2, cap2, gap


def facing_direction(fix: ForeheadFix, cap: robot_io.Capture) -> np.ndarray:
    """The direction the thermometer should ideally travel: straight into the forehead.

    Uses the forehead's surface direction measured from depth when it is believable, otherwise
    the camera's line of sight (the person is assumed to face the camera)."""
    line_of_sight = unit(fix.point_w - cap.camera_position)
    if fix.normal_w is not None and angle_deg(-fix.normal_w, line_of_sight) <= 35.0:
        return unit(-fix.normal_w)
    return line_of_sight


def plan_approach(fix: ForeheadFix, cap: robot_io.Capture, aim: bool = False, check_angle: bool = True):
    """Approach plan + the orientation to command + a list of problems (empty = OK to go)."""
    if cap.T_world_gripper is None or cap.gripper_ov is None:
        raise NoFace("can't read the gripper frame, so the thermometer can't be placed")
    ideal = facing_direction(fix, cap)
    plan = plan_forehead_approach(fix.point_w, cap.T_world_gripper[:3, :3], settings.THERMO_AXIS_IN_GRIPPER,
                                  robot_io.thermo_tip_in_gripper(), settings.STANDOFF_MM, settings.PRE_APPROACH_MM,
                                  settings.APPROACH_STEPS, aim_direction=ideal if aim else None)
    ov = matrix_to_ov(plan.R_gripper) if aim else cap.gripper_ov
    problems = []
    off = angle_deg(plan.axis, ideal)
    if check_angle and off > settings.MAX_AIM_ANGLE_DEG:
        problems.append(f"the thermometer would hit the forehead {off:.0f} deg away from straight on "
                        f"(max {settings.MAX_AIM_ANGLE_DEG:.0f}). Re-teach the look pose so the face is near the "
                        f"middle of the picture and the thermometer points at it, or add --aim.")
    for p in [plan.gripper_pre] + plan.steps:
        for why in reach_problems(p, settings.ARM_BASE_XY, settings.REACH_MM, settings.Z_LIMITS_MM):
            problems.append(f"gripper target {np.round(p).tolist()} is {why}")
    return plan, ov, problems


def person_obstacles(forehead_w, axis_w):
    """A head sphere and a body box for the planner, so the arm routes around the person."""
    from viam.proto.common import Geometry, GeometriesInFrame, Pose, RectangularPrism, Sphere, Vector3
    a = unit(axis_w)
    head = np.asarray(forehead_w) + a * (0.8 * settings.HEAD_RADIUS_MM)
    geoms = [Geometry(center=Pose(x=head[0], y=head[1], z=head[2], o_z=1), label="person-head",
                      sphere=Sphere(radius_mm=settings.HEAD_RADIUS_MM))]
    flat = np.array([a[0], a[1], 0.0])
    if np.linalg.norm(flat) > 0.3:
        flat = unit(flat)
        body = np.asarray(forehead_w) + flat * 150 + np.array([0.0, 0.0, -450.0])
        geoms.append(Geometry(center=Pose(x=body[0], y=body[1], z=body[2], o_z=1,
                                          theta=math.degrees(math.atan2(flat[1], flat[0]))),
                              box=RectangularPrism(dims_mm=Vector3(x=300, y=480, z=600)), label="person-body"))
    return [GeometriesInFrame(reference_frame="world", geometries=geoms)]


def save_view(cap, fix: ForeheadFix | None, lines, tag="look") -> str:
    os.makedirs(OUT, exist_ok=True)
    faces = fix.faces if fix else []
    img = draw_faces(cap.color, faces, fix.face if fix else None, lines)
    path = os.path.join(OUT, f"thermometer_{tag}_{time.strftime('%H%M%S')}.png")
    cv2.imwrite(path, img)
    return path


def _fmt(v) -> str:
    return "(" + ", ".join(f"{x:.0f}" for x in v) + ")"


def print_plan(plan: ForeheadApproach):
    print(f"Plan (world mm): forehead {_fmt(plan.forehead)}, thermometer pointing {np.round(plan.axis, 2).tolist()}")
    print(f"  1. gripper to {_fmt(plan.gripper_pre)}: tip {np.linalg.norm(plan.tip_pre - plan.forehead):.0f} mm from the forehead")
    for i, p in enumerate(plan.steps, 2):
        print(f"  {i}. straight to {_fmt(p)}")
    print(f"  The tip stops at {_fmt(plan.tip_final)}, {np.linalg.norm(plan.tip_final - plan.forehead):.0f} mm from the forehead.")


def describe(fix: ForeheadFix) -> str:
    x, y, z = fix.point_w
    return (f"forehead at world ({x:.0f}, {y:.0f}, {z:.0f}) mm, {fix.depth_mm:.0f} mm from the camera "
            f"(depth from {fix.depth_from}" + (", surface direction measured)" if fix.normal_w is not None else ")"))


async def trigger_measurement(rig: robot_io.Rig):
    """Hook: if your thermometer needs a button pressed, do it here (e.g. close the gripper a bit
    more with the gripper's do_command). By default we just hold still."""
    return None


async def run(rig: robot_io.Rig, finder, execute=False, yes=False, aim=False, recheck=True) -> dict:
    log = {"started": time.strftime("%Y-%m-%d %H:%M:%S"), "execute": execute}
    if execute and settings.LOOK_AT_PERSON_JOINTS is not None:
        await robot_io.ask("Press Enter to move to the look-at-person pose", yes)
        await rig.move_joints(settings.LOOK_AT_PERSON_JOINTS)

    fix, cap, gap = await two_looks(rig, finder)
    print(describe(fix) + f"; two looks agreed within {gap:.1f} mm")
    plan, ov, problems = plan_approach(fix, cap, aim)
    log.update(forehead=np.round(fix.point_w, 1).tolist(), plan=plan.summary(), orientation=list(ov), problems=problems)
    view = save_view(cap, fix, [describe(fix)[:60], f"tip stops {settings.STANDOFF_MM:.0f} mm away"])
    print(f"Picture: {view}")
    print_plan(plan)
    if problems:
        print("STOPPED, nothing moved:\n  - " + "\n  - ".join(problems))
        log["result"] = "refused"
        return log
    if not execute:
        print("Dry run: nothing moved. Add --execute to do it.")
        log["result"] = "dry run"
        return log

    obstacles = person_obstacles(fix.point_w, plan.axis) if settings.USE_PERSON_OBSTACLES else None
    moved_closer = cancelled = False
    try:
        await robot_io.ask(f"Press Enter to move the thermometer to {settings.STANDOFF_MM + settings.PRE_APPROACH_MM:.0f} mm "
                           f"in front of the forehead", yes)
        await rig.move_gripper(plan.gripper_pre, ov, obstacles=obstacles)

        if recheck:   # closer now: look again before the last stretch
            cap2 = await rig.capture()
            try:
                fix2 = locate_forehead(cap2, finder)
            except NoFace as e:
                raise NoFace(f"can't see the forehead from the pre-approach ({e}). The thermometer may block "
                             f"the camera; use --no-recheck only if the person will stay still.") from e
            moved = float(np.linalg.norm(fix2.point_w - fix.point_w))
            print(f"Re-check: {describe(fix2)}; {moved:.0f} mm from the first estimate")
            if moved > settings.RECHECK_TOLERANCE_MM:
                raise NoFace(f"the forehead is {moved:.0f} mm from where it was: the person moved")
            plan2, _, problems2 = plan_approach(fix2, cap2, aim=False, check_angle=False)   # orientation is set
            if problems2:
                raise NoFace("; ".join(problems2))
            plan, fix = plan2, fix2
            log["recheck_mm"] = round(moved, 1)

        await robot_io.ask(f"Press Enter for the slow straight approach ({len(plan.steps)} short moves)", yes)
        moved_closer = True
        for p in plan.steps:
            await rig.move_gripper(p, ov, linear=True, obstacles=obstacles, max_vel=settings.APPROACH_MAX_VEL_DEGS)
            await asyncio.sleep(0.2)
        print(f"Measuring: holding {settings.MEASURE_SECONDS:.0f} s with the tip {settings.STANDOFF_MM:.0f} mm away")
        await trigger_measurement(rig)
        await asyncio.sleep(settings.MEASURE_SECONDS)
        log["result"] = "measured"
        log["tip_final"] = np.round(plan.tip_final, 1).tolist()
    except asyncio.CancelledError:
        cancelled = True   # Ctrl+C: stop where we are, don't move on our own
        raise
    finally:
        if moved_closer and not cancelled:
            print("Backing away.")
            try:
                await rig.move_gripper(plan.gripper_pre, ov, linear=True, obstacles=obstacles)
                if settings.LOOK_AT_PERSON_JOINTS is not None:
                    await rig.move_joints(settings.LOOK_AT_PERSON_JOINTS)
            except Exception as e:   # never hide the original problem
                print(f"(could not back away automatically: {e}; use the Viam app)")
    return log


async def watch(rig: robot_io.Rig, finder, seconds: float | None = None):
    """Keep looking and printing where the forehead is. Ctrl+C to stop."""
    show = True
    t_end = None if seconds is None else time.time() + seconds
    while t_end is None or time.time() < t_end:
        cap = await rig.capture(settle_s=0.0)
        try:
            fix = locate_forehead(cap, finder)
            line = describe(fix)
        except NoFace as e:
            fix, line = None, f"no forehead: {e}"
        print(line)
        img = draw_faces(cap.color, fix.faces if fix else [], fix.face if fix else None, [line[:70]])
        if show:
            try:
                cv2.imshow("thermometer watch (q to quit)", img)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            except cv2.error:
                show = False
                os.makedirs(OUT, exist_ok=True)
                print(f"(no window available; saving {OUT}/thermometer_watch.png instead)")
        if not show:
            cv2.imwrite(os.path.join(OUT, "thermometer_watch.png"), img)


async def _main(args):
    if args.offline:
        cap = robot_io.load_capture(args.offline)
        finder = FaceFinder(score_threshold=settings.FACE_MIN_SCORE, up_ratio=settings.FOREHEAD_UP_RATIO)
        try:
            fix = locate_forehead(cap, finder)
        except NoFace as e:
            print(f"STOPPED: {e}")
            return 1
        print(describe(fix))
        plan, ov, problems = plan_approach(fix, cap, args.aim)
        print_plan(plan)
        print("Problems:" if problems else "No problems found.", *problems, sep="\n  ")
        print(f"Picture: {save_view(cap, fix, [describe(fix)[:60]])}")
        return 0

    rig = await robot_io.make_rig(args)
    if args.sim:
        import sim
        finder = sim.SimFaceFinder(rig.world, settings.FOREHEAD_UP_RATIO)
    else:
        finder = FaceFinder(score_threshold=settings.FACE_MIN_SCORE, up_ratio=settings.FOREHEAD_UP_RATIO)
    try:
        if args.watch:
            await watch(rig, finder, seconds=args.seconds)
            return 0
        log = await run(rig, finder, execute=args.execute, yes=args.yes, aim=args.aim, recheck=not args.no_recheck)
        os.makedirs(OUT, exist_ok=True)
        with open(os.path.join(OUT, f"thermometer_{time.strftime('%H%M%S')}.json"), "w") as f:
            json.dump(log, f, indent=2)
        return 0 if log.get("result") in ("measured", "dry run") else 1
    except (NoFace, robot_io.UnsafeTarget) as e:
        print(f"STOPPED: {e}")
        return 1
    except asyncio.CancelledError:
        await rig.stop()
        raise
    except Exception as e:   # a motion or connection error: say so plainly, keep the details
        traceback.print_exc()
        print(f"\nSTOPPED ({type(e).__name__}): {e}\nCheck the arm in the Viam app before trying again.")
        return 1
    finally:
        await rig.close()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--watch", action="store_true", help="keep printing where the forehead is (no motion)")
    ap.add_argument("--seconds", type=float, help="with --watch: stop after this many seconds")
    ap.add_argument("--execute", action="store_true", help="actually move (asks before each move)")
    ap.add_argument("--yes", action="store_true", help="don't ask before moves")
    ap.add_argument("--aim", action="store_true", help="also turn the thermometer to point straight at the forehead")
    ap.add_argument("--no-recheck", action="store_true", help="skip the second look from the pre-approach")
    ap.add_argument("--offline", metavar="DIR", help="use a saved snapshot instead of the robot (plan only)")
    robot_io.add_sim_args(ap)
    args = ap.parse_args(argv)
    args.scene = "person"
    return robot_io.run_main(_main(args))


if __name__ == "__main__":
    raise SystemExit(main())
