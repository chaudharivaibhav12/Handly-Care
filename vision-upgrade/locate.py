"""Find the bottle or the tissue with the depth camera and grab it where it really is.

Before: the arm went to a taught spot at a fixed height.
Now:    look -> find the table -> find the object -> measure its top -> grab at that height.
Move the bottle 10 cm, or swap in a taller one, and it still works.

    python locate.py --list                  # what the camera sees on the table, with sizes
    python locate.py bottle                  # find it and print the grasp (nothing moves)
    python locate.py tissue --execute        # find it and pick it
    python locate.py bottle --execute --verify   # ...then look again to check it's gone
    python locate.py bottle --sim --execute  # try everything on the simulator
    python locate.py bottle --offline captures/my_table   # use a saved snapshot

Use it from your own routine:
    found = await locate.find(rig, "bottle")   # two looks, checked
    found.grasp, found.hover, found.lift       # gripper targets in world mm
"""
from __future__ import annotations

import argparse
import asyncio
import math
import os
import time
import traceback
from dataclasses import dataclass, field

import cv2
import numpy as np

import robot_io
import settings
from depth_tools import Blob, colorize_heights, find_objects

OUT = "out"


@dataclass
class Found:
    name: str
    blob: Blob
    top_w: np.ndarray       # top of what we grab (cap top / tissue tip), world mm
    grasp: np.ndarray       # gripper target when the fingers close
    hover: np.ndarray       # above it, before going straight down
    lift: np.ndarray        # where to go after closing
    yaw_deg: float          # direction of the object's long side in world (for boxes)
    notes: list = field(default_factory=list)
    view: str = ""          # saved picture of what the camera saw

    def summary(self) -> dict:
        r = lambda v: [round(float(x), 1) for x in v]
        return {"object": self.name, "top": r(self.top_w), "grasp": r(self.grasp), "hover": r(self.hover),
                "lift": r(self.lift), "yaw_deg": round(self.yaw_deg, 1),
                "height_above_table_mm": round(self.blob.highest_mm, 1), "notes": self.notes}


class NotFound(Exception):
    pass


def _fmt(v) -> str:
    return "(" + ", ".join(f"{x:.0f}" for x in v) + ")"


def print_found(found: Found):
    print(f"Found the {found.name}: top at {_fmt(found.top_w)} mm (world), "
          f"{found.blob.highest_mm:.0f} mm above the table. " + " ".join(found.notes))
    print(f"  gripper frame: hover {_fmt(found.hover)} -> straight down to {_fmt(found.grasp)} -> close -> "
          f"lift to {_fmt(found.lift)}  (fingertips {settings.GRIPPER_TIP_OFFSET_MM:.0f} mm below the frame)")


def mismatch(blob: Blob, profile: dict) -> list[str]:
    """Why a blob is not this kind of object (empty list = it matches)."""
    why = []
    for key, value, label in (("height_mm", blob.highest_mm, "height"), ("length_mm", blob.length_mm, "length"),
                              ("width_mm", blob.width_mm, "width")):
        lo, hi = profile[key]
        if not lo <= value <= hi:
            why.append(f"{label} {value:.0f} not in {lo}-{hi}")
    return why


def detect(cap: robot_io.Capture):
    """Table + every object on it, using the settings."""
    return find_objects(cap.depth, cap.intr, roi=settings.LOCATE_ROI, ignore_boxes=settings.IGNORE_BOXES_PX,
                        valid_mm=settings.DEPTH_VALID_MM, plane_tol_mm=settings.PLANE_TOLERANCE_MM,
                        min_height_mm=settings.MIN_OBJECT_HEIGHT_MM, max_height_mm=settings.MAX_OBJECT_HEIGHT_MM,
                        min_area_px=settings.MIN_BLOB_AREA_PX, top_band_mm=settings.TOP_BAND_MM)


def locate_in_capture(cap: robot_io.Capture, name: str):
    """Find one object in one picture. Returns (Found, blobs, height_map). Raises NotFound."""
    if name not in settings.OBJECTS:
        raise NotFound(f"'{name}' is not in settings.OBJECTS ({', '.join(settings.OBJECTS)})")
    profile = settings.OBJECTS[name]
    blobs, plane, height_map = detect(cap)
    matches = [b for b in blobs if not mismatch(b, profile)]
    if not matches:
        lines = [f"  {b.row()}  <- {'; '.join(mismatch(b, profile))}" for b in blobs] or ["  (nothing above the table)"]
        raise NotFound(f"no {name} found. What I see:\n" + "\n".join(lines) +
                       f"\nIf one of these IS the {name}, widen its ranges in settings.OBJECTS.")
    if len(matches) > 1:
        # Two candidates: refuse rather than guess (unless one is clearly bigger).
        a, b = sorted(matches, key=lambda m: -m.area_px)[:2]
        if a.area_px < 2 * b.area_px:
            raise NotFound(f"{len(matches)} things look like a {name} (#{a.id} and #{b.id}); "
                           f"remove one or tighten settings.OBJECTS['{name}']")
    blob = max(matches, key=lambda m: m.area_px)
    notes = []
    if blob.touches_edge and not profile.get("needs_tissue_up_mm"):
        raise NotFound(f"the {name} is cut off at the edge of the picture, so I can't measure it well. "
                       f"Move it toward the middle, or change the overview pose.")

    if name == "tissue" or profile.get("needs_tissue_up_mm"):
        need = profile.get("needs_tissue_up_mm", 12)
        up = (blob.peak_height_mm or 0.0) - blob.plateau_height_mm
        if blob.peak_point_cam is None or up < need:
            raise NotFound(f"I see the box but no tissue sticking out (need {need} mm, see {max(0.0, up):.0f} mm). "
                           f"Pull one up a little and try again.")
        top_w = cap.to_world(blob.peak_point_cam)
        below = min(profile["grasp_below_top_mm"], 0.6 * up)   # never pinch down into the box
        notes.append(f"tissue sticks up {up:.0f} mm above the box")
    else:
        top_w = cap.to_world(blob.top_point_cam)
        below = profile["grasp_below_top_mm"]

    # Fingertip targets, then where the gripper FRAME must go for the fingertips to get there.
    tip = np.array([top_w[0], top_w[1], top_w[2] - below])
    to_frame = lambda p: robot_io.frame_for_fingertips(p, settings.DOWN_ORIENTATION)
    grasp = to_frame(tip)
    hover = to_frame(tip + np.array([0.0, 0.0, below + settings.HOVER_MM]))
    lift = to_frame(tip + np.array([0.0, 0.0, profile.get("lift_mm", 120)]))
    axis_w = cap.T_world_cam[:3, :3] @ blob.long_axis_cam
    yaw = math.degrees(math.atan2(axis_w[1], axis_w[0]))
    return Found(name, blob, top_w, grasp, hover, lift, yaw, notes), blobs, height_map


def draw(cap: robot_io.Capture, blobs, height_map, found: Found | None = None) -> np.ndarray:
    left = cap.color.copy()
    for b in blobs:
        chosen = found is not None and b is found.blob
        color = (0, 220, 0) if chosen else (0, 200, 255)
        if b.contour is not None:
            cv2.drawContours(left, [b.contour], -1, color, 2)
        x, y, _, _ = b.bbox
        label = f"#{b.id} {b.highest_mm:.0f}mm" + (f" {found.name}" if chosen else "")
        cv2.putText(left, label, (x, max(15, y - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3)
        cv2.putText(left, label, (x, max(15, y - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1)
        if chosen:
            px = b.peak_pixel if (found.name == "tissue" and b.peak_pixel) else b.top_pixel
            cv2.drawMarker(left, (int(px[0]), int(px[1])), (0, 0, 255), cv2.MARKER_CROSS, 22, 2)
    return np.hstack([left, colorize_heights(height_map)])


def save_view(cap, blobs, height_map, found, tag) -> str:
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, f"locate_{tag}_{time.strftime('%H%M%S')}.png")
    cv2.imwrite(path, draw(cap, blobs, height_map, found))
    return path


async def find(rig: robot_io.Rig, name: str, looks: int = 2) -> Found:
    """Look `looks` times; the answers must agree, otherwise something is moving: refuse."""
    results = []
    for _ in range(looks):
        cap = await rig.capture()
        found, blobs, hm = locate_in_capture(cap, name)
        results.append(found)
    spread = max(float(np.linalg.norm(r.grasp - results[0].grasp)) for r in results)
    if spread > settings.TWO_LOOK_TOLERANCE_OBJ_MM:
        raise NotFound(f"two looks disagree by {spread:.0f} mm; is something moving?")
    best = results[-1]
    best.grasp = np.mean([r.grasp for r in results], axis=0)
    best.hover = np.mean([r.hover for r in results], axis=0)
    best.lift = np.mean([r.lift for r in results], axis=0)
    best.notes.append(f"{looks} looks agreed within {spread:.1f} mm")
    best.view = save_view(cap, blobs, hm, best, name)
    return best


async def pick(rig: robot_io.Rig, found: Found, yes: bool = False) -> bool:
    """Open, hover, straight down, close, straight up. Returns whether the gripper felt it."""
    ov = settings.DOWN_ORIENTATION
    for p in (found.hover, found.grasp, found.lift):
        rig.check_target(p)
    await robot_io.ask(f"Press Enter to open the gripper and go above the {found.name} at {np.round(found.hover).tolist()}", yes)
    await rig.open_gripper()
    await rig.move_gripper(found.hover, ov)
    await robot_io.ask(f"Press Enter to go straight down to z = {found.grasp[2]:.0f} mm", yes)
    await rig.move_gripper(found.grasp, ov, linear=True)
    held = await rig.grab()
    print("Gripper says:", "holding something" if held else "closed on nothing (check the height)")
    await rig.move_gripper(found.lift, ov, linear=True)
    return held


async def verify_gone(rig: robot_io.Rig, found: Found) -> bool:
    """Look again from the overview: is the object gone from where it was?"""
    if settings.OBJECTS[found.name].get("needs_tissue_up_mm"):
        print("(Not checking by looking: the next tissue usually pops up in the same place. "
              "Trusting the gripper instead.)")
        return True
    if settings.OVERVIEW_JOINTS is None:
        print("(set OVERVIEW_JOINTS in settings.py to verify by looking again)")
        return False
    await rig.move_joints(settings.OVERVIEW_JOINTS)
    cap = await rig.capture()
    try:
        again, blobs, hm = locate_in_capture(cap, found.name)
    except NotFound:
        print(f"Verified: no {found.name} left where it was.")
        return True
    moved = float(np.linalg.norm(again.top_w[:2] - found.top_w[:2]))
    if moved < 30:
        print(f"NOT verified: the {found.name} is still there ({moved:.0f} mm from before). The grasp missed.")
        save_view(cap, blobs, hm, again, f"{found.name}_still_there")
        return False
    print(f"Verified: the {found.name} is no longer where it was.")
    return True


async def _main(args):
    if args.offline:
        cap = robot_io.load_capture(args.offline)
        return _report_offline(cap, args)
    rig = await robot_io.make_rig(args)
    try:
        if settings.OVERVIEW_JOINTS is not None and (args.execute or args.go_overview):
            await robot_io.ask("Press Enter to move to the overview pose", args.yes)
            await rig.move_joints(settings.OVERVIEW_JOINTS)
        if args.list or not args.object:
            cap = await rig.capture()
            return _list(cap)
        found = await find(rig, args.object)
        print_found(found)
        print(f"Picture: {found.view}")
        if args.execute:
            held = await pick(rig, found, args.yes)
            if args.verify and not await verify_gone(rig, found):
                return 3
            return 0 if held else 2
        print("Dry run: nothing moved. Add --execute to pick it.")
        return 0
    except (NotFound, robot_io.UnsafeTarget) as e:
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


def _list(cap) -> int:
    blobs, plane, hm = detect(cap)
    print(f"Table found ({100 * plane.inlier_fraction:.0f}% of the picture is flat). Objects:")
    for b in blobs:
        top = cap.to_world(b.peak_point_cam if b.peak_point_cam is not None else b.top_point_cam)
        print(f"  {b.row()}   top at world {np.round(top).tolist()}")
        for name, profile in settings.OBJECTS.items():
            why = mismatch(b, profile)
            print(f"      {name}: {'MATCH' if not why else 'no (' + '; '.join(why) + ')'}")
    if not blobs:
        print("  nothing above the table. Is the camera looking at it, and is depth aligned?")
    print(f"Picture: {save_view(cap, blobs, hm, None, 'list')}")
    return 0


def _report_offline(cap, args) -> int:
    if args.list or not args.object:
        return _list(cap)
    try:
        found, blobs, hm = locate_in_capture(cap, args.object)
    except NotFound as e:
        print(f"STOPPED: {e}")
        return 1
    print_found(found)
    print(f"Picture: {save_view(cap, blobs, hm, found, args.object)}")
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("object", nargs="?", help="a name from settings.OBJECTS, e.g. bottle or tissue")
    ap.add_argument("--list", action="store_true", help="show everything on the table and how it matches")
    ap.add_argument("--execute", action="store_true", help="actually move (asks before each move)")
    ap.add_argument("--yes", action="store_true", help="don't ask before moves")
    ap.add_argument("--verify", action="store_true", help="after picking, look again to confirm")
    ap.add_argument("--go-overview", action="store_true", help="move to OVERVIEW_JOINTS before looking")
    ap.add_argument("--offline", metavar="DIR", help="use a saved snapshot instead of the robot")
    robot_io.add_sim_args(ap)
    ap.add_argument("--sim-spot", type=int, help="simulator only: put the bottle at another spot (0-3)")
    args = ap.parse_args(argv)
    args.scene = "table"
    return robot_io.run_main(_main(args))


if __name__ == "__main__":
    raise SystemExit(main())
