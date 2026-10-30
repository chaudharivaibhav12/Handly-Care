"""Optional: make a taught routine (the drawer) follow its object, using a printed tag.

A fixed thing like the drawer is easiest to locate with an AprilTag: a printed black-and-
white square the camera finds very reliably, with its exact position and which way it faces.

  python markers.py make 0                 # tag_0.png: print it, measure the black square, set --size-mm
  python markers.py show                   # every tag in view, with world position and facing

Teach the drawer routine ONCE, relative to the tag on the drawer front:
  python markers.py anchor drawer          # from a pose that sees the tag: remember where it is
  python markers.py add drawer             # jog to each waypoint and add it (--linear, --grip open/close)
Then any time, even after the drawer was moved or turned:
  python markers.py replay drawer          # find the tag, shift every waypoint, print them
  python markers.py replay drawer --execute
"""
from __future__ import annotations

import argparse
import json
import math
import os
from dataclasses import dataclass

import cv2
import numpy as np

import robot_io
import settings
from depth_tools import robust_depth
from geometry import Intrinsics, reanchor, reanchor_orientation

TAG_DICT = cv2.aruco.DICT_APRILTAG_36h11
TAG_SIZE_MM = 50.0     # side of the black square as printed. Measure it with a ruler!
ROUTINES = "routines"


@dataclass
class Tag:
    id: int
    corners_px: np.ndarray    # 4 x 2
    center_cam: np.ndarray
    R_cam_tag: np.ndarray     # tag axes in the camera frame; column 2 points out of the tag face

    @property
    def normal_cam(self) -> np.ndarray:
        return self.R_cam_tag[:, 2]


def find_tags(color: np.ndarray, intr: Intrinsics, size_mm: float = TAG_SIZE_MM, depth: np.ndarray | None = None):
    """All tags in the picture, with 3D pose in the camera frame."""
    gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
    detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(TAG_DICT), cv2.aruco.DetectorParameters())
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is None:
        return []
    s = size_mm / 2
    obj = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], np.float64)   # order OpenCV's IPPE_SQUARE wants
    K = np.array([[intr.fx, 0, intr.cx], [0, intr.fy, intr.cy], [0, 0, 1]], np.float64)
    tags = []
    for c, i in zip(corners, ids.ravel()):
        px = c.reshape(4, 2).astype(np.float64)
        ok, rvec, tvec = cv2.solvePnP(obj, px, K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            continue
        R, _ = cv2.Rodrigues(rvec)
        center = tvec.ravel()
        if depth is not None:   # the depth camera knows the distance better than the printed size does
            side_px = float(np.linalg.norm(px[1] - px[0]))
            d = robust_depth(depth, *px.mean(axis=0), max(2, int(side_px / 8)), settings.DEPTH_VALID_MM)
            if d is not None and abs(d - center[2]) < 0.25 * center[2]:
                center = center * (d / center[2])
        if R[:, 2] @ (-center) < 0:   # make the normal face the camera (out of the printed side)
            R = R @ np.diag([1.0, -1.0, -1.0])
        tags.append(Tag(int(i), px, center, R))
    return tags


def tag_anchor(tag: Tag, cap: robot_io.Capture):
    """(x, y, z, yaw_deg) of a tag in world. Yaw = the way it faces (or its x axis if lying flat)."""
    center = cap.to_world(tag.center_cam)
    R_w = cap.T_world_cam[:3, :3] @ tag.R_cam_tag
    n = R_w[:, 2]
    ref = R_w[:, 0] if abs(n[2]) > 0.8 else n
    yaw = math.degrees(math.atan2(ref[1], ref[0]))
    return (float(center[0]), float(center[1]), float(center[2]), yaw), n


def _routine_path(name):
    return os.path.join(ROUTINES, f"{name}.json")


def _load_routine(name):
    path = _routine_path(name)
    if not os.path.exists(path):
        return {"name": name, "anchor": None, "tag_id": None, "waypoints": []}
    with open(path) as f:
        return json.load(f)


def _save_routine(r):
    os.makedirs(ROUTINES, exist_ok=True)
    with open(_routine_path(r["name"]), "w") as f:
        json.dump(r, f, indent=2)


async def _see_tag(rig, tag_id=None, size_mm=TAG_SIZE_MM):
    cap = await rig.capture()
    tags = find_tags(cap.color, cap.intr, size_mm, cap.depth)
    if tag_id is not None:
        tags = [t for t in tags if t.id == tag_id]
    if not tags:
        raise SystemExit("No tag in view" + (f" with id {tag_id}" if tag_id is not None else "") + ".")
    if len(tags) > 1:
        raise SystemExit(f"{len(tags)} tags in view ({[t.id for t in tags]}); use --tag to pick one.")
    return tags[0], cap


async def _main(args) -> int:
    if args.step == "make":
        img = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(TAG_DICT), args.id, 600)
        img = cv2.copyMakeBorder(img, 120, 120, 120, 120, cv2.BORDER_CONSTANT, value=255)   # white margin
        path = f"tag_{args.id}.png"
        cv2.imwrite(path, img)
        print(f"Saved {path}. Print it, measure the BLACK square's side in mm, and pass --size-mm.")
        return 0

    rig = await robot_io.make_rig(args)
    try:
        if args.step == "show":
            cap = await rig.capture()
            tags = find_tags(cap.color, cap.intr, args.size_mm, cap.depth)
            for t in tags:
                (x, y, z, yaw), n = tag_anchor(t, cap)
                print(f"tag {t.id}: world ({x:.0f}, {y:.0f}, {z:.0f}) mm, faces {np.round(n, 2).tolist()} (yaw {yaw:.0f} deg)")
            if not tags:
                print("No tags in view.")
            return 0

        routine = _load_routine(args.name)
        if args.step == "anchor":
            tag, cap = await _see_tag(rig, args.tag, args.size_mm)
            routine["anchor"], _ = tag_anchor(tag, cap)
            routine["tag_id"] = tag.id
            _save_routine(routine)
            print(f"'{args.name}' is anchored to tag {tag.id} at {np.round(routine['anchor'], 1).tolist()}")
            return 0

        if args.step == "add":
            xyz, ov = await rig.gripper_pose()
            routine["waypoints"].append({"xyz": xyz.tolist(), "ov": list(ov), "linear": args.linear, "grip": args.grip})
            _save_routine(routine)
            print(f"waypoint {len(routine['waypoints'])}: {np.round(xyz).tolist()} (linear={args.linear}, grip={args.grip})")
            return 0

        if args.step == "replay":
            if routine["anchor"] is None or not routine["waypoints"]:
                raise SystemExit(f"'{args.name}' needs an anchor and waypoints first (see --help)")
            tag, cap = await _see_tag(rig, routine["tag_id"], args.size_mm)
            now, _ = tag_anchor(tag, cap)
            then = routine["anchor"]
            print(f"tag {tag.id} moved {math.dist(now[:3], then[:3]):.0f} mm and turned {now[3] - then[3]:+.0f} deg since teaching")
            pts = reanchor([w["xyz"] for w in routine["waypoints"]], then, now)
            plan = []
            for w, p in zip(routine["waypoints"], pts):
                ov = reanchor_orientation(w["ov"], then[3], now[3])
                plan.append((p, ov, w))
                print(f"  -> {np.round(p).tolist()}" + (" straight" if w["linear"] else "") + (f", then {w['grip']}" if w["grip"] else ""))
            if not args.execute:
                print("Dry run: nothing moved. Add --execute.")
                return 0
            for p, _, _ in plan:
                rig.check_target(p)
            await robot_io.ask("Press Enter to run the routine", args.yes)
            for p, ov, w in plan:
                await rig.move_gripper(p, ov, linear=w["linear"])
                if w["grip"] == "open":
                    await rig.open_gripper()
                elif w["grip"] == "close":
                    await rig.grab()
            return 0
    finally:
        await rig.close()
    return 1


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("step", choices=["make", "show", "anchor", "add", "replay"])
    ap.add_argument("name", nargs="?", default="drawer", help="routine name (default: drawer)")
    ap.add_argument("--id", type=int, default=0, help="with make: which tag to draw")
    ap.add_argument("--tag", type=int, help="which tag id to use if several are in view")
    ap.add_argument("--size-mm", type=float, default=TAG_SIZE_MM, help="printed black square side")
    ap.add_argument("--linear", action="store_true", help="with add: reach this waypoint in a straight line")
    ap.add_argument("--grip", choices=["open", "close"], help="with add: open/close the gripper after reaching it")
    ap.add_argument("--execute", action="store_true")
    ap.add_argument("--yes", action="store_true")
    robot_io.add_sim_args(ap)
    args = ap.parse_args(argv)
    args.scene = "table"
    return robot_io.run_main(_main(args))


if __name__ == "__main__":
    raise SystemExit(main())
