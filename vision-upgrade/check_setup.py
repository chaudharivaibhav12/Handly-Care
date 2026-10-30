"""First thing at Open Lab: does the machine give us everything the perception code needs?

    python check_setup.py            # checks, and saves 3 pictures in out/
    python check_setup.py --joints   # also prints the arm's joint angles (paste into settings.py)
    python check_setup.py --sim      # try it on the simulator

It checks: the part names in settings.py exist; the camera sends color AND depth of the same
size; it reports intrinsics; the camera and gripper frames exist and sit where a wrist camera
should. Then look at out/setup_alignment.png: the red lines (edges in the depth image) must
sit on the outlines of objects in the color image. If they are shifted, depth is not aligned
to color: add "align_color_depth": true to the RealSense camera's attributes.
"""
from __future__ import annotations

import argparse
import os

import cv2
import numpy as np

import robot_io
import settings
from depth_tools import colorize_depth, depth_edges_overlay
from geometry import angle_deg

OUT = "out"


async def _main(args) -> int:
    rig = await robot_io.make_rig(args)
    results = []

    def report(status, msg):
        results.append(status)
        print(f"{status:5} {msg}")

    try:
        names = sorted(r.name for r in rig.machine.resource_names)
        print("Parts on this machine:", ", ".join(names))
        for name, what in ((settings.CAMERA, "camera"), (settings.ARM, "arm"),
                           (settings.GRIPPER, "gripper"), (settings.MOTION, "motion service")):
            if name in names:
                report("PASS", f"{what} '{name}' found")
            else:
                report("FAIL", f"{what} '{name}' NOT found: change it in settings.py")

        try:
            cap = await rig.capture(apply_correction=False)
        except Exception as e:
            report("FAIL", f"couldn't take a 3D picture: {e}")
            report("HINT", "the camera needs color + depth sensors, intrinsics, and a frame whose parent is the "
                           "arm (CONFIGURE -> camera -> Frame). Ask a Viam engineer if the frame is missing.")
            return 1
        h, w = cap.color.shape[:2]
        report("PASS", f"color {w}x{h}; intrinsics fx={cap.intr.fx:.0f} fy={cap.intr.fy:.0f} "
                       f"cx={cap.intr.cx:.0f} cy={cap.intr.cy:.0f}")
        lo, hi = settings.DEPTH_VALID_MM
        ok = (cap.depth >= lo) & (cap.depth <= hi)
        if ok.mean() > 0.6:
            report("PASS", f"{ok.mean():.0%} of depth pixels have a reading (median {np.median(cap.depth[ok]):.0f} mm)")
        else:
            report("WARN", f"only {ok.mean():.0%} of depth pixels have a reading: too close, too far, or shiny/black surfaces?")
        for note in cap.notes:
            report("WARN", note)

        os.makedirs(OUT, exist_ok=True)
        cv2.imwrite(os.path.join(OUT, "setup_color.png"), cap.color)
        cv2.imwrite(os.path.join(OUT, "setup_depth.png"), colorize_depth(cap.depth, settings.DEPTH_VALID_MM))
        cv2.imwrite(os.path.join(OUT, "setup_alignment.png"), depth_edges_overlay(cap.color, cap.depth))
        report("LOOK", f"open {OUT}/setup_alignment.png: red depth edges must sit on object outlines")

        cam = cap.camera_position
        report("INFO", f"camera is at world ({cam[0]:.0f}, {cam[1]:.0f}, {cam[2]:.0f}) mm, looking along "
                       f"{np.round(cap.T_world_cam[:3, 2], 2).tolist()}")
        if cap.T_world_gripper is None:
            report("FAIL", "the gripper frame is missing, so targets can't be turned into gripper moves")
        else:
            g = cap.T_world_gripper[:3, 3]
            gap = float(np.linalg.norm(cam - g))
            report("PASS" if 20 <= gap <= 250 else "WARN",
                   f"camera is {gap:.0f} mm from the gripper frame origin (a wrist camera is usually 50-200)")
            report("INFO", f"gripper at world ({g[0]:.0f}, {g[1]:.0f}, {g[2]:.0f}); camera looks "
                           f"{angle_deg(cap.T_world_cam[:3, 2], cap.T_world_gripper[:3, 2]):.0f} deg away from the "
                           f"gripper's pointing direction; gripper orientation {np.round(cap.gripper_ov, 2).tolist()}")
        if rig.correction is not None:
            report("INFO", f"camera correction in use: {np.round(rig.correction, 1).tolist()} mm (camera frame)")
        else:
            report("INFO", "no camera correction yet: run the touch test (calib_check.py)")

        if args.joints:
            report("INFO", f"joints now: {await rig.joints()}  <- paste into settings.py if this is a pose to keep")
    finally:
        await rig.close()

    bad = results.count("FAIL")
    print(f"\n{'All good.' if not bad else f'{bad} problem(s) to fix first.'} Pictures are in {OUT}/.")
    return 1 if bad else 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--joints", action="store_true", help="also print the current joint angles")
    robot_io.add_sim_args(ap)
    args = ap.parse_args(argv)
    args.scene = "table"
    return robot_io.run_main(_main(args))


if __name__ == "__main__":
    raise SystemExit(main())
