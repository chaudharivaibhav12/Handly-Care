"""Save what the camera sees right now, so you can work on it later without the robot.

    python snapshot.py              -> captures/<time>/
    python snapshot.py my_table     -> captures/my_table/

Each folder has color.png, depth.png (16-bit, mm), and meta.json (intrinsics and exactly where
the camera and gripper were). Then, at home:
    python locate.py --offline captures/my_table --list
    python thermometer.py --offline captures/me_in_chair
"""
from __future__ import annotations

import argparse
import os
import time

import robot_io


async def _main(args) -> int:
    rig = await robot_io.make_rig(args)
    try:
        cap = await rig.capture(apply_correction=not args.raw)
    finally:
        await rig.close()
    folder = robot_io.save_capture(cap, os.path.join("captures", args.name or time.strftime("%Y%m%d_%H%M%S")))
    print(f"Saved {folder} ({cap.color.shape[1]}x{cap.color.shape[0]})")
    for note in cap.notes:
        print("note:", note)
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("name", nargs="?", help="folder name under captures/")
    ap.add_argument("--raw", action="store_true", help="don't apply the camera correction")
    ap.add_argument("--scene", default="table", choices=["table", "person"], help="simulator only")
    robot_io.add_sim_args(ap)
    return robot_io.run_main(_main(ap.parse_args(argv)))


if __name__ == "__main__":
    raise SystemExit(main())
