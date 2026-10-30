"""The touch test: is the camera telling the truth about where things are?

Last month's teams found the camera was off by 2-9 cm. Viam's own guide calls 10-20 mm at
50-100 cm "good". Check it before trusting any 3D point.

Put ONE small object on the table (a bottle cap, a die, a 2-3 cm block). Then:
  1. python calib_check.py look     (from the overview pose) where the camera thinks its top is
  2. In the Viam app, jog the gripper (pointing down) until the fingertips' center just
     touches the middle of the object's top.
  3. python calib_check.py touch    where the gripper really is
Move the object and repeat 1-3 at 3-4 spots spread over the table. Then:
  4. python calib_check.py report          the error at each spot
  5. python calib_check.py report --save   keep the average as a correction (camera_correction.json)
     python calib_check.py reset           start over

On the simulator (with a deliberately wrong camera mount):
  python calib_check.py look --sim --sim-mount-error 12 -8 6 --sim-spot 0   (then touch, spots 1-3...)
"""
from __future__ import annotations

import argparse
import json
import os
import time

import numpy as np

import robot_io
import settings
from depth_tools import find_objects
from geometry import apply_T


def _load() -> list:
    if not os.path.exists(settings.CALIB_POINTS_FILE):
        return []
    with open(settings.CALIB_POINTS_FILE) as f:
        return json.load(f)


def _save(points: list):
    with open(settings.CALIB_POINTS_FILE, "w") as f:
        json.dump(points, f, indent=2)


def analyze(points: list) -> dict:
    """Errors at each touched spot, in world and camera frames, and the average offset."""
    done = [p for p in points if p.get("touch") is not None]
    rows = []
    for p in done:
        e_w = np.array(p["touch"]) - np.array(p["computed_world"])
        R = np.array(p["T_world_cam"])[:3, :3]
        rows.append({"id": p["id"], "error_world": e_w, "error_cam": R.T @ e_w})
    if not rows:
        return {"rows": [], "n": 0}
    E = np.array([r["error_cam"] for r in rows])
    mean = E.mean(axis=0)
    spread = float(np.max(np.linalg.norm(E - mean, axis=1)))
    return {"rows": rows, "n": len(rows), "mean_cam": mean, "spread": spread,
            "worst_before": float(max(np.linalg.norm(r["error_world"]) for r in rows))}


async def _look(args) -> int:
    rig = await robot_io.make_rig(args)
    try:
        cap = await rig.capture(apply_correction=False)   # always measure the raw camera
    finally:
        await rig.close()
    blobs, _, _ = find_objects(cap.depth, cap.intr, roi=settings.LOCATE_ROI, ignore_boxes=settings.IGNORE_BOXES_PX,
                               valid_mm=settings.DEPTH_VALID_MM, min_height_mm=settings.MIN_OBJECT_HEIGHT_MM)
    if not blobs:
        print("I don't see anything on the table. Put a small object in view.")
        return 1
    blob = min(blobs, key=lambda b: b.area_px)   # the small calibration object
    if len(blobs) > 1:
        print(f"(I see {len(blobs)} things; using the smallest one. Clear the rest for a clean test.)")
    if blob.touches_edge:
        print("The object is cut off at the edge of the picture. Move it toward the middle and look again.")
        return 1
    world = cap.to_world(blob.top_point_cam)
    points = _load()
    points.append({"id": len(points) + 1, "time": time.strftime("%H:%M:%S"),
                   "cam_point": blob.top_point_cam.tolist(), "computed_world": world.tolist(),
                   "T_world_cam": cap.T_world_cam.tolist(), "touch": None})
    _save(points)
    print(f"Spot {len(points)}: the camera says the top is at world ({world[0]:.0f}, {world[1]:.0f}, {world[2]:.0f}) mm.")
    print("Now jog the gripper until the fingertips' center touches the middle of that top, then run:")
    print("  python calib_check.py touch" + (" --sim" if args.sim else ""))
    return 0


async def _touch(args) -> int:
    points = _load()
    if not points or points[-1].get("touch") is not None:
        print("Run 'python calib_check.py look' first.")
        return 1
    rig = await robot_io.make_rig(args)
    try:
        if args.sim:   # the simulator "jogs" the fingertips onto the real top for you
            import sim
            truth = sim.true_top_near(rig.world, points[-1]["computed_world"])
            await rig.move_gripper(robot_io.frame_for_fingertips(truth, settings.DOWN_ORIENTATION),
                                   settings.DOWN_ORIENTATION)
        T = await rig.probe(rig.gripper_name)
        xyz = apply_T(T, robot_io.tip_in_gripper())   # the fingertips, not the gripper frame origin
    finally:
        await rig.close()
    points[-1]["touch"] = xyz.tolist()
    _save(points)
    e = xyz - np.array(points[-1]["computed_world"])
    print(f"Spot {points[-1]['id']}: fingertips at ({xyz[0]:.0f}, {xyz[1]:.0f}, {xyz[2]:.0f}); "
          f"camera was off by {np.linalg.norm(e):.0f} mm (x {e[0]:+.0f}, y {e[1]:+.0f}, z {e[2]:+.0f}).")
    return 0


def _report(args) -> int:
    a = analyze(_load())
    if a["n"] == 0:
        print("No touched spots yet.")
        return 1
    print(f"{'spot':>4}  {'error x':>8} {'y':>6} {'z':>6}  {'total':>6}   (world mm: touched - camera)")
    for r in a["rows"]:
        e = r["error_world"]
        print(f"{r['id']:>4}  {e[0]:8.1f} {e[1]:6.1f} {e[2]:6.1f}  {np.linalg.norm(e):6.1f}")
    print(f"Worst error now: {a['worst_before']:.0f} mm. "
          f"If we add the average offset, the spots disagree by at most {a['spread']:.0f} mm.")
    if a["worst_before"] <= 12:
        print("Good: the camera is already within ~1 cm. No correction needed.")
    elif a["n"] < 3:
        print("Do at least 3 spots before trusting a correction.")
    elif a["spread"] <= 8:
        print("The error is the same everywhere: a fixed offset. A correction will fix it "
              "(it is applied in the camera's own frame, so it follows the camera when the arm moves).")
    else:
        print("The error changes from spot to spot, so it is not a simple offset: the camera frame's "
              "orientation in the Viam config is probably off. Ask a Viam engineer to check the camera frame.")
    if args.save:
        if a["n"] < 3:
            print("Not saved: need at least 3 spots.")
            return 1
        with open(settings.CAMERA_CORRECTION_FILE, "w") as f:
            json.dump({"offset_cam_mm": a["mean_cam"].tolist(), "spots": a["n"], "spread_mm": a["spread"],
                       "made": time.strftime("%Y-%m-%d %H:%M")}, f, indent=2)
        print(f"Saved {settings.CAMERA_CORRECTION_FILE}. Every script now uses it. "
              f"Delete the file if the camera is moved or re-mounted, and redo the test.")
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("step", choices=["look", "touch", "report", "reset"])
    ap.add_argument("--save", action="store_true", help="with report: save the correction")
    robot_io.add_sim_args(ap)
    ap.add_argument("--sim-spot", type=int, help="simulator only: which spot the object is at (0-3)")
    args = ap.parse_args(argv)
    args.scene = "table"
    if args.sim:   # same file names the simulator uses
        settings.CAMERA_CORRECTION_FILE = "sim_camera_correction.json"
        settings.CALIB_POINTS_FILE = "sim_calib_points.json"
    if args.step == "reset":
        for path in (settings.CALIB_POINTS_FILE,):
            if os.path.exists(path):
                os.remove(path)
        print("Cleared the touch-test spots (the saved correction, if any, is kept).")
        return 0
    if args.step == "report":
        return _report(args)
    return robot_io.run_main(_look(args) if args.step == "look" else _touch(args))


if __name__ == "__main__":
    raise SystemExit(main())
