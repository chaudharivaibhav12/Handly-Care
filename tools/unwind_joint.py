"""Slowly turn ONE joint to a target, in steps, checking the gripper height after each step; optional lift first.
python tools/unwind_joint.py JOINT_INDEX TARGET_DEG [--lift DEG] [--zmin MM] [--step DEG]
--lift: first move joints 2 and 3 (index 1, 2) back by DEG each (raises the arm on the xArm6), only that.
Stops (no further moves) the moment the gripper is below zmin."""
import argparse, asyncio, os, sys
sys.path.insert(0, os.getcwd())
from dotenv import load_dotenv
load_dotenv(".env")
from viam.proto.component.arm import JointPositions, MoveOptions
from robot import RealRobot
async def main(a):
    r = await RealRobot.connect()
    opts = MoveOptions(max_vel_degs_per_sec=10.0, max_acc_degs_per_sec2=20.0)
    async def go(j, label):
        await r.arm.move_through_joint_positions([JointPositions(values=j)], options=opts)
        p = await r.where()
        print(f"  {label}: joints {[round(v, 1) for v in j]}  gripper x {p.x:.0f} y {p.y:.0f} z {p.z:.0f}")
        return p
    try:
        j = list((await r.arm.get_joint_positions()).values)
        p = await r.where()
        print(f"  start: joints {[round(v, 1) for v in j]}  gripper x {p.x:.0f} y {p.y:.0f} z {p.z:.0f}")
        if a.lift:
            j[1] -= a.lift; j[2] -= a.lift
            p = await go(list(j), f"lift (j2, j3 -{a.lift:g})")
            if p.z < a.zmin:
                print(f"STOP: gripper at z {p.z:.0f} < {a.zmin:g} after the lift"); return
        while abs(j[a.joint] - a.target) > 0.5:
            d = max(-a.step, min(a.step, a.target - j[a.joint]))
            j[a.joint] += d
            p = await go(list(j), f"joint {a.joint + 1} -> {j[a.joint]:.1f}")
            if p.z < a.zmin:
                print(f"STOP: gripper at z {p.z:.0f} < {a.zmin:g}"); return
        print("  done")
    finally:
        await r.close()
ap = argparse.ArgumentParser()
ap.add_argument("joint", type=int); ap.add_argument("target", type=float)
ap.add_argument("--lift", type=float, default=0.0); ap.add_argument("--zmin", type=float, default=200.0)
ap.add_argument("--step", type=float, default=30.0)
asyncio.run(main(ap.parse_args()))
