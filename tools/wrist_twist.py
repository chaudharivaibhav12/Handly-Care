"""Turn only the last wrist joint by DEG from where the arm is, pause, and turn back (gripper unchanged).
Use it with the gripper empty to check which sign is anticlockwise seen from above before unscrewing a cap.

    python tools/wrist_twist.py -120
"""
import asyncio
import os
import sys

sys.path.insert(0, os.getcwd())
from dotenv import load_dotenv  # noqa: E402

load_dotenv(".env")
import config  # noqa: E402
from robot import RealRobot  # noqa: E402


async def main(args):
    if len(args) != 1:
        print(__doc__)
        return False
    deg = float(args[0])
    if abs(deg) > 180:
        print("keep it within +-180 deg for this test")
        return False
    from viam.proto.component.arm import JointPositions, MoveOptions
    opts = MoveOptions(max_vel_degs_per_sec=config.CAP_TWIST_VEL_DEGS, max_acc_degs_per_sec2=config.CAP_TWIST_ACC_DEGS)
    r = await RealRobot.connect()
    try:
        start = list((await r.arm.get_joint_positions()).values)
        turned = start[:5] + [start[5] + deg]
        if abs(turned[5]) > config.WRIST_LIMIT_DEG:
            print(f"refused: wrist would go to {turned[5]:.0f} deg (limit {config.WRIST_LIMIT_DEG:.0f})")
            return False
        print(f"wrist {start[5]:.1f} -> {turned[5]:.1f} deg ... watch the direction")
        await r.arm.move_through_joint_positions([JointPositions(values=turned)], options=opts)
        await asyncio.sleep(1.5)
        await r.arm.move_through_joint_positions([JointPositions(values=start)], options=opts)
        print("back. Was that anticlockwise seen from above? If not, flip the sign of CAP_TWIST_DEG in config.py")
        return True
    finally:
        await r.close()


if __name__ == "__main__":
    sys.exit(0 if asyncio.run(main(sys.argv[1:])) else 1)
