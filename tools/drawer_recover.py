"""Recover from a drawer-open run that stopped while gripping the knob (e.g. 'context canceled' mid-pull).

    python tools/drawer_recover.py        # dry run: shows where the arm is and what it would do
    python tools/drawer_recover.py --go   # do it

Same ending as `drawer open` (LOG 6:57 recovery): finish the pull to open_far -> release the knob -> retreat -> high.
Refuses unless the arm is near the end of the taught pull (every joint within NEAR_DEG of the pull path), so it
can't sweep the arm somewhere untaught. Drawer is left OPEN; shut it with `python app.py drawer close`.
"""
import asyncio
import os
import sys

sys.path.insert(0, os.getcwd())
from dotenv import load_dotenv  # noqa: E402

load_dotenv(".env")
import config  # noqa: E402
import taught  # noqa: E402
from robot import RealRobot, retry  # noqa: E402

NEAR_DEG = 8.0
PULL_END = ["pull2", "pull3", "open", "open_far"]


def plan(now, data):
    """-> (nearest pose name, its max joint gap in deg). The arm must be close to the late part of the pull."""
    gaps = {n: max(abs(a - b) for a, b in zip(now, data[n]["joints"])) for n in PULL_END if n in data}
    name = min(gaps, key=gaps.get)
    return name, gaps[name]


async def main(go):
    from viam.proto.component.arm import JointPositions, MoveOptions
    data = taught.load()
    taught.add_open_far(data, config.DRAWER_EXTRA_OPEN_MM)
    r = await RealRobot.connect()
    try:
        now = list((await r.arm.get_joint_positions()).values)
        name, gap = plan(now, data)
        print(f"arm joints {[round(v, 1) for v in now]}; nearest taught pull pose: {name} ({gap:.1f} deg away)")
        if gap > NEAR_DEG:
            print(f"REFUSED: more than {NEAR_DEG:.0f} deg from the pull path. Don't guess: ask before moving.")
            return False
        print("plan: open_far (finish the pull) -> open gripper -> retreat -> high, at "
              f"{config.DRAWER_MAX_VEL_DEGS:.0f} deg/s. Drawer stays open.")
        if not go:
            print("dry run only: add --go to do it (hand on the E-stop)")
            return True
        opts = MoveOptions(max_vel_degs_per_sec=config.DRAWER_MAX_VEL_DEGS,
                           max_acc_degs_per_sec2=config.DRAWER_MAX_ACC_DEGS)

        async def through(names):
            await retry("joint move", lambda: r.arm.move_through_joint_positions(
                [JointPositions(values=data[n]["joints"]) for n in names], options=opts))
            print(f"  at {names[-1]}")
        await through(["open_far"])
        await r.gripper.open()
        print("  knob released")
        await through(["retreat", "high"])
        print("done: arm at 'high', drawer open. Next: python app.py drawer close")
        return True
    finally:
        await r.close()


if __name__ == "__main__":
    sys.exit(0 if asyncio.run(main("--go" in sys.argv)) else 1)
