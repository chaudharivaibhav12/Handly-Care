"""Go to taught joint poses (from near 'high'), e.g. to re-teach the next one by hand from there.

    python tools/goto_pose.py therm_above therm_grip --grab   # pick up the thermometer and stop there

Same speed cap and start-near-'high' guard as the routines (robot.replay). Optional --grab closes the gripper at
the end. Then: manual mode ON, move the arm, manual mode OFF, `python app.py joints <name>` to save.
"""
import asyncio
import os
import sys

sys.path.insert(0, os.getcwd())
from dotenv import load_dotenv  # noqa: E402

load_dotenv(".env")
import taught  # noqa: E402
from robot import RealRobot  # noqa: E402


async def main(args):
    grab = "--grab" in args
    names = [a for a in args if not a.startswith("--")]
    data = taught.load()
    missing = [n for n in names if n not in data]
    if not names or missing:
        print(__doc__ if not names else f"not taught: {missing}")
        return False
    steps = [("release", [])] + [("move", [n]) for n in names] + ([("grab", [])] if grab else [])
    r = await RealRobot.connect()
    try:
        await r.replay(data, steps)
        print(f"at {names[-1]}" + (", gripper closed" if grab else ""))
        return True
    finally:
        await r.close()


if __name__ == "__main__":
    sys.exit(0 if asyncio.run(main(sys.argv[1:])) else 1)
