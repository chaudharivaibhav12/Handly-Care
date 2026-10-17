"""Find the thermometer's two gripper positions by hand (no arm motion, gripper only).

    python tools/gripper_pos.py get          # where the fingers are now (0 = closed .. 850 = open, xArm)
    python tools/gripper_pos.py set 400      # move the fingers to 400
    python tools/gripper_pos.py open         # fully open

Hold the thermometer between the fingers (button between them), then step down from open, e.g. 500, 450, 420...:
  THERM_HOLD_POS  = the first number where it's held firmly but the light does NOT come on
  THERM_PRESS_POS = the first smaller number where the light comes on (button clicks)
Put both in config.py. Keep a hand on the thermometer: an open gripper drops it.
"""
import asyncio
import os
import sys

sys.path.insert(0, os.getcwd())
from dotenv import load_dotenv  # noqa: E402

load_dotenv(".env")
from robot import RealRobot  # noqa: E402


async def main(args):
    if not args or args[0] not in ("get", "set", "open") or (args[0] == "set" and len(args) != 2):
        print(__doc__)
        return False
    r = await RealRobot.connect()
    try:
        if args[0] == "open":
            await r.gripper.open()
        elif args[0] == "set":
            pos = int(args[1])
            if not 0 <= pos <= 850:
                print("position must be 0..850")
                return False
            print("set ->", await r.gripper.do_command({"set": pos}))
            await asyncio.sleep(0.8)
        print("gripper now:", await r.gripper.do_command({"get": True}))
        return True
    finally:
        await r.close()


if __name__ == "__main__":
    sys.exit(0 if asyncio.run(main(sys.argv[1:])) else 1)
