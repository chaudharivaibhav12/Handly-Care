"""Open the gripper only (no arm motion). Have someone hold whatever it's carrying first."""
import asyncio, os, sys
sys.path.insert(0, os.getcwd())
from dotenv import load_dotenv
load_dotenv(".env")
from robot import RealRobot
async def main():
    r = await RealRobot.connect()
    try:
        await r.gripper.open()
        print("gripper open")
    finally:
        await r.close()
asyncio.run(main())
