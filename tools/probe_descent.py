"""Step straight down from the current (tilted) pose in STEP mm steps to Z_MIN, keeping the tilt; report the first
refusal with the planner's reason; then straight back up to the start. python tools/probe_descent.py Z_MIN [STEP]"""
import asyncio, os, sys
sys.path.insert(0, os.getcwd())
from dotenv import load_dotenv
load_dotenv(".env")
import config
from robot import RealRobot, _why, arm_halted
async def main(zmin, step):
    r = await RealRobot.connect()
    try:
        p = await r.where()
        o, th = (p.o_x, p.o_y, p.o_z), p.theta
        tol = 10.0   # deg, as tried for the drawer at 5:07 PM
        z = p.z
        while z - step >= zmin:
            try:
                await r.move_to(p.x, p.y, z - step, theta=th, o=o, must_be_straight=True, orient_tol=tol)
                z -= step
                print(f"  ok z {z:.0f}")
            except Exception as e:
                if arm_halted(e):
                    raise
                print(f"  REFUSED going to z {z - step:.0f}: {_why(e)}")
                break
        if z != p.z:
            await r.move_to(p.x, p.y, p.z, theta=th, o=o, must_be_straight=True, orient_tol=tol)
            print(f"  back up at z {p.z:.0f}")
    finally:
        await r.close()
asyncio.run(main(float(sys.argv[1]), float(sys.argv[2]) if len(sys.argv) > 2 else 30.0))
