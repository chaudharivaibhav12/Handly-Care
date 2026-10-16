import asyncio, os, sys, math
sys.path.insert(0, os.getcwd())
from dotenv import load_dotenv
load_dotenv(".env")
import config
from robot import RealRobot
async def main():
    r = await RealRobot.connect()
    try:
        p = await r.where()
        print(f"start {p.x:.0f} {p.y:.0f} {p.z:.0f}")
        await r.go_overview()   # lifts first if low; low + tilted -> joint lean-back first (Fri 7:20)
        q = await r.where()
        err = math.dist((q.x, q.y, q.z), config.OVERVIEW)
        print(f"overview {q.x:.1f} {q.y:.1f} {q.z:.1f} theta {q.theta:.0f}; {err:.1f} mm from OVERVIEW")
    finally:
        await r.close()
asyncio.run(main())
