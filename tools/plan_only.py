"""Look once (camera only, no arm motion), then show the AI plan for each request: python tools/plan_only.py "req1" "req2" ...
Each request is planned on its own (as if said fresh), against the same table."""
import asyncio, os, sys, time
sys.path.insert(0, os.getcwd())
from dotenv import load_dotenv
load_dotenv(".env")
import app, planner, perception, places
from robot import RealRobot
async def main(reqs):
    r = await RealRobot.connect()
    try:
        scene = await perception.find(r)
    finally:
        await r.close()
    print("table:", {o: p for o, p in scene.items() if p})
    for q in reqs:
        t = time.time()
        plan = planner.make_plan([q], scene)
        print(f"\n> {q}   ({time.time() - t:.1f} s)")
        print("  ", plan)
        for a in plan.get("actions", []) if isinstance(plan, dict) else []:
            try:
                print("   target", a["object"], a["destination"], "->", places.resolve(a["object"], a["destination"], scene))
            except Exception as e:  # noqa: BLE001
                print("   target", a, "->", type(e).__name__, e)
app.safe_console()
asyncio.run(main(sys.argv[1:]))
