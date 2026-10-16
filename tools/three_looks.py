import asyncio, io, json, math, os, sys, time
sys.path.insert(0, os.getcwd())
from dotenv import load_dotenv
load_dotenv(".env")
from PIL import Image, ImageDraw, ImageFont
import config, perception
from robot import RealRobot
exec(open(os.path.join(os.path.dirname(__file__), "homog.py")).read())
async def main(out_img, out_json):
    r = await RealRobot.connect()
    runs = []
    try:
        for k in range(3):
            t = time.time()
            scene = await perception.find(r)
            runs.append({o: (None if p is None else [round(p[0], 1), round(p[1], 1)]) for o, p in scene.items()})
            print(f"run {k + 1} ({time.time() - t:.1f}s):", {o: p for o, p in runs[-1].items()})
        imgs, _ = await r.camera.get_images(filter_source_names=["color"])
        img = Image.open(io.BytesIO(imgs[0].data)).convert("RGB")
    finally:
        await r.close()
    summary = {}
    for o in config.OBJECTS:
        if not config.OBJECTS[o].get("found"):
            continue
        ps = [run.get(o) for run in runs]
        seen = [p for p in ps if p]
        cx = sum(p[0] for p in seen) / len(seen) if seen else None
        cy = sum(p[1] for p in seen) / len(seen) if seen else None
        spread = max(math.dist(p, (cx, cy)) for p in seen) if seen else None
        i = config.OBJECTS[o]
        summary[o] = {"seen": f"{len(seen)}/3", "mean": [cx, cy], "max_dev_mm": spread, "size": i["size"],
                      "grasp_height": i["grasp_height"], "graspable": i["graspable"], "why_not": i["why_not"]}
        print(f"{o:18s} seen {len(seen)}/3  mean ({cx:.0f},{cy:.0f})  spread {spread:.1f} mm  size {i['size']}  graspable {i['graspable']} {i['why_not']}")
    json.dump({"runs": runs, "summary": summary}, open(out_json, "w"), indent=1)
    P = homography(json.load(open("results/overview-map.json")))
    dr = ImageDraw.Draw(img); f = ImageFont.truetype("arial.ttf", 18)
    for o, s in summary.items():
        if s["mean"][0] is None: continue
        (x, y), (w, d, h) = s["mean"], s["size"]
        col = (0, 200, 0) if s["graspable"] else (230, 0, 0)
        dr.polygon([P(x - w/2, y - d/2), P(x + w/2, y - d/2), P(x + w/2, y + d/2), P(x - w/2, y + d/2)], outline=col, width=4)
        u, v = P(x, y); dr.text((u - 40, v - 10), f"{o}\n{w:.0f}x{d:.0f}x{h:.0f}", fill=col, font=f)
    img.save(out_img)
asyncio.run(main(sys.argv[1], sys.argv[2]))
