"""Run perception.find (anything mode) and draw each object's footprint + label on a fresh photo."""
import asyncio, io, json, os, sys
sys.path.insert(0, os.getcwd())
from dotenv import load_dotenv
load_dotenv(".env")
from PIL import Image, ImageDraw, ImageFont
import config, perception
from robot import RealRobot
exec(open(os.path.join(os.path.dirname(__file__), "homog.py")).read())
async def main(out):
    r = await RealRobot.connect()
    try:
        scene = await perception.find(r)
        imgs, _ = await r.camera.get_images(filter_source_names=["color"])
        img = Image.open(io.BytesIO(imgs[0].data)).convert("RGB")
    finally:
        await r.close()
    P = homography(json.load(open("results/overview-map.json")))
    dr = ImageDraw.Draw(img)
    f = ImageFont.truetype("arial.ttf", 18)
    for obj, pos in scene.items():
        if pos is None:
            continue
        info = config.OBJECTS[obj]; w, d, h = info["size"]; x, y = pos
        poly = [P(x - w / 2, y - d / 2), P(x + w / 2, y - d / 2), P(x + w / 2, y + d / 2), P(x - w / 2, y + d / 2)]
        col = (0, 200, 0) if info["graspable"] else (230, 0, 0)
        dr.polygon(poly, outline=col, width=4)
        u, v = P(x, y)
        dr.text((u - 40, v - 10), f"{obj}\n{w:.0f}x{d:.0f}x{h:.0f}", fill=col, font=f)
        print(f"{obj:16s} ({x:.0f},{y:.0f}) {w:.0f}x{d:.0f}x{h:.0f} graspable={info['graspable']}")
    img.save(out)
asyncio.run(main(sys.argv[1]))
