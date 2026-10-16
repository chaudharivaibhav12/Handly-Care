"""Render the height map (world grid) as an image: gray = table, color = height, black = no points."""
import asyncio, os, sys, statistics
sys.path.insert(0, os.getcwd())
from dotenv import load_dotenv
load_dotenv(".env")
from PIL import Image, ImageDraw
import config, anything, heightmap
from robot import RealRobot
async def main(out):
    r = await RealRobot.connect()
    try:
        data, _ = await r.camera.get_point_cloud()
        O, cols = await anything.frame_affine(r, config.CAMERA)
    finally:
        await r.close()
    grid = heightmap.grid_from_points(heightmap.points_from_cloud(data, O, cols))
    table = heightmap.fit_table(grid)
    c = config.CELL_MM
    counts = [g[1] for g in grid.values()]
    print("cells", len(grid), "median pts/cell", statistics.median(counts))
    x0, x1, y0, y1 = 0, 560, -160, 760          # world mm shown
    W, H = int((y1 - y0) / c), int((x1 - x0) / c)
    img = Image.new("RGB", (W, H), (0, 0, 0))
    px = img.load()
    med = statistics.median(counts)
    for (i, j), g in grid.items():
        X, Y = (i + 0.5) * c, (j + 0.5) * c
        if not (x0 <= X < x1 and y0 <= Y < y1): continue
        h = g[0] / g[1] - table(X, Y)
        u, v = int((y1 - Y) / c), int((x1 - X) / c)       # image: left = +y, top = +x (like the camera view)
        if h < 4: col = (90, 90, 90)
        elif h < 8: col = (140, 140, 60)
        else:
            t = min(1.0, h / 80); col = (int(255 * t), int(255 * (1 - t)), 60)
        if g[1] < 0.3 * med: col = (col[0] // 3, col[1] // 3, 200)   # sparse cell: blue tint
        px[u, v] = col
    img = img.resize((W * 3, H * 3), Image.NEAREST)
    img.save(out)
asyncio.run(main(sys.argv[1]))
