"""Draw a world grid + target spots onto color.jpg using the plane homography from map.json."""
import json, math, sys, os
sys.path.insert(0, sys.argv[3])
from PIL import Image, ImageDraw, ImageFont
d = sys.argv[1]; out = sys.argv[2]
info = json.load(open(os.path.join(d, "map.json")))
W, H = info["W"], info["H"]
pix = {"tl": (0, 0), "tr": (W, 0), "br": (W, H), "bl": (0, H)}
pairs = [(info["corners"][k][:2], pix[k]) for k in pix]

def solve(A, b):
    n = len(A); M = [row[:] + [b[i]] for i, row in enumerate(A)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(M[r][c])); M[c], M[p] = M[p], M[c]
        for r in range(n):
            if r != c:
                f = M[r][c] / M[c][c]
                M[r] = [M[r][k] - f * M[c][k] for k in range(n + 1)]
    return [M[i][n] / M[i][i] for i in range(n)]
A, b = [], []
for (x, y), (u, v) in pairs:
    A.append([x, y, 1, 0, 0, 0, -u * x, -u * y]); b.append(u)
    A.append([0, 0, 0, x, y, 1, -v * x, -v * y]); b.append(v)
h = solve(A, b)
def P(x, y):
    w = h[6] * x + h[7] * y + 1
    return ((h[0] * x + h[1] * y + h[2]) / w, (h[3] * x + h[4] * y + h[5]) / w)

img = Image.open(os.path.join(d, "color.jpg")).convert("RGB")
dr = ImageDraw.Draw(img)
try:
    f = ImageFont.truetype("arial.ttf", 20); fs = ImageFont.truetype("arial.ttf", 15)
except OSError:
    f = fs = ImageFont.load_default()
CLEAN = os.environ.get("CLEAN") == "1"
G = (120, 120, 120)
for x in ([] if CLEAN else range(0, 701, 100)):
    dr.line([P(x, -100), P(x, 900)], fill=G, width=1)
for y in ([] if CLEAN else range(-100, 901, 100)):
    dr.line([P(-100, y), P(700, y)], fill=G, width=1)
for x in ([] if CLEAN else range(100, 601, 100)):
    for y in range(0, 801, 200):
        u, v = P(x, y)
        if 0 < u < W - 60 and 0 < v < H - 20:
            dr.text((u + 3, v + 2), f"{x},{y}", fill=G, font=fs)
# keep-out line and reach arcs
dr.line([P(480, -100), P(480, 900)], fill=(220, 40, 40), width=4)
u, v = P(480, 700); dr.text((u + 6, v - 24), "keep x < 480 (wall)", fill=(220, 40, 40), font=f)
for R, lab in (() if CLEAN else ((650, "max reach"), (200, "too close"))):
    pts = [P(R * math.cos(a / 100), R * math.sin(a / 100)) for a in range(-160, 160)]
    dr.line(pts, fill=(150, 60, 200), width=3)
    u, v = P(R * math.cos(1.2), R * math.sin(1.2)); dr.text((u + 4, v), lab, fill=(150, 60, 200), font=f)
spots = json.loads(os.environ["SPOTS"])
for name, (x, y, kind) in spots.items():
    if kind == "poly":
        pts = json.loads(os.environ["POLY_" + name])
        dr.polygon([P(a, b) for a, b in pts], outline=(30, 90, 200), width=4)
        u, v = P(x, y); dr.text((u - 40, v - 22), name + " (real)", fill=(255, 255, 255), font=f)
        dr.text((u - 40, v + 2), f"({x},{y})", fill=(255, 255, 255), font=fs)
    elif kind == "zone":
        poly = [P(x - 75, y - 75), P(x + 75, y - 75), P(x + 75, y + 75), P(x - 75, y + 75)]
        dr.polygon(poly, outline=(0, 140, 110), width=4)
        u, v = P(x, y); dr.text((u - 40, v - 22), name, fill=(0, 110, 90), font=f)
        dr.text((u - 40, v + 2), f"({x},{y})", fill=(0, 110, 90), font=fs)
    else:
        u, v = P(x, y); s = 14
        dr.line([(u - s, v - s), (u + s, v + s)], fill=(230, 90, 30), width=5)
        dr.line([(u - s, v + s), (u + s, v - s)], fill=(230, 90, 30), width=5)
        dr.text((u + 18, v - 20), name, fill=(200, 70, 20), font=f)
        dr.text((u + 18, v + 2), f"({x},{y})", fill=(200, 70, 20), font=fs)
img.save(out, quality=92)
print("saved", out)
for k in ("tl", "tr", "br", "bl"):
    print(k, [round(c) for c in info["corners"][k][:2]], "->", [round(c) for c in P(*info["corners"][k][:2])])
