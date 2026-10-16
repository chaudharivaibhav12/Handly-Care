"""MODE "anything": find ANY object on the table by its 3D shape, then name it with Claude.

    depth camera -> point-cloud segmenter (Viam obstacles_pointcloud, config.ANYTHING_SEGMENTER)
        -> one blob per thing sticking up from the table: world x, y + measured size (mm)
        -> filter (arm base, the tray itself, noise) -> vote over several looks
        -> identity: same thing as before? (tracked by position, then by size, then by name)
        -> new things: Claude looks at a crop of each and names it ("blue mug")
        -> registered in config.OBJECTS with measured size, grasp height and graspable/why-not

Everything downstream (planner, places, executor, robot) keeps using config.OBJECTS, so it
works unchanged. Whether the arm CAN pick something up comes from measurements, not a list:
too wide for the gripper, too flat, or too tall -> refused with the measured reason.
"""
import asyncio
import base64
import io
import json
import math
import re

import calibration
import config

_known = {}            # id -> {"pos": (x, y) last seen or expected, "name": str}
_intrinsics = {}       # id(robot) -> (fx, fy, cx, cy, W, H)


def reset():
    """Forget every discovered object (tests; a fresh table)."""
    _known.clear()
    for o in [o for o, i in config.OBJECTS.items() if i.get("found")]:
        del config.OBJECTS[o]


# ---------------- measuring one blob ----------------

def grip_plan(size, top_d=None):
    """(ok, reason, grasp_height mm, where) from the measured size. Grip in the middle if the footprint
    fits the gripper; else, for tapered things (an upside-down cup, a bottle), near the top if the top fits."""
    w, d, h = size
    opening = config.GRIPPER_MAX_OPEN_MM - config.GRASP_MARGIN_MM
    if h < config.MIN_GRASP_HEIGHT_MM:
        return False, f"it's too flat to grab ({h:.0f} mm tall)", None, None
    if h > config.MAX_GRASP_OBJECT_HEIGHT_MM:
        return False, f"it's too tall for me ({h / 10:.0f} cm)", None, None
    top_ok = top_d is not None and top_d <= opening and h >= config.TOP_GRIP_DEPTH_MM + config.MIN_GRASP_HEIGHT_MM
    if top_ok and (d > opening or top_d + 10 < d):     # grip where it's narrowest (real cup read 93x73, top ~60)
        return True, "", round(h - config.TOP_GRIP_DEPTH_MM, 1), "top"
    if d <= opening:
        if h > config.TALL_RATIO * d:      # tall and thin: grip higher so it can't swing and tip (Fri 2:23)
            return True, "", round(max(grasp_height(h), min(h * 0.65, h - 12.0)), 1), "high"
        return True, "", grasp_height(h), "middle"
    return False, (f"it's {d / 10:.0f} cm wide and my gripper only opens to "
                   f"{config.GRIPPER_MAX_OPEN_MM / 10:.1f} cm"), None, None


def graspable(size, top_d=None):
    """(ok, reason) from the measured size (footprint w >= d, height h), in mm."""
    ok, why, _, _ = grip_plan(size, top_d)
    return ok, why


def grasp_height(h):
    """Where the fingertips close, mm above the table: middle, but at least 10 mm under the top."""
    return round(max(8.0, min(h * 0.5, h - 10.0)), 1)


def skip_reason(b):
    """Why a blob isn't an object we should consider (None = keep it)."""
    from perception import in_table_area
    if math.hypot(b["x"], b["y"]) < config.MIN_REACH_MM * 0.6:
        return "the arm itself"
    if not in_table_area(b["x"], b["y"]):
        return "off the table"
    if b["h"] < config.MIN_OBJECT_HEIGHT_MM or b["w"] < config.MIN_OBJECT_WIDTH_MM:
        return "too small (noise)"
    if b["w"] > config.ANYTHING_MAX_DIM_MM:
        return "too big (segmentation junk)"
    for name, (cx, cy, r) in config.FIXTURES.items():
        if math.dist((b["x"], b["y"]), (cx, cy)) < r and b["w"] > config.FIXTURE_MIN_DIM_MM \
                and b["h"] < config.FIXTURE_MAX_HEIGHT_MM:
            return f"the {name} itself"
        if math.dist((b["x"], b["y"]), (cx, cy)) < r and b["h"] < config.FIXTURE_RIM_MM:
            return f"the {name}'s rim"
    return None


async def _camera_model(robot):
    key = id(robot)
    if key not in _intrinsics:
        ip = (await robot.camera.get_properties()).intrinsic_parameters
        _intrinsics[key] = (ip.focal_x_px, ip.focal_y_px, ip.center_x_px, ip.center_y_px,
                            ip.width_px, ip.height_px)
    return _intrinsics[key]


async def frame_affine(robot, ref):
    """(origin, [world vector of the frame's x, y, z axes]) from 4 transforms."""
    import perception
    from viam.proto.common import Pose
    if ref == "world":
        return (0.0, 0.0, 0.0), [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)]
    o, ex, ey, ez = [await perception.to_world(robot, ref, Pose(x=a, y=b, z=c, o_z=1))
                     for a, b, c in ((0, 0, 0), (100, 0, 0), (0, 100, 0), (0, 0, 100))]
    O = (o.x, o.y, o.z)
    return O, [tuple((e - b) / 100.0 for e, b in zip((p.x, p.y, p.z), O)) for p in (ex, ey, ez)]


async def frame_map(robot, ref):
    """A local function frame->world built from 4 transforms (instead of one RPC per fragment)."""
    O, cols = await frame_affine(robot, ref)
    return lambda x, y, z: tuple(O[i] + x * cols[0][i] + y * cols[1][i] + z * cols[2][i] for i in range(3))


async def cloud_blobs(robot, cal):
    """DEC-003: objects from our own height map of the full point cloud. [blob, ...] like blobs_once."""
    import heightmap
    try:
        data, _ = await robot.camera.get_point_cloud()
    except Exception as e:  # noqa: BLE001
        print(f"  point cloud failed: {e}")
        return []
    O, cols = await frame_affine(robot, config.CAMERA)
    grid = heightmap.grid_from_points(heightmap.points_from_cloud(data, O, cols))
    cam = None
    out = []
    for b in heightmap.blobs_from_grid(grid, heightmap.fit_table(grid)):
        b["u"] = b["v"] = b["pr"] = None
        X, Y, Z = b["cam"]
        if Z > 0:
            try:
                cam = cam or await _camera_model(robot)
                fx, fy, cx, cy, _, _ = cam
                b["u"], b["v"], b["pr"] = fx * X / Z + cx, fy * Y / Z + cy, fx * b["w"] / 2.0 / Z
            except Exception:  # noqa: BLE001
                pass
        b["x"], b["y"] = calibration.apply(cal, b["x"], b["y"])
        if skip_reason(b) is None:
            out.append(b)
    return out


def _solve3(A, b):
    import copy
    M = [row[:] + [v] for row, v in zip(copy.deepcopy(A), b)]
    for c in range(3):
        p = max(range(c, 3), key=lambda r: abs(M[r][c]))
        if abs(M[p][c]) < 1e-9:
            return None
        M[c], M[p] = M[p], M[c]
        for r in range(3):
            if r != c:
                f = M[r][c] / M[c][c]
                M[r] = [M[r][k] - f * M[c][k] for k in range(4)]
    return [M[i][3] / M[i][i] for i in range(3)]


def table_plane(frags):
    """Fit the table (z = a x + b y + c, world mm) to the flat noise fragments the segmenter left
    behind. The real table is tilted/warped by ±15 mm in depth, so one constant doesn't work.
    Falls back to the constant TABLE_TOP_WORLD_Z when there isn't enough noise to fit."""
    pts = [((f["xmin"] + f["xmax"]) / 2, (f["ymin"] + f["ymax"]) / 2, (f["top"] + f["bot"]) / 2)
           for f in frags if f["top"] - f["bot"] < config.FLAT_FRAGMENT_MM]
    flat = lambda x, y: config.TABLE_TOP_WORLD_Z  # noqa: E731
    for _ in range(2):                          # fit, drop outliers, refit
        if len(pts) < 6:
            return flat
        A = [[sum(p[i] * p[j] for p in pts) for j in range(2)] + [sum(p[i] for p in pts)] for i in range(2)]
        A.append([sum(p[0] for p in pts), sum(p[1] for p in pts), len(pts)])
        b = [sum(p[0] * p[2] for p in pts), sum(p[1] * p[2] for p in pts), sum(p[2] for p in pts)]
        sol = _solve3(A, b)
        if sol is None:
            return flat
        a, bb, c = sol
        flat = lambda x, y, a=a, bb=bb, c=c: a * x + bb * y + c  # noqa: E731
        pts = [p for p in pts if abs(p[2] - flat(p[0], p[1])) < 10]
    return flat


def merge(frags, table):
    """Fragments -> objects: drop pieces that are table-level, then union pieces whose footprints
    touch (within MERGE_GAP_MM). The segmenter splits one mug into 4-5 pieces."""
    keep = []
    for f in frags:
        f["h"] = f["top"] - table((f["xmin"] + f["xmax"]) / 2, (f["ymin"] + f["ymax"]) / 2)
        if f["h"] >= config.MIN_OBJECT_HEIGHT_MM:
            if "pcd" in f and "pts" not in f:     # decided by height above the table, NOT thickness:
                f["pts"] = [f["to_w"](*p) for p in pcd_points(f["pcd"])]   # a flat top is a thin piece
            keep.append(f)
    parent = list(range(len(keep)))

    def root(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i
    g = config.MERGE_GAP_MM
    for i in range(len(keep)):
        for j in range(i + 1, len(keep)):
            a, b = keep[i], keep[j]
            if a["xmin"] - g <= b["xmax"] and b["xmin"] - g <= a["xmax"] and                     a["ymin"] - g <= b["ymax"] and b["ymin"] - g <= a["ymax"]:
                parent[root(i)] = root(j)
    groups = {}
    for i, f in enumerate(keep):
        groups.setdefault(root(i), []).append(f)
    out = []
    for fs in groups.values():
        x0, x1 = min(f["xmin"] for f in fs), max(f["xmax"] for f in fs)
        y0, y1 = min(f["ymin"] for f in fs), max(f["ymax"] for f in fs)
        cams = [f["cam"] for f in fs if f["cam"] is not None]
        b = {"x": (x0 + x1) / 2, "y": (y0 + y1) / 2,
             "w": max(x1 - x0, y1 - y0), "d": min(x1 - x0, y1 - y0), "h": max(f["h"] for f in fs),
             "cam": tuple(sum(c[k] for c in cams) / len(cams) for k in range(3)) if cams else None,
             "pieces": len(fs)}
        pts = [p for f in fs for p in f.get("pts", ())]
        if pts:                                   # real segments: measure from the 3D points
            m = measure_points(pts, table)
            if m is None:
                continue
            b["x"], b["y"], b["w"], b["d"], b["h"] = m
        out.append(b)
    return out


def pcd_points(data, every=3):
    """Points (x, y, z) in mm from a Viam PCD (binary, fields x y z [rgb], float meters)."""
    import struct
    i = data.find(b"DATA binary\n")
    if i < 0:
        return []
    header = data[:i].decode(errors="replace").split("\n")
    fields = next((ln.split()[1:] for ln in header if ln.startswith("FIELDS")), ["x", "y", "z"])
    stride = 4 * len(fields)
    body = data[i + len(b"DATA binary\n"):]
    out = []
    for k in range(0, len(body) - stride + 1, stride * every):
        x, y, z = struct.unpack_from("<fff", body, k)
        out.append((x * 1000.0, y * 1000.0, z * 1000.0))
    return out


def _pct(vals, q):
    vals = sorted(vals)
    return vals[min(len(vals) - 1, max(0, int(q * (len(vals) - 1))))]


def measure_points(pts, table):
    """Robust size from an object's world points: height = 95th percentile above the local table
    (flying-pixel streaks lie BELOW the top, so they don't inflate it); footprint = 5-95th
    percentile of the points that really stick up. Returns (x, y, w, d, h) or None."""
    up = [(x, y, z - table(x, y)) for x, y, z in pts]
    up = [p for p in up if p[2] >= config.MIN_OBJECT_HEIGHT_MM]
    if len(up) < config.MIN_OBJECT_POINTS:
        return None
    h = _pct([p[2] for p in up], 0.95)
    xs, ys = [p[0] for p in up], [p[1] for p in up]
    x0, x1, y0, y1 = _pct(xs, 0.05), _pct(xs, 0.95), _pct(ys, 0.05), _pct(ys, 0.95)
    return (x0 + x1) / 2, (y0 + y1) / 2, max(x1 - x0, y1 - y0), min(x1 - x0, y1 - y0), h


async def blobs_once(robot, cal):
    """One look: [blob, ...]; blob = {x, y, w, d, h, u, v, pr} (world mm; pixel center + radius)."""
    import perception
    if getattr(config, "ANYTHING_SOURCE", "segmenter") == "cloud":
        return await cloud_blobs(robot, cal)
    try:
        objs = await perception.segmenter_for(robot, config.ANYTHING_SEGMENTER)             .get_object_point_clouds(config.CAMERA)
    except Exception as e:  # noqa: BLE001
        print(f"  segmenter {config.ANYTHING_SEGMENTER} failed: {e}")
        return []
    maps, frags = {}, []
    for o in objs:
        geoms = list(o.geometries.geometries)
        if not geoms or not geoms[0].HasField("box"):
            continue
        g = geoms[0]
        d, c = g.box.dims_mm, g.center
        ref = o.geometries.reference_frame or config.CAMERA
        if ref not in maps:
            maps[ref] = await frame_map(robot, ref)
        to_w = maps[ref]
        corners = [to_w(c.x + sx * d.x / 2, c.y + sy * d.y / 2, c.z + sz * d.z / 2)
                   for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
        f = {"xmin": min(p[0] for p in corners), "xmax": max(p[0] for p in corners),
             "ymin": min(p[1] for p in corners), "ymax": max(p[1] for p in corners),
             "top": max(p[2] for p in corners), "bot": min(p[2] for p in corners),
             "cam": (c.x, c.y, c.z) if ref == config.CAMERA and c.z > 0 else None}
        if getattr(o, "point_cloud", b""):
            f["pcd"], f["to_w"] = o.point_cloud, to_w    # parsed later, only for pieces above the table
        frags.append(f)
    out = []
    cam = None
    for b in merge(frags, table_plane(frags)):
        b["u"] = b["v"] = b["pr"] = None
        if b["cam"] is not None:                  # pixel position, for the naming crop (approximate)
            try:
                cam = cam or await _camera_model(robot)
                fx, fy, cx, cy, _, _ = cam
                X, Y, Z = b["cam"]
                b["u"], b["v"], b["pr"] = fx * X / Z + cx, fy * Y / Z + cy, fx * b["w"] / 2.0 / Z
            except Exception:  # noqa: BLE001
                pass
        b["x"], b["y"] = calibration.apply(cal, b["x"], b["y"])
        if skip_reason(b) is None:
            out.append(b)
    return dedupe(out)


def dedupe(blobs):
    """Two candidates refined onto the same object (centers closer than either's half-width): keep the larger."""
    out = []
    for b in sorted(blobs, key=lambda b: -b["w"] * b["d"]):
        if all(math.dist((b["x"], b["y"]), (o["x"], o["y"])) > max(o["w"], b["w"]) / 2 for o in out):
            out.append(b)
    return out


def vote(looks, min_hits=None):
    """Class-agnostic: cluster blobs by position across looks; keep ones seen in >= min_hits looks.
    Returns averaged blobs (pixel position from the most recent sighting)."""
    min_hits = config.MIN_HITS if min_hits is None else min_hits
    clusters = []
    for i, look in enumerate(looks):
        for b in look:
            for c in clusters:
                cx = sum(p["x"] for _, p in c) / len(c)
                cy = sum(p["y"] for _, p in c) / len(c)
                if math.dist((b["x"], b["y"]), (cx, cy)) <= config.MATCH_RADIUS_MM:
                    c.append((i, b))
                    break
            else:
                clusters.append([(i, b)])
    out = []
    for c in clusters:
        if len({i for i, _ in c}) < min_hits:
            continue
        avg = {k: sum(p[k] for _, p in c) / len(c) for k in ("x", "y", "w", "d", "h")}
        last = max(c, key=lambda ip: ip[0])[1]
        avg.update(u=last["u"], v=last["v"], pr=last["pr"], rgb=last.get("rgb"))
        tops = [p.get("top_d") for _, p in c]
        avg["top_d"] = sum(tops) / len(tops) if all(t is not None for t in tops) else None
        out.append(avg)
    return out


# ---------------- naming ----------------

COLORS = {"red": (200, 40, 40), "orange": (230, 130, 40), "yellow": (230, 210, 50),
          "green": (50, 160, 70), "blue": (40, 90, 200), "purple": (130, 60, 170),
          "pink": (230, 120, 170), "white": (235, 235, 235), "gray": (128, 128, 128),
          "black": (30, 30, 30), "brown": (120, 80, 40)}


def crop(jpeg, b):
    """JPEG bytes of the region around blob b, or None."""
    if not jpeg or b.get("u") is None:
        return None
    try:
        from PIL import Image
        img = Image.open(io.BytesIO(jpeg)).convert("RGB")
        r = max(40.0, (b["pr"] or 40.0) * 1.6)
        box = (int(max(0, b["u"] - r)), int(max(0, b["v"] - r)),
               int(min(img.width, b["u"] + r)), int(min(img.height, b["v"] + r)))
        if box[2] - box[0] < 8 or box[3] - box[1] < 8:
            return None
        buf = io.BytesIO()
        img.crop(box).save(buf, format="JPEG", quality=85)
        return buf.getvalue()
    except Exception:  # noqa: BLE001
        return None


def color_of(jpeg_crop):
    """Rough color name of the middle of a crop, or None."""
    if not jpeg_crop:
        return None
    try:
        from PIL import Image
        img = Image.open(io.BytesIO(jpeg_crop)).convert("RGB")
        w, h = img.size
        px = img.crop((w // 3, h // 3, max(w // 3 + 1, 2 * w // 3), max(h // 3 + 1, 2 * h // 3))).resize((1, 1))
        rgb = px.getpixel((0, 0))
        return min(COLORS, key=lambda c: sum((a - b) ** 2 for a, b in zip(rgb, COLORS[c])))
    except Exception:  # noqa: BLE001
        return None


def color_name(rgb):
    """Nearest basic color name for an (r, g, b) average, or None."""
    if not rgb:
        return None
    return min(COLORS, key=lambda c: sum((a - b) ** 2 for a, b in zip(rgb, COLORS[c])))


def local_name(b, jpeg_crop=None):
    """No AI: a plain name from color + size. Always works."""
    color = color_of(jpeg_crop) or color_name(b.get("rgb"))
    name = f"{color} object" if color else ("small object" if b["w"] < 60 else "object")
    return {"name": name,
            "description": f"{'a ' + color if color else 'an'} object about "
                           f"{b['w'] / 10:.0f} x {b['d'] / 10:.0f} x {b['h'] / 10:.0f} cm",
            "stackable": False, "safe": True, "why_unsafe": ""}


NAMER_PROMPT = """These are {n} objects on a desk, each photographed from directly above by a robot arm's
camera. The measured size of each (footprint x height) is given. For each object, in order, give:
- "name": 1-3 lowercase words a person would use, color first if useful ("blue mug", "yellow highlighter")
- "description": one short phrase including what it is used for ("a blue mug, for drinks")
- "stackable": true only if it has a flat, stable top something could rest on
- "safe": false if a robot gripper should not pick it up (sharp, hot, open liquid, glass, phone/laptop/electronics)
- "why_unsafe": a short reason if not safe, else ""
Reply with a JSON list only, one entry per object, in order:
[{{"n": 1, "name": "...", "description": "...", "stackable": false, "safe": true, "why_unsafe": ""}}]"""


def name_with_ai(blobs, crops, client=None):
    """One Claude call for all new objects. Returns a list of dicts (same order) or raises."""
    if client is None:
        import ai
        client = ai.client(config.NAMER_TIMEOUT_S)
    content = []
    for i, (b, c) in enumerate(zip(blobs, crops), 1):
        content.append({"type": "text", "text": f"Object {i}: about {b['w'] / 10:.0f} x {b['d'] / 10:.0f} cm,"
                                                f" {b['h'] / 10:.0f} cm tall."})
        if c:
            content.append({"type": "image", "source": {"type": "base64", "media_type": "image/jpeg",
                                                        "data": base64.standard_b64encode(c).decode()}})
    content.append({"type": "text", "text": NAMER_PROMPT.format(n=len(blobs))})
    import ai
    kw = dict(model=ai.model(config.NAMER_MODEL), max_tokens=4000, output_config={"effort": "low"},
              messages=[{"role": "user", "content": content}])
    try:   # server-side refusal fallback (default routing); older SDKs / OpenRouter may not know the field
        if ai.via_openrouter():
            raise TypeError("no beta fallbacks on OpenRouter")
        msg = client.beta.messages.create(betas=["server-side-fallback-2026-07-01"], fallbacks="default", **kw)
    except TypeError:
        msg = client.messages.create(**kw)
    text = "".join(getattr(b, "text", "") for b in msg.content if getattr(b, "type", "text") == "text")
    m = re.search(r"\[.*\]", text, re.S)
    data = json.loads(m.group(0)) if m else None
    if not isinstance(data, list) or len(data) != len(blobs):
        raise ValueError("namer reply didn't list every object")
    out = []
    for d in data:
        name = str(d.get("name", "")).strip().lower()
        if not name:
            raise ValueError("namer gave an empty name")
        out.append({"name": name, "description": str(d.get("description", name)),
                    "stackable": bool(d.get("stackable")), "safe": d.get("safe", True) is not False,
                    "why_unsafe": str(d.get("why_unsafe", ""))})
    return out


def namer(blobs, crops, use_ai):
    """Names for new blobs: Claude if possible, else color + size. Never raises."""
    if use_ai:
        try:
            return name_with_ai(blobs, crops)
        except Exception as e:  # noqa: BLE001
            print(f"  naming with AI failed ({type(e).__name__}); using plain names")
    return [local_name(b, c) for b, c in zip(blobs, crops)]


async def color_image(robot):
    try:
        images, _ = await robot.camera.get_images(filter_source_names=["color"])
        for im in images:
            if "jpeg" in im.mime_type or "jpg" in im.mime_type:
                return im.data
    except Exception:  # noqa: BLE001
        pass
    return None


# ---------------- identity + registry ----------------

def slug(name):
    return re.sub(r"[^a-z0-9]+", "_", name.lower()).strip("_") or "object"


def _similar(size_a, size_b):
    return all(abs(a - b) <= max(20.0, 0.3 * max(a, b)) for a, b in zip(size_a, size_b))


def register(obj, b, info=None):
    """Create or refresh config.OBJECTS[obj] from a measured blob (+ naming info for new ones)."""
    old = config.OBJECTS.get(obj)
    size = (b["w"], b["d"], b["h"])
    top_d = b.get("top_d")
    if old and old.get("found"):
        size = tuple(round(0.5 * (a + c), 1) for a, c in zip(old["size"], size))   # smooth
        if top_d is not None and old.get("top_d") is not None:
            top_d = round(0.5 * (top_d + old["top_d"]), 1)
        info = info or old["_info"]
    size = tuple(round(v, 1) for v in size)
    ok, why, gh, where = grip_plan(size, top_d)
    if ok and not info["safe"]:
        ok, why = False, info["why_unsafe"] or "it isn't safe for me to handle"
    desc = info["description"] + f" (about {size[0] / 10:.0f} x {size[1] / 10:.0f} x {size[2] / 10:.0f} cm)"
    if not ok:
        desc += f"; CAN'T pick it up: {why}"
    elif where == "top":
        desc += "; I grip it near the top (narrower there)"
    config.OBJECTS[obj] = {
        "description": desc, "labels": [info["name"]], "size": size, "top_d": top_d, "grip": where,
        "grasp_height": gh if gh is not None else grasp_height(size[2]),
        "stackable": bool(info["stackable"] and ok and size[1] >= 40 and size[2] <= 80),
        "graspable": ok, "why_not": why, "found": True, "_info": info,
    }
    _known.setdefault(obj, {})["pos"] = (b["x"], b["y"])
    _known[obj]["name"] = info["name"]


def expect(obj, xy):
    """The arm just put `obj` at xy: look for it there next time."""
    if obj in _known:
        _known[obj]["pos"] = tuple(xy)


async def identify(blobs, name_new):
    """Match blobs to known objects; name the rest with `await name_new(blobs) -> [info]`.
    Returns the scene: {object_id: (x, y) or None} for every discovered object."""
    present = {}
    unmatched = list(range(len(blobs)))
    missing = [o for o in _known if o in config.OBJECTS]
    pairs = sorted((math.dist(_known[o]["pos"], (blobs[i]["x"], blobs[i]["y"])), o, i)
                   for o in missing for i in unmatched)
    matched = {}
    for dist, o, i in pairs:                      # 1. by position (it stayed, or the arm moved it)
        if dist > config.TRACK_RADIUS_MM:
            break
        if o not in matched and i not in matched.values():
            matched[o] = i
    unmatched = [i for i in unmatched if i not in matched.values()]
    missing = [o for o in missing if o not in matched]
    if len(unmatched) == 1 and len(missing) == 1 and \
            _similar(config.OBJECTS[missing[0]]["size"], tuple(blobs[unmatched[0]][k] for k in "wdh")):
        matched[missing.pop()] = unmatched.pop()  # 2. one thing moved by a person: same size -> same thing
    for o, i in matched.items():
        register(o, blobs[i])
        present[o] = (blobs[i]["x"], blobs[i]["y"])
    if unmatched:                                 # 3. new (or moved and looks different): name them
        infos = await name_new([blobs[i] for i in unmatched])
        for i, info in zip(unmatched, infos):
            obj = next((o for o in missing if _known[o].get("name") == info["name"]), None)
            if obj:                               # a missing object turned up elsewhere
                missing.remove(obj)
                register(obj, blobs[i])
            else:
                base = slug(info["name"])
                obj, n = base, 2
                while obj in config.OBJECTS:
                    obj, n = f"{base}_{n}", n + 1
                register(obj, blobs[i], info)
            present[obj] = (blobs[i]["x"], blobs[i]["y"])
    return {o: present.get(o) for o, i in config.OBJECTS.items() if i.get("found")}


async def find(robot, cal, frames, use_ai=None):
    """Look `frames` times (arm already at overview) and return {object_id: (x, y) or None}."""
    import os
    if use_ai is None:
        import ai
        use_ai = ai.available()
    for o in [o for o, i in config.OBJECTS.items() if not i.get("found")]:
        del config.OBJECTS[o]                     # the fixed catalog doesn't apply in this mode
    looks = []
    for i in range(frames):
        looks.append(await blobs_once(robot, cal))
        if i < frames - 1:
            await asyncio.sleep(config.FRAME_GAP_S)

    async def name_new(new):
        jpeg = await color_image(robot) if any(b.get("u") is not None for b in new) else None
        crops = [crop(jpeg, b) for b in new]
        return await asyncio.to_thread(namer, new, crops, use_ai)

    return await identify(vote(looks), name_new)
