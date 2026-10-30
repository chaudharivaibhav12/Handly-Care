"""Depth images: decode them, read them without getting fooled, and find objects on a table.

The idea used for the bottle and the tissue box, in four steps:
  1. Turn every depth pixel into a 3D point (geometry.deproject).
  2. Find the biggest flat surface: that's the table (a RANSAC plane fit).
  3. Anything sticking up more than ~12 mm above it is an object; group touching pixels.
  4. For each object, measure where its top is and how big it is. Those numbers replace
     the fixed heights and positions that were hard-coded before.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import cv2
import numpy as np

from geometry import Intrinsics, deproject_pixels, unit

DEPTH_MAGIC = b"DEPTHMAP"


# ---------------------------------------------------------------------------
# Decoding what Viam sends
# ---------------------------------------------------------------------------


def _plausible_fraction(d: np.ndarray) -> float:
    nz = d[d > 0]
    if nz.size == 0:
        return 0.0
    return float(np.mean((nz >= 100) & (nz <= 10000)))


def decode_viam_depth(data: bytes) -> np.ndarray:
    """Viam's raw depth format (MIME type image/vnd.viam.dep) -> H x W uint16 array in mm.

    Layout: the 8 bytes "DEPTHMAP", width and height as 8-byte big-endian numbers, then one
    2-byte value per pixel (big-endian). A camera set to little_endian_depth flips the pixel
    bytes, so both byte orders are tried and the one that looks like real distances wins.
    """
    if len(data) < 24 or data[:8] != DEPTH_MAGIC:
        raise ValueError("not a Viam depth map (no DEPTHMAP header)")
    w = int.from_bytes(data[8:16], "big")
    h = int.from_bytes(data[16:24], "big")
    n = w * h
    if w <= 0 or h <= 0 or len(data) < 24 + 2 * n:
        raise ValueError(f"depth map header says {w}x{h} but only {len(data)} bytes arrived")
    big = np.frombuffer(data, dtype=">u2", count=n, offset=24).reshape(h, w)
    little = np.frombuffer(data, dtype="<u2", count=n, offset=24).reshape(h, w)
    best = big if _plausible_fraction(big) >= _plausible_fraction(little) else little
    return best.astype(np.uint16)


def encode_viam_depth(depth: np.ndarray) -> bytes:
    """The reverse of decode_viam_depth (used by the simulator and the tests)."""
    d = np.ascontiguousarray(depth, dtype=">u2")
    h, w = d.shape
    return DEPTH_MAGIC + w.to_bytes(8, "big") + h.to_bytes(8, "big") + d.tobytes()


def decode_depth(data: bytes, mime_type: str) -> np.ndarray:
    """Depth from either Viam's raw format or a 16-bit PNG."""
    mt = str(mime_type)
    if mt.startswith("image/vnd.viam.dep") or data[:8] == DEPTH_MAGIC:
        return decode_viam_depth(data)
    img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_UNCHANGED)
    if img is None or img.ndim != 2:
        raise ValueError(f"can't read depth image of type {mt}")
    return img.astype(np.uint16)


def parse_pcd(data: bytes) -> np.ndarray:
    """Viam's binary PCD (x y z [rgb], meters, camera frame) -> (N, 3) float32 array."""
    i = data.find(b"DATA binary\n")
    if i < 0:
        raise ValueError("not a binary PCD point cloud")
    header = data[:i].decode(errors="replace").split("\n")
    fields = next((ln.split()[1:] for ln in header if ln.startswith("FIELDS")), ["x", "y", "z"])
    sizes = next((ln.split()[1:] for ln in header if ln.startswith("SIZE")), ["4"] * len(fields))
    if any(s != "4" for s in sizes):
        raise ValueError("unexpected PCD field sizes")
    body = data[i + len(b"DATA binary\n"):]
    nf = len(fields)
    n = len(body) // (4 * nf)
    rows = np.frombuffer(body, dtype="<f4", count=n * nf).reshape(n, nf)
    return rows[:, [fields.index("x"), fields.index("y"), fields.index("z")]]


def encode_pcd(points_m: np.ndarray) -> bytes:
    """(N, 3) meters -> Viam-style binary PCD (used by the simulator and the tests)."""
    pts = np.ascontiguousarray(points_m, dtype="<f4")
    n = len(pts)
    header = (f"VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {n}\nHEIGHT 1\n"
              f"VIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA binary\n")
    return header.encode() + pts.tobytes()


def depth_from_pcd(data: bytes, intr: Intrinsics, max_px_error: float = 1.5):
    """Depth image (mm, color-sized) from the camera's point cloud, or None if it can't be trusted.

    The RealSense module builds its point cloud from depth ALREADY ALIGNED to color, one point per
    color pixel, in pixel order. So point k is pixel (k % width, k // width), whatever the
    align_color_depth setting. We check that on a sample: project each point back with the
    intrinsics and it must land on its own pixel.
    """
    pts = parse_pcd(data)
    w, h = intr.width, intr.height
    if len(pts) != w * h:
        return None
    z = pts[:, 2].astype(np.float64)
    ok = np.isfinite(z) & (z > 0.05)
    if np.count_nonzero(ok) < 100:
        return None
    idx = np.flatnonzero(ok)
    sample = idx[np.linspace(0, len(idx) - 1, min(400, len(idx))).astype(int)]
    p = pts[sample].astype(np.float64)
    u = p[:, 0] * intr.fx / p[:, 2] + intr.cx
    v = p[:, 1] * intr.fy / p[:, 2] + intr.cy
    err = np.hypot(u - sample % w, v - sample // w)
    if np.median(err) > max_px_error:
        return None
    depth = np.where(ok, np.round(z * 1000.0), 0).clip(0, 65535).astype(np.uint16)
    return depth.reshape(h, w)


def decode_color(data: bytes, mime_type: str) -> np.ndarray:
    """Color image (JPEG, PNG or Viam's raw RGBA) -> BGR array for OpenCV."""
    mt = str(mime_type)
    if mt.startswith("image/vnd.viam.rgba"):
        w = int.from_bytes(data[4:8], "big")
        h = int.from_bytes(data[8:12], "big")
        rgba = np.frombuffer(data, np.uint8, count=w * h * 4, offset=12).reshape(h, w, 4)
        return cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR)
    img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        raise ValueError(f"can't read color image of type {mt}")
    return img


# ---------------------------------------------------------------------------
# Reading depth at a point without getting fooled
# ---------------------------------------------------------------------------


def robust_depth(depth: np.ndarray, u: float, v: float, radius: int,
                 valid_mm=(150, 2500), near_band_mm: float = 30.0, min_fraction: float = 0.2):
    """Depth at pixel (u, v), or None if there isn't enough good data around it.

    One pixel is never trusted: depth images have holes (0 = no reading) and edges where
    a pixel half-sees the background. We look at a small window, drop holes and silly
    values, keep the nearest surface (the forehead, not the wall behind), and take its median.
    """
    h, w = depth.shape
    r = max(1, int(radius))
    u0, u1 = max(0, int(round(u)) - r), min(w, int(round(u)) + r + 1)
    v0, v1 = max(0, int(round(v)) - r), min(h, int(round(v)) + r + 1)
    if u0 >= u1 or v0 >= v1:
        return None
    patch = depth[v0:v1, u0:u1].astype(np.float64)
    vals = patch[(patch >= valid_mm[0]) & (patch <= valid_mm[1])]
    if vals.size < max(3, min_fraction * patch.size):
        return None
    near = np.percentile(vals, 20)
    band = vals[np.abs(vals - near) <= near_band_mm]
    return float(np.median(band)) if band.size else None


def surface_normal(depth: np.ndarray, intr: Intrinsics, u: float, v: float, radius: int,
                   valid_mm=(150, 2500), max_rms_mm: float = 5.0):
    """Which way the surface around (u, v) faces: a unit vector in the camera frame pointing
    back toward the camera, or None if the patch is too holey or not flat enough (hair, edges)."""
    h, w = depth.shape
    r = max(2, int(radius))
    u0, u1 = max(0, int(round(u)) - r), min(w, int(round(u)) + r + 1)
    v0, v1 = max(0, int(round(v)) - r), min(h, int(round(v)) + r + 1)
    if u0 >= u1 or v0 >= v1:
        return None
    patch = depth[v0:v1, u0:u1].astype(np.float64)
    vs, us = np.nonzero((patch >= valid_mm[0]) & (patch <= valid_mm[1]))
    if vs.size < max(12, 0.3 * patch.size):
        return None
    pts = deproject_pixels(us + u0, vs + v0, patch[vs, us], intr)
    med = np.median(pts[:, 2])
    pts = pts[np.abs(pts[:, 2] - med) < 40]          # drop the background behind the head
    if len(pts) < 12:
        return None
    c = pts.mean(axis=0)
    _, s, Vt = np.linalg.svd(pts - c, full_matrices=False)
    n = Vt[-1]
    if n @ (-c) < 0:
        n = -n
    rms = s[-1] / np.sqrt(len(pts))
    facing = float(n @ unit(-c))   # a surface seen almost edge-on is not believable
    return unit(n) if rms <= max_rms_mm and facing >= 0.34 else None


# ---------------------------------------------------------------------------
# The table: biggest flat surface
# ---------------------------------------------------------------------------


@dataclass
class Plane:
    normal: np.ndarray   # unit vector, pointing toward the camera (camera frame)
    point: np.ndarray    # a point on the plane (camera frame, mm)
    inlier_fraction: float

    def height(self, points: np.ndarray) -> np.ndarray:
        """Signed height of points above the plane, in mm."""
        return (np.asarray(points) - self.point) @ self.normal


def fit_plane(points: np.ndarray, tol_mm: float = 6.0, iters: int = 300,
              seed: int = 0, max_points: int = 20000) -> Plane:
    """RANSAC: try many random 3-point planes, keep the one that most points lie on."""
    P = np.asarray(points, dtype=np.float64)
    if len(P) < 50:
        raise ValueError("not enough depth points to find the table")
    rng = np.random.default_rng(seed)
    S = P if len(P) <= max_points else P[rng.choice(len(P), max_points, replace=False)]
    best_n, best_d, best_count = None, 0.0, -1
    for _ in range(iters):
        a, b, c = S[rng.choice(len(S), 3, replace=False)]
        n = np.cross(b - a, c - a)
        nn = np.linalg.norm(n)
        if nn < 1e-6:
            continue
        n /= nn
        d = -float(n @ a)
        count = int(np.count_nonzero(np.abs(S @ n + d) < tol_mm))
        if count > best_count:
            best_n, best_d, best_count = n, d, count
    if best_n is None:
        raise ValueError("could not fit a plane")
    inl = np.abs(S @ best_n + best_d) < tol_mm
    Q = S[inl]
    centroid = Q.mean(axis=0)
    _, _, Vt = np.linalg.svd(Q - centroid, full_matrices=False)
    n = Vt[-1]
    if n @ (-centroid) < 0:   # the camera sits at the origin: make the normal face it
        n = -n
    frac = float(np.mean(np.abs((S - centroid) @ n) < tol_mm))
    return Plane(unit(n), centroid, frac)


# ---------------------------------------------------------------------------
# Objects standing on the table
# ---------------------------------------------------------------------------


@dataclass
class Blob:
    """One thing sticking up from the table, measured in mm (camera frame)."""

    id: int
    area_px: int
    bbox: tuple                 # x, y, w, h in pixels
    top_height_mm: float        # highest point above the table (97th percentile, so noise can't win)
    plateau_height_mm: float    # the most common "upper" height: a box lid, a bottle shoulder
    top_point_cam: np.ndarray   # middle of the top surface, camera frame
    top_pixel: tuple            # (u, v) of that point
    footprint_center_cam: np.ndarray   # middle of the footprint, on the table plane
    length_mm: float            # footprint, long side
    width_mm: float             # footprint, short side
    long_axis_cam: np.ndarray   # direction of the long side (camera frame, unit)
    contour: np.ndarray = field(repr=False, default=None)
    # Something small sticking up above the plateau (a tissue out of its box), if any:
    peak_height_mm: float | None = None
    peak_point_cam: np.ndarray | None = None
    peak_pixel: tuple | None = None
    peak_area_px: int = 0
    touches_edge: bool = False   # cut off by the edge of the search area: its numbers may be off

    @property
    def highest_mm(self) -> float:
        """Tallest point, counting anything sticking up above the flat top."""
        return max(self.top_height_mm, self.peak_height_mm or 0.0)

    def row(self) -> str:
        peak = (f", something sticks up {self.peak_height_mm - self.plateau_height_mm:.0f} mm above its top"
                if self.peak_height_mm is not None else "")
        edge = " [CUT OFF at the edge of the picture]" if self.touches_edge else ""
        return (f"#{self.id}: top {self.top_height_mm:5.0f} mm above table (flat part {self.plateau_height_mm:.0f}), "
                f"footprint {self.length_mm:4.0f} x {self.width_mm:4.0f} mm, {self.area_px} px{peak}{edge}")


def roi_mask(shape, roi, ignore_boxes=()) -> np.ndarray:
    h, w = shape
    m = np.zeros((h, w), bool)
    x0, y0, x1, y1 = roi
    m[int(y0 * h):int(y1 * h), int(x0 * w):int(x1 * w)] = True
    for bx0, by0, bx1, by1 in ignore_boxes:
        m[max(0, by0):max(0, by1), max(0, bx0):max(0, bx1)] = False
    return m


def _plane_basis(n: np.ndarray):
    helper = np.array([1.0, 0.0, 0.0]) if abs(n[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    e1 = unit(np.cross(n, helper))
    return e1, np.cross(n, e1)


def find_objects(depth: np.ndarray, intr: Intrinsics, roi=(0, 0, 1, 1), ignore_boxes=(),
                 valid_mm=(150, 2500), plane_tol_mm: float = 6.0, min_height_mm: float = 12.0,
                 max_height_mm: float = 350.0, min_area_px: int = 250, top_band_mm: float = 8.0,
                 plane: Plane | None = None):
    """Find the table and every object on it. Returns (blobs, plane, height_map).

    height_map[v, u] = mm above the table (NaN where there is no depth reading).
    """
    h, w = depth.shape
    valid = (depth >= valid_mm[0]) & (depth <= valid_mm[1]) & roi_mask((h, w), roi, ignore_boxes)
    vs, us = np.nonzero(valid)
    if vs.size < 100:
        raise ValueError("almost no depth readings: is the camera too close, or depth not aligned?")
    pts = deproject_pixels(us, vs, depth[vs, us], intr)
    if plane is None:
        plane = fit_plane(pts, plane_tol_mm)
    heights = plane.height(pts)
    height_map = np.full((h, w), np.nan, np.float32)
    height_map[vs, us] = heights

    with np.errstate(invalid="ignore"):
        mask = ((height_map > min_height_mm) & (height_map < max_height_mm)).astype(np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)

    e1, e2 = _plane_basis(plane.normal)
    blobs = []
    for k in range(1, count):
        area = int(stats[k, cv2.CC_STAT_AREA])
        if area < min_area_px:
            continue
        sel = labels == k
        bv, bu = np.nonzero(sel)
        bh = height_map[bv, bu].astype(np.float64)
        bp = deproject_pixels(bu, bv, depth[bv, bu], intr)

        top = float(np.percentile(bh, 97))
        top_sel = bh >= top - top_band_mm
        top_point = np.median(bp[top_sel], axis=0)
        top_pixel = (float(np.median(bu[top_sel])), float(np.median(bv[top_sel])))

        upper = bh[bh >= 0.5 * top]
        hist, edges = np.histogram(upper, bins=max(1, int((upper.max() - upper.min()) / 4.0) + 1))
        mode = 0.5 * (edges[np.argmax(hist)] + edges[np.argmax(hist) + 1])
        near_mode = upper[np.abs(upper - mode) <= 4.0]
        plateau = float(np.median(near_mode)) if near_mode.size else float(mode)

        flat = bp - np.outer((bp - plane.point) @ plane.normal, plane.normal)
        uv = np.stack([(flat - plane.point) @ e1, (flat - plane.point) @ e2], axis=1)
        mean_uv = uv.mean(axis=0)
        c = uv - mean_uv
        evals, evecs = np.linalg.eigh(c.T @ c / max(1, len(c) - 1))
        major, minor = evecs[:, 1], evecs[:, 0]
        along, across = c @ major, c @ minor
        a_lo, a_hi = np.percentile(along, [1, 99])
        c_lo, c_hi = np.percentile(across, [1, 99])
        length, width = float(a_hi - a_lo), float(c_hi - c_lo)
        center_uv = mean_uv + 0.5 * (a_lo + a_hi) * major + 0.5 * (c_lo + c_hi) * minor
        center = plane.point + center_uv[0] * e1 + center_uv[1] * e2
        long_axis = unit(major[0] * e1 + major[1] * e2)

        contours, _ = cv2.findContours(sel.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        x, y, bw, bh_px = (int(v) for v in stats[k, :4])
        blob = Blob(len(blobs) + 1, area, (x, y, bw, bh_px), top, plateau, top_point, top_pixel,
                    center, length, width, long_axis, max(contours, key=cv2.contourArea) if contours else None)
        rx0, ry0, rx1, ry1 = int(roi[0] * w), int(roi[1] * h), int(roi[2] * w), int(roi[3] * h)
        blob.touches_edge = x <= rx0 + 1 or y <= ry0 + 1 or x + bw >= rx1 - 1 or y + bh_px >= ry1 - 1

        # A small part rising above the flat top (the tissue sticking out of the box)?
        above = bh > plateau + top_band_mm
        if np.count_nonzero(above) >= 20:
            sub = np.zeros((h, w), np.uint8)
            sub[bv[above], bu[above]] = 1
            n_sub, sub_labels, sub_stats, _ = cv2.connectedComponentsWithStats(sub, connectivity=8)
            j = 1 + int(np.argmax(sub_stats[1:, cv2.CC_STAT_AREA]))
            if sub_stats[j, cv2.CC_STAT_AREA] >= 20:
                pick = sub_labels[bv, bu] == j
                peak = float(np.percentile(bh[pick], 95))
                band = pick & (bh >= peak - top_band_mm)
                blob.peak_height_mm = peak
                blob.peak_point_cam = np.median(bp[band], axis=0)
                blob.peak_pixel = (float(np.median(bu[band])), float(np.median(bv[band])))
                blob.peak_area_px = int(sub_stats[j, cv2.CC_STAT_AREA])
        blobs.append(blob)
    blobs.sort(key=lambda b: -b.area_px)
    for i, b in enumerate(blobs, 1):
        b.id = i
    return blobs, plane, height_map


# ---------------------------------------------------------------------------
# Pictures for humans (and for the demo screen)
# ---------------------------------------------------------------------------


def colorize_depth(depth: np.ndarray, valid_mm=(150, 2500)) -> np.ndarray:
    d = depth.astype(np.float32)
    ok = (d >= valid_mm[0]) & (d <= valid_mm[1])
    if not ok.any():
        return np.zeros(depth.shape + (3,), np.uint8)
    lo, hi = np.percentile(d[ok], 2), np.percentile(d[ok], 98)
    norm = np.clip((d - lo) / max(1.0, hi - lo), 0, 1)
    img = cv2.applyColorMap((255 * (1 - norm)).astype(np.uint8), cv2.COLORMAP_TURBO)
    img[~ok] = 0
    return img


def colorize_heights(height_map: np.ndarray, max_mm: float = 200.0) -> np.ndarray:
    hm = np.nan_to_num(height_map, nan=-1.0)
    norm = np.clip(hm / max_mm, 0, 1)
    img = cv2.applyColorMap((255 * norm).astype(np.uint8), cv2.COLORMAP_TURBO)
    img[hm < 0] = (40, 40, 40)
    return img


def depth_edges_overlay(color: np.ndarray, depth: np.ndarray) -> np.ndarray:
    """Color image with depth edges drawn in red. If depth is aligned to color, the red
    lines hug the outlines of objects. If they are shifted or scaled, alignment is off."""
    d8 = cv2.normalize(cv2.medianBlur(depth.astype(np.float32), 5), None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    edges = cv2.Canny(d8, 20, 60)
    out = color.copy()
    out[edges > 0] = (0, 0, 255)
    return out
