"""Finding objects with our own height map of the camera's full point cloud (DEC-003).

    full point cloud (921,600 points, 1 per pixel, meters, camera frame, with RGB)
      -> world mm (one affine map, no RPC per point)
      -> a CELL_MM grid: mean height + color per cell
      -> fit the table plane (it is tilted ~±15 mm in depth) with trimmed least squares
      -> cells sticking up >= MIN_OBJECT_HEIGHT_MM -> drop lone cells -> connected blobs
      -> per blob: footprint (cell extents), height (95th percentile), color, pixel for the naming crop

Why not the Viam obstacles-pointcloud segments: they return object EDGES (2 cm flying-pixel columns),
not flat tops, so EXP-002 missed a flat object and read footprints 30-60% too small.
"""
import array
import math
import sys

import config


def parse_pcd(data):
    """-> (xyz floats array, rgb uint32 array or None, n points). Viam PCD: binary, x y z [rgb], meters."""
    i = data.find(b"DATA binary\n")
    if i < 0:
        raise ValueError("not a binary PCD")
    header = data[:i].decode(errors="replace").split("\n")
    fields = next((ln.split()[1:] for ln in header if ln.startswith("FIELDS")), ["x", "y", "z"])
    body = data[i + len(b"DATA binary\n"):]
    nf = len(fields)
    n = len(body) // (4 * nf)
    f = array.array("f")
    f.frombytes(body[: n * 4 * nf])
    rgb = None
    if "rgb" in fields:
        rgb = array.array("I")
        rgb.frombytes(body[: n * 4 * nf])
    if sys.byteorder != "little":
        f.byteswap()
        if rgb is not None:
            rgb.byteswap()
    return f, rgb, n, nf, fields.index("rgb") if "rgb" in fields else None


def grid_from_points(points):
    """points: iterable of (wx, wy, wz, cx, cy, cz, (r, g, b) or None) in mm.
    -> {(i, j): [sum_z, n, sum_cx, sum_cy, sum_cz, sum_r, sum_g, sum_b, n_rgb]}"""
    c = config.CELL_MM
    grid = {}
    for wx, wy, wz, cx, cy, cz, col in points:
        k = (int(math.floor(wx / c)), int(math.floor(wy / c)))
        g = grid.get(k)
        if g is None:
            g = grid[k] = [0.0, 0, 0.0, 0.0, 0.0, 0, 0, 0, 0]
        g[0] += wz
        g[1] += 1
        g[2] += cx
        g[3] += cy
        g[4] += cz
        if col is not None:
            g[5] += col[0]
            g[6] += col[1]
            g[7] += col[2]
            g[8] += 1
    return grid


def fit_table(grid):
    """Trimmed least-squares plane z = a x + b y + c over the cells' mean heights (the table is most cells)."""
    c = config.CELL_MM
    pts = [((i + 0.5) * c, (j + 0.5) * c, g[0] / g[1]) for (i, j), g in grid.items() if g[1] >= 2]
    plane = None
    for limit in (40.0, 15.0, 8.0):
        if len(pts) < 10:
            break
        sxx = sum(p[0] * p[0] for p in pts); sxy = sum(p[0] * p[1] for p in pts)
        syy = sum(p[1] * p[1] for p in pts); sx = sum(p[0] for p in pts); sy = sum(p[1] for p in pts)
        sxz = sum(p[0] * p[2] for p in pts); syz = sum(p[1] * p[2] for p in pts); sz = sum(p[2] for p in pts)
        n = len(pts)
        sol = _solve3([[sxx, sxy, sx], [sxy, syy, sy], [sx, sy, n]], [sxz, syz, sz])
        if sol is None:
            break
        plane = sol
        a, b, k = sol
        if limit == 40.0:   # first pass: median residual decides what "table" is (objects are a minority)
            res = sorted(abs(p[2] - (a * p[0] + b * p[1] + k)) for p in pts)
            limit = max(limit, res[len(res) // 2] * 3)
        pts = [p for p in pts if abs(p[2] - (a * p[0] + b * p[1] + k)) < limit]
    if plane is None:
        z0 = config.TABLE_TOP_WORLD_Z
        return lambda x, y: z0
    a, b, k = plane
    return lambda x, y: a * x + b * y + k


def _solve3(A, b):
    M = [row[:] + [v] for row, v in zip(A, b)]
    for col in range(3):
        p = max(range(col, 3), key=lambda r: abs(M[r][col]))
        if abs(M[p][col]) < 1e-9:
            return None
        M[col], M[p] = M[p], M[col]
        for r in range(3):
            if r != col:
                f = M[r][col] / M[col][col]
                M[r] = [M[r][q] - f * M[col][q] for q in range(4)]
    return [M[i][3] / M[i][i] for i in range(3)]


def _pct(vals, q):
    vals = sorted(vals)
    return vals[min(len(vals) - 1, max(0, int(q * (len(vals) - 1))))]


def blobs_from_grid(grid, table):
    """Cells above the table -> connected blobs: [{x, y, w, d, h, cam, rgb, cells}] (world mm)."""
    c = config.CELL_MM
    height = {}
    for (i, j), g in grid.items():
        h = g[0] / g[1] - table((i + 0.5) * c, (j + 0.5) * c)
        if config.MIN_OBJECT_HEIGHT_MM <= h <= config.MAX_BLOB_HEIGHT_MM:
            height[(i, j)] = h
    nbrs = [(di, dj) for di in (-1, 0, 1) for dj in (-1, 0, 1) if di or dj]
    up = {k for k in height if sum((k[0] + di, k[1] + dj) in height for di, dj in nbrs) >= 2}  # no lone specks
    seen, out = set(), []
    for start in up:
        if start in seen:
            continue
        comp, stack = [], [start]
        seen.add(start)
        while stack:
            k = stack.pop()
            comp.append(k)
            for di, dj in nbrs:
                q = (k[0] + di, k[1] + dj)
                if q in up and q not in seen:
                    seen.add(q)
                    stack.append(q)
        if len(comp) < config.MIN_OBJECT_CELLS or max(height[k] for k in comp) < config.MIN_OBJECT_PEAK_MM:
            continue
        i0, i1 = min(k[0] for k in comp), max(k[0] for k in comp)
        j0, j1 = min(k[1] for k in comp), max(k[1] for k in comp)
        # cell extents minus one cell: edge cells are only partly the object (read ~1 cm too wide, Fri 2:45)
        wx, wy = max(c, (i1 - i0) * c), max(c, (j1 - j0) * c)
        h = _pct([height[k] for k in comp], 0.95)
        band = [k for k in comp if height[k] >= h - config.TOP_BAND_MM]   # the part a near-top grip holds
        ti0, ti1 = min(k[0] for k in band), max(k[0] for k in band)
        tj0, tj1 = min(k[1] for k in band), max(k[1] for k in band)
        tx, ty = max(c, (ti1 - ti0) * c), max(c, (tj1 - tj0) * c)
        n = sum(grid[k][1] for k in comp)
        nrgb = sum(grid[k][8] for k in comp)
        out.append({
            "x": (i0 + i1 + 1) * c / 2, "y": (j0 + j1 + 1) * c / 2,
            "w": max(wx, wy), "d": min(wx, wy), "h": h, "top_d": min(tx, ty),
            "cam": tuple(sum(grid[k][q] for k in comp) / n for q in (2, 3, 4)),
            "rgb": tuple(sum(grid[k][q] for k in comp) / nrgb for q in (5, 6, 7)) if nrgb else None,
            "cells": len(comp),
        })
    return out


def points_from_cloud(data, O, cols, step=None):
    """PCD bytes (camera frame, meters) -> world points via the affine (O, cols), every `step`-th point."""
    f, rgb, n, nf, ri = parse_pcd(data)
    step = step or config.CLOUD_STEP
    (ox, oy, oz), (ax, ay, az), (bx, by, bz), (kx, ky, kz) = O, cols[0], cols[1], cols[2]
    out = []
    for p in range(0, n, step):
        q = p * nf
        z = f[q + 2]
        if not (0.05 < z < config.CLOUD_MAX_RANGE_M):
            continue
        x, y = f[q] * 1000.0, f[q + 1] * 1000.0
        z *= 1000.0
        col = None
        if ri is not None:
            v = rgb[q + ri]
            col = ((v >> 16) & 255, (v >> 8) & 255, v & 255)
        out.append((ox + x * ax + y * bx + z * kx, oy + x * ay + y * by + z * ky,
                    oz + x * az + y * bz + z * kz, x, y, z, col))
    return out
