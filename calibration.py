"""Fix the camera's position error, the problem every team hit last month (2-9 cm off).

How it works: put a few objects on TAUGHT spots (slots/zones from Milestone 1, where the
gripper itself was centered, so those positions are the truth). The camera reports where
it thinks they are. We fit the best correction from "camera says" to "truth":
  - 3+ spread-out points: an affine fit (fixes shift, rotation, and scale)
  - 1-2 points, or points in a line: shift only
and apply it to every detection afterwards. Pure Python, no numpy.
"""
import json
import math
import os

import config

IDENTITY = {"kind": "none", "m": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], "points": 0,
            "before_mm": None, "after_mm": None}


def apply(cal, x, y):
    m = cal["m"]
    return (m[0][0] * x + m[0][1] * y + m[0][2],
            m[1][0] * x + m[1][1] * y + m[1][2])


def _solve3(a, b):
    """Solve a 3x3 linear system with Gaussian elimination. Returns None if singular."""
    a = [row[:] + [b[i]] for i, row in enumerate(a)]
    for col in range(3):
        piv = max(range(col, 3), key=lambda r: abs(a[r][col]))
        if abs(a[piv][col]) < 1e-9:
            return None
        a[col], a[piv] = a[piv], a[col]
        for r in range(3):
            if r != col:
                f = a[r][col] / a[col][col]
                a[r] = [a[r][k] - f * a[col][k] for k in range(4)]
    return [a[i][3] / a[i][i] for i in range(3)]


def _spread_mm(points):
    """Smaller principal spread (std dev, mm) of the points: ~0 if they lie on a line."""
    n = len(points)
    mx, my = sum(p[0] for p in points) / n, sum(p[1] for p in points) / n
    sxx = sum((p[0] - mx) ** 2 for p in points) / n
    syy = sum((p[1] - my) ** 2 for p in points) / n
    sxy = sum((p[0] - mx) * (p[1] - my) for p in points) / n
    tr, det = sxx + syy, sxx * syy - sxy * sxy
    smallest = tr / 2 - math.sqrt(max(tr * tr / 4 - det, 0.0))
    return math.sqrt(max(smallest, 0.0))


def _rms(cal, pairs):
    errs = [math.dist(apply(cal, *seen), true) for seen, true in pairs]
    return math.sqrt(sum(e * e for e in errs) / len(errs))


def fit(pairs, min_spread_mm=40.0):
    """pairs: [((seen_x, seen_y), (true_x, true_y)), ...] -> calibration dict."""
    if not pairs:
        raise ValueError("need at least one object on a taught spot")
    n = len(pairs)
    shift = (sum(t[0] - s[0] for s, t in pairs) / n, sum(t[1] - s[1] for s, t in pairs) / n)
    cal = {"kind": "shift", "m": [[1.0, 0.0, shift[0]], [0.0, 1.0, shift[1]]]}

    seen = [s for s, _ in pairs]
    if n >= 3 and _spread_mm(seen) >= min_spread_mm:
        # least squares: [x y 1] @ [a b c] = true_x, same for y
        ata = [[0.0] * 3 for _ in range(3)]
        atx, aty = [0.0] * 3, [0.0] * 3
        for (sx, sy), (tx, ty) in pairs:
            row = (sx, sy, 1.0)
            for i in range(3):
                for j in range(3):
                    ata[i][j] += row[i] * row[j]
                atx[i] += row[i] * tx
                aty[i] += row[i] * ty
        rx, ry = _solve3(ata, atx), _solve3(ata, aty)
        if rx and ry:
            affine = {"kind": "affine", "m": [rx, ry]}
            sx_ = math.hypot(rx[0], ry[0])
            sy_ = math.hypot(rx[1], ry[1])
            angle = math.degrees(math.atan2(ry[0], rx[0]))
            # A wild fit means a mismatched point, not a real camera error: keep the shift.
            if abs(sx_ - 1) <= 0.15 and abs(sy_ - 1) <= 0.15 and abs(angle) <= 10:
                cal = affine
    cal["points"] = n
    cal["before_mm"] = round(_rms(IDENTITY, pairs), 1)
    cal["after_mm"] = round(_rms(cal, pairs), 1)
    return cal


def load(path=None):
    path = path or config.CALIBRATION_PATH
    if not os.path.exists(path):
        return dict(IDENTITY)
    try:
        with open(path) as f:
            cal = json.load(f)
        m = cal["m"]
        if len(m) == 2 and all(len(r) == 3 for r in m):
            return cal
    except (OSError, ValueError, KeyError, TypeError):
        pass
    print(f"  ignoring unreadable {path}; using no correction")
    return dict(IDENTITY)


def save(cal, path=None):
    with open(path or config.CALIBRATION_PATH, "w") as f:
        json.dump(cal, f, indent=1)
