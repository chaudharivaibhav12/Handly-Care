"""Is the medicine drawer box still where its poses were taught? Find its yellow knob with the camera.

Sidekick opens the drawer by replaying taught joint poses with no camera (DEC-006), so if the box
moved even 2 cm, the fingers miss the knob. Before replaying, look at the knob:

    python care.py remember-drawer --execute   # once, right after teaching the drawer poses (goes to the overview)
    python care.py check-drawer --execute      # any time later: how far did the knob move?

`care.py medicine` runs the check itself and refuses if the box moved more than
DRAWER_MOVED_TOLERANCE_MM.
"""
from __future__ import annotations

import json
import os
import time
from dataclasses import dataclass

import cv2
import numpy as np

import settings
from geometry import deproject


class KnobNotFound(Exception):
    pass


@dataclass
class Knob:
    pixel: tuple
    area_px: int
    depth_mm: float
    point_w: np.ndarray


def find_knobs(cap) -> list[Knob]:
    """Every yellow blob with a depth reading, biggest first, in world mm."""
    hsv = cv2.cvtColor(cap.color, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(settings.KNOB_HSV_LOW), np.array(settings.KNOB_HSV_HIGH))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, labels, stats, centers = cv2.connectedComponentsWithStats(mask, connectivity=8)
    lo, hi = settings.DEPTH_VALID_MM
    knobs = []
    for k in range(1, n):
        area = int(stats[k, cv2.CC_STAT_AREA])
        if area < settings.KNOB_MIN_AREA_PX:
            continue
        vs, us = np.nonzero(labels == k)
        d = cap.depth[vs, us].astype(np.float64)
        d = d[(d >= lo) & (d <= hi)]
        if d.size < 5:
            continue
        u, v = (float(c) for c in centers[k])
        depth = float(np.median(d))
        knobs.append(Knob((u, v), area, depth, cap.to_world(deproject(u, v, depth, cap.intr))))
    knobs.sort(key=lambda kb: -kb.area_px)
    return knobs


def remember(cap, path: str | None = None) -> dict:
    knobs = find_knobs(cap)
    if not knobs:
        raise KnobNotFound("no yellow knob in view. Is the drawer box in the picture (arm at the overview)?")
    data = {"knobs": [kb.point_w.round(1).tolist() for kb in knobs[:3]], "time": time.strftime("%Y-%m-%d %H:%M")}
    with open(path or settings.KNOB_FILE, "w") as f:
        json.dump(data, f, indent=2)
    return data


def load(path: str | None = None) -> dict:
    path = path or settings.KNOB_FILE
    if not os.path.exists(path):
        raise KnobNotFound("I don't know where the drawer was taught. Drawer closed, box on its marks: "
                          "python care.py remember-drawer --execute")
    with open(path) as f:
        return json.load(f)


def check(cap, saved: dict):
    """How far the box moved since `remember`: (mm, [(remembered, now), ...]). Raises KnobNotFound."""
    now = find_knobs(cap)
    pairs = []
    for p in saved["knobs"]:
        p = np.asarray(p, dtype=np.float64)
        near = [kb for kb in now if np.linalg.norm(kb.point_w[:2] - p[:2]) <= settings.KNOB_SEARCH_RADIUS_MM]
        if near:
            best = min(near, key=lambda kb: np.linalg.norm(kb.point_w - p))
            pairs.append((p, best.point_w))
    if not pairs:
        raise KnobNotFound(f"I can't see the drawer's yellow knob within {settings.KNOB_SEARCH_RADIUS_MM:.0f} mm of "
                           f"where it was. The box may have moved a lot, or something blocks the view.")
    moved = float(np.median([np.linalg.norm(b - a) for a, b in pairs]))
    return moved, pairs


def draw(cap, knobs) -> np.ndarray:
    out = cap.color.copy()
    for kb in knobs:
        u, v = (int(round(c)) for c in kb.pixel)
        cv2.circle(out, (u, v), max(8, int(np.sqrt(kb.area_px))), (0, 0, 255), 2)
        cv2.putText(out, f"{kb.depth_mm:.0f} mm", (u + 12, v), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    return out
