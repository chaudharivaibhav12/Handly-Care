"""Where is the person? From their face to a spot the arm can hand things to.

The forehead point comes from thermometer.locate_forehead (face landmarks + depth). The
hand-over point sits in front of the person: HANDOVER_FORWARD_MM toward the robot from the
forehead and HANDOVER_BELOW_MM below it, about chest height, where a hand can take it.
"""
from __future__ import annotations

import numpy as np

import settings
from geometry import unit


def toward_robot(forehead_w, camera_w) -> np.ndarray:
    """Horizontal direction from the person toward the camera (they face it when it looks at them)."""
    d = np.asarray(camera_w, dtype=np.float64) - np.asarray(forehead_w, dtype=np.float64)
    d[2] = 0.0
    return unit(d)


def handover_point(forehead_w, camera_w, forward_mm: float | None = None, below_mm: float | None = None) -> np.ndarray:
    forward = settings.HANDOVER_FORWARD_MM if forward_mm is None else forward_mm
    below = settings.HANDOVER_BELOW_MM if below_mm is None else below_mm
    t = toward_robot(forehead_w, camera_w)
    return np.asarray(forehead_w, dtype=np.float64) + t * forward + np.array([0.0, 0.0, -below])


def obstacles(forehead_w, camera_w):
    """Head sphere + body box for the motion planner, facing the robot."""
    import thermometer
    return thermometer.person_obstacles(forehead_w, -toward_robot(forehead_w, camera_w))
