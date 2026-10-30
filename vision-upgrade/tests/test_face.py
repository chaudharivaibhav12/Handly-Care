import math

import numpy as np
import pytest

from face import FaceFinder, forehead_from_landmarks

UPRIGHT = np.array([[300, 200], [340, 200], [320, 225], [305, 260], [335, 260]], float)


def test_forehead_is_above_the_eyes_by_half_the_eye_mouth_distance():
    (u, v), eye_mouth = forehead_from_landmarks(UPRIGHT, 0.55)
    assert eye_mouth == pytest.approx(60)
    assert (u, v) == pytest.approx((320, 200 - 0.55 * 60))


def test_forehead_follows_a_tilted_head():
    a = math.radians(30)
    R = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
    center = np.array([320.0, 230.0])
    tilted = (UPRIGHT - center) @ R.T + center
    (u, v), _ = forehead_from_landmarks(tilted, 0.55)
    expected = (np.array([320, 200 - 33.0]) - center) @ R.T + center
    assert (u, v) == pytest.approx(tuple(expected))


def test_model_loads_and_ignores_a_blank_picture():
    finder = FaceFinder()
    assert finder.detect(np.full((480, 640, 3), 128, np.uint8)) == []
