"""Camera correction: recovers known errors, refuses wild fits, survives bad files."""
import json
import math

import pytest

import calibration
import config

TRUTH = [(450, -150), (550, -50), (450, 100), (350, 0)]


def camera_sees(points, dx=0.0, dy=0.0, angle_deg=0.0, scale=1.0, center=(450, 0)):
    """Simulate a mis-calibrated camera: rotate/scale around `center`, then shift."""
    a = math.radians(angle_deg)
    out = []
    for x, y in points:
        px, py = x - center[0], y - center[1]
        rx = scale * (px * math.cos(a) - py * math.sin(a)) + center[0] + dx
        ry = scale * (px * math.sin(a) + py * math.cos(a)) + center[1] + dy
        out.append((rx, ry))
    return out


def pairs_for(seen, truth=TRUTH):
    return list(zip(seen, truth))


def corrected_error(cal, seen, truth=TRUTH):
    return max(math.dist(calibration.apply(cal, *s), t) for s, t in zip(seen, truth))


def test_pure_shift_like_thirdarm_is_removed():
    seen = camera_sees(TRUTH, dx=-13.6, dy=-16.6)          # ThirdArm's measured ~21 mm miss
    cal = calibration.fit(pairs_for(seen))
    assert corrected_error(cal, seen) < 0.5
    assert cal["before_mm"] > 20 and cal["after_mm"] < 0.5


def test_big_shift_like_the_winners_teammate_is_removed():
    seen = camera_sees(TRUTH, dx=-76.2, dy=-88.9)
    cal = calibration.fit(pairs_for(seen))
    assert corrected_error(cal, seen) < 0.5


def test_rotation_and_scale_need_the_affine_fit():
    seen = camera_sees(TRUTH, dx=10, dy=-5, angle_deg=4, scale=1.05)
    cal = calibration.fit(pairs_for(seen))
    assert cal["kind"] == "affine"
    assert corrected_error(cal, seen) < 0.5
    # and it generalizes to a point it wasn't fitted on
    new_truth = [(500, 200)]
    new_seen = camera_sees(new_truth, dx=10, dy=-5, angle_deg=4, scale=1.05)
    assert corrected_error(cal, new_seen, new_truth) < 0.5


def test_single_point_gives_a_shift():
    cal = calibration.fit([((440, -160), (450, -150))])
    assert cal["kind"] == "shift" and calibration.apply(cal, 440, -160) == pytest.approx((450, -150))


def test_points_in_a_line_fall_back_to_shift():
    line = [(400, 0), (450, 0), (500, 0)]
    seen = camera_sees(line, dx=12)
    cal = calibration.fit(pairs_for(seen, line))
    assert cal["kind"] == "shift"
    assert corrected_error(cal, seen, line) < 0.5


def test_a_mismatched_point_doesnt_produce_a_wild_correction():
    # someone typed red_cube=slot1 but it was on slot3: the affine fit would be wild
    seen = camera_sees(TRUTH, dx=5)
    truth = [TRUTH[2], TRUTH[1], TRUTH[0], TRUTH[3]]
    cal = calibration.fit(pairs_for(seen, truth))
    assert cal["kind"] == "shift"
    assert cal["after_mm"] > 10          # and the report shows something is off


def test_empty_is_an_error():
    with pytest.raises(ValueError):
        calibration.fit([])


def test_save_and_load_round_trip():
    cal = calibration.fit(pairs_for(camera_sees(TRUTH, dx=7, dy=-3)))
    calibration.save(cal)
    again = calibration.load()
    assert again["m"] == cal["m"]


def test_missing_file_means_no_correction():
    cal = calibration.load()
    assert calibration.apply(cal, 123, 456) == (123, 456)


@pytest.mark.parametrize("content", ["not json", "{}", json.dumps({"m": [[1, 2]]}), json.dumps({"m": "x"})])
def test_corrupt_file_means_no_correction(content):
    with open(config.CALIBRATION_PATH, "w") as f:
        f.write(content)
    cal = calibration.load()
    assert calibration.apply(cal, 10, 20) == (10, 20)


def test_nearly_collinear_noisy_points_still_give_a_shift():
    truth = [(400, 0), (450, 0), (500, 0), (550, 0)]
    seen = [(410, 3), (460, -2), (510, 2.5), (560, -3)]
    cal = calibration.fit(pairs_for(seen, truth))
    assert cal["kind"] == "shift"
    # a point off the line is shifted, not wildly stretched
    assert math.dist(calibration.apply(cal, 480, 100), (470, 100)) < 1
