import math

import numpy as np
import pytest

import geometry as g

INTR = g.Intrinsics(615.0, 610.0, 321.0, 239.0, 640, 480)
rng = np.random.default_rng(7)


def random_rotation():
    q = rng.normal(size=4)
    q /= np.linalg.norm(q)
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


# ---- camera model ---------------------------------------------------------

def test_deproject_center_pixel_is_straight_ahead():
    assert np.allclose(g.deproject(INTR.cx, INTR.cy, 500, INTR), [0, 0, 500])


def test_deproject_then_project_round_trip():
    us, vs, ds = rng.uniform(0, 640, 50), rng.uniform(0, 480, 50), rng.uniform(200, 2000, 50)
    pts = g.deproject_pixels(us, vs, ds, INTR)
    assert np.allclose(g.project(pts, INTR), np.stack([us, vs], 1))
    assert np.allclose(pts[:, 2], ds)


def test_deproject_known_numbers():
    # 100 px right of center at 1 m with fx 615 -> 1000 * 100 / 615 mm to the right
    p = g.deproject(INTR.cx + 100, INTR.cy - 61, 1000, INTR)
    assert p[0] == pytest.approx(1000 * 100 / 615)
    assert p[1] == pytest.approx(-1000 * 61 / 610)


def test_intrinsics_scale_with_image_size():
    half = INTR.scaled_to(320, 240)
    assert (half.fx, half.cx, half.width) == pytest.approx((307.5, 160.5, 320))
    assert INTR.scaled_to(640, 480) is INTR


# ---- transforms -------------------------------------------------------------

def test_probe_recovers_a_transform():
    R, t = random_rotation(), np.array([100.0, -50.0, 300.0])
    T = g.make_T(R, t)
    step = 100.0
    pts = [g.apply_T(T, p) for p in ([0, 0, 0], [step, 0, 0], [0, step, 0], [0, 0, step])]
    assert np.allclose(g.T_from_probe(*pts, step), T)


def test_probe_rejects_a_broken_frame():
    with pytest.raises(ValueError):
        g.T_from_probe([0, 0, 0], [100, 0, 0], [100, 0, 0], [0, 0, 100], 100.0)


def test_invert_T():
    T = g.make_T(random_rotation(), [1, 2, 3])
    assert np.allclose(g.invert_T(T) @ T, np.eye(4))


# ---- orientation vectors ------------------------------------------------------

def test_pointing_down_has_z_axis_down():
    R = g.ov_to_matrix(0, 0, -1, 0)
    assert np.allclose(R[:, 2], [0, 0, -1])


def test_ov_z_axis_is_the_vector():
    for _ in range(50):
        v = rng.normal(size=3)
        R = g.ov_to_matrix(*v, rng.uniform(-180, 180))
        assert np.allclose(R[:, 2], v / np.linalg.norm(v))
        assert np.allclose(R.T @ R, np.eye(3))


def test_matrix_to_ov_round_trip_including_straight_up_and_down():
    cases = [random_rotation() for _ in range(100)]
    cases += [g.ov_to_matrix(0, 0, s, th) for s in (1, -1) for th in (-170, -90, 0, 33, 180)]
    for R in cases:
        assert np.allclose(g.ov_to_matrix(*g.matrix_to_ov(R)), R, atol=1e-9)


def test_matches_viams_own_spatialmath():
    sm = pytest.importorskip("viam.spatialmath")
    try:
        sm.OrientationVector(0, 0, 1, 0)
    except Exception as e:   # the native library isn't available on every platform
        pytest.skip(f"viam spatialmath native lib unavailable: {e}")
    for _ in range(100):
        v, th = rng.normal(size=3), rng.uniform(-180, 180)
        q = sm.OrientationVector(*v, math.radians(th)).to_quaternion()
        R = g.ov_to_matrix(*v, th)
        for i, e in enumerate(np.eye(3)):
            r = q.rotate_vector(sm.Vector3(*e))
            assert np.allclose([r.x, r.y, r.z], R[:, i], atol=1e-9)
    # and the other way: our matrix_to_ov must mean the same rotation to Viam
    for _ in range(50):
        R = random_rotation()
        ox, oy, oz, th = g.matrix_to_ov(R)
        q = sm.OrientationVector(ox, oy, oz, math.radians(th)).to_quaternion()
        r = q.rotate_vector(sm.Vector3(1, 0, 0))
        assert np.allclose([r.x, r.y, r.z], R[:, 0], atol=1e-6)


def test_rotation_between():
    for _ in range(50):
        a, b = rng.normal(size=3), rng.normal(size=3)
        R = g.rotation_between(a, b)
        assert np.allclose(R @ g.unit(a), g.unit(b))
        assert np.allclose(R.T @ R, np.eye(3))
    assert np.allclose(g.rotation_between([1, 0, 0], [1, 0, 0]), np.eye(3))
    R = g.rotation_between([0, 0, 1], [0, 0, -1])
    assert np.allclose(R @ [0, 0, 1], [0, 0, -1])


# ---- thermometer approach ------------------------------------------------------

def test_tip_stops_standoff_in_front_of_forehead_along_its_axis():
    R = g.ov_to_matrix(1, 0, 0, -90)          # gripper pointing along +x
    F = np.array([700.0, 30.0, 480.0])
    tip_g = (0.0, 0.0, 60.0)
    plan = g.plan_forehead_approach(F, R, (0, 0, 1), tip_g, standoff_mm=45, pre_extra_mm=150, n_steps=3)
    assert np.allclose(plan.axis, [1, 0, 0])
    assert np.allclose(plan.tip_final, [655, 30, 480])
    assert np.allclose(g.tip_position(plan.gripper_final, R, tip_g), plan.tip_final)
    assert np.linalg.norm(plan.tip_pre - F) == pytest.approx(195)
    assert np.allclose(plan.steps[-1], plan.gripper_final) and len(plan.steps) == 3
    assert np.allclose(plan.R_gripper, R)      # orientation kept


def test_sideways_thermometer_offset_is_respected():
    R = g.ov_to_matrix(0, 0, -1, 0)           # gripper pointing down, thermometer sticks out sideways
    F = np.array([600.0, 0.0, 400.0])
    axis_g, tip_g = (1, 0, 0), (80.0, 0.0, 30.0)
    plan = g.plan_forehead_approach(F, R, axis_g, tip_g, 40, 100, 2)
    tip = g.tip_position(plan.gripper_final, R, tip_g)
    assert np.allclose(tip, plan.tip_final)
    assert np.allclose(F - tip, 40 * plan.axis)
    assert np.allclose(plan.axis, R @ np.array(axis_g, float))


def test_aim_turns_the_thermometer_to_the_given_direction():
    R = g.ov_to_matrix(1, 0, 0, -90)
    d = g.unit([1.0, 0.3, -0.1])
    plan = g.plan_forehead_approach([700, 100, 450], R, (0, 0, 1), (0, 0, 60), 45, 150, 3, aim_direction=d)
    assert np.allclose(plan.axis, d)
    assert g.angle_deg(plan.R_gripper[:, 2], d) < 1e-6


def test_reach_problems():
    assert g.reach_problems([400, 0, 200], (0, 0), (150, 700), (-50, 900)) == []
    assert len(g.reach_problems([900, 0, 200], (0, 0), (150, 700), (-50, 900))) == 1
    assert len(g.reach_problems([50, 0, -100], (0, 0), (150, 700), (-50, 900))) == 2


def test_in_box():
    box = {"x": (0, 10), "y": (0, 10), "z": (0, 10)}
    assert g.in_box([5, 5, 5], box) and not g.in_box([5, 11, 5], box) and g.in_box([99, 99, 99], None)


# ---- re-anchoring a taught routine ----------------------------------------------

def test_reanchor_moves_and_turns_waypoints_with_the_object():
    taught = (500.0, 0.0, 100.0, 90.0)
    pts = np.array([[500.0, -100.0, 100.0], [500.0, -200.0, 150.0]])   # straight out in front of the drawer
    same = g.reanchor(pts, taught, taught)
    assert np.allclose(same, pts)
    moved = g.reanchor(pts, taught, (450.0, 50.0, 110.0, 0.0))        # drawer shifted and turned -90 deg
    assert np.allclose(moved[0], [350.0, 50.0, 110.0])
    assert np.allclose(moved[1], [250.0, 50.0, 160.0])


def test_reanchor_orientation_turns_with_the_object():
    ov = g.reanchor_orientation((1, 0, 0, -90), 0.0, 90.0)
    assert np.allclose(g.ov_to_matrix(*ov)[:, 2], [0, 1, 0], atol=1e-9)
