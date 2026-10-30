import cv2
import numpy as np
import pytest

import depth_tools as dt
import sim
from geometry import apply_T, make_T, ov_to_matrix

rng = np.random.default_rng(3)


# ---- decoding -------------------------------------------------------------------

def test_viam_depth_round_trip():
    d = rng.integers(200, 3000, (48, 64)).astype(np.uint16)
    data = dt.encode_viam_depth(d)
    assert data[:8] == b"DEPTHMAP" and len(data) == 24 + 2 * d.size
    assert np.array_equal(dt.decode_viam_depth(data), d)
    assert np.array_equal(dt.decode_depth(data, "image/vnd.viam.dep"), d)


def test_little_endian_depth_is_detected():
    d = rng.integers(300, 1500, (20, 30)).astype(np.uint16)
    data = b"DEPTHMAP" + (30).to_bytes(8, "big") + (20).to_bytes(8, "big") + d.astype("<u2").tobytes()
    assert np.array_equal(dt.decode_viam_depth(data), d)


def test_bad_depth_is_refused():
    with pytest.raises(ValueError):
        dt.decode_viam_depth(b"NOTDEPTH" + bytes(40))
    with pytest.raises(ValueError):
        dt.decode_viam_depth(b"DEPTHMAP" + (100).to_bytes(8, "big") + (100).to_bytes(8, "big") + bytes(10))


def test_png_depth_and_color_formats():
    d = rng.integers(200, 3000, (10, 12)).astype(np.uint16)
    ok, png = cv2.imencode(".png", d)
    assert np.array_equal(dt.decode_depth(png.tobytes(), "image/png"), d)
    img = rng.integers(0, 255, (10, 12, 3)).astype(np.uint8)
    ok, png = cv2.imencode(".png", img)
    assert np.array_equal(dt.decode_color(png.tobytes(), "image/png"), img)
    rgba = np.dstack([img[..., ::-1], np.full((10, 12), 255, np.uint8)])
    raw = b"RGBA" + (12).to_bytes(4, "big") + (10).to_bytes(4, "big") + rgba.tobytes()
    assert np.array_equal(dt.decode_color(raw, "image/vnd.viam.rgba"), img)


# ---- robust depth -----------------------------------------------------------------

def test_robust_depth_ignores_holes_and_the_wall_behind():
    d = np.full((50, 50), 1500, np.uint16)    # wall
    d[20:30, 20:30] = 600                      # forehead
    d[24, 24] = 0                              # a hole
    d[20:30, 28:30] = 1500                     # the patch half-sees the wall
    assert dt.robust_depth(d, 25, 25, 5) == pytest.approx(600)


def test_robust_depth_gives_up_without_data():
    d = np.zeros((50, 50), np.uint16)
    d[0, 0] = 700
    assert dt.robust_depth(d, 25, 25, 4) is None
    assert dt.robust_depth(d, -100, -100, 4) is None


# ---- table plane ------------------------------------------------------------------

def test_fit_plane_finds_a_tilted_table_among_junk():
    n_true = np.array([0.1, -0.2, -1.0])
    n_true /= np.linalg.norm(n_true)
    p0 = np.array([0.0, 0.0, 600.0])
    a, b = np.cross(n_true, [1, 0, 0]), None
    a /= np.linalg.norm(a)
    b = np.cross(n_true, a)
    uv = rng.uniform(-300, 300, (3000, 2))
    table = p0 + uv[:, :1] * a + uv[:, 1:] * b + rng.normal(0, 1, (3000, 1)) * n_true
    junk = rng.uniform([-300, -300, 300], [300, 300, 590], (800, 3))
    plane = dt.fit_plane(np.vstack([table, junk]), tol_mm=5)
    assert abs(abs(plane.normal @ n_true) - 1) < 1e-3
    assert plane.normal @ (-plane.point) > 0       # faces the camera
    assert abs(plane.height(p0[None])[0]) < 2


# ---- objects on the table ------------------------------------------------------------

def find(world):
    img, depth = world.render()
    blobs, plane, hm = dt.find_objects(depth, sim.INTR)
    return blobs, world.T_world_cam_config


def test_bottle_cap_top_is_found_within_3mm():
    blobs, T = find(sim.table_world(seed=1))
    bottle = min(blobs, key=lambda b: b.area_px)
    top = apply_T(T, bottle.top_point_cam)
    assert np.linalg.norm(top - sim.TABLE_TRUTH["bottle_cap_center"]) < 3
    assert 35 < bottle.length_mm < 50 and 30 < bottle.width_mm < 50


def test_tissue_sticking_out_is_found_as_a_peak():
    blobs, T = find(sim.table_world(seed=2))
    box = max(blobs, key=lambda b: b.area_px)
    assert box.plateau_height_mm == pytest.approx(95, abs=3)
    assert box.peak_height_mm - box.plateau_height_mm > 30
    assert np.linalg.norm(apply_T(T, box.peak_point_cam) - sim.TABLE_TRUTH["tissue_tip_center"]) < 4
    assert 215 < box.length_mm < 235 and 110 < box.width_mm < 130


def test_works_when_the_camera_is_tilted():
    w = sim.table_world(seed=4)
    tilt = ov_to_matrix(0.25, 0.1, -1.0, 15)     # gripper not straight down
    w.T_world_gripper = make_T(tilt, (380.0, 60.0, 450.0))
    blobs, T = find(w)
    bottle = min(blobs, key=lambda b: b.area_px)
    assert np.linalg.norm(apply_T(T, bottle.top_point_cam) - sim.TABLE_TRUTH["bottle_cap_center"]) < 4


def test_objects_cut_off_by_the_edge_are_flagged():
    w = sim.table_world(seed=5, bottle_xy=(470.0, 175.0))   # half out of the picture
    img, depth = w.render()
    blobs, _, _ = dt.find_objects(depth, sim.INTR, roi=(0.05, 0.05, 0.95, 0.95))
    bottle = min(blobs, key=lambda b: b.area_px)
    assert bottle.touches_edge
    assert not max(blobs, key=lambda b: b.area_px).touches_edge


def test_flat_table_has_no_objects():
    w = sim.table_world(seed=6)
    for s in w.shapes:
        if getattr(s, "name", ""):
            s.held = True
    blobs, _ = find(w)
    assert blobs == []


def test_surface_normal_of_a_tilted_wall():
    intr = sim.INTR
    us, vs = np.meshgrid(np.arange(640), np.arange(480))
    n_true = np.array([0.3, -0.2, -1.0])
    n_true /= np.linalg.norm(n_true)
    # plane through (0, 0, 600) with normal n_true: depth z along each pixel ray
    d = np.stack([(us - intr.cx) / intr.fx, (vs - intr.cy) / intr.fy, np.ones_like(us, float)], -1)
    t = (n_true @ np.array([0, 0, 600.0])) / (d @ n_true)
    depth = np.round(t).astype(np.uint16)
    n = dt.surface_normal(depth, intr, 320, 240, 12)
    assert n is not None and abs(n @ n_true) > 0.995 and n @ np.array([0, 0, -1.0]) > 0


def test_surface_normal_refuses_a_ragged_patch():
    depth = rng.integers(400, 900, (100, 100)).astype(np.uint16)
    assert dt.surface_normal(depth, sim.INTR, 50, 50, 10) is None
