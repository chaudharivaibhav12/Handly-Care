"""heightmap.py (DEC-003): objects from our own height map of the full point cloud.
Each case is the real-table problem that broke the Viam segmenter in EXP-002."""
import math
import random
import struct

import pytest

import anything
import calibration
import config
import heightmap
import perception
from conftest import run


def tilted(x, y):
    return 10 + 0.04 * x - 0.03 * y            # the real table read ~±15 mm off flat


def table_points(rng, x0=100, x1=500, y0=-100, y1=400, step=4):
    """Table surface with ±2 mm depth noise, gray."""
    return [(x, y, tilted(x, y) + rng.uniform(-2, 2), 0, 0, 600, (200, 200, 200))
            for x in range(x0, x1, step) for y in range(y0, y1, step)]


def box_points(x, y, w, d, h, rgb, step=3):
    """The TOP of a box seen from above (that's all the camera sees), resting on the tilted table."""
    return [(px, py, tilted(px, py) + h, 0, 0, 600 - h, rgb)
            for px in range(int(x - w / 2), int(x + w / 2), step) for py in range(int(y - d / 2), int(y + d / 2), step)]


def without(points, x, y, w, d):
    return [p for p in points if not (x - w / 2 <= p[0] < x + w / 2 and y - d / 2 <= p[1] < y + d / 2)]


def find(points):
    grid = heightmap.grid_from_points(points)
    return heightmap.blobs_from_grid(grid, heightmap.fit_table(grid))


def scene(rng):
    pts = table_points(rng)
    pts = without(pts, 200, 100, 60, 60) + box_points(200, 100, 60, 60, 50, (210, 40, 40))     # red cube
    pts = without(pts, 400, 250, 75, 75) + box_points(400, 250, 75, 75, 25, (40, 90, 200))    # flat blue box, 25 mm
    return pts


def test_finds_a_cube_and_a_flat_square_with_true_sizes():
    blobs = sorted(find(scene(random.Random(1))), key=lambda b: b["x"])
    assert len(blobs) == 2
    cube, square = blobs
    assert (cube["x"], cube["y"]) == pytest.approx((200, 100), abs=5)
    assert cube["w"] == pytest.approx(60, abs=10) and cube["h"] == pytest.approx(50, abs=4)
    assert square["w"] == pytest.approx(75, abs=10) and square["h"] == pytest.approx(25, abs=4)  # EXP-002 missed it
    assert anything.color_name(cube["rgb"]) == "red" and anything.color_name(square["rgb"]) == "blue"


def test_tilted_table_is_not_an_object():
    assert find(table_points(random.Random(2))) == []


def test_lone_spikes_and_specks_are_ignored():
    rng = random.Random(3)
    pts = table_points(rng) + [(300, 200, 200, 0, 0, 400, None)] * 5          # flying pixels, one cell
    pts += box_points(150, 300, 6, 6, 30, (0, 0, 0), step=2)                    # a 6 mm speck
    assert find(pts) == []


def test_two_objects_close_but_not_touching_stay_separate():
    rng = random.Random(4)
    pts = table_points(rng)
    for x in (200, 270):                                                         # 10 mm gap
        pts = without(pts, x, 100, 60, 60) + box_points(x, 100, 60, 60, 40, (40, 160, 70))
    assert len(find(pts)) == 2


def test_pcd_points_are_mapped_to_world_mm():
    pcd = b"VERSION .7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F U\nDATA binary\n" + \
        struct.pack("<fffI", 0.1, 0.2, 0.6, (255 << 16) | (10 << 8) | 5) + struct.pack("<fffI", 0, 0, 0, 0)
    O, cols = (100.0, 200.0, 650.0), [(0, 1, 0), (1, 0, 0), (0, 0, -1)]       # camera looking down
    pts = heightmap.points_from_cloud(pcd, O, cols, step=1)
    assert len(pts) == 1                                                         # z=0 point dropped
    wx, wy, wz, cx, cy, cz, col = pts[0]
    assert (wx, wy, wz) == pytest.approx((300, 300, 50)) and col == (255, 10, 5)


def test_find_uses_the_cloud_end_to_end(monkeypatch):
    """perception.find in anything mode with ANYTHING_SOURCE='cloud': cloud -> named objects."""
    monkeypatch.setattr(config, "MODE", "anything")
    monkeypatch.setattr(config, "ANYTHING_SOURCE", "cloud")
    monkeypatch.setattr(config, "OBJECTS", {})
    monkeypatch.setattr(anything, "_known", {})
    monkeypatch.setattr(config, "FIXTURES", {})
    pts = [p for p in scene(random.Random(5))]
    body = b"".join(struct.pack("<fffI", y / 1000, x / 1000, (650 - z) / 1000,
                                (int(c[0]) << 16) | (int(c[1]) << 8) | int(c[2])) for x, y, z, _, _, _, c in pts)
    pcd = b"FIELDS x y z rgb\nDATA binary\n" + body

    class Cam:
        async def get_point_cloud(self):
            return pcd, "pointcloud/pcd"

        async def get_images(self, **kw):
            return [], None

        async def get_properties(self):
            raise RuntimeError("no intrinsics in this test")

    class Robot:
        camera = Cam()

        async def go_overview(self):
            pass

    async def to_world(robot, ref, pose):            # camera looking straight down from z=650
        import types
        return types.SimpleNamespace(x=pose.y, y=pose.x, z=650 - pose.z)
    monkeypatch.setattr(perception, "to_world", to_world)
    monkeypatch.setattr(config, "CLOUD_STEP", 1)
    found = run(perception.find(Robot(), cal=dict(calibration.IDENTITY)))
    assert set(found) == {"red_object", "blue_object"}
    assert config.OBJECTS["blue_object"]["size"][2] == pytest.approx(25, abs=4)
    assert found["red_object"] == pytest.approx((200, 100), abs=5)


def test_sensor_noise_specks_up_to_14mm_are_not_objects():
    """Real height map Fri 2:25 PM: the whole table speckled 4-20 mm by depth noise -> false objects at 8 mm."""
    rng = random.Random(6)
    pts = table_points(rng)
    pts = without(pts, 300, 200, 30, 30) + box_points(300, 200, 30, 30, 14, (128, 128, 128))
    assert find(pts) == []


def cone_points(x, y, r_bottom, r_top, h, rgb, step=2):
    """A tapered object seen from above: flat top disc (r_top) + sloping side down to r_bottom."""
    pts = []
    for px in range(int(x - r_bottom), int(x + r_bottom) + 1, step):
        for py in range(int(y - r_bottom), int(y + r_bottom) + 1, step):
            r = math.hypot(px - x, py - y)
            if r > r_bottom:
                continue
            z = h if r <= r_top else h * (r_bottom - r) / (r_bottom - r_top)
            pts.append((px, py, tilted(px, py) + z, 0, 0, 600 - z, rgb))
    return pts


def test_upside_down_cup_is_gripped_near_its_narrow_top(monkeypatch):
    """Vaibhav's white cup upside down: 9 cm rim on the table, 6 cm on top, 10.5 cm tall (Fri 2:55 PM)."""
    import anything
    monkeypatch.setattr(config, "GRIPPER_MAX_OPEN_MM", 84.0)
    rng = random.Random(7)
    pts = without(table_points(rng), 300, 150, 92, 92) + cone_points(300, 150, 45, 30, 105, (235, 235, 235))
    (cup,) = find(pts)
    assert cup["w"] == pytest.approx(90, abs=10) and cup["h"] == pytest.approx(105, abs=5)
    assert cup["top_d"] <= 74                                     # the top fits the gripper
    ok, why, gh, where = anything.grip_plan((cup["w"], cup["d"], cup["h"]), cup["top_d"])
    assert ok and where == "top" and gh == pytest.approx(cup["h"] - config.TOP_GRIP_DEPTH_MM, abs=0.1)


def test_cup_right_way_up_is_still_refused_its_top_is_widest(monkeypatch):
    import anything
    monkeypatch.setattr(config, "GRIPPER_MAX_OPEN_MM", 84.0)
    ok, why, _, _ = anything.grip_plan((90, 90, 105), top_d=90)
    assert not ok and "9 cm wide" in why


def test_narrow_objects_are_still_gripped_in_the_middle(monkeypatch):
    import anything
    monkeypatch.setattr(config, "GRIPPER_MAX_OPEN_MM", 84.0)
    assert anything.grip_plan((30, 30, 30), top_d=30)[2:] == (15, "middle")


def test_tapered_object_that_barely_fits_is_still_gripped_at_its_narrow_top(monkeypatch):
    """Real cup read 93 x 73 x 101 (Fri 3:05 PM): 73 squeaks under the limit, but the middle is ~75 wide.
    Grip where it's narrowest."""
    import anything
    monkeypatch.setattr(config, "GRIPPER_MAX_OPEN_MM", 84.0)
    assert anything.grip_plan((93, 73, 101), top_d=58)[3] == "top"


def test_tall_thin_objects_are_gripped_higher_so_they_dont_tip(monkeypatch):
    """Fri 2:23 PM: the standing red cylinder (3 x 3 x 6 cm, gripped at mid-height) tipped over in the tray."""
    import anything
    monkeypatch.setattr(config, "GRIPPER_MAX_OPEN_MM", 84.0)
    ok, _, gh, where = anything.grip_plan((43, 33, 58), top_d=33)
    assert ok and where == "high" and gh == pytest.approx(37.7, abs=0.1)
    assert anything.grip_plan((50, 50, 50), top_d=50)[3] == "middle"          # a cube: unchanged


def test_real_red_cylinder_counts_as_tall_thin(monkeypatch):
    """EXP-004: the cylinder that tipped measured 52 x 37 x 55 (ratio 1.49): at TALL_RATIO 1.5 it was still
    gripped in the middle. It must get the high grip."""
    import anything
    monkeypatch.setattr(config, "GRIPPER_MAX_OPEN_MM", 84.0)
    ok, _, gh, where = anything.grip_plan((52, 37, 55), top_d=37)
    assert ok and where == "high" and gh == pytest.approx(35.8, abs=0.1)
