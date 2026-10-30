"""Whole flows on the simulator: the same code paths the real robot runs."""
import json
import os

import cv2
import numpy as np
import pytest

import calib_check
import check_setup
import locate
import markers
import robot_io
import settings
import sim
import snapshot
import thermometer
from conftest import run
from geometry import Intrinsics, angle_deg, make_T, ov_to_matrix, tip_position


def true_forehead(world, finder):
    """Where the ray through the detected forehead pixel really hits the head."""
    face = finder.detect(None)[0]
    T = world.T_world_cam_true
    d = np.array([(face.forehead[0] - sim.INTR.cx) / sim.INTR.fx, (face.forehead[1] - sim.INTR.cy) / sim.INTR.fy, 1.0])
    O, D = T[:3, 3], (T[:3, :3] @ d)[None]
    t = sim.Sphere(world.person["center"], world.person["radius"], (0, 0, 0)).hit(O, D)[0]
    return O + t * D[0]


# ---- thermometer -------------------------------------------------------------------

def test_thermometer_tip_ends_standoff_from_the_real_forehead():
    rig = sim.make_rig("person")
    finder = sim.SimFaceFinder(rig.world)
    truth = true_forehead(rig.world, finder)
    log = run(thermometer.run(rig, finder, execute=True, yes=True))
    assert log["result"] == "measured", log
    assert np.linalg.norm(np.array(log["forehead"]) - truth) < 4
    moves = rig.world.moves
    closest = min(moves, key=lambda m: np.linalg.norm(tip_position(m["to"][:3, 3], m["to"][:3, :3],
                                                                   settings.THERMO_TIP_IN_GRIPPER_MM) - truth))
    tip = tip_position(closest["to"][:3, 3], closest["to"][:3, :3], settings.THERMO_TIP_IN_GRIPPER_MM)
    assert np.linalg.norm(tip - truth) == pytest.approx(settings.STANDOFF_MM, abs=5)
    assert all(m["obstacles"] == 2 for m in moves[:-1])           # head + body given to the planner
    assert sum(m["linear"] for m in moves) == settings.APPROACH_STEPS + 1   # straight in, straight out
    # it backed away at the end
    assert np.linalg.norm(rig.world.T_world_gripper[:3, 3] - sim.person_world().T_world_gripper[:3, 3]) < 1


def test_thermometer_follows_the_person_when_they_sit_differently():
    for head in ((790.0, -120.0, 420.0), (760.0, 110.0, 520.0)):
        rig = sim.make_rig("person", head_center=head)
        finder = sim.SimFaceFinder(rig.world)
        truth = true_forehead(rig.world, finder)
        log = run(thermometer.run(rig, finder, execute=True, yes=True))
        assert log["result"] == "measured", log
        assert np.linalg.norm(np.array(log["tip_final"]) - truth) == pytest.approx(settings.STANDOFF_MM, abs=5)


def test_dry_run_moves_nothing():
    rig = sim.make_rig("person")
    log = run(thermometer.run(rig, sim.SimFaceFinder(rig.world), execute=False))
    assert log["result"] == "dry run" and rig.world.moves == []


def test_refuses_when_the_person_moves_between_looks():
    rig = sim.make_rig("person")
    finder = sim.SimFaceFinder(rig.world)
    real_detect = finder.detect
    calls = {"n": 0}

    def fidgety(img):
        calls["n"] += 1
        if calls["n"] == 2:
            rig.world.person["center"] = rig.world.person["center"] + np.array([0.0, 40.0, 0.0])
            for s in rig.world.shapes:
                if getattr(s, "name", "") == "head":
                    s.c = rig.world.person["center"]
        return real_detect(img)

    finder.detect = fidgety
    with pytest.raises(thermometer.NoFace, match="moved"):
        run(thermometer.run(rig, finder, execute=True, yes=True))
    assert rig.world.moves == []


def test_refuses_off_axis_approach_unless_aiming():
    settings.MAX_AIM_ANGLE_DEG = 10.0
    head = (790.0, 150.0, 450.0)          # in view, but well to the side
    rig = sim.make_rig("person", head_center=head)
    log = run(thermometer.run(rig, sim.SimFaceFinder(rig.world), execute=True, yes=True))
    assert log["result"] == "refused" and any("straight on" in p for p in log["problems"])
    assert rig.world.moves == []
    rig = sim.make_rig("person", head_center=head)
    finder = sim.SimFaceFinder(rig.world)
    truth = true_forehead(rig.world, finder)
    log = run(thermometer.run(rig, finder, execute=True, yes=True, aim=True))
    assert log["result"] == "measured", log
    assert np.linalg.norm(np.array(log["tip_final"]) - truth) == pytest.approx(settings.STANDOFF_MM, abs=5)


def test_refuses_targets_out_of_reach():
    rig = sim.make_rig("person", head_center=(1000.0, 30.0, 450.0))
    log = run(thermometer.run(rig, sim.SimFaceFinder(rig.world), execute=True, yes=True))
    assert log["result"] == "refused" and any("from the arm base" in p for p in log["problems"])
    assert rig.world.moves == []


def test_no_face_is_a_clean_stop():
    rig = sim.make_rig("person")
    rig.world.person = None
    with pytest.raises(thermometer.NoFace):
        run(thermometer.run(rig, sim.SimFaceFinder(rig.world), execute=True, yes=True))


# ---- bottle and tissue --------------------------------------------------------------

def test_bottle_is_grasped_at_its_measured_height():
    rig = sim.make_rig("table")
    found = run(locate.find(rig, "bottle"))
    cap = sim.TABLE_TRUTH["bottle_cap_center"]
    assert np.linalg.norm(found.grasp[:2] - cap[:2]) < 3
    assert found.grasp[2] == pytest.approx(cap[2] - settings.OBJECTS["bottle"]["grasp_below_top_mm"], abs=3)
    assert run(locate.pick(rig, found, yes=True))
    assert run(locate.verify_gone(rig, found))


def test_a_moved_bottle_is_still_found():
    for xy in sim.CALIB_SPOTS:
        rig = sim.make_rig("table", bottle_xy=xy)
        found = run(locate.find(rig, "bottle"))
        assert np.linalg.norm(found.grasp[:2] - np.array(xy)) < 3


def test_two_bottles_means_refuse_not_guess():
    rig = sim.make_rig("table")
    rig.world.shapes += [sim.Cylinder(560.0, 40.0, 22.0, 0.0, 95.0, (40, 120, 230), "bottle2"),
                         sim.Cylinder(560.0, 40.0, 15.0, 95.0, 115.0, (245, 245, 245), "bottle2")]
    with pytest.raises(locate.NotFound, match="look like a bottle"):
        run(locate.find(rig, "bottle"))


def test_tissue_is_pinched_above_the_box():
    rig = sim.make_rig("table")
    found = run(locate.find(rig, "tissue"))
    tip = sim.TABLE_TRUTH["tissue_tip_center"]
    assert np.linalg.norm(found.top_w - tip) < 4
    assert found.grasp[2] > 95 + 5                       # never down into the box
    assert run(locate.pick(rig, found, yes=True))


def test_no_tissue_sticking_out_is_refused():
    rig = sim.make_rig("table")
    for s in rig.world.shapes:
        if getattr(s, "name", "") == "tissue":
            s.held = True
    with pytest.raises(locate.NotFound, match="no tissue sticking out"):
        run(locate.find(rig, "tissue"))


def test_unknown_object_name():
    rig = sim.make_rig("table")
    with pytest.raises(locate.NotFound, match="not in settings.OBJECTS"):
        run(locate.find(rig, "laptop"))


def test_cli_dry_run_and_execute(capsys):
    assert locate.main(["bottle", "--sim"]) == 0
    assert "Dry run" in capsys.readouterr().out
    assert locate.main(["tissue", "--sim", "--execute", "--yes", "--verify"]) == 0
    assert locate.main(["--list", "--sim"]) == 0
    assert "MATCH" in capsys.readouterr().out


# ---- calibration ---------------------------------------------------------------------

def test_touch_test_finds_and_fixes_a_wrong_camera_mount():
    err = [12.0, -8.0, 6.0]
    for k in range(4):
        base = ["--sim", "--sim-mount-error", *map(str, err), "--sim-spot", str(k)]
        assert calib_check.main(["look", *base]) == 0
        assert calib_check.main(["touch", *base]) == 0
    assert calib_check.main(["report", "--sim", "--save"]) == 0
    with open("sim_camera_correction.json") as f:
        offset = np.array(json.load(f)["offset_cam_mm"])
    assert np.linalg.norm(offset - err) < 3
    for xy in sim.CALIB_SPOTS:   # now the camera is right again
        rig = sim.make_rig("table", cam_mount_error=err, bottle_xy=xy)
        found = run(locate.find(rig, "bottle"))
        assert np.linalg.norm(found.top_w - np.array([*xy, 115.0])) < 3


def test_analysis_flags_inconsistent_errors():
    T = np.eye(4).tolist()
    pts = [{"id": i, "computed_world": [0, 0, 0], "T_world_cam": T, "touch": t}
           for i, t in enumerate([[10, 0, 0], [-10, 0, 0], [0, 25, 0]], 1)]
    a = calib_check.analyze(pts)
    assert a["n"] == 3 and a["spread"] > 8


# ---- capture details -----------------------------------------------------------------

class UnalignedCamera(sim.FakeCamera):
    async def get_images(self, filter_source_names=None, **kw):
        imgs, md = await super().get_images(filter_source_names, **kw)
        for im in imgs:
            if im.name == "depth":
                from depth_tools import decode_viam_depth, encode_viam_depth
                d = decode_viam_depth(im.data)
                im.data = encode_viam_depth(cv2.resize(d, (320, 240), interpolation=cv2.INTER_NEAREST))
        return imgs, md


def test_unaligned_depth_is_reported():
    settings.DEPTH_FROM = "images"
    rig = sim.make_rig("table")
    rig.camera = UnalignedCamera(rig.world)
    cap = run(rig.capture(settle_s=0))
    assert cap.depth.shape == cap.color.shape[:2]
    assert any("NOT aligned" in n for n in cap.notes)


def test_point_cloud_gives_aligned_depth_even_when_the_depth_image_is_not():
    rig = sim.make_rig("table")
    rig.camera = UnalignedCamera(rig.world)
    cap = run(rig.capture(settle_s=0))
    assert any("point cloud" in n for n in cap.notes)
    found, _, _ = locate.locate_in_capture(cap, "bottle")
    assert np.linalg.norm(found.top_w - sim.TABLE_TRUTH["bottle_cap_center"]) < 3


def test_no_point_cloud_falls_back_to_the_depth_image():
    rig = sim.make_rig("table")
    rig.world.no_point_cloud = True
    cap = run(rig.capture(settle_s=0))
    assert any("point cloud unavailable" in n for n in cap.notes)
    found, _, _ = locate.locate_in_capture(cap, "bottle")
    assert np.linalg.norm(found.top_w - sim.TABLE_TRUTH["bottle_cap_center"]) < 3


def test_fingertip_offset_is_respected_for_picks_and_the_thermometer():
    """Like armfarm13: the gripper frame sits 48 mm behind the real fingertips."""
    rig = sim.make_rig("table", tip_offset=48.0)
    found = run(locate.find(rig, "bottle"))
    cap = sim.TABLE_TRUTH["bottle_cap_center"]
    fingers_z = found.grasp[2] - 48.0                     # pointing down: tips are 48 mm below the frame
    assert fingers_z == pytest.approx(cap[2] - settings.OBJECTS["bottle"]["grasp_below_top_mm"], abs=3)
    assert run(locate.pick(rig, found, yes=True))          # the pretend gripper feels it at the real tips
    rig = sim.make_rig("person", tip_offset=48.0)
    finder = sim.SimFaceFinder(rig.world)
    truth = true_forehead(rig.world, finder)
    log = run(thermometer.run(rig, finder, execute=True, yes=True))
    assert log["result"] == "measured"
    closest = min(rig.world.moves, key=lambda m: np.linalg.norm(m["to"][:3, 3] - truth))
    tip = closest["to"][:3, 3] + closest["to"][:3, :3] @ (np.array(settings.THERMO_TIP_IN_GRIPPER_MM) + [0, 0, 48.0])
    assert np.linalg.norm(tip - truth) == pytest.approx(settings.STANDOFF_MM, abs=5)


def test_capture_retries_if_the_camera_moves():
    rig = sim.make_rig("table")
    real = rig.camera.get_images
    calls = {"n": 0}

    async def shaky(*a, **kw):
        calls["n"] += 1
        if calls["n"] == 1:   # the arm is still settling during the first picture
            rig.world.T_world_gripper = rig.world.T_world_gripper @ make_T(np.eye(3), (5.0, 0.0, 0.0))
        return await real(*a, **kw)

    rig.camera.get_images = shaky
    run(rig.capture(settle_s=0))
    assert calls["n"] == 2


def test_snapshot_then_offline_gives_the_same_answer():
    assert snapshot.main(["table1", "--sim"]) == 0
    cap = robot_io.load_capture(os.path.join("captures", "table1"))
    found, _, _ = locate.locate_in_capture(cap, "bottle")
    assert np.linalg.norm(found.top_w - sim.TABLE_TRUTH["bottle_cap_center"]) < 3
    assert locate.main(["bottle", "--offline", os.path.join("captures", "table1")]) == 0


def test_check_setup_passes_on_the_simulator():
    assert check_setup.main(["--sim", "--joints"]) == 0
    assert os.path.exists(os.path.join("out", "setup_alignment.png"))


def test_unsafe_targets_never_reach_the_motion_service():
    rig = sim.make_rig("table")
    with pytest.raises(robot_io.UnsafeTarget):
        run(rig.move_gripper([1500.0, 0.0, 100.0], settings.DOWN_ORIENTATION))
    assert rig.world.moves == []


# ---- tags (drawer) ---------------------------------------------------------------------

def render_tag(center_cam, R_cam_tag, size_mm, intr, tag_id=3):
    tag = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(markers.TAG_DICT), tag_id, 200)
    s = size_mm / 2
    corners3d = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]]) @ R_cam_tag.T + center_cam
    px = np.stack([corners3d[:, 0] * intr.fx / corners3d[:, 2] + intr.cx,
                   corners3d[:, 1] * intr.fy / corners3d[:, 2] + intr.cy], 1).astype(np.float32)
    src = np.array([[0, 0], [199, 0], [199, 199], [0, 199]], np.float32)
    H = cv2.getPerspectiveTransform(src, px)
    img = cv2.warpPerspective(tag, H, (intr.width, intr.height), borderValue=255)
    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)


def test_tag_pose_is_recovered():
    intr = Intrinsics(615.0, 615.0, 320.0, 240.0, 640, 480)
    turn = ov_to_matrix(0.0, 0.0, 1.0, 0.0)
    a = np.radians(25)
    Ry = np.array([[np.cos(a), 0, np.sin(a)], [0, 1, 0], [-np.sin(a), 0, np.cos(a)]])
    # tag facing the camera (its z toward the camera), turned 25 deg
    R_cam_tag = Ry @ np.diag([1.0, -1.0, -1.0]) @ turn
    center = np.array([30.0, -20.0, 450.0])
    img = render_tag(center, R_cam_tag, 60.0, intr)
    tags = markers.find_tags(img, intr, size_mm=60.0)
    assert len(tags) == 1 and tags[0].id == 3
    assert np.linalg.norm(tags[0].center_cam - center) < 5
    assert angle_deg(tags[0].normal_cam, R_cam_tag[:, 2]) < 3


def test_forehead_direction_is_measured_from_depth():
    rig = sim.make_rig("person")
    finder = sim.SimFaceFinder(rig.world)
    cap = run(rig.capture(settle_s=0))
    fix = thermometer.locate_forehead(cap, finder)
    true_normal = (fix.point_w - rig.world.person["center"]) / rig.world.person["radius"]
    assert fix.normal_w is not None and angle_deg(fix.normal_w, true_normal) < 8


def test_drawer_routine_follows_the_tag(monkeypatch):
    """Teach relative to a tag, move the 'drawer' 80 mm and turn it 30 deg, replay: the waypoints follow."""
    pose = {"center": np.array([0.0, 0.0, 500.0]), "R": np.diag([1.0, -1.0, -1.0])}

    def fake_find_tags(color, intr, size_mm=50.0, depth=None):
        return [markers.Tag(7, np.zeros((4, 2)), pose["center"].copy(), pose["R"].copy())]

    monkeypatch.setattr(markers, "find_tags", fake_find_tags)
    args = ["--sim"]
    assert markers.main(["anchor", "drawer", *args]) == 0
    with open(os.path.join("routines", "drawer.json")) as f:
        taught = json.load(f)["anchor"]
    # two waypoints taught at the current sim pose: write them directly (the sim can't be jogged by hand)
    r = json.load(open(os.path.join("routines", "drawer.json")))
    wps = [[500.0, -50.0, 150.0], [500.0, -150.0, 150.0]]
    r["waypoints"] = [{"xyz": p, "ov": [0.0, 0.0, -1.0, 0.0], "linear": True, "grip": None} for p in wps]
    json.dump(r, open(os.path.join("routines", "drawer.json"), "w"))
    # move the tag: 80 mm along the camera's x, and turn it 30 deg about the camera's line of sight
    a = np.radians(30)
    turn = np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])
    pose["center"] = np.array([80.0, 0.0, 500.0])
    pose["R"] = turn @ np.diag([1.0, -1.0, -1.0])
    rig_moves = []
    real_make = robot_io.make_rig

    async def capturing_make_rig(a):
        rig = await real_make(a)
        rig_moves.append(rig.world.moves)
        return rig

    monkeypatch.setattr(robot_io, "make_rig", capturing_make_rig)
    assert markers.main(["replay", "drawer", "--execute", "--yes", *args]) == 0
    moves = rig_moves[-1]
    assert len(moves) == 2 and all(m["linear"] for m in moves)
    # the expected waypoints, computed independently
    rig = sim.make_rig("table")
    cap = run(rig.capture(settle_s=0))
    now, _ = markers.tag_anchor(markers.Tag(7, np.zeros((4, 2)), pose["center"], pose["R"]), cap)
    from geometry import reanchor
    expected = reanchor(wps, taught, now)
    for m, e in zip(moves, expected):
        assert np.allclose(m["to"][:3, 3], e, atol=1e-6)
    assert abs(math_angle(now[3] - taught[3])) > 5   # it really turned


def math_angle(deg):
    return (deg + 180) % 360 - 180
