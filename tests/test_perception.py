"""Perception in world coordinates, with fake Viam segmenters built from real Viam types."""
import pytest
from viam.proto.common import (GeometriesInFrame, Geometry, PointCloudObject, Pose,
                               RectangularPrism, Vector3)

import calibration
import config
import perception
from conftest import run, sim

OFFSET = (400.0, 0.0)   # fake camera->world: world = camera + OFFSET


def pco(label, x, y, dims=(50, 50, 50), frame="cam"):
    """One segmented object, like detections-to-segments returns, in camera mm."""
    g = Geometry(center=Pose(x=x - OFFSET[0], y=y - OFFSET[1], z=300),
                 box=RectangularPrism(dims_mm=Vector3(x=dims[0], y=dims[1], z=dims[2])), label=label)
    return PointCloudObject(point_cloud=b"", geometries=GeometriesInFrame(reference_frame=frame, geometries=[g]))


class FakeSegmenter:
    def __init__(self, looks=None, error=None):
        self.looks, self.error, self.calls = looks or [[]], error, 0

    async def get_object_point_clouds(self, camera):
        assert camera == config.CAMERA
        self.calls += 1
        if self.error:
            raise self.error
        return self.looks[min(self.calls - 1, len(self.looks) - 1)]


class FakeRealRobot:
    def __init__(self):
        self.machine = object()
        self.calls = []

    async def go_overview(self):
        self.calls.append("overview")


@pytest.fixture
def world(monkeypatch):
    async def to_world(robot, ref, pose):
        robot.calls.append(f"tf:{ref}")
        return Pose(x=pose.x + OFFSET[0], y=pose.y + OFFSET[1], z=pose.z)
    monkeypatch.setattr(perception, "to_world", to_world)


def install(monkeypatch, segmenters):
    monkeypatch.setattr(config, "SEGMENTERS", {name: fixed for name, (fixed, _) in segmenters.items()})
    monkeypatch.setattr(perception, "segmenter_for", lambda r, name: segmenters[name][1])


# ---------------- labels ----------------

def test_labels_map_to_objects_case_insensitive():
    assert perception.object_for_label("Cup") == "cup"
    assert perception.object_for_label("red block") == "red_cube"
    assert perception.object_for_label("blue_cube") == "blue_cube"
    assert perception.object_for_label("laptop") is None
    assert perception.object_for_label(None) is None


# ---------------- one look ----------------

def test_one_look_maps_labels_to_world_positions(monkeypatch, world):
    seg = FakeSegmenter([[pco("cup", 420, 120), pco("red block", 480, -100), pco("laptop", 500, 0)]])
    install(monkeypatch, {"objects-3d": (None, seg)})
    r = FakeRealRobot()
    found = run(perception.detections_once(r, dict(calibration.IDENTITY)))
    assert sorted(found) == sorted([("cup", 420, 120), ("red_cube", 480, -100)])
    assert "tf:cam" in r.calls


def test_fixed_segmenter_names_everything_it_finds(monkeypatch, world):
    seg = FakeSegmenter([[pco("", 450, 50)]])     # color detectors give no useful label
    install(monkeypatch, {"seg-blue": ("blue_cube", seg)})
    found = run(perception.detections_once(FakeRealRobot(), dict(calibration.IDENTITY)))
    assert found == [("blue_cube", 450, 50)]


def test_junk_segments_are_dropped(monkeypatch, world):
    table_blob = pco("cup", 450, 0, dims=(600, 400, 20))     # the table itself
    off_table = pco("cup", 1500, 900)                       # e.g. carpet on the floor
    seg = FakeSegmenter([[table_blob, off_table]])
    install(monkeypatch, {"objects-3d": (None, seg)})
    assert run(perception.detections_once(FakeRealRobot(), dict(calibration.IDENTITY))) == []


def test_calibration_is_applied(monkeypatch, world):
    seg = FakeSegmenter([[pco("cup", 420, 120)]])
    install(monkeypatch, {"objects-3d": (None, seg)})
    cal = calibration.fit([((420, 120), (440, 100))])       # camera is 20 mm off in x and y
    found = run(perception.detections_once(FakeRealRobot(), cal))
    assert found == [("cup", pytest.approx(440), pytest.approx(100))]


def test_a_broken_segmenter_doesnt_stop_the_others(monkeypatch, world):
    good = FakeSegmenter([[pco("", 450, 50)]])
    install(monkeypatch, {"seg-red": ("red_cube", FakeSegmenter(error=RuntimeError("no such service"))),
                          "seg-blue": ("blue_cube", good)})
    assert run(perception.detections_once(FakeRealRobot(), dict(calibration.IDENTITY))) == [("blue_cube", 450, 50)]


def test_world_frame_objects_are_not_transformed(monkeypatch):
    calls = []

    async def to_world(robot, ref, pose):
        calls.append(ref)
        return pose
    monkeypatch.setattr(perception, "to_world", to_world)
    seg = FakeSegmenter([[pco("cup", 420, 120, frame="world")]])
    install(monkeypatch, {"objects-3d": (None, seg)})
    run(perception.detections_once(FakeRealRobot(), dict(calibration.IDENTITY)))
    assert calls == ["world"]


# ---------------- voting across looks ----------------

def test_vote_keeps_steady_objects():
    looks = [[("cup", 420, 120)], [("cup", 423, 118)], [("cup", 419, 121)]]
    pos = perception.vote(looks)["cup"]
    assert pos == pytest.approx((420.67, 119.67), abs=0.1)


def test_vote_drops_flicker():
    looks = [[("cup", 420, 120)], [], []]
    assert perception.vote(looks, min_hits=2)["cup"] is None


def test_vote_phantom_loses_to_the_real_object():
    looks = [[("cup", 420, 120), ("cup", 600, -300)], [("cup", 421, 119)], [("cup", 419, 121)]]
    assert perception.vote(looks)["cup"] == pytest.approx((420, 120), abs=1)


def test_vote_two_equally_real_cups_is_refused():
    looks = [[("cup", 420, 120), ("cup", 300, -200)]] * 3
    assert perception.vote(looks)["cup"] is None


def test_vote_counts_looks_not_detections():
    # the same frame reporting a duplicate must not count twice
    looks = [[("cup", 420, 120), ("cup", 421, 121)], [], []]
    assert perception.vote(looks, min_hits=2)["cup"] is None


def test_vote_reports_every_catalog_object():
    assert set(perception.vote([[]])) == set(config.OBJECTS)


# ---------------- find() ----------------

def test_find_goes_to_overview_first_and_votes(monkeypatch, world):
    monkeypatch.setattr(config, "OBSERVE_FRAMES", 3)
    seg = FakeSegmenter([[pco("cup", 420, 120)], [pco("cup", 421, 119), pco("cup", 600, -300)], [pco("cup", 419, 120)]])
    install(monkeypatch, {"objects-3d": (None, seg)})
    r = FakeRealRobot()
    s = run(perception.find(r))
    assert r.calls[0] == "overview" and seg.calls == 3
    assert s["cup"] == pytest.approx((420, 119.67), abs=0.5) and s["marker"] is None


def test_find_uses_saved_calibration(monkeypatch, world):
    calibration.save(calibration.fit([((420, 120), (430, 120))]))
    seg = FakeSegmenter([[pco("cup", 420, 120)]])
    install(monkeypatch, {"objects-3d": (None, seg)})
    assert run(perception.find(FakeRealRobot()))["cup"] == pytest.approx((430, 120))


def test_find_in_slots_mode_returns_taught_positions(monkeypatch):
    monkeypatch.setattr(config, "MODE", "slots")

    async def observe(robot, already_there=False):
        assert already_there
        return {"cup": "tray", "red_cube": None, "blue_cube": "slot2", "marker": None}
    monkeypatch.setattr(perception.vision, "observe", observe)
    s = run(perception.find(FakeRealRobot()))
    assert s["cup"] == config.ZONES["tray"] and s["blue_cube"] == config.SLOTS["slot2"] and s["red_cube"] is None


def test_find_on_sim_reads_the_truth():
    r = sim()
    assert run(perception.find(r)) == r.seen() and r.overview_visits == 1


def test_zone_of():
    assert perception.zone_of(config.ZONES["tray"]) == "tray"
    assert perception.zone_of((450, 0)) is None
    assert perception.zone_of(None) is None
