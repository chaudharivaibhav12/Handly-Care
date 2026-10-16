"""Vision: pixel -> spot, picking the right detection, and observe() with fake Viam detectors."""
import types

import config
import vision
from conftest import run


def det(x0, y0, x1, y1, conf=0.9):
    return types.SimpleNamespace(x_min=x0, y_min=y0, x_max=x1, y_max=y1, confidence=conf)


def center_of(spot):
    x0, y0, x1, y1 = config.REGIONS[spot]
    return (x0 + x1) / 2, (y0 + y1) / 2


def box_in(spot, half=10, conf=0.9):
    cx, cy = center_of(spot)
    return det(cx - half, cy - half, cx + half, cy + half, conf)


# ---------------- pixel regions ----------------

def test_every_region_center_maps_to_its_own_spot():
    for spot in config.REGIONS:
        assert vision.spot_for_pixel(*center_of(spot)) == spot


def test_region_edges_left_top_inclusive_right_bottom_exclusive():
    x0, y0, x1, y1 = config.REGIONS["slot1"]
    assert vision.spot_for_pixel(x0, y0) == "slot1"
    assert vision.spot_for_pixel(x1, y0) != "slot1"


def test_pixel_outside_all_regions_is_none():
    assert vision.spot_for_pixel(-5, -5) is None
    assert vision.spot_for_pixel(10_000, 10_000) is None


# ---------------- choosing a detection ----------------

def test_low_confidence_detections_are_ignored():
    assert vision.best_detection([det(0, 0, 50, 50, conf=0.1)]) is None


def test_huge_background_blob_is_ignored():
    # ThirdArm saw a 409x719 px "blue" region in an image corner on this hardware.
    big_bg = det(0, 0, 409, 719)
    assert not vision.plausible(big_bg)
    assert vision.spot_in_frame([big_bg, box_in("slot2")]) == "slot2"


def test_tiny_speck_is_ignored():
    assert not vision.plausible(det(5, 5, 8, 8))


def test_object_sized_detection_is_kept():
    assert vision.plausible(box_in("slot1"))


def test_no_detections_is_none():
    assert vision.best_detection([]) is None
    assert vision.spot_in_frame([]) is None


def test_detection_outside_every_region_is_ignored():
    # e.g. something on the floor or the carpet
    assert vision.spot_in_frame([det(2000, 2000, 2040, 2040)]) is None


def test_two_spots_in_one_frame_is_ambiguous():
    assert vision.spot_in_frame([box_in("slot1"), box_in("tray")]) is None


# ---------------- voting across frames ----------------

def test_vote_majority_wins():
    assert vision.vote(["tray", "tray", None]) == "tray"


def test_vote_single_flicker_is_not_enough():
    assert vision.vote([None, "left", None], min_hits=2) is None


def test_vote_tie_is_refused():
    assert vision.vote(["tray", "left"], min_hits=1) is None


def test_vote_phantom_loses_to_real_spot():
    assert vision.vote(["slot1", "slot1", "right"], min_hits=2) == "slot1"


# ---------------- observe() on a "real" robot with fake detectors ----------------

class FakeDetector:
    def __init__(self, dets=None, error=None):
        self.dets, self.error, self.cameras = dets or [], error, []

    async def get_detections_from_camera(self, camera):
        self.cameras.append(camera)
        if self.error:
            raise self.error
        return self.dets


class FakeRealRobot:
    """Looks like RealRobot to vision.observe (no `.table`)."""
    def __init__(self):
        self.machine = object()
        self.calls = []

    async def go_overview(self):
        self.calls.append("overview")


def install(monkeypatch, detectors, robot):
    def detector_for(r, name):
        robot.calls.append(f"detect:{name}")
        return detectors[name]
    monkeypatch.setattr(vision, "detector_for", detector_for)


def test_observe_maps_each_object_to_its_spot(monkeypatch):
    r = FakeRealRobot()
    dets = {
        "det-red": FakeDetector([box_in("slot1")]),
        "det-blue": FakeDetector([box_in("tray")]),
        "det-white": FakeDetector([]),                       # not visible
        "det-yellow": FakeDetector([box_in("to_me")]),
    }
    install(monkeypatch, dets, r)
    scene = run(vision.observe(r))
    assert scene == {"red_cube": "slot1", "blue_cube": "tray", "cup": None, "marker": "to_me"}
    assert r.calls[0] == "overview"                          # moves to overview BEFORE looking
    assert all(d.cameras == [config.CAMERA] * config.OBSERVE_FRAMES for d in dets.values())


def test_observe_survives_a_broken_detector(monkeypatch):
    r = FakeRealRobot()
    dets = {name: FakeDetector([box_in("slot2")]) for name in ["det-red", "det-blue", "det-white", "det-yellow"]}
    dets["det-red"] = FakeDetector(error=RuntimeError("service not found"))
    install(monkeypatch, dets, r)
    scene = run(vision.observe(r))
    assert scene["red_cube"] is None and scene["blue_cube"] == "slot2"


def test_observe_ignores_noise_and_uses_the_main_blob(monkeypatch):
    r = FakeRealRobot()
    noise = det(*[v for v in (5, 5, 8, 8)])                  # tiny speck in slot1
    main = box_in("left", half=40)
    dets = {name: FakeDetector([]) for name in ["det-red", "det-blue", "det-white", "det-yellow"]}
    dets["det-red"] = FakeDetector([noise, main])
    install(monkeypatch, dets, r)
    assert run(vision.observe(r))["red_cube"] == "left"


class SequenceDetector:
    """Returns a different list of detections on each call (one per frame)."""
    def __init__(self, frames):
        self.frames, self.i = frames, 0

    async def get_detections_from_camera(self, camera):
        out = self.frames[min(self.i, len(self.frames) - 1)]
        self.i += 1
        return out


def test_observe_looks_several_times_and_votes(monkeypatch):
    monkeypatch.setattr(config, "OBSERVE_FRAMES", 3)
    monkeypatch.setattr(config, "MIN_HITS", 2)
    r = FakeRealRobot()
    dets = {name: FakeDetector([]) for name in ["det-red", "det-blue", "det-white", "det-yellow"]}
    # red: a phantom in "right" for one frame, really in slot1 for two
    dets["det-red"] = SequenceDetector([[box_in("right")], [box_in("slot1")], [box_in("slot1")]])
    # blue: flickers into view once only -> not trusted
    dets["det-blue"] = SequenceDetector([[], [box_in("tray")], []])
    install(monkeypatch, dets, r)
    scene = run(vision.observe(r))
    assert scene["red_cube"] == "slot1"
    assert scene["blue_cube"] is None
    assert r.calls.count("detect:det-red") == 3


def test_observe_ignores_background_blob_every_frame(monkeypatch):
    r = FakeRealRobot()
    bg = det(0, 0, 409, 719)
    dets = {name: FakeDetector([]) for name in ["det-red", "det-blue", "det-white", "det-yellow"]}
    dets["det-blue"] = FakeDetector([bg, box_in("slot2")])
    install(monkeypatch, dets, r)
    assert run(vision.observe(r))["blue_cube"] == "slot2"


def test_observe_can_skip_moving_when_already_at_overview(monkeypatch):
    r = FakeRealRobot()
    install(monkeypatch, {n: FakeDetector([]) for n in ["det-red", "det-blue", "det-white", "det-yellow"]}, r)
    run(vision.observe(r, already_there=True))
    assert "overview" not in r.calls


# ---------------- snapshot ----------------

class FakeImage:
    def __init__(self, name, mime, data):
        self.name, self.mime_type, self.data = name, mime, data


class FakeCamera:
    async def get_images(self):
        return [FakeImage("depth", "image/vnd.viam.dep", b"DEPTH"),
                FakeImage("color", "image/jpeg", b"COLOR")], None


def test_snapshot_with_no_color_image_saves_nothing(monkeypatch, tmp_path):
    class DepthOnly:
        async def get_images(self):
            return [FakeImage("depth", "image/vnd.viam.dep", b"DEPTH")], None
    r = FakeRealRobot()
    r.camera = DepthOnly()
    assert run(vision.snapshot(r, str(tmp_path / "overview.jpg"))) is None


def test_snapshot_saves_the_color_image_not_depth(monkeypatch, tmp_path):
    r = FakeRealRobot()
    r.camera = FakeCamera()
    install(monkeypatch, {n: FakeDetector([box_in("slot3")]) for n in
                          ["det-red", "det-blue", "det-white", "det-yellow"]}, r)
    path = run(vision.snapshot(r, str(tmp_path / "overview.jpg")))
    assert path.endswith(".jpg") and open(path, "rb").read() == b"COLOR"


def test_wide_flat_strip_is_ignored():
    # e.g. a colored edge of the table: very wide, object-height
    assert not vision.plausible(det(0, 300, 500, 340))


def test_tall_thin_strip_is_ignored():
    assert not vision.plausible(det(300, 0, 340, 500))
