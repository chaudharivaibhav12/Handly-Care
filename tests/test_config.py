"""config.py sanity checks: the real config must pass, and each check must catch its mistake."""
import copy
import types

import pytest

import checks
import config


def cfg_copy(**changes):
    fields = {k: copy.deepcopy(getattr(config, k)) for k in dir(config) if k.isupper()}
    fields.update(changes)
    return types.SimpleNamespace(**fields)


def problems(**changes):
    return checks.config_problems(cfg_copy(**changes))


def with_object(obj, **fields):
    objs = copy.deepcopy(config.OBJECTS)
    objs[obj].update(fields)
    return objs


def test_shipped_config_is_clean():
    assert checks.config_problems(config) == []


def test_shipped_config_is_clean_in_slots_mode_too():
    assert problems(MODE="slots") == []


def test_all_spots_is_slots_plus_zones():
    assert config.ALL_SPOTS == {**config.SLOTS, **config.ZONES}


def test_bad_mode_is_caught():
    assert any("MODE" in p for p in problems(MODE="magic"))


# ---------------- objects ----------------

def test_missing_size_is_caught():
    objs = copy.deepcopy(config.OBJECTS)
    del objs["cup"]["size"]
    assert any("'cup' needs size" in p for p in problems(OBJECTS=objs))


def test_grasp_height_above_the_object_is_caught():
    assert any("grasp_height" in p for p in problems(OBJECTS=with_object("red_cube", grasp_height=80)))


def test_missing_grasp_height_is_caught():
    objs = copy.deepcopy(config.OBJECTS)
    del objs["marker"]["grasp_height"]
    assert any("'marker' has no grasp_height" in p for p in problems(OBJECTS=objs))


def test_too_wide_for_the_gripper_is_caught():
    assert any("wider than the gripper" in p for p in problems(OBJECTS=with_object("cup", size=(95, 95, 80))))


def test_missing_description_is_caught():
    assert any("description" in p for p in problems(OBJECTS=with_object("cup", description="")))


# ---------------- recognition ----------------

def test_segmenter_for_unknown_object_is_caught():
    assert any("unknown object" in p for p in problems(SEGMENTERS={"seg-x": "laptop"}))


def test_object_with_no_way_to_be_found_is_caught():
    assert any("no detector labels" in p for p in problems(OBJECTS=with_object("cup", labels=[])))


def test_object_found_by_its_own_segmenter_needs_no_labels():
    objs = with_object("cup", labels=[])
    assert problems(OBJECTS=objs, SEGMENTERS={"objects-3d": None, "seg-cup": "cup"}) == []


def test_label_shared_by_two_objects_is_caught():
    assert any("labels used by two" in p for p in problems(OBJECTS=with_object("marker", labels=["cup"])))


def test_no_segmenters_is_caught():
    assert any("SEGMENTERS is empty" in p for p in problems(SEGMENTERS={}))


def test_slots_mode_duplicate_color_is_caught():
    objs = with_object("blue_cube", color="red")
    assert any("share a color" in p for p in problems(MODE="slots", OBJECTS=objs))


def test_slots_mode_duplicate_detector_is_caught():
    objs = with_object("blue_cube", detector="det-red")
    assert any("detector" in p for p in problems(MODE="slots", OBJECTS=objs))


# ---------------- places ----------------

def test_spots_too_close_are_caught():
    zones = dict(config.ZONES, tray=config.SLOTS["slot1"])
    assert any("closer than" in p for p in problems(ZONES=zones))


def test_unreachable_spot_is_caught():
    assert any("outside WORKSPACE or reach" in p for p in problems(ZONES=dict(config.ZONES, tray=(900, 0))))


def test_slot_inside_a_zone_is_caught():
    slots = dict(config.SLOTS, slot2=(570, 10))
    assert any("inside zone 'to_me'" in p for p in problems(SLOTS=slots))


def test_overlapping_zones_are_caught():
    zones = dict(config.ZONES, left=(300, -150))
    assert any("zones 'tray' and 'left' overlap" in p for p in problems(ZONES=zones))


def test_name_used_as_slot_and_zone_is_caught():
    slots = dict(config.SLOTS, tray=(400, 200))
    assert any("both slot and zone" in p for p in problems(SLOTS=slots))


@pytest.mark.parametrize("bad", ["x", "+z", "left", ""])
def test_bad_user_direction_is_caught(bad):
    assert any("USER_DIRECTION" in p for p in problems(USER_DIRECTION=bad))


def test_reach_limits_must_make_sense():
    assert any("MIN_REACH_MM" in p for p in problems(MIN_REACH_MM=700))


def test_zero_tolerance_is_caught():
    assert any("PLACE_TOLERANCE_MM" in p for p in problems(PLACE_TOLERANCE_MM=0))


# ---------------- slots-mode regions and misc ----------------

def test_spot_without_region_is_caught_in_slots_mode():
    regions = dict(config.REGIONS)
    regions.pop("tray")
    assert any("'tray' has no camera REGION" in p for p in problems(MODE="slots", REGIONS=regions))


def test_overlapping_regions_are_caught():
    regions = dict(config.REGIONS, slot2=config.REGIONS["slot1"])
    assert any("REGIONS" in p and "overlap" in p for p in problems(REGIONS=regions))


def test_backwards_region_is_caught():
    regions = dict(config.REGIONS, slot1=(100, 100, 50, 50))
    assert any("min >= max" in p for p in problems(REGIONS=regions))


def test_low_hover_is_caught():
    assert any("HOVER" in p for p in problems(HOVER=20))


def test_bad_box_size_window_is_caught():
    assert any("MIN_BOX_PX" in p for p in problems(MIN_BOX_PX=200, MAX_BOX_PX=100))


def test_min_hits_above_frames_is_caught():
    assert any("MIN_HITS" in p for p in problems(OBSERVE_FRAMES=2, MIN_HITS=3))


def test_calibration_spots_are_spread_out():
    """Taught slots double as calibration points: they must not be in a line."""
    import calibration
    assert calibration._spread_mm(list(config.SLOTS.values())) >= 40
