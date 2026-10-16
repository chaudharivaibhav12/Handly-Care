"""MODE "anything" (DEC-002): find any object by 3D shape, name it, decide graspability by size."""
import io
import types

import pytest

import anything
import calibration
import config
import perception
import planner
from conftest import run

TOP = 15.0   # TABLE_TOP_WORLD_Z used in these tests


def blob_obj(x, y, w, d, h):
    """A fake segmenter object: a box in world coordinates sitting on the table."""
    g = types.SimpleNamespace(center=types.SimpleNamespace(x=x, y=y, z=TOP + h / 2),
                              box=types.SimpleNamespace(dims_mm=types.SimpleNamespace(x=w, y=d, z=h)),
                              label="")
    g.HasField = lambda f: f == "box"
    return types.SimpleNamespace(geometries=types.SimpleNamespace(reference_frame="world", geometries=[g]))


CUBE = (450, -150, 50, 50, 50)
CUP = (520, 10, 65, 65, 80)
LAPTOP = (380, 250, 310, 220, 20)


class FakeSeg:
    def __init__(self, looks):
        self.looks = list(looks)

    async def get_object_point_clouds(self, camera):
        return [blob_obj(*b) for b in (self.looks.pop(0) if self.looks else [])]


class FakeCam:
    async def get_images(self, **kw):
        return [], None


class FakeRobot:
    def __init__(self):
        self.camera = FakeCam()

    async def go_overview(self):
        pass


NAMES = {50: ("red block", "a red toy block", True, True, ""),
         65: ("paper cup", "a paper cup, for drinks", False, True, ""),
         310: ("laptop", "a laptop computer", True, True, "")}


def fake_namer(calls):
    def namer(blobs, crops, use_ai):
        calls.append(len(blobs))
        out = []
        for b in blobs:
            name, desc, stack, safe, why = NAMES[round(b["w"])]
            out.append({"name": name, "description": desc, "stackable": stack, "safe": safe, "why_unsafe": why})
        return out
    return namer


@pytest.fixture
def mode(monkeypatch):
    monkeypatch.setattr(config, "MODE", "anything")
    monkeypatch.setattr(config, "ANYTHING_SOURCE", "segmenter")   # these tests drive the segmenter path
    monkeypatch.setattr(config, "OBJECTS", dict(config.OBJECTS))
    monkeypatch.setattr(config, "TABLE_TOP_WORLD_Z", TOP)
    monkeypatch.setattr(config, "FIXTURES", {"tray": (300, -300, 150)})
    monkeypatch.setattr(config, "GRIPPER_MAX_OPEN_MM", 84.0)
    monkeypatch.setattr(anything, "_known", {})
    calls = []
    monkeypatch.setattr(anything, "namer", fake_namer(calls))
    return calls


def look(monkeypatch, *looks):
    seg = FakeSeg(looks)
    monkeypatch.setattr(perception, "segmenter_for", lambda robot, name: seg)


def find(robot=None):
    return run(perception.find(robot or FakeRobot(), cal=dict(calibration.IDENTITY)))


# ---------------- measuring ----------------

def test_graspable_comes_from_measured_size(mode):
    assert anything.graspable((50, 50, 50)) == (True, "")
    ok, why = anything.graspable((310, 220, 20))
    assert not ok and "22 cm wide" in why and "8.4 cm" in why
    assert anything.graspable((60, 50, 8))[1].startswith("it's too flat")
    assert anything.graspable((60, 50, 260))[1].startswith("it's too tall")
    assert anything.graspable((80, 75, 50))[0] is False      # 75 > 84 - 10 margin
    assert anything.graspable((80, 74, 50))[0] is True


def test_grasp_height_is_middle_but_below_the_top():
    assert anything.grasp_height(50) == 25
    assert anything.grasp_height(120) == 60
    assert anything.grasp_height(12) == 8                    # never lower than 8 mm


@pytest.mark.parametrize("blob, reason", [
    ((60, 40, 50, 50, 50), "the arm itself"),
    ((450, 0, 60, 60, 3), "too small (noise)"),
    ((300, -300, 230, 160, 25), "the tray itself"),
    ((400, -300, 48, 10, 20), "the tray's rim"),            # seen on the real arm, Fri 1:35 PM
    ((950, 0, 50, 50, 50), "off the table"),
    ((450, 0, 600, 400, 30), "too big (segmentation junk)"),
])
def test_blobs_that_are_not_objects_are_skipped(mode, blob, reason):
    x, y, w, d, h = blob
    assert anything.skip_reason({"x": x, "y": y, "w": w, "d": d, "h": h}) == reason


def test_a_cube_inside_the_tray_is_still_an_object(mode):
    assert anything.skip_reason({"x": 300, "y": -300, "w": 50, "d": 50, "h": 50}) is None


# ---------------- finding + naming ----------------

def test_find_discovers_names_and_measures_everything(mode, monkeypatch):
    look(monkeypatch, [CUBE, CUP, LAPTOP], [CUBE, CUP, LAPTOP], [CUBE, CUP, LAPTOP])
    scene = find()
    assert set(scene) == {"red_block", "paper_cup", "laptop"}
    assert scene["red_block"] == pytest.approx((450, -150))
    assert "red_cube" not in config.OBJECTS                  # the fixed catalog is gone in this mode
    cube, laptop = config.OBJECTS["red_block"], config.OBJECTS["laptop"]
    assert cube["size"] == (50, 50, 50) and cube["grasp_height"] == 25 and cube["graspable"]
    assert cube["stackable"] and not config.OBJECTS["paper_cup"]["stackable"]
    assert not laptop["graspable"] and "22 cm wide" in laptop["why_not"]
    assert "CAN'T pick it up" in laptop["description"]
    assert mode == [3]                                       # ONE naming call for all three


def test_phantom_seen_in_one_look_is_ignored(mode, monkeypatch):
    look(monkeypatch, [CUBE, (300, 200, 50, 50, 50)], [CUBE], [CUBE])
    assert set(find()) == {"red_block"}


def test_arm_moving_an_object_keeps_its_identity_without_renaming(mode, monkeypatch):
    look(monkeypatch, [CUBE], [CUBE], [CUBE])
    find()
    perception.moved("red_block", (300, 250))                # executor calls this after a move
    look(monkeypatch, [(305, 246, 50, 50, 50)] * 1, [(305, 246, 50, 50, 50)], [(305, 246, 50, 50, 50)])
    scene = find()
    assert scene == {"red_block": pytest.approx((305, 246))} and mode == [1]


def test_person_moving_one_object_keeps_its_identity_by_size(mode, monkeypatch):
    look(monkeypatch, [CUBE, CUP], [CUBE, CUP], [CUBE, CUP])
    find()
    moved_cup = (300, 300, 65, 65, 80)
    look(monkeypatch, [CUBE, moved_cup], [CUBE, moved_cup], [CUBE, moved_cup])
    scene = find()
    assert scene["paper_cup"] == pytest.approx((300, 300)) and mode == [2]   # no second naming call


def test_object_gone_is_reported_as_not_seen(mode, monkeypatch):
    look(monkeypatch, [CUBE, CUP], [CUBE, CUP], [CUBE, CUP])
    find()
    look(monkeypatch, [CUBE], [CUBE], [CUBE])
    assert find()["paper_cup"] is None


def test_two_things_with_the_same_name_get_different_ids(mode, monkeypatch):
    other = (300, 200, 50, 50, 50)
    look(monkeypatch, [CUBE, other], [CUBE, other], [CUBE, other])
    assert set(find()) == {"red_block", "red_block_2"}


def test_unsafe_according_to_the_namer_is_not_graspable(mode, monkeypatch):
    monkeypatch.setattr(anything, "namer", lambda blobs, crops, use_ai: [
        {"name": "glass", "description": "a drinking glass", "stackable": False, "safe": False,
         "why_unsafe": "it's glass and could break"}])
    look(monkeypatch, [CUP], [CUP], [CUP])
    find()
    assert config.OBJECTS["glass"]["graspable"] is False
    assert config.OBJECTS["glass"]["why_not"] == "it's glass and could break"


def test_naming_failure_falls_back_to_plain_names(mode, monkeypatch):
    monkeypatch.undo()   # real namer
    monkeypatch.setattr(config, "MODE", "anything")
    monkeypatch.setattr(config, "OBJECTS", dict(config.OBJECTS))
    monkeypatch.setattr(config, "TABLE_TOP_WORLD_Z", TOP)
    monkeypatch.setattr(anything, "_known", {})

    def boom(*a, **k):
        raise TimeoutError("slow network")
    monkeypatch.setattr(anything, "name_with_ai", boom)
    infos = anything.namer([{"x": 0, "y": 0, "w": 50, "d": 50, "h": 50}], [None], use_ai=True)
    assert infos[0]["name"] == "small object" and "5 x 5 x 5 cm" in infos[0]["description"]


# ---------------- the Claude call ----------------

class FakeClient:
    def __init__(self, text):
        self.calls = []
        reply = types.SimpleNamespace(content=[types.SimpleNamespace(type="text", text=text)])
        create = lambda **kw: (self.calls.append(kw), reply)[1]  # noqa: E731
        self.beta = types.SimpleNamespace(messages=types.SimpleNamespace(create=create))
        self.messages = types.SimpleNamespace(create=create)


def test_name_with_ai_sends_crops_and_parses_the_list():
    fake = FakeClient('Here: [{"n": 1, "name": "Blue Mug", "description": "a blue mug, for drinks",'
                      ' "stackable": false, "safe": true, "why_unsafe": ""}]')
    out = anything.name_with_ai([{"w": 80, "d": 80, "h": 95}], [b"\xff\xd8jpeg"], client=fake)
    assert out[0]["name"] == "blue mug" and out[0]["safe"] is True
    call = fake.calls[0]
    assert call["model"] == config.NAMER_MODEL and call["output_config"] == {"effort": "low"}
    assert call["fallbacks"] == "default" and "temperature" not in call
    kinds = [b["type"] for b in call["messages"][0]["content"]]
    assert kinds.count("image") == 1 and kinds[-1] == "text"


def test_name_with_ai_rejects_a_reply_missing_objects():
    fake = FakeClient('[{"n": 1, "name": "mug"}]')
    with pytest.raises(ValueError):
        anything.name_with_ai([{"w": 1, "d": 1, "h": 1}] * 2, [None, None], client=fake)


def test_crop_and_color_from_a_real_image():
    from PIL import Image
    img = Image.new("RGB", (200, 100), (240, 240, 240))
    img.paste((210, 30, 30), (80, 30, 120, 70))
    buf = io.BytesIO()
    img.save(buf, format="JPEG")
    c = anything.crop(buf.getvalue(), {"u": 100, "v": 50, "pr": 20})
    assert c and anything.color_of(c) == "red"
    assert anything.crop(buf.getvalue(), {"u": None, "v": None, "pr": None}) is None


# ---------------- the planner with discovered objects ----------------

def discovered(monkeypatch):
    monkeypatch.setattr(config, "OBJECTS", {})
    monkeypatch.setattr(anything, "_known", {})
    infos = [("yellow highlighter", "a yellow highlighter, for writing", (120, 20, 20), (450, -150)),
             ("laptop", "a laptop computer", (310, 220, 20), (380, 250))]
    for name, desc, size, (x, y) in infos:
        anything.register(anything.slug(name), {"x": x, "y": y, "w": size[0], "d": size[1], "h": size[2]},
                          {"name": name, "description": desc, "stackable": False, "safe": True, "why_unsafe": ""})
    return {"yellow_highlighter": (450, -150), "laptop": (380, 250)}


def test_keyword_planner_hands_over_a_discovered_writing_tool(mode, monkeypatch):
    scene = discovered(monkeypatch)
    p = planner.make_plan(["Give me something to write with"], scene)
    assert p["actions"] == [{"object": "yellow_highlighter", "destination": "to_me"}]


def test_refusal_gives_the_measured_reason(mode, monkeypatch):
    scene = discovered(monkeypatch)
    p = planner.make_plan(["Pick up my laptop"], scene)
    assert p["actions"] == [] and "22 cm wide" in p["say"]


def test_validate_drops_an_ungraspable_object_from_an_ai_plan(mode, monkeypatch):
    scene = discovered(monkeypatch)
    p = planner.validate({"say": "ok", "actions": [{"object": "laptop", "destination": "tray"},
                                                   {"object": "yellow_highlighter", "destination": "tray"}]},
                         scene)
    assert [a["object"] for a in p["actions"]] == ["yellow_highlighter"]
    assert any("can't pick up the laptop" in x for x in p["problems"])


# ---------------- fragments (what the real segmenter returns, Fri 1:15 PM) ----------------

def frag(x, y, w, d, z0, z1):
    """A world-frame fragment: footprint centered at x, y; spans z0..z1."""
    return {"xmin": x - w / 2, "xmax": x + w / 2, "ymin": y - d / 2, "ymax": y + d / 2,
            "top": z1, "bot": z0, "cam": None}


def tilted(x, y):
    return 5 + 0.04 * x - 0.03 * y          # a table ~±15 mm off flat across the view


def test_pieces_of_one_object_merge_and_table_noise_is_dropped(mode):
    noise = [frag(x, y, 15, 10, tilted(x, y) - 2, tilted(x, y) + 3)
             for x in (150, 250, 350, 450) for y in (-100, 100, 300)]
    mug = [frag(400 + dx, 200 + dy, 35, 35, tilted(400, 200) + 10, tilted(400, 200) + 95)
           for dx in (-17, 17) for dy in (-17, 17)]          # one mug, 4 touching pieces
    objs = anything.merge(noise + mug, anything.table_plane(noise + mug))
    assert len(objs) == 1
    m = objs[0]
    assert m["pieces"] == 4 and m["w"] == pytest.approx(69) and m["h"] == pytest.approx(95, abs=1)


def test_tilted_table_is_fitted_not_assumed(mode):
    noise = [frag(x, y, 15, 10, tilted(x, y) - 2, tilted(x, y) + 2)
             for x in (150, 300, 450) for y in (-100, 100, 300)]
    plane = anything.table_plane(noise)
    assert plane(450, -100) == pytest.approx(tilted(450, -100), abs=1)
    assert anything.table_plane(noise[:3])(0, 0) == TOP     # too little noise: the constant


def test_far_apart_pieces_stay_separate_objects(mode):
    a = frag(300, 0, 50, 50, TOP, TOP + 50)
    b = frag(300, 80, 50, 50, TOP, TOP + 50)                 # 30 mm gap > MERGE_GAP_MM
    assert len(anything.merge([a, b], lambda x, y: TOP)) == 2


def test_points_measure_true_height_despite_flying_pixel_streaks(mode):
    """Real segments (Fri 1:30 PM) are 2 cm columns from the object's top down to the table:
    flying pixels at the edge. The 95th-percentile top must ignore them and a few spikes."""
    import struct
    top = [(x, y, TOP + 60) for x in range(400, 450, 3) for y in range(0, 50, 3)]        # 50x50 top, 60 mm
    streak = [(400, 0, TOP + z) for z in range(10, 60, 2)]                                # edge streak
    spikes = [(425, 25, TOP + 300)] * 3                                                   # 3 wild points
    pts = top + streak + spikes
    x, y, w, d, h = anything.measure_points(pts, lambda x, y: TOP)
    assert h == pytest.approx(60) and w == pytest.approx(45, abs=6) and (x, y) == pytest.approx((424, 24), abs=3)
    pcd = b"VERSION .7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F I\nDATA binary\n" + b"".join(
        struct.pack("<fffI", px / 1000, py / 1000, pz / 1000, 0) for px, py, pz in pts[:9])
    assert anything.pcd_points(pcd, every=1)[0] == pytest.approx(pts[0], abs=0.01)


def test_flat_top_of_an_object_is_kept_not_treated_as_table(mode):
    """Bug found Fri 1:45 PM: points were only read from THICK pieces, so the flat top of a block
    (a thin piece, 23 mm above the table) was thrown away and its footprint came out 30 instead of 75."""
    import struct
    noise = [frag(x, y, 15, 10, TOP - 2, TOP + 2) for x in (150, 300, 450) for y in (-100, 100, 300)]
    top_pts = [(x, y, TOP + 23) for x in range(300, 376, 3) for y in range(0, 76, 3)]
    pcd = b"FIELDS x y z rgb\nDATA binary\n" + b"".join(
        struct.pack("<fffI", x / 1000, y / 1000, z / 1000, 0) for x, y, z in top_pts)
    flat_top = frag(337, 37, 75, 75, TOP + 21, TOP + 24)               # 3 mm thick, 23 mm up
    flat_top.update(pcd=pcd, to_w=lambda x, y, z: (x, y, z))
    objs = anything.merge(noise + [flat_top], anything.table_plane(noise + [flat_top]))
    assert len(objs) == 1 and objs[0]["w"] == pytest.approx(70, abs=6) and objs[0]["h"] == pytest.approx(23, abs=1)
