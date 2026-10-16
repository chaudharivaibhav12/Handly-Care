"""Placement geometry: directions, zones, relations, stacking, blockers, reach, done-ness."""
import math

import pytest

import config
import places
from conftest import scene


def near(a, b, tol=0.5):
    return math.dist(a[:2], b[:2]) <= tol


# ---------------- directions (from the person's point of view) ----------------

@pytest.mark.parametrize("user, front, left", [
    ("+x", (1, 0), (0, -1)),
    ("-x", (-1, 0), (0, 1)),
    ("+y", (0, 1), (1, 0)),
    ("-y", (0, -1), (-1, 0)),
])
def test_directions_follow_where_the_person_stands(monkeypatch, user, front, left):
    monkeypatch.setattr(config, "USER_DIRECTION", user)
    d = places.directions()
    assert d["in_front_of"] == pytest.approx(front)
    assert d["left_of"] == pytest.approx(left)
    assert d["right_of"] == pytest.approx((-left[0], -left[1]))
    assert d["behind"] == pytest.approx((-front[0], -front[1]))


# ---------------- parsing ----------------

@pytest.mark.parametrize("dest, expect", [
    ("tray", ("zone", "tray")),
    ("left_of:cup", ("left_of", "cup")),
    ("on:blue_cube", ("on", "blue_cube")),
])
def test_parse_valid(dest, expect):
    assert places.parse(dest) == expect


@pytest.mark.parametrize("dest", ["", None, "slot1", "floor", "under:cup", "left_of:laptop", "(100,200)", 5])
def test_parse_invalid_is_a_place_error(dest):
    with pytest.raises(places.PlaceError):
        places.parse(dest)


def test_anchor_of():
    assert places.anchor_of("next_to:cup") == "cup"
    assert places.anchor_of("tray") is None
    assert places.anchor_of("junk") is None


# ---------------- zones ----------------

def test_zone_uses_its_center_when_free():
    x, y, lift = places.resolve("red_cube", "tray", scene())
    assert near((x, y), config.ZONES["tray"]) and lift == 0


def test_zone_finds_a_free_spot_when_center_is_taken():
    s = scene(cup="tray")
    x, y, _ = places.resolve("red_cube", "tray", s)
    assert not near((x, y), config.ZONES["tray"], tol=1)
    assert math.dist((x, y), config.ZONES["tray"]) <= config.ZONE_RADIUS_MM
    assert places.blocker("red_cube", x, y, s) is None


def test_full_zone_is_refused(monkeypatch):
    monkeypatch.setattr(config, "ZONE_RADIUS_MM", 30.0)   # room for exactly one thing
    with pytest.raises(places.PlaceError, match="full"):
        places.resolve("red_cube", "tray", scene(cup="tray"))


# ---------------- relations ----------------

def gap(a, b):
    return places.radius(a) + places.radius(b) + config.NEXT_TO_GAP_MM


@pytest.mark.parametrize("rel", ["left_of", "right_of", "in_front_of", "behind"])
def test_side_relations_land_on_the_right_side(rel):
    # only the cup and the mover on the table, well apart
    s = {"cup": (450, 0), "red_cube": (300, 250), "blue_cube": None, "marker": None}
    x, y, lift = places.resolve("red_cube", f"{rel}:cup", s)
    ux, uy = places.directions()[rel]
    want = (450 + ux * gap("cup", "red_cube"), 0 + uy * gap("cup", "red_cube"))
    assert near((x, y), want) and lift == 0


def test_next_to_picks_a_free_reachable_side_near_the_middle():
    s = {"cup": (450, 0), "red_cube": (300, 250), "blue_cube": None, "marker": None}
    x, y, _ = places.resolve("red_cube", "next_to:cup", s)
    assert abs(math.dist((x, y), (450, 0)) - gap("cup", "red_cube")) < 0.5
    assert places.reachable(x, y) and places.blocker("red_cube", x, y, s) is None


def test_relation_refuses_when_side_is_blocked():
    left = places.directions()["left_of"]
    d = gap("cup", "red_cube")
    s = scene(cup=(450, 0), blue_cube=(450 + left[0] * d, left[1] * d), red_cube=(350, 200))
    with pytest.raises(places.PlaceError, match="blue cube is in the way"):
        places.resolve("red_cube", "left_of:cup", s)


def test_next_to_goes_around_a_blocked_side():
    left = places.directions()["left_of"]
    d = gap("cup", "red_cube")
    s = scene(cup=(450, 0), blue_cube=(450 + left[0] * d, left[1] * d), red_cube=(350, 250))
    x, y, _ = places.resolve("red_cube", "next_to:cup", s)
    assert places.blocker("red_cube", x, y, s) is None


def test_relation_refuses_out_of_reach():
    # anchor near the far edge: "behind" (away from the person, -x) would leave the table
    s = scene(cup=(210, 0), red_cube=(450, 200))
    with pytest.raises(places.PlaceError, match="reach"):
        places.resolve("red_cube", "behind:cup", s)


def test_relation_to_invisible_anchor_is_refused():
    with pytest.raises(places.PlaceError, match="can't see the cup"):
        places.resolve("red_cube", "next_to:cup", scene(cup=None))


def test_relation_to_itself_is_refused():
    with pytest.raises(places.PlaceError):
        places.resolve("cup", "next_to:cup", scene())


# ---------------- stacking ----------------

def test_stack_on_a_cube_lands_on_it_and_lifts_by_its_height():
    s = scene()
    x, y, lift = places.resolve("red_cube", "on:blue_cube", s)
    assert near((x, y), s["blue_cube"]) and lift == config.OBJECTS["blue_cube"]["size"][2]


def test_cannot_stack_on_a_cup():
    with pytest.raises(places.PlaceError, match="can't stack"):
        places.resolve("red_cube", "on:cup", scene())


def test_cannot_stack_on_something_already_occupied():
    s = scene(marker=xy_of(scene()["blue_cube"]))
    with pytest.raises(places.PlaceError, match="already"):
        places.resolve("red_cube", "on:blue_cube", s)


def xy_of(p):
    return p


# ---------------- reach, blockers ----------------

def test_reachable_limits():
    assert places.reachable(400, 0)
    assert not places.reachable(100, 0)            # outside workspace / too close to base
    assert not places.reachable(700, 400)          # beyond reach
    assert not places.reachable(400, 1000)


def test_blocker_ignores_itself_and_unseen_objects():
    s = scene(blue_cube=None)
    rx, ry = s["red_cube"]
    assert places.blocker("red_cube", rx, ry, s) is None


# ---------------- done / error ----------------

def test_done_in_zone_within_radius():
    zx, zy = config.ZONES["tray"]
    assert places.done("cup", "tray", scene(cup=(zx + 30, zy)))
    assert not places.done("cup", "tray", scene(cup=(zx + 200, zy)))


def test_done_relation_uses_tight_tolerance():
    s = {"cup": (450, 0), "red_cube": (300, 250), "blue_cube": None, "marker": None}
    x, y, _ = places.resolve("red_cube", "left_of:cup", s)
    s["red_cube"] = (x + 5, y)
    assert places.done("red_cube", "left_of:cup", s)
    s["red_cube"] = (x + config.PLACE_TOLERANCE_MM + 10, y)
    assert not places.done("red_cube", "left_of:cup", s)


def test_relation_breaks_when_the_anchor_moves():
    s = {"cup": (450, 0), "red_cube": (300, 250), "blue_cube": None, "marker": None}
    x, y, _ = places.resolve("red_cube", "left_of:cup", s)
    s["red_cube"] = (x, y)
    assert places.done("red_cube", "left_of:cup", s)
    s["cup"] = (450, 150)
    assert not places.done("red_cube", "left_of:cup", s)


def test_next_to_any_side_counts():
    s = scene(cup=(450, 0))
    d = gap("cup", "red_cube")
    for v in [(d, 0), (-d, 0), (0, d), (0, -d)]:
        s["red_cube"] = (450 + v[0], v[1])
        assert places.done("red_cube", "next_to:cup", s)


def test_on_done_when_over_the_anchor():
    s = scene()
    s["red_cube"] = s["blue_cube"]
    assert places.done("red_cube", "on:blue_cube", s)


def test_not_done_when_unseen():
    assert not places.done("cup", "tray", scene(cup=None))
    assert places.error_mm("cup", "tray", scene(cup=None)) is None


def test_zone_lift_raises_the_drop_height(monkeypatch):
    """A tray floor above the table: resolve() returns its height as the lift."""
    import places
    monkeypatch.setattr(config, "ZONE_LIFT", {"tray": 12.0})
    _, _, lift = places.resolve("red_cube", "tray", {"red_cube": (450, -150)})
    assert lift == 12.0
    _, _, lift = places.resolve("red_cube", "left", {"red_cube": (450, -150)})
    assert lift == 0.0


def test_person_on_plus_y_has_their_left_on_plus_x(monkeypatch):
    """armfarm13: the person sits on the +y side, so their left is +x (the `left` zone, x=395)."""
    import places
    monkeypatch.setattr(config, "USER_DIRECTION", "+y")
    d = places.directions()
    assert d["left_of"][0] > 0 and abs(d["left_of"][1]) < 1e-9
    assert d["in_front_of"][1] > 0   # "in front of" = toward the person


def test_next_to_never_targets_the_wall_strip(monkeypatch):
    """EXP-006 req 2 (Fri 3:47): next_to the blue block at (454, 60) picked (454, 141); the camera hits wall-front there."""
    monkeypatch.setattr(config, "WORKSPACE", {"x": (80.0, 480.0), "y": (-60.0, 640.0)})
    monkeypatch.setattr(config, "NO_GO", [(440.0, 480.0, 100.0, 640.0)])
    monkeypatch.setattr(config, "USER_DIRECTION", "+y")
    assert not places.reachable(454.0, 141.0) and places.reachable(456.0, 76.0)
    scene = {"blue_cube": (454.0, 60.0), "red_cube": (200.0, 300.0)}
    try:
        x, y, _ = places.resolve("red_cube", "next_to:blue_cube", scene)
        assert places.reachable(x, y)
    except places.PlaceError as e:
        assert "reach" in str(e)
