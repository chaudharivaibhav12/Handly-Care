"""The adaptive loop, end to end in simulation: does it pick from anywhere, place precisely,
check its own work, nudge, repair, take interruptions only at checkpoints, and stop safely?"""
import asyncio
import json
import math

import config
import executor
import places
from conftest import run, sim


def go(robot, request, inbox_msgs=()):
    status = executor.Status(echo=False)

    async def main():
        inbox = asyncio.Queue()
        for m in inbox_msgs:
            inbox.put_nowait(m)
        await executor.run_task(robot, request, inbox, status)
        return inbox
    inbox = run(main())
    return status, inbox


def say_during_move(n, message):
    """A person types `message` while move number `n` is happening."""
    inbox = asyncio.Queue()

    def on_move(robot, obj, src, dst):
        if len(robot.moves) == n:
            inbox.put_nowait(message)
    return inbox, on_move


def go_with(robot, request, inbox):
    status = executor.Status(echo=False)
    run(executor.run_task(robot, request, inbox, status))
    return status


def movers(robot):
    return [m[0] for m in robot.moves]


# ---------------- happy paths ----------------

def test_picks_from_anywhere_and_delivers():
    r = sim({"red_cube": (480, -120), "blue_cube": (500, 90), "cup": (420, 200), "marker": (380, -40)})
    st, _ = go(r, "Give me something to write with")
    assert r.spot("marker") == "to_me"
    assert r.moves == [("marker", (380, -40), tuple(config.ZONES["to_me"]), True)]
    assert st.data["done"] == [{"object": "marker", "destination": "to_me"}]
    assert st.data["errors_mm"] == [0.0] and st.data["state"] == "idle"


def test_looks_again_after_every_move():
    r = sim()
    go(r, "Put the cup in the tray, everything else to the left")
    assert len(r.moves) == 4 and r.overview_visits == 1 + len(r.moves)


def test_zone_sort_ends_in_the_right_places_without_overlaps():
    r = sim()
    go(r, "Put the cup in the tray, everything else to the left")
    s = r.seen()
    assert places.done("cup", "tray", s)
    for o in ("red_cube", "blue_cube", "marker"):
        assert places.done(o, "left", s)
    for o in s:   # three things share the left zone, none overlapping
        assert places.blocker(o, *s[o], s) is None


def test_stacking_lifts_by_the_lower_cube_height():
    r = sim()
    go(r, "put the red cube on top of the blue cube")
    assert r.pos["red_cube"] == r.pos["blue_cube"]
    assert r.lifts == [config.OBJECTS["blue_cube"]["size"][2]]


def test_relation_placement_is_precise():
    r = sim()
    st, _ = go(r, "put the red cube next to the cup")
    assert places.done("red_cube", "next_to:cup", r.seen())
    assert places.error_mm("red_cube", "next_to:cup", r.seen()) < 1


def test_anchor_is_moved_before_what_goes_next_to_it():
    r = sim()
    go(r, "put the red cube next to the cup, and the cup in the tray")
    assert movers(r) == ["cup", "red_cube"]
    s = r.seen()
    assert places.done("cup", "tray", s) and places.done("red_cube", "next_to:cup", s)


def test_refusal_never_moves_anything():
    r = sim()
    st, _ = go(r, "Pick up my laptop")
    assert r.moves == [] and "say" in st.kinds()


def test_already_satisfied_request_does_nothing():
    r = sim(dict(sim().pos, marker=tuple(config.ZONES["to_me"])))
    go(r, "Give me something to write with")
    assert r.moves == []


# ---------------- precision: measure, nudge, give up ----------------

def test_placement_that_lands_off_is_nudged_from_where_it_landed():
    off = {"n": 0}

    def err(obj, n):
        off["n"] += 1
        return (40.0, 0.0) if off["n"] == 1 else (3.0, 0.0)   # first try 40 mm off, then close
    r = sim(place_error=err)
    st, _ = go(r, "put the red cube left of the cup")
    assert movers(r) == ["red_cube", "red_cube"]
    first_target = r.moves[0][2]
    assert r.moves[1][1] == (first_target[0] + 40.0, first_target[1])   # picked from where it landed
    assert any("off; adjusting" in t for t in st.texts())
    assert st.data["errors_mm"] == [3.0]


def test_zone_placements_tolerate_more_than_relations():
    r = sim(place_error=(40.0, 0.0))        # 40 mm off is still inside an 18 cm zone
    go(r, "give me the blue cube")
    assert len(r.moves) == 1 and places.done("blue_cube", "to_me", r.seen())


def test_always_off_gives_up_and_asks_for_help(monkeypatch):
    monkeypatch.setattr(config, "MAX_RETRIES", 1)
    r = sim(place_error=(60.0, 0.0))
    st, _ = go(r, "put the red cube left of the cup")
    assert len(r.moves) == 1 + config.MAX_RETRIES
    assert any("Could someone help" in t for t in st.texts())


# ---------------- interruptions (only at safe checkpoints) ----------------

def test_change_of_mind_mid_task():
    inbox, hook = say_during_move(1, "Actually, put the red cube next to the cup instead")
    r = sim(on_move=hook)
    st = go_with(r, "Put the cup in the tray, and the red cube on top of the blue cube", inbox)
    assert movers(r) == ["cup", "red_cube"]
    assert places.done("red_cube", "next_to:cup", r.seen())
    assert r.pos["red_cube"] != r.pos["blue_cube"]
    assert "replan" in st.kinds()


def test_leave_mid_task():
    inbox, hook = say_during_move(1, "Actually, leave the marker")
    r = sim(on_move=hook)
    go_with(r, "Put the cup in the tray, everything else to the left", inbox)
    assert "marker" not in movers(r) and r.spot("marker") == "slot4"


def test_interrupt_is_not_applied_mid_move():
    inbox, hook = say_during_move(1, "Actually, leave the cup")
    r = sim(on_move=hook)
    go_with(r, "Put the cup in the tray, everything else to the left", inbox)
    assert r.moves[0][0] == "cup" and r.moves[0][3] is True


def test_follow_up_can_add_new_work():
    inbox, hook = say_during_move(1, "and give me the blue cube")
    r = sim(on_move=hook)
    go_with(r, "Give me something to write with", inbox)
    s = r.seen()
    assert places.done("marker", "to_me", s) and places.done("blue_cube", "to_me", s)


def test_end_of_input_marker_is_left_for_the_caller():
    _, inbox = go(sim(), "Give me something to write with", inbox_msgs=[None])
    assert inbox.get_nowait() is None


# ---------------- checking its own work, repairing the table ----------------

class FailFirst:
    """Grab misses the first `n` times for one object."""
    def __init__(self, obj, n):
        self.obj, self.n = obj, n

    def wrap(self, robot):
        real = robot.move_object

        async def move_object(obj, src, dst, lift=0.0, **kw):
            if obj == self.obj and self.n > 0:
                self.n -= 1
                robot.moves.append((obj, tuple(src), tuple(dst), False))
                return False
            return await real(obj, src, dst, lift)
        robot.move_object = move_object
        return robot


def test_failed_grab_is_retried_and_succeeds(monkeypatch):
    monkeypatch.setattr(config, "MAX_RETRIES", 1)
    r = FailFirst("marker", 1).wrap(sim())
    st, _ = go(r, "Give me something to write with")
    assert r.spot("marker") == "to_me" and [m[3] for m in r.moves] == [False, True]


def test_gives_up_on_one_object_but_finishes_the_rest(monkeypatch):
    monkeypatch.setattr(config, "MAX_RETRIES", 1)
    r = FailFirst("cup", 99).wrap(sim())
    st, _ = go(r, "Put the cup in the tray, everything else to the left")
    assert len([m for m in r.moves if m[0] == "cup"]) == 1 + config.MAX_RETRIES
    assert any("Could someone help" in t for t in st.texts())
    for o in ("red_cube", "blue_cube", "marker"):
        assert places.done(o, "left", r.seen())


def test_dropped_on_the_way_is_picked_up_from_where_it_fell(monkeypatch):
    r = sim()
    real = r.move_object
    state = {"first": True}

    async def drops_it(obj, src, dst, lift=0.0, **kw):
        if state["first"]:
            state["first"] = False
            r.pos[obj] = (480.0, 250.0)          # fell here
            r.moves.append((obj, tuple(src), tuple(dst), True))
            return True
        return await real(obj, src, dst, lift)
    r.move_object = drops_it
    go(r, "Give me something to write with")
    assert r.spot("marker") == "to_me" and r.moves[1][1] == (480.0, 250.0)


def test_notices_a_finished_object_was_moved_and_fixes_it():
    def on_move(robot, obj, src, dst):
        if obj == "red_cube":                     # someone takes the cup out of the tray
            robot.pos["cup"] = (450.0, 250.0)
    r = sim(on_move=on_move)
    st, _ = go(r, "Put the cup in the tray, everything else to the left")
    assert places.done("cup", "tray", r.seen())
    assert any("no longer where it should be" in t for t in st.texts())


def test_keeps_an_arrangement_true_when_the_anchor_moves():
    """'red cube next to the cup': someone moves the cup; the robot moves the cube to follow."""
    moved = {"done": False}

    def on_move(robot, obj, src, dst):
        if obj == "marker" and not moved["done"]:   # cube already placed; now the cup moves
            moved["done"] = True
            robot.pos["cup"] = (480.0, 250.0)
    r = sim(on_move=on_move)
    st, _ = go(r, "put the red cube next to the cup, and the marker in the tray")
    assert movers(r) == ["red_cube", "marker", "red_cube"]
    assert places.done("red_cube", "next_to:cup", r.seen())
    assert any("no longer where it should be" in t for t in st.texts())


def test_object_that_disappears_is_skipped_with_a_message():
    def on_move(robot, obj, src, dst):
        if len(robot.moves) == 1:
            robot.pos["marker"] = None            # taken off the table
    r = sim(on_move=on_move)
    st, _ = go(r, "Put the cup in the tray, everything else to the left")
    assert "marker" not in movers(r)
    assert any("can't see the marker" in t for t in st.texts())


def test_out_of_reach_object_is_refused_without_moving():
    r = sim(dict(sim().pos, marker=(720.0, 0.0)))   # seen, but too far
    st, _ = go(r, "Give me something to write with")
    assert r.moves == [] and any("out of my reach" in t for t in st.texts())


def test_destination_that_becomes_impossible_is_refused(monkeypatch):
    monkeypatch.setattr(config, "ZONE_RADIUS_MM", 30.0)    # the tray fits one thing
    r = sim(dict(sim().pos, cup=tuple(config.ZONES["tray"])))
    st, _ = go(r, "put the blue cube in the tray")
    assert r.moves == [] and any("full" in t for t in st.texts())


def test_never_loops_forever_if_someone_keeps_undoing_work():
    def on_move(robot, obj, src, dst):
        robot.pos["marker"] = tuple(config.SLOTS["slot4"])
    r = sim(on_move=on_move)
    st, _ = go(r, "Give me something to write with")
    assert len(r.moves) <= 3 * len(config.OBJECTS) + 5
    assert any("stopped" in t or "help" in t for t in st.texts())


# ---------------- status for the dashboard ----------------

def test_status_file_has_everything_a_dashboard_needs():
    r = sim(place_error=(4.0, 3.0))
    go(r, "put the red cube next to the cup")
    data = json.load(open(config.STATUS_PATH))
    for key in ["requests", "scene", "goals", "remaining", "done", "current", "state", "log",
                "errors_mm", "last_error_mm"]:
        assert key in data
    assert data["last_error_mm"] == 5.0 and data["state"] == "idle"
    assert math.dist(data["scene"]["red_cube"], r.pos["red_cube"]) < 1e-6


def test_status_file_problems_never_stop_the_robot(monkeypatch):
    monkeypatch.setattr(config, "STATUS_PATH", "/nonexistent-dir/status.json")
    r = sim()
    go(r, "Give me something to write with")
    assert r.spot("marker") == "to_me"


def test_next_moves_an_anchor_before_what_depends_on_it():
    goals = {"red_cube": "next_to:cup", "marker": "on:red_cube", "cup": "tray"}
    assert executor._next(goals, ["red_cube", "marker", "cup"]) == "cup"
    assert executor._next(goals, ["red_cube", "marker"]) == "red_cube"
    # a cycle still makes progress
    cyc = {"red_cube": "next_to:cup", "cup": "next_to:red_cube"}
    assert executor._next(cyc, ["red_cube", "cup"]) == "red_cube"


def _unplannable(r, bad, holding):
    from robot import MoveError
    real = r.move_object

    async def move_object(obj, src, dst, lift=0.0, **kw):
        if obj == bad:
            raise MoveError(f"no safe path to there, so I put the {obj} back", holding=holding)
        return await real(obj, src, dst, lift)
    r.move_object = move_object
    return r


def test_unplannable_move_is_reported_and_the_rest_still_happens():
    """EXP-006 req 2 (Fri 3:39): a planner refusal mid-carry crashed the whole app."""
    r = _unplannable(sim(), "cup", holding=False)
    st, _ = go(r, "Put the cup in the tray, everything else to the left")
    assert any("safe path" in t for t in st.texts())
    for o in ("red_cube", "blue_cube", "marker"):
        assert places.done(o, "left", r.seen())


def test_still_holding_after_a_failed_move_stops_and_asks_for_help():
    r = _unplannable(sim(), "cup", holding=True)
    st, _ = go(r, "Put the cup in the tray")
    assert any("Please take it from me" in t for t in st.texts())
