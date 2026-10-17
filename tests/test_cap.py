"""Bottle cap (Sat 1:35 PM): a person holds the bottle; 3 re-grip twists of the last joint, then lift the cap off."""
import asyncio

import pytest

import config
import taught
from conftest import run
from robot import MoveError, RealRobot
from test_drawer import FakeArm, teach_all
from test_thermometer import FakeGripper, _no_sleep


def setup(monkeypatch, wrist=100.0):
    monkeypatch.setattr(asyncio, "sleep", _no_sleep)
    teach_all()
    taught.save("cap_above", [5.0, 5, 5, 5, 5, wrist], {"x": 0, "y": 0, "z": 200})
    taught.save("cap_grip", [6.0, 6, 6, 6, 6, wrist], {"x": 0, "y": 0, "z": 150})
    log = []
    seen = []
    arm = FakeArm()

    async def through(positions, options=None, **kw):
        seen.append([list(p.values) for p in positions])
    arm.move_through_joint_positions = through
    r = RealRobot(machine=None, arm=arm, gripper=FakeGripper(log), motion=None, camera=None)
    r.retry_delay = 0
    return r, seen, log


def test_three_regrip_twists_then_lift_without_turning_back(monkeypatch):
    r, seen, log = setup(monkeypatch)
    run(r.cap())
    wrist = [s[-1][5] for s in seen]
    # above, down, then (twist, back) x2, twist, lift still turned
    assert wrist == [100.0, 100.0, -20.0, 100.0, -20.0, 100.0, -20.0, -20.0]
    assert seen[-1][0][:5] == [5.0] * 5                              # the lift ends at cap_above's arm joints
    assert log == ["release", "grab", "release", "grab", "release", "grab"]   # ends holding the cap


def test_never_turns_back_while_holding_the_cap(monkeypatch):
    r, seen, log = setup(monkeypatch)
    seq = taught.cap_sequence(taught.load(), 3, -120.0)
    holding = False
    for kind, args in seq:
        holding = True if kind == "grab" else False if kind == "release" else holding
        if kind == "turned" and args[1] == 0.0:
            assert not holding


def test_refuses_to_twist_past_the_wrist_limit(monkeypatch):
    r, seen, log = setup(monkeypatch, wrist=-300.0)                   # -300 - 120 = -420: past -360
    with pytest.raises(MoveError, match="wrist limit"):
        run(r.cap())


def test_needs_its_poses():
    teach_all()
    with pytest.raises(KeyError, match="cap_above"):
        taught.cap_sequence(taught.load())


def _teach_pour():
    for i, n in enumerate(["bottle_approach", "bottle_grip", "bottle_lift", "pour"]):
        taught.save(n, [30.0 + i] * 6, {"x": 400.0, "y": 80.0, "z": 260.0, "o_x": 1.0, "o_y": 0.0, "o_z": 0.0,
                                        "theta": 0.0})


def test_pour_tilts_slowly_straightens_and_stands_the_bottle_back(monkeypatch):
    """Sat 2:10 PM (Vaibhav's poses): hold at 550, lift beside the slot 1 cup, slow tilt, slow upright, back on slot 2."""
    r, seen, log = setup(monkeypatch)
    _teach_pour()
    speeds = []
    arm_through = r.arm.move_through_joint_positions

    async def through(positions, options=None, **kw):
        speeds.append(options.max_vel_degs_per_sec)
        return await arm_through(positions, options, **kw)
    r.arm.move_through_joint_positions = through
    r.arm.at = taught.load()["bottle_approach"]["joints"]            # the planner brought it there
    d = taught.load()
    run(r.replay(d, taught.pour_sequence(d, 3.0, 550), start="bottle_approach"))
    by = {tuple(v["joints"]): n for n, v in d.items()}
    assert [[by[tuple(j)] for j in s] for s in seen] == [["bottle_grip"], ["bottle_lift"], ["pour"], ["bottle_lift"],
                                                         ["bottle_grip"], ["bottle_approach"]]
    assert speeds[2] == speeds[3] == config.POUR_VEL_DEGS            # the tilt in and the straightening are slow
    assert log == [("set", 550), "release"]                          # holds at 550, lets go only back on slot 2


def test_pour_stops_if_the_planner_arrives_in_another_arm_shape(monkeypatch):
    """'high' -> bottle_approach is a 95 deg jump: planned, and the blind replay only continues from the taught shape."""
    r, seen, log = setup(monkeypatch)
    _teach_pour()

    class Motion:
        async def move(self, **kw):
            return True
    r.motion = Motion()
    r.arm.at = [90.0] * 6                                             # somewhere else entirely
    with pytest.raises(MoveError, match="different arm shape"):
        run(r.pour())
    assert seen == []                                                 # no blind replay


def test_water_runs_cap_drop_then_pour():
    import app
    from conftest import sim
    robot = sim()
    assert run(app.run_command(robot, ["water"])) is True
    assert robot.drawer_moves == ["cap", "drop_cap", "to_high", "pour"]


def test_pour_through_a_taught_waypoint_never_uses_the_planner(monkeypatch):
    """Sat 2:19 PM: the planner refused bottle_approach (configured wall-front box). With bottle_via taught, the arm
    replays high -> bottle_via -> bottle_approach and back the same way."""
    r, seen, log = setup(monkeypatch)
    _teach_pour()
    taught.save("bottle_via", [20.0] * 6, {"x": 0, "y": 0, "z": 300})
    r.motion = None                                                   # any planner call would crash
    run(r.pour())
    d = taught.load()
    by = {tuple(v["joints"]): n for n, v in d.items()}
    path = [[by[tuple(j)] for j in s] for s in seen]
    assert path[0] == ["bottle_via", "bottle_approach"] and path[-1] == ["bottle_via", "high"]
    assert ["pour"] in path


def test_water_goes_from_the_cup_straight_to_the_bottle_when_taught():
    """Sat 2:35 PM: no detour to 'high' between dropping the cap and picking up the bottle."""
    import app
    from conftest import sim
    teach_all()
    taught.save("cap_drop", [7.0] * 6, {"x": 353.0, "y": 42.0, "z": 154.0, "theta": 170.0})
    taught.save("bottle_via", [8.0] * 6, {"x": 254.0, "y": 129.0, "z": 388.0})
    robot = sim()
    assert run(app.run_command(robot, ["water"])) is True
    assert robot.drawer_moves == ["cap", "drop_cap", "pour"]


def test_pour_puts_the_bottle_down_from_straight_above_its_spot(monkeypatch):
    """Sat 2:45 PM: lift -> grip swung the bottle's bottom into the table first. With bottle_above taught, it goes
    up off the spot, over, and straight back down onto it (both ways)."""
    _teach_pour()
    taught.save("bottle_above", [40.0] * 6, {"x": 308.0, "y": 102.0, "z": 180.0})
    seq = taught.pour_sequence(taught.load(), 3.0, 550)
    moves = [args for kind, args in seq if kind == "move"]
    assert ["bottle_above", "bottle_lift"] in moves and ["bottle_above", "bottle_grip"] in moves
