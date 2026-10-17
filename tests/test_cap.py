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
