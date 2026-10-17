"""Thermometer (Sat): point the IR thermometer at the ball. Its button is one click, and held down the light
goes off after ~2 s, so either click only at the ball ("press") or swing there fast after the grip ("fast")."""
import asyncio

import pytest

import config
import taught
import voice
from conftest import run
from robot import RealRobot
from test_drawer import FakeArm, names_of, teach_all


class FakeGripper:
    def __init__(self, log):
        self.log = log

    async def open(self, **kw):
        self.log.append("release")

    async def grab(self, **kw):
        self.log.append("grab")
        return True

    async def do_command(self, cmd, **kw):
        self.log.append(("set", cmd["set"]))
        return {}


async def _no_sleep(*a, **k):
    return None


def setup(monkeypatch, mode, hold=420, press=380):
    monkeypatch.setattr(asyncio, "sleep", _no_sleep)
    monkeypatch.setattr(config, "THERM_MODE", mode)
    monkeypatch.setattr(config, "THERM_HOLD_POS", hold)
    monkeypatch.setattr(config, "THERM_PRESS_POS", press)
    teach_all()
    for i, n in enumerate(["therm_above", "therm_grip", "therm_aim"]):
        taught.save(n, [20.0 + i] * 6, {"x": 100.0, "y": 300.0, "z": 150.0})
    log = []
    arm = FakeArm()
    r = RealRobot(machine=None, arm=arm, gripper=FakeGripper(log), motion=None, camera=None)
    r.retry_delay = 0
    return r, arm, log


def test_press_mode_clicks_only_at_the_ball(monkeypatch):
    r, arm, log = setup(monkeypatch, "press")
    run(r.thermometer())
    assert names_of(arm) == [["therm_above"], ["therm_grip"], ["therm_aim"], ["therm_grip"], ["therm_above", "high"]]
    # holds gently on the way, clicks at the ball, lets the button go, puts it back, lets go of it
    assert log == ["release", ("set", 420), ("set", 380), ("set", 420), "release"]
    assert all(o.max_vel_degs_per_sec == config.DRAWER_MAX_VEL_DEGS for _, o in arm.moves)   # nothing fast


def test_fast_mode_swings_fast_only_from_grip_to_ball(monkeypatch):
    r, arm, log = setup(monkeypatch, "fast")
    run(r.thermometer())
    assert names_of(arm) == [["therm_above"], ["therm_grip"], ["therm_aim"], ["therm_grip"], ["therm_above", "high"]]
    assert log == ["release", "grab", "release"]
    speeds = [o.max_vel_degs_per_sec for _, o in arm.moves]
    assert speeds == [config.DRAWER_MAX_VEL_DEGS] * 2 + [config.THERM_FAST_VEL_DEGS] + [config.DRAWER_MAX_VEL_DEGS] * 2


def test_press_mode_needs_measured_gripper_positions(monkeypatch):
    r, arm, log = setup(monkeypatch, "press", hold=None, press=None)
    with pytest.raises(KeyError, match="THERM_HOLD_POS"):
        run(r.thermometer())
    assert arm.moves == [] and log == []


def test_needs_its_poses_taught(monkeypatch):
    monkeypatch.setattr(asyncio, "sleep", _no_sleep)
    teach_all()
    log = []
    r = RealRobot(machine=None, arm=FakeArm(), gripper=FakeGripper(log), motion=None, camera=None)
    with pytest.raises(KeyError, match="therm_above"):
        run(r.thermometer())


@pytest.mark.parametrize("text", ["can you check my temperature", "I think I have a fever", "I feel hot"])
def test_voice_asks_for_the_thermometer(text):
    assert voice.interpret(text, use_ai=False)["action"] == "thermometer"


def test_press_is_a_short_click_then_back_to_hold():
    """Sat 11 AM: held down the thermometer doesn't measure: press for click_s only, then hold while it reads."""
    teach_all()
    for n in ["therm_above", "therm_grip", "therm_aim"]:
        taught.save(n, [1.0] * 6, {"x": 0, "y": 0, "z": 0})
    seq = taught.therm_sequence(taught.load(), "press", 2.0, 260, 250, 0.5)
    i = seq.index(("move", ["therm_aim"]))
    assert seq[i + 1:i + 3] == [("click", [250, 260, 0.5]), ("wait", [2.0])]
