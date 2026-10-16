"""Shared test setup. Run all tests with:  python -m pytest -q"""
import asyncio
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import config  # noqa: E402
from robot import SimRobot  # noqa: E402

# The geometry the tests were written for (the pre-event placeholder table). Tests pin it so
# teaching real positions into config.py never breaks them; real values are checked by `app.py check`.
TEST_SLOTS = {"slot1": (450, -150), "slot2": (500, -60), "slot3": (450, 100), "slot4": (350, 0)}
TEST_ZONES = {"tray": (300, -300), "left": (300, 300), "right": (550, -300), "to_me": (600, 0)}

# Starting layout, by taught spot name (readable). Values may also be (x, y).
START = {"red_cube": "slot1", "blue_cube": "slot2", "cup": "slot3", "marker": "slot4"}


@pytest.fixture(autouse=True)
def isolate(monkeypatch, tmp_path):
    # Never call the real AI in tests, never litter files in the repo, never wait.
    monkeypatch.delenv("ANTHROPIC_API_KEY", raising=False)
    monkeypatch.delenv("OPENROUTER_API_KEY", raising=False)
    monkeypatch.setattr(config, "STATUS_PATH", str(tmp_path / "status.json"))
    monkeypatch.setattr(config, "CALIBRATION_PATH", str(tmp_path / "calibration.json"))
    monkeypatch.setattr(config, "TRIALS_PATH", str(tmp_path / "trials.csv"))
    monkeypatch.setattr(config, "DRAWER_JOINTS_PATH", str(tmp_path / "drawer_joints.json"))
    monkeypatch.setattr(config, "SETTLE_S", 0)
    monkeypatch.setattr(config, "FRAME_GAP_S", 0)
    monkeypatch.setattr(config, "MODE", "vision")
    monkeypatch.setattr(config, "USER_DIRECTION", "+x")
    monkeypatch.setattr(config, "SLOTS", dict(TEST_SLOTS))
    monkeypatch.setattr(config, "ZONES", dict(TEST_ZONES))
    monkeypatch.setattr(config, "ALL_SPOTS", {**TEST_SLOTS, **TEST_ZONES})
    monkeypatch.setattr(config, "ZONE_LIFT", {})
    monkeypatch.setattr(config, "ZONE_RADIUS_MM", 90.0)
    monkeypatch.setattr(config, "WORKSPACE", {"x": (200.0, 700.0), "y": (-400.0, 400.0)})
    monkeypatch.setattr(config, "NO_GO", [])
    monkeypatch.chdir(tmp_path)


def run(coro):
    return asyncio.run(coro)


def xy(where):
    """'slot1' -> its taught (x, y); (x, y) -> itself; None -> None."""
    if where is None:
        return None
    return tuple(config.ALL_SPOTS[where]) if isinstance(where, str) else tuple(where)


def scene(**changes):
    """A scene of positions: START, with some objects moved (spot name, (x, y) or None)."""
    s = {o: xy(w) for o, w in START.items()}
    for o, w in changes.items():
        s[o] = xy(w)
    return s


def sim(layout=None, **kw):
    return SimRobot(dict(START if layout is None else layout), **kw)
