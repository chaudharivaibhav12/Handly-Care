"""The app: feed files, setup commands (test, objects, calibrate, pick), and the demo end to end."""
import json
import math
import os
import re
import subprocess
import sys

import app
import calibration
import config
import executor
import places
from conftest import run, sim
from robot import SimRobot

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def test_read_feed_parses_delays_and_skips_comments(tmp_path):
    f = tmp_path / "feed.txt"
    f.write_text("# comment\n\n0|hello\n1.5| there | with bar\n")
    assert app.read_feed(str(f)) == [(0.0, "hello"), (1.5, "there | with bar")]


def test_shipped_demo_file_parses():
    texts = [t for _, t in app.read_feed(os.path.join(ROOT, "demo.txt"))]
    assert texts[0] == "Give me something to write with"
    assert any("on top of" in t for t in texts) and any("instead" in t for t in texts)


def test_sim_layout_starts_off_the_taught_spots_and_out_of_zones():
    r = SimRobot(app.SIM_LAYOUT)
    for o in config.OBJECTS:
        assert r.spot(o) is None and places.reachable(*r.pos[o])
        assert all(math.dist(r.pos[o], z) > config.ZONE_RADIUS_MM for z in config.ZONES.values())


def test_check_command_passes_on_shipped_config():
    assert run(app.main(["check"])) is True


# ---------------- Milestone commands ----------------

def test_test_command_rejects_bad_names():
    r = sim()
    assert run(app.run_command(r, ["test", "laptop", "slot1", "tray"])) is False
    assert r.moves == []


def test_test_command_moves_between_taught_spots():
    r = sim()
    assert run(app.run_command(r, ["test", "red_cube", "slot1", "tray"])) is True
    assert r.spot("red_cube") == "tray"


def test_objects_command_lists_what_it_sees(capsys):
    r = sim(dict(app.SIM_LAYOUT, marker=(720.0, 0.0)))
    assert run(app.run_command(r, ["objects"])) is True
    out = capsys.readouterr().out
    assert "cup" in out and "x=   420" in out and "OUT OF REACH" in out


def test_pick_command_finds_and_delivers(capsys):
    r = sim(app.SIM_LAYOUT)
    assert run(app.run_command(r, ["pick", "cup", "tray"])) is True
    assert places.done("cup", "tray", r.seen())
    assert "mm from target" in capsys.readouterr().out


def test_pick_command_refuses_unknown_or_out_of_reach():
    r = sim(dict(app.SIM_LAYOUT, cup=(720.0, 0.0)))
    assert run(app.run_command(r, ["pick", "cup", "tray"])) is False
    assert run(app.run_command(r, ["pick", "laptop", "tray"])) is False
    assert r.moves == []


class OffsetCameraSim(SimRobot):
    """A sim whose 'camera' reports every object shifted, like a mis-calibrated wrist camera."""
    def __init__(self, layout, offset):
        super().__init__(layout)
        self.offset = offset

    def seen(self):
        return {o: (None if p is None else (p[0] + self.offset[0], p[1] + self.offset[1]))
                for o, p in super().seen().items()}


def test_calibrate_command_measures_and_saves_the_correction(capsys):
    layout = {"red_cube": "slot1", "blue_cube": "slot2", "cup": "slot3", "marker": "slot4"}
    r = OffsetCameraSim(layout, offset=(-20.0, 15.0))
    ok = run(app.run_command(r, ["calibrate", "red_cube=slot1", "blue_cube=slot2", "cup=slot3", "marker=slot4"]))
    assert ok
    cal = json.load(open(config.CALIBRATION_PATH))
    assert cal["before_mm"] == 25.0 and cal["after_mm"] < 0.5
    assert math.dist(calibration.apply(cal, 430, 15), (450, 0)) < 0.5   # camera said (430, 15)
    out = capsys.readouterr().out
    assert "off by 25 mm" in out and "25.0 mm -> 0.0 mm" in out


def test_calibrate_command_rejects_bad_input():
    r = sim()
    assert run(app.run_command(r, ["calibrate", "laptop=slot1"])) is False
    assert run(app.run_command(r, ["calibrate"])) is False
    assert not os.path.exists(config.CALIBRATION_PATH)


# ---------------- the demo ----------------

def test_demo_in_process_with_a_change_of_mind():
    """The demo script, deterministic: each sim move takes 0.1 s and the change of
    mind arrives while the cup is moving."""
    r = sim(app.SIM_LAYOUT, move_seconds=0.1)
    feed = [(0, "Give me something to write with"),
            (0.3, "Put the cup in the tray, and the red cube on top of the blue cube"),
            (0.05, "Actually, put the red cube next to the cup instead"),
            (0.8, "Pick up my laptop")]
    status = run(app.interactive(r, feed=feed, status=executor.Status(echo=False)))
    s = r.seen()
    assert places.done("marker", "to_me", s) and places.done("cup", "tray", s)
    assert places.done("red_cube", "next_to:cup", s) and s["red_cube"] != s["blue_cube"]
    assert "replan" in status.kinds()
    assert any("can't do that" in t for t in status.texts())


def test_cli_demo_end_to_end():
    """Runs exactly what you'll rehearse: python app.py --sim --seed 3 --feed demo.txt"""
    # ANTHROPIC_API_KEY="" (not absent): load_dotenv() won't override it with the real key from .env,
    # so the test stays offline. UTF-8: the app prints emoji; Windows' default decoder gave stdout=None.
    env = dict(os.environ, ANTHROPIC_API_KEY="", OPENROUTER_API_KEY="", PYTHONIOENCODING="utf-8")
    out = subprocess.run([sys.executable, "app.py", "--sim", "--seed", "3", "--feed", "demo.txt"],
                         cwd=ROOT, env=env, capture_output=True, text=True, encoding="utf-8",
                         errors="replace", timeout=60)
    for leftover in ("status.json",):
        p = os.path.join(ROOT, leftover)
        if os.path.exists(p):
            os.remove(p)
    assert out.returncode == 0, out.stderr
    final = out.stdout.strip().splitlines()[-1]
    assert re.search(r"cup \([^)]*\) \[tray\]", final)
    assert re.search(r"marker \([^)]*\) \[to_me\]", final)
    assert "re-planning" in out.stdout and "can't do that safely" in out.stdout
    assert "mm from the target" in out.stdout


# ---------------- trial log for the test pilot ----------------

def test_pick_and_test_runs_are_logged_and_summarized(capsys):
    import trials
    r = sim(app.SIM_LAYOUT, place_error=(4.0, 3.0))
    for _ in range(5):
        run(app.run_command(r, ["pick", "cup", "tray"]))
        r.pos["cup"] = (420.0, 200.0)                  # pilot puts it back
    run(app.run_command(r, ["test", "red_cube", "slot1", "tray"]))  # red_cube isn't on slot1: a miss
    rows = trials.read()
    assert len(rows) == 6 and rows[0]["command"] == "pick" and rows[0]["error_mm"] == "5.0"
    s = trials.summary()
    assert s["cup"] == {"tries": 5, "grabbed": 5, "on_target": 5, "mean_mm": 5.0, "max_mm": 5.0}
    assert s["red_cube"]["grabbed"] == 0
    assert run(app.run_command(r, ["stats"])) is True
    out = capsys.readouterr().out
    assert "demo-ready" in out and "need 5+ tries" in out


def test_off_target_trials_count_against_readiness():
    import trials
    rows = [{"object": "cup", "grabbed": "1", "error_mm": e} for e in ["5", "8", "45", "50", "6"]]
    s = trials.summary(rows, ok_mm=20)["cup"]
    assert s["on_target"] == 3 and s["max_mm"] == 50.0


def test_stats_with_no_trials():
    import trials
    assert "no trials yet" in trials.report()


# ---------------- Windows console safety ----------------

def test_feed_always_ends_even_if_printing_fails(monkeypatch):
    """On Windows a cp1252 console can't print emoji; that used to hang the demo forever."""
    import asyncio
    import builtins

    def broken_print(*a, **k):
        raise UnicodeEncodeError("charmap", "x", 0, 1, "can't encode")
    monkeypatch.setattr(builtins, "print", broken_print)

    async def main():
        q = asyncio.Queue()
        await app.start_feed(asyncio.get_running_loop(), q, [(0, "hello"), (0, "there")])
        return [q.get_nowait() for _ in range(q.qsize())]
    assert run(main()) == ["hello", "there", None]


def test_cli_demo_survives_a_non_utf8_console():
    env = dict(os.environ, ANTHROPIC_API_KEY="", OPENROUTER_API_KEY="")   # never the real keys from .env
    env["PYTHONIOENCODING"] = "cp1252"          # what Windows gives a captured process
    out = subprocess.run([sys.executable, "app.py", "--sim", "--seed", "3", "--feed", "demo.txt"],
                         cwd=ROOT, env=env, capture_output=True, timeout=60)
    p = os.path.join(ROOT, "status.json")
    if os.path.exists(p):
        os.remove(p)
    assert out.returncode == 0, out.stderr.decode("utf-8", "replace")
    assert b"can't do that safely" in out.stdout


def test_calibrate_auto_pairs_each_slot_with_the_nearest_object():
    """`calibrate auto` (anything mode: objects have no fixed names)."""
    import app
    slots = {"slot1": (450, -150), "slot2": (500, -60), "slot3": (450, 100)}
    raw = {"red_object": (455, -128), "cup": (506, -37), "far_thing": (200, 300), "gone": None}
    pairs = app.auto_pairs(raw, slots, radius=80)
    assert pairs == [((455, -128), (450, -150)), ((506, -37), (500, -60))]   # slot3: nothing near


def test_object_on_a_spot_is_the_nearest_one_within_reach():
    import app
    scene = {"brown_object": (193, 163), "cup": (286, 67), "gone": None}
    assert app.object_on(scene, (197, 164)) == "brown_object"
    assert app.object_on(scene, (450, 300)) is None


def test_trial_note_survives_no_keyboard(monkeypatch):
    """Fri 5:25 PM: a drawer run worked, then input() hit EOF and the trial was never logged."""
    import app
    monkeypatch.setattr(app.sys.stdin, "isatty", lambda: True, raising=False)

    def eof(prompt=""):
        raise EOFError
    monkeypatch.setattr("builtins.input", eof)
    assert app.ask_note("?") == ""


def test_a_tissue_run_that_sees_no_box_goes_back_to_high(monkeypatch):
    """Sat 12 PM: a failed tissue left the arm at bird's-eye, so the demo's medicine was refused (60 deg from 'high')."""
    robot = sim()                                   # the sim table has no tissue box
    monkeypatch.setattr(app.perception, "find", lambda r: _scene(r))
    assert run(app.run_command(robot, ["tissue"])) is False
    assert robot.drawer_moves[-1] == "to_high"


async def _scene(robot):
    return robot.seen()
