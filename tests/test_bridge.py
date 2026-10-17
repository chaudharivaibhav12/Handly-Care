"""bridge.py: the dashboard can start only known routines, one at a time, see progress, and stop them."""
import json
import sys
import threading
import time
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer

import pytest

import bridge

FAKE = {   # stand-in routines: a quick one that prints app.py-like lines, a failing one, a slow one
    "quick": (["quick"], 2, [("", "Starting"), ("tissue_box at", "Pulling")]),
    "fail": (["fail"], 2, [("", "Starting")]),
    "slow": (["slow"], 60, [("", "Waiting")]),
}
SCRIPTS = {
    "quick": "print('  tissue_box at (1, 2)'); print('tissue: done')",
    "fail": "print('  hello'); print('stopped: no safe path to the tissues'); raise SystemExit(1)",
    "slow": "import time; print('waiting', flush=True); time.sleep(30)",
}


def fake_command(args):
    return [sys.executable, "-c", SCRIPTS[args[0]]]


def wait_result(r, timeout=10):
    end = time.time() + timeout
    while time.time() < end:
        s = r.status()
        if s["result"] and not s["busy"]:
            return s
        time.sleep(0.05)
    raise AssertionError("routine never finished")


def test_real_routines_are_only_the_tested_ones():
    assert set(bridge.ROUTINES) == {"medicine", "tissue", "thermometer", "to_high"}
    r = bridge.Runner(sim=True)
    assert r._app_command(["medicine"])[-2:] == ["medicine", "--sim"]


def test_unknown_routine_refused():
    r = bridge.Runner(routines=FAKE, command=fake_command)
    with pytest.raises(ValueError):
        r.start("drawer")
    assert r.status()["busy"] is False


def test_run_reports_steps_and_done():
    r = bridge.Runner(routines=FAKE, command=fake_command)
    run = r.start("quick")
    s = wait_result(r)
    assert s["result"] == {"id": run, "task": "quick", "ok": True, "message": "Done", "seconds": s["result"]["seconds"]}
    assert "tissue: done" in s["lines"]


def test_failure_carries_apps_reason():
    r = bridge.Runner(routines=FAKE, command=fake_command)
    r.start("fail")
    s = wait_result(r)
    assert s["result"]["ok"] is False
    assert s["result"]["message"] == "stopped: no safe path to the tissues"


def test_one_run_at_a_time_and_stop():
    r = bridge.Runner(routines=FAKE, command=fake_command)
    r.start("slow")
    assert r.status()["busy"] is True
    with pytest.raises(RuntimeError):
        r.start("quick")              # never two routines on one arm
    assert r.stop(grace=3) is True
    s = wait_result(r)
    assert s["result"]["ok"] is False and s["result"]["message"] == "Stopped from the dashboard"
    r.start("quick")                  # free again after the stop
    assert wait_result(r)["result"]["ok"] is True


def test_next_step_follows_markers():
    phases = bridge.ROUTINES["medicine"][2]
    step = phases[0][1]
    for line in ["  high -> approach: 5.6 s", "  retreat -> high: 3.8 s", "  pill_lift -> pill_above: 2.9 s",
                 "  handoff: 4.6 s", "  high -> retreat: 3.1 s"]:
        step = bridge.next_step(step, line, phases)
    assert step == "Closing the drawer"


def test_http_run_status_and_origin_check():
    r = bridge.Runner(routines=FAKE, command=fake_command)
    srv = ThreadingHTTPServer(("127.0.0.1", 0), bridge.make_handler(r))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{srv.server_port}"

    def post(path, body, origin=None):
        req = urllib.request.Request(base + path, json.dumps(body).encode(), {"Content-Type": "application/json"})
        if origin:
            req.add_header("Origin", origin)
        try:
            with urllib.request.urlopen(req) as res:
                return res.status, json.load(res)
        except urllib.error.HTTPError as e:
            return e.code, json.load(e)
    try:
        assert post("/api/run", {"task": "quick"}, origin="http://evil.example")[0] == 403
        assert post("/api/run", {"task": "rm -rf"})[0] == 400
        code, body = post("/api/run", {"task": "quick"}, origin=f"http://localhost:{srv.server_port}")
        assert code == 200 and body["run"] == 1
        wait_result(r)
        with urllib.request.urlopen(base + "/api/status") as res:
            assert json.load(res)["result"]["ok"] is True
        with urllib.request.urlopen(base + "/") as res:     # serves the dashboard page
            assert b"Handly" in res.read()
    finally:
        srv.shutdown()
        srv.server_close()


BREAK_SCRIPT = """
import asyncio, app
async def main():
    app.stop_on_break()
    print('ready', flush=True)
    try:
        await asyncio.sleep(30)
    except asyncio.CancelledError:
        print('arm stopped', flush=True)   # app.main() calls robot.stop() here
        raise
asyncio.run(main())
"""


@pytest.mark.skipif(sys.platform != "win32", reason="Ctrl+Break is Windows-only")
def test_dashboard_stop_reaches_apps_stop_path():
    r = bridge.Runner(routines={"run": (["run"], 30, [("", "Running")])},
                      command=lambda args: [sys.executable, "-c", BREAK_SCRIPT])
    r.start("run")
    end = time.time() + 15
    while "ready" not in r.status()["lines"]:
        assert time.time() < end, r.status()["lines"]
        time.sleep(0.05)
    r.stop(grace=5)
    s = wait_result(r)
    assert s["result"]["message"] == "Stopped from the dashboard"
    assert "arm stopped" in r.lines         # cancelled cleanly (robot.stop() path), not killed


def test_sim_runs_never_touch_the_real_trials(tmp_path):
    import os
    import subprocess
    real = os.path.join(bridge.ROOT, "trials.csv")
    before = open(real, "rb").read() if os.path.exists(real) else None
    out = subprocess.run([sys.executable, "app.py", "--sim", "medicine"], cwd=bridge.ROOT,
                         capture_output=True, text=True, timeout=60, stdin=subprocess.DEVNULL)
    assert "medicine: done" in out.stdout, out.stdout + out.stderr
    after = open(real, "rb").read() if os.path.exists(real) else None
    assert after == before


def test_output_is_clean_and_app_never_asks():
    # Fri 8:10 PM: the panel showed Viam's color codes and ended on "did the patient get it? (y/n...)"
    # (NUL stdin looks like a TTY on Windows, so app.py asked a question nobody could answer).
    script = ("import os, app; print('\x1b[33;36mINFO\x1b[0m dial'); "
              "print('note=' + repr(app.ask_note('did the patient get it? ')))")
    r = bridge.Runner(routines={"run": (["run"], 5, [("", "Running")])},
                      command=lambda args: [sys.executable, "-c", script])
    r.start("run")
    s = wait_result(r)
    assert s["lines"] == ["INFO dial", "note=''"]
