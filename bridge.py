"""Bridge: the Handly Care dashboard (handly-care/) drives Sidekick's tested routines.

    python bridge.py            # real arm  -> open http://localhost:8765
    python bridge.py --sim      # simulated arm, for trying the UI

The page asks for a routine by name; this runs exactly `python app.py <routine>` (one at a time) and reports
its progress. No Viam keys in the browser, and the browser can only start the routines listed in ROUTINES.
Stop = Ctrl+Break to the run, so app.py stops the arm on its own connection (the hardware E-stop stays primary).
"""
import argparse
import json
import os
import re
import signal
import subprocess
import sys
import threading
import time
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

ANSI = re.compile(r"\x1b\[[0-9;]*m")   # Viam log colors
ROOT = os.path.dirname(os.path.abspath(__file__))
WEB = os.path.join(ROOT, "handly-care")
PORT = 8765

# name -> (app.py args, typical seconds on the real arm, [(output text that ends a phase, next phase label)])
ROUTINES = {
    "medicine": (["medicine"], 62, [
        ("", "Opening the drawer"),
        ("retreat -> high", "Taking out the pill box"),
        ("pill_lift", "Handing it to the patient"),
        ("handoff:", "Moving back"),
        ("high -> retreat", "Closing the drawer"),
    ]),
    "tissue": (["tissue"], 74, [
        ("", "Looking for the tissue box"),
        ("tissue_box at", "Pulling one tissue out"),
        ("handing over", "Handing it over"),
        ("pill_above -> high", "Returning to the start pose"),
        ("joint", "Returning to the start pose"),
    ]),
    "to_high": (["to_high"], 30, [("", "Moving to the start pose")]),
    "cap": (["cap"], 60, [
        ("", "Reaching for the cap"),
        ("cap_grip:", "Unscrewing the cap"),
        ("cap_above wrist", "Lifting the cap off"),
        ("cap dropped", "Returning to the start pose"),
    ]),
    "pour": (["pour"], 60, [
        ("", "Picking up the bottle"),
        ("bottle_lift:", "Pouring the water"),
        ("slow bottle_lift", "Putting the bottle back"),
    ]),
    "water": (["water"], 120, [
        ("", "Reaching for the cap"),
        ("cap_grip:", "Unscrewing the cap"),
        ("cap_above wrist", "Lifting the cap off"),
        ("cap dropped", "Returning to the start pose"),
        ("bottle_grip:", "Picking up the bottle"),
        ("bottle_lift:", "Pouring the water"),
        ("slow bottle_lift", "Putting the bottle back"),
    ]),
    "thermometer": (["thermometer"], 35, [
        ("", "Picking up the thermometer"),
        ("therm_aim", "Measuring"),
        ("therm_above -> high", "Putting it back"),
    ]),
}


class Runner:
    """At most one app.py routine at a time; thread-safe status for the page."""

    def __init__(self, sim=False, routines=None, command=None, log_dir=None):
        self.sim = sim
        self.log_dir = log_dir      # every run's full output saved here (Sat 12 PM: a failed tissue's reason was lost)
        self.full = []
        self.routines = routines or ROUTINES
        self.command = command or self._app_command
        self.lock = threading.Lock()
        self.proc = None
        self.run_id = 0
        self.task = ""
        self.step = ""
        self.started = 0.0
        self.lines = []
        self.result = None      # {"id", "task", "ok", "message", "seconds"} of the last finished run
        self.stopping = False

    def _app_command(self, args):
        return [sys.executable, "-u", os.path.join(ROOT, "app.py"), *args] + (["--sim"] if self.sim else [])

    @property
    def busy(self):
        return self.proc is not None and self.proc.poll() is None

    def start(self, name):
        """Start a routine -> run id. ValueError for an unknown routine, RuntimeError if one is running."""
        if name not in self.routines:
            raise ValueError(f"unknown routine '{name}' (allowed: {', '.join(self.routines)})")
        with self.lock:
            if self.busy or (self.proc is not None and self.result is None):
                raise RuntimeError(f"the arm is busy with '{self.task}'")
            args, _, phases = self.routines[name]
            flags = getattr(subprocess, "CREATE_NEW_PROCESS_GROUP", 0)     # so Stop can send Ctrl+Break
            env = dict(os.environ, PYTHONIOENCODING="utf-8", SIDEKICK_NO_PROMPT="1")   # no y/n question: nobody can answer it
            self.proc = subprocess.Popen(self.command(args), cwd=ROOT, stdin=subprocess.DEVNULL,
                                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                                         encoding="utf-8", errors="replace", creationflags=flags, env=env)
            self.run_id += 1
            self.task, self.step, self.started = name, phases[0][1], time.time()
            self.lines, self.full, self.result, self.stopping = [], [], None, False
            threading.Thread(target=self._read, args=(self.proc, self.run_id, phases), daemon=True).start()
            return self.run_id

    def _read(self, proc, run_id, phases):
        for raw in proc.stdout:
            line = ANSI.sub("", raw).rstrip()
            if not line.strip():
                continue
            with self.lock:
                self.lines = (self.lines + [line])[-40:]
                self.full.append(line)
                self.step = next_step(self.step, line, phases)
        code = proc.wait()
        with self.lock:
            secs = round(time.time() - self.started)
            ok = code == 0 and not self.stopping
            if self.stopping:
                msg = "Stopped from the dashboard"
            elif ok:
                msg = "Done"
            else:
                msg = last_reason(self.lines) or f"app.py exited with code {code}"
            self.result = {"id": run_id, "task": self.task, "ok": ok, "message": msg, "seconds": secs}
            if self.log_dir:
                try:
                    os.makedirs(self.log_dir, exist_ok=True)
                    name = time.strftime("%H%M%S", time.localtime(self.started)) + f"-{self.task}-{'ok' if ok else 'FAIL'}.txt"
                    with open(os.path.join(self.log_dir, name), "w", encoding="utf-8") as f:
                        f.write("\n".join(self.full + [f"[bridge] exit {code}: {msg} ({secs} s)"]) + "\n")
                except OSError:
                    pass

    def stop(self, grace=6.0):
        """Ask the run to stop the arm (Ctrl+Break -> app.py calls robot.stop()); kill it if it doesn't exit."""
        with self.lock:
            proc = self.proc
            if proc is None or proc.poll() is not None:
                return False
            self.stopping = True
        try:
            proc.send_signal(signal.CTRL_BREAK_EVENT if os.name == "nt" else signal.SIGINT)
        except (OSError, ValueError):
            pass
        try:
            proc.wait(grace)
        except subprocess.TimeoutExpired:
            proc.kill()
        return True

    def status(self):
        with self.lock:
            busy = self.busy or (self.proc is not None and self.result is None)
            expected = self.routines[self.task][1] if self.task else 1
            elapsed = time.time() - self.started if busy else 0
            return {"connected": True, "sim": self.sim, "busy": busy, "task": self.task if busy else "",
                    "run": self.run_id, "step": self.step if busy else "",
                    "progress": round(min(0.95, elapsed / expected), 2) if busy else 0,
                    "elapsed": round(elapsed), "lines": self.lines[-12:], "result": self.result,
                    "routines": list(self.routines)}


def next_step(current, line, phases):
    """The phase label after this output line (phases = [(text that ends the previous phase, label), ...])."""
    step = current
    for marker, label in phases[1:]:
        if marker in line:
            step = label
    return step


def last_reason(lines):
    """The most useful line of a failed run: app.py's own 'stopped:'/'can't' message, else the last line."""
    for line in reversed(lines):
        s = line.strip()
        if s.startswith(("stopped:", "can't", "couldn't", "I can't", "teach ")) or "Error" in s:
            return s
    return lines[-1].strip() if lines else ""


def make_handler(runner, interpret=None):
    class Handler(SimpleHTTPRequestHandler):
        def __init__(self, *a, **kw):
            super().__init__(*a, directory=WEB, **kw)

        def log_message(self, fmt, *args):     # quiet: the page polls twice a second
            if not self.path.startswith("/api/status"):
                super().log_message(fmt, *args)

        def _json(self, code, body):
            data = json.dumps(body).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def end_headers(self):
            if not self.path.startswith("/api/"):
                self.send_header("Cache-Control", "no-cache")
            super().end_headers()

        def do_GET(self):
            if self.path.startswith("/api/status"):
                return self._json(200, runner.status())
            if self.path in ("/", ""):
                self.path = "/index.html"
            elif self.path == "/favicon.svg":
                self.path = "/public/favicon.svg"
            return super().do_GET()

        def do_POST(self):
            # Same-origin only: a page from another site can't start the arm.
            origin = self.headers.get("Origin")
            if origin and origin not in (f"http://localhost:{self.server.server_port}",
                                         f"http://127.0.0.1:{self.server.server_port}"):
                return self._json(403, {"error": "wrong origin"})
            try:
                body = json.loads(self.rfile.read(int(self.headers.get("Content-Length") or 0)) or b"{}")
            except ValueError:
                return self._json(400, {"error": "bad JSON"})
            if self.path == "/api/run":
                try:
                    run = runner.start(str(body.get("task", "")))
                except ValueError as e:
                    return self._json(400, {"error": str(e)})
                except RuntimeError as e:
                    return self._json(409, {"error": str(e)})
                print(f"[bridge] started {body.get('task')} (run {run})")
                return self._json(200, {"run": run})
            if self.path == "/api/stop":
                stopped = runner.stop()
                print("[bridge] STOP from the dashboard" + ("" if stopped else " (nothing running)"))
                return self._json(200, {"stopped": stopped})
            if self.path == "/api/say":
                return self._json(200, say(runner, str(body.get("text", "")), interpret))
            return self._json(404, {"error": "not found"})
    return Handler


def say(runner, text, interpret=None):
    """A spoken request -> what the page should do. "stop" stops right here, even mid-run; anything else is
    only a suggestion: the page shows it and the nurse clicks Start (the arm never moves on voice alone)."""
    import voice
    interpret = interpret or voice.interpret
    if voice.is_stop(text):                  # before the AI and before the busy check: always instant
        stopped = runner.stop()
        print(f"[bridge] voice STOP: {text!r}" + ("" if stopped else " (nothing running)"))
        return {"action": "stop", "say": "Stopping." if stopped else "Okay, I'm not moving.", "source": "local",
                "text": text}
    st = runner.status()
    if st["busy"]:
        return {"action": "busy", "say": f"One moment, I'm still busy with {st['task'].replace('_', ' ')}.",
                "source": "local", "text": text}
    out = dict(interpret(text), text=text)
    print(f"[bridge] voice {text!r} -> {out['action']} ({out['source']})")
    return out


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("--sim", action="store_true", help="run app.py --sim (no arm)")
    p.add_argument("--port", type=int, default=PORT)
    args = p.parse_args(argv)
    try:                                     # AI key for voice requests (app.py loads it itself)
        from dotenv import load_dotenv
        load_dotenv(os.path.join(ROOT, ".env"))
    except ImportError:
        pass
    runner = Runner(sim=args.sim, log_dir=None if args.sim else os.path.join(ROOT, "results", "bridge-runs"))
    server = ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(runner))   # this laptop only
    print(f"Handly Care bridge{' (SIMULATED arm)' if args.sim else ''}: open http://localhost:{args.port}  (Ctrl+C to quit)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nbridge: stopping")
        runner.stop()
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
