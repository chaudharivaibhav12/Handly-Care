"""A log of every test move, so the teammate running the arm builds real numbers.

`test`, `pick` and `try` append a row to trials.csv; `python app.py stats` summarizes:
success rate and placement error per object. These are the numbers that tell you
whether something is demo-ready (4/5 or better), and they make a pitch line:
"placed within N mm on average over M trials".
"""
import csv
import os
import time

import config

FIELDS = ["time", "command", "object", "from", "to", "grabbed", "error_mm", "note"]


def log(command, obj, src, dst, grabbed, error_mm=None, note=""):
    path = config.TRIALS_PATH
    new = not os.path.exists(path)
    try:
        with open(path, "a", newline="") as f:
            w = csv.DictWriter(f, fieldnames=FIELDS)
            if new:
                w.writeheader()
            w.writerow({
                "time": time.strftime("%H:%M:%S"), "command": command, "object": obj,
                "from": src, "to": dst, "grabbed": int(bool(grabbed)),
                "error_mm": "" if error_mm is None else round(error_mm, 1), "note": note,
            })
    except OSError:
        pass   # a log file must never stop the robot


def read():
    if not os.path.exists(config.TRIALS_PATH):
        return []
    with open(config.TRIALS_PATH, newline="") as f:
        return list(csv.DictReader(f))


def summary(rows=None, ok_mm=None):
    """{object: {"tries", "grabbed", "on_target", "mean_mm", "max_mm"}}.
    on_target: grabbed AND (error unknown for `test`, or error <= ok_mm)."""
    rows = read() if rows is None else rows
    ok_mm = config.PLACE_TOLERANCE_MM if ok_mm is None else ok_mm
    out = {}
    for r in rows:
        s = out.setdefault(r["object"], {"tries": 0, "grabbed": 0, "on_target": 0, "errors": []})
        s["tries"] += 1
        grabbed = r["grabbed"] == "1"
        s["grabbed"] += grabbed
        err = float(r["error_mm"]) if r["error_mm"] not in ("", None) else None
        if err is not None:
            s["errors"].append(err)
        if grabbed and (err is None or err <= ok_mm):
            s["on_target"] += 1
    for s in out.values():
        e = s.pop("errors")
        s["mean_mm"] = round(sum(e) / len(e), 1) if e else None
        s["max_mm"] = round(max(e), 1) if e else None
    return out


def report():
    stats = summary()
    if not stats:
        return "no trials yet: run `python app.py test ...` or `python app.py pick ...`"
    lines = [f"{'object':10s} {'tries':>5s} {'grabbed':>8s} {'on target':>10s} {'mean mm':>8s} {'max mm':>7s}  verdict"]
    for obj, s in sorted(stats.items()):
        rate = s["on_target"] / s["tries"]
        verdict = "demo-ready" if s["tries"] >= 5 and rate >= 0.8 else ("needs work" if s["tries"] >= 5 else "need 5+ tries")
        mean = "-" if s["mean_mm"] is None else f"{s['mean_mm']:.1f}"
        mx = "-" if s["max_mm"] is None else f"{s['max_mm']:.1f}"
        lines.append(f"{obj:10s} {s['tries']:5d} {s['grabbed']:8d} {s['on_target']:10d} {mean:>8s} {mx:>7s}  {verdict}")
    return "\n".join(lines)
