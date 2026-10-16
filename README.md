# Sidekick

> New Claude session? Read `CLAUDE.md` first, then `STATUS.md`.

A desk helper you just talk to. It finds objects **anywhere on the table**, places them
**precisely** ("next to the cup", "on top of the blue block", "in the tray"), measures how
far off each placement landed, fixes it, and keeps the arrangement true when you change
your mind or someone moves things. Built on Viam with an xArm/UR arm, a two-finger gripper
and a wrist depth camera.

## Files

| File | Job |
|---|---|
| `config.py` | Every name, position and setting you tune on the day. Start here. |
| `perception.py` | Where is everything, in world mm: 3D segmenter → frame system → camera correction → vote over 3 looks. |
| `calibration.py` | Fixes the camera's 2-9 cm position error using objects on taught spots. |
| `places.py` | Where exactly to put something (zones, left/right/front/behind/next to, stacking) and whether it's there. |
| `robot.py` | `move_object(obj, from_xy, to_xy, lift)`: the one tested motion routine. Real + simulated robot. |
| `planner.py` | Request → AI model → small JSON plan → **checked in code** before anything moves. |
| `executor.py` | The loop: look → plan → move one object → look again → measure error → nudge / repair / re-plan. |
| `vision.py` | The fallback: `MODE = "slots"` (objects in taped slots, camera only says which slot). |
| `checks.py` | Catches config mistakes. `python app.py check` |
| `app.py` | Entry point and setup commands. |
| `trials.py` | Logs every `test`/`pick` to `trials.csv`; `python app.py stats` summarizes. |
| `TEAMMATE.md` | One-page guide for the test pilot. |
| `CLAUDE.md` · `STATUS.md` · `DECISIONS.md` | Full context for Claude · live progress · why things are the way they are |
| `docs/` | Event info, day plan, past-hackathon analysis, Viam 101 notes |
| `demo.txt` | The demo script, used to rehearse in simulation. |
| `tests/` | 250 automated tests. `python -m pytest -q` |

## Tonight / any time (no robot needed)

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
python -m pytest -q                              # ~8 seconds
python app.py --sim --seed 3 --feed demo.txt     # the whole demo, incl. a change of mind
python app.py --sim                              # type requests yourself
```

The simulator adds realistic placement error (`--place-noise 6` mm by default) so you see
the "placed 8 mm from the target" numbers and the occasional nudge. Without
`ANTHROPIC_API_KEY`, a keyword planner handles the demo phrases; with it, try looser
requests like "I need to take notes" or "tidy up around the cup".

## Commands for Friday

| Command | Milestone |
|---|---|
| `python app.py check` | After every edit to `config.py` |
| `python app.py resources` | Check the names in `config.py` match the machine |
| `python app.py where` | Print the gripper position → teach `TABLE_Z`, `SLOTS`, `ZONES`, `OVERVIEW` |
| `python app.py test red_cube slot1 tray` | M1: one move between taught spots, 5× in a row |
| `python app.py objects` | M2: what the camera sees, in world mm, per object |
| `python app.py calibrate red_cube=slot1 blue_cube=slot2 cup=slot3` | M3: camera correction from objects on taught spots |
| `python app.py pick cup tray` | M3: find it anywhere, place it, print the error in mm |
| `python app.py stats` | Success rate and mm error per object from every logged run |
| `python app.py` | M4+: talk to it |

## Viam setup (ask a Viam engineer to check it in the first hour)

- Arm, gripper (frame parented to the arm, real flange-to-fingertip offset), wrist camera
  (frame attached to the arm). The hackathon fragment usually has these.
- Camera attribute `align_color_depth: true` (the recycle team needed it for correct 3D).
- A **floor/table obstacle**, so the planner never drives into the table.
- **Detector ladder** (try in order, time-boxed; `config.SEGMENTERS` switches between them):
  1. **Pretrained detector** (a YOLO module from the Viam registry) for everyday objects: cup,
     apple, orange, bottle... Wrap it in a `detections-to-segments` service named `objects-3d`
     (`detector_name`, `camera_name`; Viam's reference used `mean_k: 5`, `sigma: 1.25`).
     Run `python app.py objects`. Good if each object is found steadily.
  2. **Train your own** (~1.5 h), like last month's winner: 100-200 photos from the robot's own
     camera, label, train (Viam's built-in training, or YOLOv8n). Use the label names in `OBJECTS[...]["labels"]`.
  3. **Color fallback**: one `color_detector` + one `detections-to-segments` per object,
     then `SEGMENTERS = {"seg-red": "red_cube", ...}`. Always works; looks more like a toy demo.
- Calibrate with whatever detector finds the cubes first (color is fine): calibration is about
  the camera, not the detector.

## Milestones (each one is a demo on its own)

1. **Reliable movement:** teach `TABLE_Z`, slots, zones, overview; `test` 5/5. Nothing else before this.
2. **See the table:** a segmenter + `objects` shows each object steadily (3 runs, same answer ±10 mm).
3. **Calibrate and pick anywhere:** `calibrate` to under 10 mm; `pick` lands in the zone 4/5.
4. **Understand and place precisely:** "put the cup in the tray and the red block next to it";
   relation placements land under `PLACE_TOLERANCE_MM`.
5. **Adapt:** change of mind mid-task; move the cup at a checkpoint and watch the block follow.
6. **Stretch:** stacking; pointing ("put it *there*"); a dashboard from `status.json` showing the error per placement.

**Fallback:** if Milestone 3 isn't working by Friday 7 PM, set `MODE = "slots"`: objects start in
taped slots and go to zones. Everything else (talking, checking, re-planning) still works.

## Lessons from last month's teams on this exact hardware

- **Camera-to-arm positions were off by 2-9 cm for everyone who used them** (ThirdArm ~21 mm,
  the winner's teammate 76 mm + 89 mm, patched by hand). `calibrate` measures and fixes it;
  re-run it if anyone bumps the camera.
- **Detections are noisy**: background blobs, things off the table, objects flickering.
  Perception drops oversized "objects" (`MAX_OBJECT_DIM_MM`), anything off the table area, and
  anything not seen in 2 of 3 looks. Two equal "cups" → it refuses rather than guesses.
- **Choose objects that look the same from any angle from above** (cubes, cups, round fruit,
  a marker standing upright). A two-finger gripper can't grab a lying marker the wrong way.
- **The configured table can be too high**: if motion refuses a grasp you know works, ask a
  Viam engineer to fix the `table` obstacle or the gripper's collision box.
- **Grip check**: the gripper's "holding" flag only means "jaws not fully open/closed". Test it
  empty; if it lies, `TRUST_GRAB_SENSOR = False` (the camera check still catches misses).
- **Connections drop** (`deadline has elapsed`, `StreamTerminatedError`): moves are retried;
  if a run stops, check the table and restart.
- **Motion is not fast**: ~6 moves + a look per object, so budget 20-40 s per object; keep the demo to 3-4 objects.

## Safety rules for the demo

- Nobody reaches into the workspace while the arm moves. Changes happen while it's at the overview pose.
- Keep anything not in `config.OBJECTS` (laptops, phones, drinks) off the table.
- One person keeps a hand near the E-stop.
