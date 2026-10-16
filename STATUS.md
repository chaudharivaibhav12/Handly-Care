# STATUS: the one page to read first (also the research MAP)

**Last updated:** Fri Oct 16, 7:27 PM (laptop clock). Demo path works on the real arm: `tissue` (camera-located, one tissue to the hand-over spot) then `medicine` (open drawer → real pill box → hand over → close), back to back. Next: backup video, dashboard bridge, thermometer.
Update after every milestone, measurement, or decision. History lives in `research/LOG.md` and the
EXP/DEC records; this page is only the current state.

## Handoff
- FOLDER: work in GithubHandly-Care now (git repo); Viam-Hacksidekick is a stale copy. (for a new session, Fri 7:27 PM laptop clock)
- DEMO (DEC-005): open the middle drawer, take the medicine capsule box out, hand it to the patient; laser thermometer later. Teammate handoff files are coming.
- DASHBOARD (7:48 PM, sim only so far): `python bridge.py` → open http://localhost:8765 → Arm Setup → Connect. "Release pill box" runs `medicine`, the tissue request / Hand Over runs `tissue`, "Arm to start pose" runs `to_high`; a confirm box shows the checklist before every run. One run at a time. Thermometer stays simulated. Use "Next event" with autoplay PAUSED (autoplay fires beats every 9 s while a 60 s routine runs). `python bridge.py --sim` = no arm.
- CHAIN (7:22): `tissue` then `medicine` back to back works (tissue ends at 'high'). `python app.py to_high` gets there from anywhere.
- TISSUE (EXP-009 running: 2/2 since taping, 2/3 overall; 3 more runs to close): `python app.py tissue` from bird's-eye: camera finds the tissue box → pinch 15 mm below the tissue top → pull up 25 cm → planner to the taught 'handoff' → release → to_high. Tissue box must be TAPED (try 1 lifted the whole box).
- MEDICINE (EXP-008; real pill box works too, LOG 7:25): `python app.py medicine` = drawer open → pills (pill_above → pill_grip → checked grab → pill_lift → handoff, hold 3 s) → drawer close, 4/4, 60 s.
- Drawer: `python app.py drawer open|close` replays 8 hand-taught JOINT poses (taught.py, drawer_joints.json; `joints <name>|list` to teach)
  at 15 deg/s. Open 4/4, close 2/2 (EXP-007, DEC-006). Blind (no planner, no camera): the BOX MUST STAY TAPED (both ends) where it was taught.
  It refuses to start if 'high' is > 45° away on any joint: start from high/retreat, not from bird's-eye.
- Harness (Branch A): picks 6-19 mm, tray upright, plain English via Claude/OpenRouter (ai.py) 4/4 at 1-10 mm (EXP-004/005/006).
- Safety fixes from today: E-stop never retried; lifts keep the tool tilt; no free move while holding a knob; carry moves avoid seen objects;
  failed carry → object put back (MoveError). Keep the ARM WIRE TIED. No-go strip near wall-front (x 440-480, y > 100).
- Names change every run → demo in ONE `python app.py` session. No glass on the demo table.
- Calibration: 8-pt affine 9.0 mm (EXP-005). Tools: tools/README.md. Ask Vaibhav before EVERY arm run (one go = one run).
- "context canceled" mid-move: editing the machine config in the Viam app (e.g. arm speed) reconfigures the arm and cancels running moves (LOG 6:58). No config edits, Viam app closed, manual mode OFF during runs.
- Arm far from 'high'? `tools/unwind_joint.py` one joint at a time with a height check (LOG 6:44, 7:13). From bird's-eye: j6, j1, j4, j5, j2, j3 (last). A cancelled step just resumes.
- Clock: trust the laptop clock.

## Goal
Demo (Sat 3:30 PM, 3 min, judged by Viam: Camila, Alyssa, Shannon): a person with limited hand
mobility tells the arm in plain English how they want their desk. It finds objects anywhere, places
them precisely (shows mm error), and adapts to a change of mind or a moved object. It refuses unsafe asks.
Judges reward a human story + verifying every step + stopping when unsure (DEC-000).

## Current direction
M1 passed (EXP-001). Branch B now finds ANY object by 3D shape and names it with Claude (DEC-002);
graspability comes from measured size, so refusals carry a measured reason. Shape finder stays as the
fallback for colored blocks. Slots fallback stays parked unless the 7 PM checkpoint fails.

## Branches
### Branch A — Core pick and place (movement, calibrate, relations, adapt) — ACTIVE
State: M1, M3, M4 passed. Picks 6-19 mm; tray 3/3 upright; language 4/4 at 1-10 mm. No-go strip near wall-front. Latest: EXP-006 (completed, CONTINUE).

### Branch B — Vision perception (now: anything mode = depth segmentation + Claude naming) — ACTIVE
State: M2 passed with our own height map (DEC-003). Steady ≤1 mm, sizes ~±1 cm. Latest: EXP-003 (completed, CONTINUE).

### Branch C — Slots fallback (MODE = "slots") — PARKED
State: tested in sim. Turned on only by the 7 PM rule (DEC-001). Latest: none.

### Branch E — Medicine drawer + temperature demo — ACTIVE
State: `medicine` 4/4, 60 s (real pill box works too); `tissue` 2/2 since taping. Latest: EXP-009 (running).

### Branch D — Saturday stretch (ONE of stacking / dashboard / pointing) — PARKED
State: waits on M1-M5 (DEC-001). Latest: none.

## Open questions
- Demo cup must be ≤ 7 cm wide (the 9 cm white cup can't be gripped). Or grip a cup by its rim?
- Stretch idea (Vaibhav, 2:45): open a water bottle by pressing its lid button. New skill → Branch D candidate, only if M1-M5 pass.
- Is the 105 mm gripper frame offset right? (Check the 3D SCENE against the real fingertips.)
- Does the motion service plan straight-line moves every time? (1 of 1 so far)
- Which detector sees our 4 objects steadily? Shape finder is first in line.

## Next 3 things to run
1. Backup video of `tissue` + `medicine` before 9 PM (real pill box already in use and working, LOG 7:25).
2. Dashboard bridge BUILT (bridge.py, LOG 7:48): first real-arm run from the UI (medicine), with the precondition checklist.
3. Thermometer: arm carries it to a taught measure pose (aim at wrist/hand, not eyes); reading shown as simulated unless integrated.

## Where we are
- Phase: **event, Fri morning**. Connected to armfarm13. First coded move OK (`overview`, 0.3 mm) after the TOOL_THETA fix.
- Clock: 2 PM detector (DEC-003) · 4 PM train-or-color (DEC-004, no change) · 7 PM slots rule (DEC due; M3 passed) · Sat 1:30 PM freeze.

## Milestones
- [x] M1 Reliable movement: `test` 5/5 between taught spots (EXP-001, 12:21)
- [x] M2 `objects` steady (±10 mm, 3 runs): ≤1 mm for objects ≥2 cm tall (EXP-003, own height map)
- [x] M3 `calibrate` (11 mm, 5 spread pts); `pick` 4/4 into zones, 6-19 mm (EXP-004)
- [x] M4 Language + relations on the real arm: 4/4 placements 1-10 mm via Claude/OpenRouter (EXP-006)
- [ ] M5 Change of mind, anchor repair, nudge on the real arm
- [ ] M6 Stretch (one of: stacking / dashboard / pointing)
- [ ] Feature freeze Sat 1:30 PM · backup video recorded · 5 rehearsals

## Measured on the real machine (fill in; then copy into config.py)
| Setting | Value | Notes |
|---|---|---|
| Resource names (ARM, GRIPPER, CAMERA, MOTION) | arm, gripper, cam, builtin | obstacles: ceiling, table, wall-front, wall-side (show as gripper type). No vision service yet. |
| OVERVIEW | (207, 283, 560), theta 178 | 12:15: no wall in view, ~88x50 cm. Was (353,406,499) then (275,380,560); z 600+ unreachable | manual mode; real pose tilted ~11°, theta 136. Code replays it straight down, theta 0. Table seems to be on the +y side. |
| TABLE_Z | 63.0 | gripper-frame z at fingertip touch (x 394, y 518), 11:18. Table obstacle top is z=-23, so the gripper frame is ~60-85 mm above the real tips. |
| OVERVIEW | | camera sees whole table |
| SLOTS slot1..slot4 | (197,164)* (277,64)* (374,-10) (455,80) | *touched. Others = camera + offset (-6,+23), ±10 mm |
| ZONES tray / to_me / left / right | (174,550) (386,414) (389,225) (167,347) | all touched. Tray floor = table (z 62.4, ZONE_LIFT 0) |
| USER_DIRECTION | +y | person sits on the +y side (image left at overview); their left = +x |
| WORKSPACE / reach | x 80-480, y -60-640, reach 200-650 | x<480 keeps clear of wall-front |
| grasp_height per object | | |
| TRUST_GRAB_SENSOR | True | EXP-001: empty grab correctly reported 'grab missed' |
| TOOL_THETA | 145 | theta=0 → self-collision wrist vs cam, every IK solution. 145 works for OVERVIEW (0.3 mm). Untested over the whole table yet. |
| STRAIGHT_LINE ok? | yes (1 lift) | 1:10: +150 mm straight lift from the tray planned without fallback. More data from EXP-001 |
| Detector in use | anything-3d (planned) | obstacles_pointcloud + Claude naming (DEC-002); fallback shape-detector |
| Calibration | affine, 8 pts: 22.5 → 9.0 mm | 2:50 PM, calibrate auto, objects on all 8 spots (EXP-005). Tray 2 mm after. Re-run if the camera/overview moves |

## Trial results (`python app.py stats`)
(paste the table here after each batch)

## Decisions made today
(time, decision, why)
- 10:45 Fri: got robot station. Machine = **armfarm13**. Running ~15 min behind the day plan.
- ~11:00 Fri: kickoff slides read → `docs/viam-kickoff-notes.md`. Slides say names are `arm`/`gripper`/`cam`
  (config has `arm-1`/`gripper-1`; fix after `resources`). Table+wall obstacles preconfigured. Shape finder added
  as a detector option. Keep app as a client script (no module rewrite).

## Problems / open issues
(what broke, what was tried, what's next)
- 11:05 `resources` crashed when config names were wrong (it connected via the full robot). Fixed: it now just dials and lists names.
- Relations near the tray can land ON its rim (sim: cube at (179,429), tray y 401-641). Needs a tray footprint keep-out before M4.
- Tray floor touched: z 62.4 = table (ZONE_LIFT 0 confirmed).
- 3:05 PM loose arm wire knocked objects on carry moves (EXP-005). Tied along the arm → 0/3. Pre-demo check: wire tied.
