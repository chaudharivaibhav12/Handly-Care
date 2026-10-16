# CLAUDE.md: read this first

You are helping **Vaibhav Chaudhari** (NYU, vc2836@nyu.edu) at a 2-day robotics hackathon.
This file is the full context from the planning session the night before (Oct 15-16, 2026).
Everything here was decided with Vaibhav. Don't re-litigate it unless he asks; build on it.

Before doing anything on a new request:
1. Read `STATUS.md`: where things stand right now (update it as you go).
2. Skim `DECISIONS.md` if the request touches scope or design.
3. After any code change: `python -m pytest -q` and `python app.py check` must both pass.

---

## The event

**Robot Hackathon: Code That Moves**, hosted by Viam (Camila Lamoratta, Alyssa Demirjian,
Shannon Bradshaw, who also ran and judged last month's "Fine Motor Skills" hackathon).
- **Where:** Viam, 1900 Broadway, Fl 6, New York, NY 10023 (next to The Smith). Give your name at the lobby desk.
- **When:** Fri Oct 16, 9:00 AM to 9:00 PM (hard close, no overnight) · Sat Oct 17, 9:00 AM to 6:00 PM.
- **Schedule:** Fri 9:30 kickoff + Viam crash course · 10:00 teams + hardware · 10:30 hacking · 1:00 lunch · 6:00 dinner.
  Sat 9:30 hacking · 12:30 lunch · **3:30 demos** · 5:00 awards.
- **Hardware per team:** UFactory xArm6 (or UR) + two-finger gripper, Linux compute, RealSense D435 or
  Orbbec Astra 2 on the wrist, webcams, Viam account. Bring a laptop.
- **Prizes (per member):** Gold SO-101 arm + $5,000 Viam credits · Silver + $2,500 · Bronze + $1,000.
- Teams of 2-3. Vaibhav was added as the final attendee. Full details: `docs/event-info.md`.

## Vaibhav's laptop
**Windows** (device "jarvis"), PowerShell, Python 3.11 in `.venv`. Project folder:
`C:\Users\babbe\OneDrive\Desktop\NYU\Github\Handly-Care` (git: github.com/chaudharivaibhav12/Handly-Care).
Moved here from `Viam-Hack\sidekick` Fri 7:55 PM; that old folder is a stale copy, don't edit it.
Give Windows commands: `.venv\Scripts\activate`, `copy .env.example .env`, backslash paths.
Console output must survive cp1252 (no crashes on emoji): `app.safe_console()` handles it; keep it.
Setup verified Fri 4 AM: `pip install -r requirements.txt` OK, all tests pass on his machine.

## Team and roles
- **Vaibhav = lead.** Owns all code, config, Viam setup, detection, calibration, planner, demo, pitch, decisions.
- **Teammate = test pilot** (1 person expected, maybe 2): runs the arm, E-stop, teaches positions
  with Vaibhav, runs logged trials (`test`, `pick`, `stats`), breaks things on purpose, plays the
  "judge" in the demo. Their guide: `TEAMMATE.md`.

## The project: Sidekick v2

**One line:** *Tell a robot arm how you want your desk, in plain English. It finds things anywhere,
places them precisely, shows how precise it was, and keeps it that way when you change your
mind or someone moves something.*
**Who it's for (open the pitch with this):** people with limited hand mobility.
**Headline:** adaptive, precise placement + checking its own work. Not voice, not object count.

Demo (3 min): pitch → judge scatters objects → "Give me something to write with" (screen: "placed
7 mm from target") → "Put the cup in the tray, and the red block next to it" → mid-task "Actually,
put the red block on the blue one" → judge slides the cup while the arm is at overview, the block
follows → "Pick up my laptop" → refuses with a reason → close (next steps).

## How the code works (`sidekick/`)

```
perception.find()  ->  planner.make_plan()  ->  executor.run_task() loop:
  look (overview pose, 3 looks, vote)          pick next goal (anchors first)
                                               places.resolve() -> exact x, y, lift
                                               robot.move_object(obj, from_xy, to_xy, lift)
                                               look again -> measure mm error -> nudge / repair / re-plan
```
| File | Role |
|---|---|
| `config.py` | ALL tunables. Every `TODO` must be measured on the real arm. |
| `perception.py` | MODE "vision": `detections-to-segments` service(s) → `transform_pose` to world → `calibration.apply` → vote over `OBSERVE_FRAMES` (needs `MIN_HITS`). MODE "slots": wraps `vision.py`. |
| `calibration.py` | Fits affine (3+ spread points) or shift from (camera-seen, taught-truth) pairs; refuses wild fits. |
| `places.py` | Destination grammar + geometry: `"tray"` zone, `left_of:/right_of:/in_front_of:/behind:/next_to:<obj>` (person's point of view via `USER_DIRECTION`), `on:<obj>` stacking (only `stackable`). Reach, blockers, `done()`, `error_mm()`. |
| `planner.py` | Claude (`PLANNER_MODEL`, 15 s timeout, no retries, temperature 0) → JSON plan → `validate()` (the safety layer) → `order()` anchors first. Keyword fallback when no key or any AI failure. |
| `executor.py` | The adaptive loop. Goals are re-checked every look: retries, nudges, repairs moved anchors, interruptions only at checkpoints, loop budget. Writes `status.json`. |
| `robot.py` | `RealRobot` (Viam motion service, gripper frame, world frame, tool pointing down, straight-line up/down) and `SimRobot` (positions, `place_error`, `flaky`, hooks). |
| `vision.py` | Slots-mode fallback: color detectors → pixel regions → vote. |
| `trials.py` | Every `test`/`pick` appended to `trials.csv`; `python app.py stats`. |
| `checks.py` | `python app.py check`: config mistakes. |
| `tests/` | 252 tests, all passing. Every safety behavior was verified by breaking it on purpose. |

**Commands:** `check` · `resources` · `where` · `overview` · `snapshot` · `test <obj> <spot> <spot>` ·
`objects` · `calibrate red_cube=slot1 blue_cube=slot2 cup=slot3` · `pick <obj> <zone>` · `stats` ·
`python app.py` (talk) · `--sim [--seed N] [--feed demo.txt] [--flaky 0.3] [--place-noise 6]`.

## Status at handoff (Fri 2:55 AM)
- Everything runs and is tested **in simulation only**. Nothing has touched real hardware yet.
- Unverified until the arm: all positions (`TABLE_Z`, `SLOTS`, `ZONES`, `OVERVIEW`, `WORKSPACE`,
  `USER_DIRECTION`), object sizes/grasp heights, resource names, segmenter labels, real AI answers
  (tested only with a fake client), gripper `grab()` honesty, whether straight-line moves get planned.
- Plan for the day: `docs/hackathon-day-plan.md`. Milestones and fallbacks below.

## Milestones (each one is demoable on its own)
1. Reliable movement: `test` 5/5 between taught spots. **Nothing else before this.**
2. See the table: `objects` steady ±10 mm over 3 runs, every object.
3. Calibrate (<10 mm) + `pick` anywhere 4/5 into a zone.
4. Language + precise relations on the real arm.
5. Adapt: change of mind, anchor moved → follower re-placed, nudge on a bad landing.
6. Stretch, ONE: stacking · dashboard from `status.json` (mm error per placement) · pointing.
**Checkpoints:** Fri 2 PM detector decision · 4 PM train-or-color · **7 PM: not at M3 → `MODE = "slots"`** ·
**Sat 1:30 PM feature freeze** (only demo-path fixes after it).

Detector ladder (`config.SEGMENTERS` switches): pretrained YOLO + `objects-3d` → train our own
(~1.5 h, photos from the robot's own camera) → color detectors, one `detections-to-segments` per object.

## Hard-won facts (from last month's teams' code on this exact hardware)
- **Camera→arm positions were off 2-9 cm for everyone** (ThirdArm ~21 mm; the winner's teammate
  76 + 89 mm). Hence `calibrate`. Re-run it whenever the camera might have moved.
- **Detections are noisy:** background blobs (409×719 px), carpet off the table, phantoms in 2-5 of 12
  frames. Hence size/area filters + voting. Never trust one detection call.
- **Similar colors are indistinguishable** (two reds 4 hue units apart).
- **Configured table can be too high**: planner refused a grasp that worked by hand (z 28 vs 41).
  Fix the `table` obstacle / gripper collision box with a Viam engineer; don't hack heights.
- **Gripper "holding" flag is weak** (only "jaws not at an end stop"). Test empty; maybe `TRUST_GRAB_SENSOR=False`.
  The winner used `gripper.do_command({"set": pos})` / `{"get": True}` (xArm gripper, 0-850 scale) to verify closure.
- **Connections drop** (`deadline has elapsed`, `StreamTerminatedError`); moves are retried.
- **Motion is slow-ish**: budget 20-40 s per object; keep the demo to 3-4 objects.
- The winner (Gaze Arm) won with a human story + verifying every step + stopping when unsure, not a fancy model.
  Full analysis: `docs/past-hackathon-analysis.md`. Viam's own reference for this hardware:
  github.com/viam-devrel/pick-and-place (uses `arm-position-saver` switches: `set_position(2)` = go;
  `1` OVERWRITES the pose, never send it).

## Viam SDK facts (checked against viam-sdk 0.84)
- `Camera.get_images(filter_source_names=["color"])` (there is no `get_image`); depth mime is `image/vnd.viam.dep`.
- `MotionClient.move(component_name: str, destination: PoseInFrame, world_state=None, constraints=None, *, extra, timeout)`.
- `Constraints(linear_constraint=[LinearConstraint(line_tolerance_mm=10)])`; the winner used 10 mm + 15° orientation tolerance.
- `VisionClient.get_object_point_clouds(camera)` → objects with `.geometries.reference_frame` and `.geometries.geometries[0].center/.box/.label`.
- `robot.transform_pose(PoseInFrame(ref, pose), "world")`. Viam 101 course notes: `docs/viam-101-notes.md`.

## How to work with Vaibhav
- Plain language, short answers; he's busy and on the floor. Lead with what to do next.
- He decides scope. Ask before big changes; propose, don't impose. Offer one clear recommendation.
- Reliability before features. If something fails twice on the real arm, shrink it.
- Keep tests passing; add a test for every bug found on hardware. Never weaken a safety check to make a demo work.
- Keep `STATUS.md` current (milestone, measured values, what broke, what's next) so any new session can pick up.
- It's a live event: watch the clock against the checkpoints and say so when one is near.

---

## Research log protocol (added Fri 11:20, see DEC-001)

Records hold *questions and evidence*: why each experiment ran, what it found (negative results too),
and what we decided. Each takes 2–3 minutes to write, never 15. Vaibhav is on the floor, so **you** draft
records from the terminal output and he confirms.

**Files:** `STATUS.md` = the map (goal, direction, `### Branch X — name — ACTIVE|PARKED|ABANDONED`,
open questions, next 3) · `research/LOG.md` one-liners · `research/experiments/EXP-NNN.md` ·
`research/decisions/DEC-NNN.md` · `DECISIONS.md` = DEC-000 (pre-event planning) · `results/` raw outputs ·
`trials.csv` raw trial rows. Validator: `python research/check.py` (must print 0 errors, like pytest and `check`).

**Rules**
1. Read `STATUS.md` before answering anything about direction, history or next steps. Answer from the
   records and cite IDs. If something isn't recorded, say so; don't reconstruct it from code or memory.
2. Records are append-only. Never edit a completed record's Results or Interpretation. If a later result
   contradicts it, write a new EXP and set only the old one's `superseded_by`. A mis-copied number may
   be fixed only with Vaibhav's OK, noted in the record.
3. No Why or no Consequence = invalid record. Refuse to close it.
4. Activating, parking, abandoning or pivoting a branch needs a DEC citing evidence EXP IDs. A checkpoint
   (2 PM detector, 4 PM train-or-color, 7 PM slots rule, Sat 1:30 freeze) always produces a DEC, even
   "no change". Advice from a teammate or a Viam engineer goes in the Reason; it doesn't replace evidence.
5. Setup = the operating point, including what didn't change: paste `python research/check.py --setup`
   (all config values + `config_hash` of config.py + calibration.json; env vars by name only). Then add
   what code can't see: objects used, lighting, camera moved since `calibrate`?, who ran the arm.
6. Hypotheses use the milestone thresholds: `test` 5/5 · `objects` ±10 mm over 3 runs · `calibrate`
   < 10 mm · `pick` 4/5 · demo path 5 clean rehearsals.
7. Negative results get the same care. An experiment that kills an idea fills `rules_out`. "Fails twice
   on the real arm → shrink it" is closed as PIVOT or PARK with a DEC.
8. Only questions are experiments. Plumbing, typos and config entry (e.g. typing in a taught pose) are
   LOG.md notes: `- Fri HH:MM note: …`. A bug found on hardware: LOG note + a regression test, named in
   the EXP's Consequence if it came from one.
9. Results cite `trials.csv` by time window and paste `python app.py stats`. Bigger outputs (snapshots,
   `objects` dumps) go to `results/EXP-NNN-*`.
10. Hacks left in place are listed per EXP and collected in "status" so nothing sim-only reaches the demo.

**Git:** a repo since Fri 7:55 PM. EXP-001..009 ran before it (`commit: ""`; `config_hash` identifies what ran);
new EXPs fill `commit`. Risky work on `exp/EXP-NNN-slug` branches, keep failed
branches, merge normally (not `--ff-only`) once the demo path still runs, never `reset --hard` / force-push
/ `git clean` (go back with `git switch -c restore-<id> <commit>`), ask before every push or merge to main,
never commit `.env` or large raw files (images, point clouds).

**Workflow phrases**
- **"new exp: <question>"** → next ID; `parent` = latest EXP on that branch; draft Why + hypothesis
  (with threshold) and ask Vaibhav to confirm (one question); fill Setup; add a `planned` LOG line.
- **"close EXP-NNN"** → Results from the actual output, then Interpretation (with confidence), Consequence,
  decision, `rules_out`, `config_hash`; update the branch's "Latest" + Next 3 in STATUS.md and the LOG line;
  run `research/check.py`.
- **"decide: <change>"** → DEC citing evidence; update the branch heading in STATUS.md.
- **"log: <note>"** → one timestamped line in LOG.md.
- **"status"** → 10 lines: changed since the last status, open, next 3, time left to the next checkpoint,
  hacks still in the code.
- **"how did we get here: <topic>"** → walk `parent`/`motivated_by` as belief → experiment → evidence → decision.
