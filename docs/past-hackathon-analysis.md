# What won last time: code analysis of Viam's Sept 2026 hackathon

Read on Oct 16, 2026 from the teams' public GitHub repos. Same venue, same hardware
(UFactory xArm6 + two-finger gripper + wrist RealSense D435, Viam), same organizers.

## The teams and what they used

| Team | Result | What it does | AI / models | Tests |
|---|---|---|---|---|
| **Gaze Arm** (rwezzy, reenj01, Taylor Ye) | **Winner** (their README) | Look at an object with your eyes, the arm picks it up; for people who can't use their hands | MediaPipe face/iris landmarks (gaze), **custom YOLOv8n trained on site** (~100-200 photos from the robot's own camera, served as Viam `mlmodel` "vision-1"), Viam `detections-to-segments` for 3D. **No LLM.** | 163 |
| **Jenga arcade** (Ammar Mahmood + 2) | Won "Fine Motor Skills" category (later dropped the "1st place" wording) | Remote players challenge the arm to Jenga over the internet | Vision + teleoperation; code not public | n/a |
| **Duet** | Not listed as a winner | Arm draws on a whiteboard with you, answering your marks | **Claude Opus 5 vision**, structured output (Pydantic schema), photo with a labelled mm grid, low effort, 12-45 s timeouts, offline fallback; OpenCV ink tracing | 186 |
| **Recycle sorter** (xiang1103) | Not listed | "Can I have two cokes": fetches drinks from a pile | YOLOE open-vocabulary detector ("can", no training), **local qwen2.5:1.5b via Ollama** to parse orders (JSON schema + enum), qwen2.5vl:7b / **Claude Opus 5** to read can labels | 111 |
| **ThirdArm** | Not listed | Pick a colored block, drop it in a box | Viam **color_detector, one per color**, point-cloud top face | 0 |
| **Juggling** | Not listed | Throw a ball | No vision; then a small model to detect "ball in funnel / hand in the way" | 0 |
| **Arm 5** | Not listed | Color sort, bottle pouring, voice | Moondream, SAM, OpenAI Whisper | 107 |
| Ball catch (gracexu24) | Not listed | Two cameras predict a thrown ball | Color tracking + trajectory fit | 66 |

Also read: Viam's own reference solution (`viam-devrel/pick-and-place`) for this hardware,
and a July SPC×Viam project (keyboard arm: ORB matching + SAM2).

## What made the winner work (from its code)

The winner's demo is a simple pick and place. What it did **around** the pick is the lesson.

1. **A person, not a feature.** README opens with who it's for (cerebral palsy, missing limbs,
   bed-bound patients). Every design choice ties back to that user ("don't make them move their head").
2. **Verify every step, never assume.** Before closing the jaws: gripper must read fully open,
   arm must be measured still within 5 mm and 2° of the target (two readbacks), jaws must stop
   short of fully closed (evidence of contact). A failed check opens the jaws and backs off.
3. **Refuse and stop instead of guessing.** Out of reach (>700 mm), planner rejects, two depth
   captures disagree by >10 mm, wrist moved during capture: it stops with the reason on screen
   and waits for a person to press R. Nothing retries automatically.
4. **Freeze the world before moving.** The camera rides the wrist, so it locks one frame,
   converts every object to world coordinates, then moves.
5. **Measure the hardware, don't trust defaults.** The config said the gripper tip was 105 mm,
   it was 150; the table model and the camera disagreed by 28 mm; one joint read -359.99° against
   a ±359° limit. They measured, wrote the numbers in the README, and built checks.
6. **Straight-line descent and lift** (`LinearConstraint`, 10 mm tolerance) and an orientation
   constraint (15°) on approach and carry.
7. **Motion off by default.** Everything runs as a dry run until `--execute`.
8. **Trained their own detector on site**, from the robot's own camera, because stock YOLO
   boxes didn't line up with the 3D objects.
9. **Started from Viam's tutorial code** and iterated in small commits (first commit 3:17 PM
   Friday, working pick ~2:42 PM Saturday, look-to-place added the night after).

## Problems teams hit (they will hit you too)

- **Camera-to-arm calibration was off for everyone who used it:** ThirdArm ~21 mm constant miss;
  winner's teammate 76 mm radial + 89 mm lateral, patched with a hard-coded correction.
- **Color detectors are noisy:** huge background blobs (409×719 px), blue carpet off the table,
  phantoms in 2-5 of 12 frames. ThirdArm needed size filter + workspace filter + 8-frame voting.
- **Similar colors are indistinguishable:** two reds 4 hue units apart.
- **Table collision model too high:** planner refused a grasp that worked by hand (z 28 vs 41).
- **Gripper "holding" flag is weak:** only means jaws aren't at an end stop.
- **Connections drop** mid-run (`deadline has elapsed`, `StreamTerminatedError`).
- **Speed:** motion planning is slow; fast moves needed module hacks (juggling).
- **Real reliability:** the recycle sorter's own logs show 5 of 8 orders fully filled and
  12 of 15 items fetched (80%), after two days of work.

## How Sidekick compares

| Lesson | Sidekick |
|---|---|
| Human story | ✓ desk helper for people with limited hand mobility |
| Verify and stop | ✓ looks again after every object, one retry, then asks for help; refuses unknown objects |
| Avoid camera-to-arm math | ✓ grasps from taught slot positions; camera only says which slot/zone |
| Noisy color detectors | **Fixed tonight:** size filter, region filter, 3-frame voting (was "biggest blob wins", which is exactly what ThirdArm found to be wrong) |
| AI call can hang | **Fixed tonight:** 15 s timeout, no silent retries, temperature 0, keyword fallback |
| Straight-line moves | ✓ (tolerance raised 5 → 10 mm to match what worked for the winner) |
| Dry-run first | ✓ `--sim` mode; on Friday use `test` one move at a time |
| Tests | ✓ 112, comparable to the top teams |
| Interaction | Talk (typed) vs. the winner's gaze. Theme overlaps with the winner (assistive), so the pitch must stress what's new: **natural language + re-planning when you change your mind** |

## Takeaways for Friday

1. Get one pick working 5/5 from taught positions before anything else (what every team that
   shipped did).
2. Spend real time on measurement in hour one: gripper tip offset, table height, the
   collision model. Ask a Viam engineer.
3. Run `python app.py snapshot` with objects placed, then set `MIN_BOX_PX`/`MAX_BOX_PX` and `REGIONS`.
4. Keep the demo to 3-4 objects; budget 20-40 s per object.
5. In the pitch, say who it's for first, then show it adapting.
