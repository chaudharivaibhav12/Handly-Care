# Sidekick v2: plan (Viam "Code That Moves" hackathon, Oct 16–17)

**One line:** Tell it how you want your desk. It finds things anywhere, puts them exactly
where you said, shows how precise it was, and keeps it that way when you change your mind
or someone moves something.

**Who it's for (first 20 s of the pitch):** people with limited hand mobility (tremor,
arthritis, limb difference) who can describe what they want but can't reach or place things.

**Why not copy Gaze (last month's winner):** same judges, same assistive theme. We take its
*engine* (vision pick from anywhere, verify every step, stop when unsure) and pair it with what
nobody showed last month: **plain language + precise relational placement + keeping an
arrangement true.**

## What it does

| Capability | How |
|---|---|
| Find objects anywhere | Viam 3D segmenter → world frame → camera correction → vote over 3 looks |
| Fix the camera's 2-9 cm error | `calibrate`: objects on taught spots → affine/shift fit, error printed before → after |
| Understand requests | Claude → JSON plan (zones, `left_of:`, `next_to:`, `on:`...), checked in code; keyword fallback offline |
| Place precisely | Geometry from object sizes, person's point of view, free-space and reach checks |
| Check its own work | Looks after every object; measures error in mm; nudges if off; asks for help after 1 retry |
| Keep it true | Every goal re-checked each look: if the cup moves, the block next to it follows |
| Change of mind | New messages applied at the next safe checkpoint, never mid-move |
| Stay safe | Refuses unknown objects, out-of-reach, full zones, impossible relations, with a spoken reason |

## Objects
Start with everyday, angle-proof objects: 2 cubes (red, blue), a small paper cup, a thick
marker standing upright (or swap for an apple/orange). Detector ladder, time-boxed:
pretrained YOLO → train our own (~1.5 h) → color detectors. `config.SEGMENTERS` switches.

## Team: lead + test pilot
- **You (lead):** all code, config, Viam setup, detection, calibration, planner, demo, pitch, decisions.
- **Teammate (test pilot):** runs the arm and the E-stop, teaches positions with you, runs logged
  trials (`test`, `pick`, `stats`), breaks things on purpose, plays the "judge" in the demo.
  Guide: `TEAMMATE.md`. Every attempt is logged to `trials.csv`; "demo-ready" = 4 of 5 on target.

## Milestones (each is a demo on its own)
1. Reliable movement: `test` 5/5 between taught spots.
2. See the table: `objects` steady (±10 mm over 3 runs) for each object.
3. Calibrate (<10 mm) and pick anywhere (`pick`, 4/5 into a zone).
4. Language + precise placement on the real arm (zones and relations under tolerance).
5. Adapt: change of mind; anchor moved → follower re-placed; nudge on a bad landing.
6. Stretch (pick one): stacking · pointing ("put it there") · dashboard with mm error per placement.

**Fallback:** not at Milestone 3 by Fri 7 PM → `MODE = "slots"` (taped slots → zones). Language,
checking and re-planning all still work.

## Demo (3 min)
1. Pitch: who it's for (20 s).
2. A judge scatters the objects anywhere on the table.
3. "Give me something to write with." → marker to the person; screen: "placed 7 mm from target".
4. "Put the cup in the tray, and the red block next to it."
5. Mid-task: "Actually, put the red block on the blue one." → re-plans at the checkpoint (or
   "in front of the cup" if stacking isn't reliable).
6. While the arm is at overview, a judge slides the cup. The robot notices and moves what
   belonged next to it. ("It keeps your desk the way you asked.")
7. "Pick up my laptop." → refuses and says why.
8. Close (20 s): next steps: pointing, voice, hand-over to the person.

## Cut on purpose (say them as next steps)
Handing into a palm, fast motion, arbitrary shapes and grasp angles, free-form coordinates.

## Evidence behind the choices
Last month's code (same room, same hardware): the winner's verify-everything engine; ThirdArm's
detection filtering and measured 21 mm camera miss; the winner's teammate's 76 + 89 mm miss;
Duet's AI timeouts and fallbacks; the recycle sorter's 80% item success after two days.
Full write-up: `past-hackathon-analysis.md`.

Code: sidekick.zip (runs in `--sim` today; 250 tests).
