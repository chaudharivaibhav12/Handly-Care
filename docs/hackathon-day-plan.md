# Sidekick v2: hackathon day plan (lead + test pilot)

Fri Oct 16 – Sat Oct 17 · Viam, 1900 Broadway, 6th floor · arrive by 9:00 AM
**Rule:** reliability before features. Missed a checkpoint → cut scope (bottom table), don't push on.

**Roles**
- **You (lead):** all code and config, Viam setup, detection, calibration, planner, demo script,
  pitch, every decision.
- **Teammate (test pilot):** runs the arm, E-stop and safety, teaches positions with you, runs
  trials (`test`, `pick`, `stats`), breaks things on purpose, plays the "judge" in the demo.
  Their guide: `TEAMMATE.md` in the code.
- Why this works: while you change code, they're measuring the last change. You never wait on
  the arm, and every decision is backed by `python app.py stats`.

**Pack:** laptop + charger, phone + charger, painter's tape, a small neutral tray (~18 cm),
2 cubes (red, blue) if you have them, a small paper cup, a thick yellow marker.

## Friday

| Time | You (lead) | Teammate (test pilot) |
|---|---|---|
| 10:00 | **Pitch Sidekick, recruit them as test pilot**, share `TEAMMATE.md` | |
| 10:30–11:00 | With a Viam engineer: gripper offset, camera frame, floor obstacle, `align_color_depth`. `.env`, `resources`, names in `config.py` | Tape slots and zones; find the E-stop; read `TEAMMATE.md` |
| 11:00–12:00 | **M1:** type in positions as they read them; `check` | Jog arm to each spot, `where`, read out x/y; `TABLE_Z` by touching the table |
| 12:00–1:00 | Pretrained YOLO + `objects-3d` segmenter; one color segmenter for the red cube | **5× `test red_cube slot1 tray`**, then other objects; gripper empty-grab test |
| 1:00 | Lunch: look at `stats` together | |
| 2:00 | **Decision:** pretrained detector good enough? | |
| 2:00–4:00 | `calibrate` with cubes on slots; then detector: add objects, or train, or color fallback | Run `objects` 3× per setup and report drift; place cubes for calibration; **5× `pick red_cube tray`** after each calibration |
| 4:00–6:00 | Tune `grasp_height`s and tolerances from the numbers; AI key + planner | **5× `pick`** per object from new places each time; `stats` |
| 6:00 | Dinner | |
| 6:45–8:30 | **M4:** language + relations on the real arm | Gives the requests by voice/typing; logs anything off |
| **7:00** | **Checkpoint:** calibrated pick not 4/5 → `MODE = "slots"` | |
| 8:30–9:00 | Push code + `config.py` + `calibration.json` + `trials.csv`; copy machine config | Phone video of everything that works |

## Saturday

| Time | You (lead) | Teammate (test pilot) |
|---|---|---|
| 9:00 | `check`, re-run `calibrate` | 5× `pick` to confirm nothing drifted overnight |
| 9:30–11:30 | **M5:** fix what they find (nudge, anchor repair, change of mind) | **Break it on purpose**: scatter, odd requests, move the cup at checkpoints |
| 11:30–12:30 | **M6 stretch, ONE:** stacking or dashboard | Trials on the stretch feature |
| **1:30** | **FEATURE FREEZE** | |
| 1:30–3:15 | Rehearse 5× (you talk, they scatter/move the cup), backup video at 2:30 | |
| 3:15 | Reset table, `calibrate`, charge laptop | |
| 3:30 | **Demos** · 5:00 Awards | |

## If you're behind

| Check at | Not working | Do this |
|---|---|---|
| Fri 12:30 | Reliable pick from a taught spot | Viam engineer now. Cubes only, one zone |
| Fri 2:00 | Pretrained detector | Train or go color while calibration continues with color segmenters |
| Fri 4:00 | Trained detector | Color fallback for every object; move on |
| Fri 7:00 | Calibrated pick anywhere | `MODE = "slots"`; demo zones + language + re-planning |
| Sat 11:30 | Relations on the real arm | Demo zones + change of mind + anchor repair in zones |
| Sat 1:30 | Anything | Freeze anyway |

**Pitch stat to collect:** "placed within N mm on average over M trials", from `python app.py stats`.
**Safety:** nobody reaches in while the arm moves; nothing but demo objects on the table; the test pilot is on the E-stop.
