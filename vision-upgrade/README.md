# Vision upgrade for Sidekick: look first, then reach

This folder answers the judges' two comments, on your own Sidekick setup:

1. **The thermometer pointed at a wall.** Now it finds the face, turns the forehead pixel plus its
   depth into a 3D point, and the thermometer tip stops a few cm in front of that forehead after a
   slow, straight approach (`care.py thermometer`, built on `thermometer.py`).
2. **The tissue and the medicine used fixed movements.** Now:
   - the tissue and the pill box are handed over **in front of the person**, wherever they sit
     (`person.py`), instead of at the one taught 'handoff' spot;
   - before Sidekick replays the drawer poses (blind, no camera), the camera checks that the drawer
     box is still where those poses were taught, and refuses if it moved (`knob.py`);
   - `locate.py` grabs a bottle cap or a tissue at the height the depth camera measures, not a
     fixed height.

It lives inside Handly-Care (`handly-care/vision-upgrade/`) and uses Sidekick's `.env`, taught poses (`drawer_joints.json`) and robot
code. **It changes none of Sidekick's files.** Every script does a dry run by default: it prints what
it sees and saves a picture, and the arm doesn't move until you add `--execute` (or use `care.py go`).
Every script also runs on a built-in simulator (`--sim`), so you can try the whole flow at home.

## Setup (Windows), once

Use Sidekick's virtual environment; it only needs numpy and OpenCV added:

```powershell
cd C:\Users\babbe\OneDrive\Desktop\NYU\Github\Handly\code\handly-care
.venv\Scripts\activate
cd vision-upgrade
pip install -r requirements.txt            # adds numpy, opencv-python, pytest (keeps your viam-sdk)
python -m pytest -q                        # 80 passed
python care.py remember-drawer --sim --execute --yes
python care.py medicine --sim --execute --yes        # the whole medicine routine on the simulator
```

If `cv2.imshow` fails, run `pip uninstall opencv-python-headless` and then `pip install opencv-python`.
The standalone scripts (`check_setup.py`, `calib_check.py`, `locate.py`, `thermometer.py`) take
`--env ..\.env`; `care.py` finds Sidekick's `.env` by itself.

## At Open Lab, in this order

| # | Do | Done when |
|---|---|---|
| 1 | `python check_setup.py --env ..\sidekick\.env`, then open `out/setup_alignment.png` | All PASS. The red depth edges sit on the object outlines. |
| 2 | Touch test: `calib_check.py look`, jog the fingertips onto the top of an object, `touch`; repeat at 3-4 spots; `report` | Worst error ≤ 12 mm, or a consistent offset saved with `report --save`. |
| 3 | New station? In `sidekick`, re-teach `high`, the drawer poses, `pill_above`, `pill_grip` and `handoff` (`python app.py joints <name>`), exactly as at the hackathon | Sidekick's own `medicine` and `tissue` work. |
| 4 | In `sidekick`: jog the arm until the camera faces the chair, the face 60-80 cm away, then `python app.py joints look_person` | `python care.py look --execute` prints the forehead and saves `out/thermometer_person_*.png` with the face boxed. |
| 5 | Drawer closed, box on its tape marks: `python care.py remember-drawer --execute` | `out/drawer_remember.png` circles the yellow knob. Repeat whenever you re-teach the drawer. |
| 6 | Measure the thermometer: `THERMO_TIP_IN_GRIPPER_MM` (from the fingertips) and `THERMO_AXIS_IN_GRIPPER` in `settings.py` | — |
| 7 | Tape a printed face to a box at head height. `python care.py thermometer`, then `--execute` 5 times | The tip stops `STANDOFF_MM` ± 1 cm from the paper forehead, 5 of 5. |
| 8 | Paper face still there: `python care.py tissue --execute`, then `python care.py medicine --execute` | Each is let go in front of the paper face. |
| 9 | A real person, with someone else holding the E-stop | — |

Check these numbers in `settings.py` at a new station:

- `GRIPPER_TIP_OFFSET_MM`: Sidekick's `TABLE_Z - TABLE_TOP_WORLD_Z` (63 - 15 = **48** at armfarm13). Viam's
  gripper frame sits that far behind the real fingertips. Get it wrong and the thermometer stops that
  much closer to the face, so measure it again.
- `HANDOVER_FORWARD_MM` / `HANDOVER_BELOW_MM`: where in front of the face the hand-over happens.
- `MAX_HANDOVER_SHIFT_MM`: how far from the taught 'handoff' it may go (farther: it stays at the taught spot).
- `KNOB_VIEW`: where the drawer check looks from. Sidekick's `overview` by default; if the knob is out of that
  picture, set a taught pose name (`high`, or one you teach) and run `remember-drawer --execute` again.
- `KNOB_HSV_LOW` / `KNOB_HSV_HIGH`: only if the drawer knob isn't yellow.

## What each routine does

| `python care.py …` | Moves |
|---|---|
| `thermometer` | 'high' → `look_person` → two looks at the face → around the person to 15 cm out → look again → 3 slow straight steps → hold → back out → `look_person` → 'high' |
| `tissue` | 'high' → `look_person` → find the person → 'high' → Sidekick's overview and perception → **Sidekick's own tissue pick** → taught 'handoff' → in front of the person (slowly, around them) → let go → taught 'handoff' → 'high' |
| `medicine` | Sidekick's overview: is the drawer box where it was taught? → 'high' → `look_person` → find the person → 'high' → **Sidekick's own drawer open and pill grab** → taught 'handoff' → in front of the person → let go → taught 'handoff' → `pill_above` → 'high' → drawer close |
| `look`, `check-drawer`, `remember-drawer` | Nothing, unless you add `--execute` (then to `look_person` or the overview first) |
| `go overview`, `go high`, `go look_person` | There, via 'high'. The drawer and pill poses are reached only by Sidekick's routines |

Every move goes through Sidekick's paths that already worked: `to_high`, `go_overview`, `replay`'s capped
joint moves, `move_to`. The new moves are the two short hops near the person, and they are planned with
the person's head and body as obstacles at a capped speed. If the planner says no, the item is handed
over at the taught spot as before.

## Files

| File | What it's for |
|---|---|
| `care.py` | Sidekick's routines with the camera in the loop (above). |
| `settings.py` | Every name, offset and limit. Lines marked **TODO** must be measured at the lab. |
| `check_setup.py` | Run this first. It checks that the camera gives color and depth, its intrinsics, and its frames. |
| `calib_check.py` | The touch test: how wrong are the camera's 3D positions? Fixes a constant offset. |
| `thermometer.py` | Face, forehead and depth in; the thermometer tip to the forehead out. |
| `person.py` | From the face to a hand-over point in front of the person, and the person as obstacles. |
| `knob.py` | Finds the drawer's yellow knob in 3D: remember it, then check it later. |
| `locate.py` | Table, objects and their tops; grabs the bottle cap or the tissue where they really are. |
| `markers.py` | Optional. Makes a taught routine (the drawer) follow a printed AprilTag. |
| `snapshot.py` | Saves a 3D picture to work on later (`--offline`). |
| `geometry.py`, `depth_tools.py`, `face.py`, `robot_io.py` | The parts the scripts are built from. |
| `sim.py` | A pretend arm, camera, table, drawer, person, and a pretend Sidekick robot. |
| `tests/` | 80 tests (`python -m pytest -q`), all passing. |

## How it works (one paragraph)

The depth camera gives a distance for each pixel. A pixel `(u, v)` with depth `d` is a 3D point in
the camera's frame: `x = (u - cx)·d / fx`, `y = (v - cy)·d / fy`, `z = d`. Viam's frame system knows
where the camera is on the wrist, so `transform_pose` turns that point into world coordinates, and
the arm is commanded in those same coordinates. Everything else here decides **which pixel** to use
(the forehead from face landmarks, the top of an object, the yellow knob) and whether to trust it:
two looks must agree, nothing may be cut off by the edge of the picture, the person must not move.
If a check fails, the script stops and says why instead of guessing. Depth comes from the camera's
point cloud, which the RealSense module already lines up with the color picture.

## Safety rules near a person

- Do the paper face first. A real person comes only after 5 of 5 good runs.
- Someone else holds the E-stop. The person sits still with their hands down, and nobody leans in.
- `--execute` asks before every stage. Use `--yes` only after it worked 5 of 5 on the paper face.
- The thermometer approach is slow and straight. The planner is told about the head and body.
- It refuses if the person moves between two looks or by the second look, if the approach angle is
  wrong, or if a target is out of reach.
- It looks at the person once, about a minute before the hand-over (the wrist camera points down
  there and can't see the face). The person should stay seated.
- Ctrl+C stops the script and the arm. It won't move on its own after that; use the Viam app.

## When something goes wrong

| You see | Likely cause and fix |
|---|---|
| `STOPPED: no face in the picture` | The arm isn't at `look_person`, or the face is turned away. `python care.py go look_person`. |
| `the drawer box is N mm from where its poses were taught` | Someone moved the box, or the drawer is open. Put it back on its marks, or re-teach and `remember-drawer --execute`. |
| `I can't see the drawer's yellow knob` | The knob is out of the picture, or the light changed its color. Check `out/drawer_check.png`; set `KNOB_VIEW` to a pose that sees it, or widen `KNOB_HSV_*`. |
| `Handing over at the taught spot: you are N mm from…` | The person sits far from where 'handoff' was taught. Re-teach 'handoff' nearer, or raise `MAX_HANDOVER_SHIFT_MM`. |
| `No safe path to the person` | The planner refused the move with the person obstacles. It handed over at the taught spot; check `HANDOVER_*`. |
| `depth is NOT aligned to color`, or red edges shifted in `setup_alignment.png` | The point cloud wasn't usable. Set `"align_color_depth": true` on the camera. |
| Touch test errors of 2-9 cm | A consistent offset → `report --save`. An inconsistent one means the camera frame's orientation is wrong (ask a Viam engineer). |
| `can't see the forehead from the pre-approach` | The thermometer blocks the camera up close. Change the grip, or `thermometer.py --no-recheck` only if the person keeps still. |

## Try it at home (simulator)

```powershell
python face.py --webcam                                 # forehead tracking on your laptop camera
python care.py remember-drawer --sim --execute --yes
python care.py medicine --sim --execute --yes
python care.py tissue --sim --execute --yes
python care.py thermometer --sim --execute --yes
python locate.py tissue --sim --execute --yes --verify
python calib_check.py look  --sim --sim-mount-error 12 -8 6 --sim-spot 0
python calib_check.py touch --sim --sim-mount-error 12 -8 6 --sim-spot 0   # spots 1-3 too, then:
python calib_check.py report --sim --save
```

The face model (`models/face_detection_yunet_2023mar.onnx`) is OpenCV's YuNet, MIT licence
(`models/YUNET_LICENSE.txt`).
