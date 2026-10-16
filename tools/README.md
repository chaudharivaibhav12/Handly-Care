# tools/: diagnostic scripts (run from the project root with the venv python)

| Script | What it does | Moves the arm? |
|---|---|---|
| `python tools/lift_then_overview.py` | lift straight up if low, then go to OVERVIEW; prints the error | yes |
| `python tools/three_looks.py OUT.jpg OUT.json` | 3x perception.find in one session: per-object spread + size; overlay photo | no (already at overview) |
| `python tools/find_overlay.py OUT.jpg` | one find, draws each object's footprint/size on a fresh photo | no |
| `python tools/heightmap_view.py OUT.png` | renders the height map (gray table, colored heights, black = no depth) | no |
| `python tools/view_map.py Z OUTDIR nomove` | photo + camera geometry (map.json) for drawing spots | no ("move" = yes) |
| `python tools/draw_spots.py DIR OUT.jpg .` | draws taught spots on DIR/color.jpg using DIR/map.json (SPOTS env var, CLEAN=1) | no |

Overlays use `results/overview-map.json` (camera geometry at OVERVIEW, uncalibrated), so boxes can sit ~1-2 cm off;
the arm itself uses calibration.json.
| `python tools/plan_only.py "req" ...` | one look, then the AI plan + resolved target for each request | no |
| `python tools/open_gripper.py` | opens the gripper only (hold what it carries first) | gripper only |
| `python tools/probe_descent.py Z_MIN [STEP]` | straight down in steps keeping the tilt; prints the first refusal; back up | yes (straight only) |
| `python tools/unwind_joint.py J TARGET [--lift D] [--zmin Z]` | slowly turn one joint (index from 0) in 30° steps, stop if the gripper drops below zmin; optional lift first | yes (slow, joint space) |
