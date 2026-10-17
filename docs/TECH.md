# Handly Care / Sidekick: what it's built with

A bedside care assistant for people with limited hand mobility. A UFactory xArm6 hands over a tissue, takes the
pill box out of a drawer and hands it over, takes a (no-contact) temperature, and opens a water bottle and pours a
cup. It's driven from a nurse-station web dashboard and by voice.

```
 Browser (Handly Care dashboard)  ── voice in (Web Speech API) / voice out (speechSynthesis)
        │  HTTP on this laptop only (localhost:8765)
        ▼
 bridge.py (Python)  ── runs ONE tested routine at a time: python app.py <routine>
        │                 asks Claude to map a spoken sentence to one allowed routine
        ▼
 app.py + robot.py (Python, Viam Python SDK)  ── taught joint replays, planned moves, camera, safety checks
        │  Viam (WebRTC/gRPC over the internet, API key)
        ▼
 armfarm13 (Viam machine): xArm6 arm + xArm gripper + wrist RealSense D435 + Viam motion service
```

---

## 1. Viam: what we used from the platform

**The machine (set up for us by Viam, "armfarm13")**. We only saw the resource names (`arm`, `gripper`, `cam`,
`builtin`, obstacle geometries). The model names below come from **Viam's kickoff talk** (docs/viam-kickoff-notes.md);
we did **not** check them in the machine's config.
| Resource | Model per the kickoff notes (unverified) | What we use it for |
|---|---|---|
| `arm` | `viam:ufactory:xArm6` (or xArm850) | every movement |
| `gripper` | `viam:ufactory:gripper` | open / grab, and **position control** via `do_command({"set": 0..850})` (thermometer click, bottle hold; ran on the arm Sat 11:21, LOG) |
| `cam` | `viam:camera:realsense` (Intel RealSense D435 on the wrist) | color images + **depth point cloud** |
| `builtin` motion service | built into viam-server | collision-aware path planning |
| `table`, `wall-front`, `wall-side`, `ceiling` | obstacle geometries in the machine's frame system | the planner keeps the arm out of them |

**Viam SDK and features we call** (Python `viam-sdk` 0.84, running on the laptop as a remote client)
- `RobotClient.at_address` with an **API key** (from `.env`, never in the browser).
- **Arm API:** `get_joint_positions`, `move_through_joint_positions` with `MoveOptions` (per-call speed and
  acceleration caps; this is how the taught routines are replayed), `get_end_position`, `stop`.
- **Motion service:** `motion.move` with a `PoseInFrame` target (tool pointing down or tilted), a `WorldState` of
  extra obstacles (objects the camera saw, so carry moves go around them), `Constraints(LinearConstraint)` for
  straight-line moves down/up, and `get_pose` for where the gripper is.
- **Frame system:** `robot.transform_pose` (camera frame → world).
- **Camera API:** `get_images` (color) and `get_point_cloud` (921,600-point depth cloud).
- **Gripper:** `open`, `grab`, `do_command` (`set` / `get` position, the way last month's winner used it).
- **Sessions:** the SDK's default safety heartbeat (it's the likely source of the intermittent `context canceled`
  we work around; see §3).

**Tried / researched and not in the final path**
- Viam `obstacles_pointcloud` vision service (`anything-3d`): it found object edges, not tops, so footprints were
  30–60% too small (EXP-002). Replaced by our own height map (DEC-003).
- `devrel:shape-finder` + `detections-to-segments` (colored-shape detector), the original detector ladder.
- `devrel:so101` module for a second arm (SO-101) to hold the bottle: researched, dropped because it couldn't hold
  the bottle firmly. A person holds it instead.

---

## 2. What we built ourselves

**Robot logic (Python, ~this repo)**
- **Taught-pose replay (`taught.py`, `robot.replay`)**: we move the arm by hand (manual mode), save the **joint
  angles** under a name (`python app.py joints <name>`), and replay them at capped speed with no planner. That's how
  the drawer, pill box, thermometer, bottle cap and pour work. Steps: move / slow / fast / grab / set (gripper
  position) / click (press and release a button) / turned (twist the last wrist joint) / wait.
- **Routines:** `tissue`, `medicine` (drawer open → pill box → hand over → drawer close), `thermometer` (gentle grip
  at 260, 0.01 s click at 250 to fire the IR thermometer at a ball), `cap` (3 grip-twist-release turns of the
  wrist, lift the cap, drop it in a cup), `pour` (side grip at 510, slow 10°/s tilt, upright, back on its spot),
  `water` (cap + pour), `to_high` (careful return to the start pose from anywhere).
- **Perception (`heightmap.py`, `anything.py`, `perception.py`)**: our own **height map** of the full depth point
  cloud (5 mm grid, fitted table plane, blobs that stick up), size + height per object, **voting over several
  looks**, then **Claude names each object from an image crop** ("tissue_box"). Graspable or not comes from the
  measured size, so refusals carry a reason.
- **Calibration (`calibration.py`)**: an affine fit of camera-seen vs. hand-taught positions (8 points, ~9 mm average
  error), refusing wild fits.
- **Planner + places (`planner.py`, `places.py`, `executor.py`)**: the original plain-English desk arranger: Claude
  turns a sentence into a JSON plan, our code validates it (the safety layer), resolves spatial words ("left of the
  cup", from the person's point of view), places objects and re-checks with the camera (mm error, nudges).
- **Safety layer:** never retry after an E-stop; lifts keep the tool's tilt; a routine started away from the start
  pose goes there carefully first only if the gripper is empty (else refuses); a failed grab says "I missed it"
  instead of handing over nothing; a move reported "canceled" within 3° of its target counts as done; resends never
  drag a gripped knob backwards; wrist-limit check for the cap twist; failed tissue returns home; the demo stops at
  the first failed task.
- **Bridge (`bridge.py`)**: Python standard library HTTP server (`http.server`) that serves the dashboard and runs
  one whitelisted routine at a time as a subprocess, reports its live phase, saves every run's full output, and
  stops it with Ctrl+Break (the routine then stops the arm on its own connection). Localhost only, same-origin only.
- **Voice intent (`voice.py`)**: "stop / halt / freeze" handled locally and instantly (no AI); "start the demo
  [in this order]" parsed locally; everything else → Claude picks exactly one allowed routine (checked in code, so
  the AI can't invent one), keyword fallback if the AI is slow or fails.
- **Research log:** every experiment and decision recorded (`research/`: EXP-001…009, DEC-001…007, LOG.md), with a
  validator; **416 automated tests** (pytest), each hardware bug got a regression test.

**Front end (Handly Care dashboard, `handly-care/`)**
- Built by a teammate: a nurse-station dashboard in **plain HTML, CSS and JavaScript (no framework)**, Apple-style
  design system, simulated rooms and a scripted night-shift story (meds due, SpO₂ alert, bathroom request).
- We replaced its direct-to-Viam browser code (`@viamrobotics/sdk` in the browser, which put the API key in the
  page) with a small client (`src/live.js`) that talks to our bridge.
- We added: **Start demo** (one checklist, then tissue → medicine → thermometer → water back to back, stops at the
  first failure), **voice bar** (Speak button / typed requests), **spoken cues** at each step ("Opening the drawer."
  "Here's your medicine." "The cap is off. You can let go of the bottle now."), routine list with live progress, full
  width arm output.
- Browser APIs: **Web Speech API** `SpeechRecognition` (Chrome/Edge speech to text) and **`speechSynthesis`**
  (text to speech); `fetch` polling twice a second.

---

## 3. AI and other tech

| What | Used for |
|---|---|
| **Claude** (Anthropic), via **OpenRouter** with the official `anthropic` Python SDK | naming objects from camera crops (Opus 5.5); turning plain English into a validated plan, and spoken requests into one routine (Sonnet 5.5). Short timeouts, no silent retries, keyword fallback. |
| Python 3.11 | everything on the laptop; `python-dotenv` for keys, `pillow` for image crops, `pytest` for tests |
| Git + GitHub | github.com/chaudharivaibhav12/Handly-Care |

**Known limits (honest list for Q&A)**
- Drawer, medicine, thermometer and pour are **taught replays**: the boxes must stay taped where they were taught.
  Next step: look with the camera before grabbing (e.g. confirm the pill box is in the open drawer).
- Intermittent Viam `context canceled` on long moves (likely the session heartbeat over venue Wi-Fi). Mitigated with
  retries that resume forward and "close enough" acceptance. Long-term fix: run the routines **on the robot as a
  Viam module** (Generic service + DoCommand), so no Wi-Fi is in the loop while the arm moves.
- The temperature shown is a **demo reading**; the thermometer is not integrated as a Viam sensor yet.
