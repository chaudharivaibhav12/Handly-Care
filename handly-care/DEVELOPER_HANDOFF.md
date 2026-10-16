# Handly Care — developer handoff

Handly Care is a nurse-station web dashboard for a demo of a bedside robot arm in a hospital
general-surgery unit. One room drives a real arm (UFactory xArm 6 via Viam); the other rooms
are simulated. This document covers what exists, how it works, and what is left to do.

**Status:** demo prototype. UI complete and tested in Chromium against a simulated arm.
**The Viam integration has never been run against real hardware.**

---

## 1. Repository layout

```
handly-care/
├── index.html              # Vite entry: full dashboard markup + CSS (generated, see §6)
├── src/
│   ├── dashboard.js        # All UI logic, demo script, rendering (generated from preview-src/ui.js)
│   └── live.js             # Viam connection, polling, taught positions, hand-over/measure sequences
├── public/favicon.svg
├── brand/                  # Logo SVGs (text outlined), brand sheet PNG, make-logo.py
├── preview-src/            # Source of truth for UI + single-file preview build
│   ├── body.html           # Markup
│   ├── style.css           # All styles + design tokens
│   ├── ui.js               # UI logic (same as src/dashboard.js minus the import line)
│   ├── mock-live.js        # Simulated arm with the same interface as live.js
│   ├── build.py            # Builds index.html + src/dashboard.js + single-file preview
│   └── handly-care-preview.html   # Self-contained preview with mock arm (no network)
├── package.json            # vite ^5.4, @viamrobotics/sdk 0.74.1 (pinned)
└── README.md               # Operator setup: connect, teach positions, safety notes
```

## 2. Run it

Requirements: Node 20+, Chrome or Edge. Firefox blocks WebRTC from `localhost`.

```bash
npm install
npm run dev        # http://localhost:5173
```

To preview the UI with no hardware, open `preview-src/handly-care-preview.html` directly in a
browser. In Arm Setup, click **Connect** with empty fields to attach the simulated arm.

## 3. Architecture

Plain JS, no framework. One global state object `S`, full re-render of each region on change,
a 250 ms interval drives the simulation clock.

```
dashboard.js
 ├─ S (state): sim clock, job queue, rooms{}, queue[] (nurse items), log[], KPIs
 ├─ SCRIPT[]          scripted demo beats, fired by timer or "Next event" (key N)
 ├─ armTask(room,…)   routes to liveTask() for the real-arm room, else simulated phases
 ├─ liveTask()        calls live.handover() or live.measure(), maps steps → progress bar
 ├─ act()/roomAct()   nurse actions on queue items and rooms (release, on my way, stop…)
 └─ render*()         renderQueue / renderArms / renderStats / renderControls / renderLive
live.js (export live, POSES)
 ├─ connect(cfg)      VIAM.createRobotClient(api-key) → ArmClient (+ optional GripperClient)
 ├─ polling 400 ms    isMoving(), getJointPositions(), getEndPosition()
 ├─ capture(key)      saves current joint angles as a named position (localStorage)
 ├─ handover()        open → above → pick → grab → above → handoff → hold → open → home
 ├─ measure()         announce → forehead → read (wait) → home
 └─ stop()            sets abort flag, arm.stop(), gripper.stop()
```

`mock-live.js` implements the same object shape (`status`, `ready`, `poses`, `pose`, `joints`,
`connect`, `capture`, `handover`, `measure`, `stop`, `on`) with timers, so the UI cannot tell
them apart. It sets `preview: true`, which skips connection-field validation.

### Simulation timing
- `CLOCK_RATIO = 30`: unit clock advances 30 s per real second (starts 02:10).
- `RESP_RATIO = 10`: simulated arm response-time scale for the KPI.
- `STEP_GAP = 9000` ms: while running, the next script beat fires automatically after 9 s.
  "Next event" fires immediately. For filming, pause and step manually.

### Persistence (localStorage, per browser)
- `bedside-arm-poses`: taught joint angles.
- `bedside-arm-connection`: credentials, only if "Remember on this computer" is ticked.
- `bedside-arm-settings`: `{ room, hold }`.

## 4. Demo content

Unit: 5 North, general surgery, night shift. Nurse Frank Sinatra, RN; aide J. Ortiz, CNA. All fictional.

| Room | Patient | Procedure | Flags |
|---|---|---|---|
| 10 | M.K., 74 F | Lap. cholecystectomy, POD 1 | Fall risk |
| 11 | J.R., 81 M | Inguinal hernia repair, POD 1 | Sleep apnea, Delirium risk |
| 12 | A.T., 68 F | Lap. appendectomy, POD 2 | — (default real-arm room) |

Script (`SCRIPT[]` in ui.js):
1. Room 12 meds due → Nurse Queue "Approval" → **Release pill box** → arm hands pill box.
2. Room 12 voice request → arm hands water bottle.
3. Room 11 SpO₂ 88% falling → "Critical", arm locked for that room → **On my way** → **Mark attended**.
4. Room 12 temperature check → arm moves near forehead → 38.4 °C → "Review" item.
5. Room 10 bathroom request → not an arm task (fall risk) → **I'll go** / **Send CNA**.

Arm rules shown in the UI: arm on its own (water, small items) · nurse release (meds) ·
arm measures, nurse reviews (temperature) · speaks before it moves · nurse only (vitals,
getting out of bed, confusion, pain).

## 5. Viam integration (live.js)

- SDK calls used: `createRobotClient`, `ArmClient.getJointPositions / moveToJointPositions /
  getEndPosition / isMoving / stop`, `GripperClient.open / grab / stop`.
- Connection: `signalingAddress: 'https://app.viam.com:443'`, credentials
  `{ type: 'api-key', authEntity: keyId, payload: key }` from the machine's CONNECT tab.
- Joint units: the TS docs example uses degrees; Python is degrees, Go is radians. Verify on the
  real arm before trusting saved angles.
- Taught positions (5): `home`, `above`, `pick`, `handoff`, `forehead`. All must be set before
  `live.ready` is true. Without them, requests for the live room fall back to simulation and log why.
- Without a gripper name, gripper steps are skipped.
- `measure()` assumes a non-contact thermometer is mounted on the arm. The 38.4 °C reading is
  hard-coded; no thermometer is integrated.

## 6. Build pipeline

`preview-src/` is the source of truth. `build.py` (run from the original workspace layout)
writes three outputs from `body.html` + `style.css` + `ui.js`:
- `index.html` (Vite entry, with favicon link)
- `src/dashboard.js` (`import { live, POSES } from './live.js'` + ui.js)
- single-file preview (mock-live.js + ui.js inlined)

Paths inside build.py point at the original scratch layout (`web/`, `bedside-arm-live/`).
Adjust them, or drop the generator and edit `index.html` / `src/dashboard.js` directly.
Recommended: move to a normal Vite setup with `style.css` and `ui.js` as modules and delete build.py.

## 7. Design system

Apple-inspired, light mode only (dark mode was removed on request; `color-scheme: light`).
Tokens in `:root` of style.css:

- Surfaces: `--bg #f5f5f7`, `--panel #fff`, `--side #fbfbfd`, `--line #e5e5ea`
- Text: `--text #1d1d1f`, `--text2 #6e6e73`, `--text3 #86868b`
- Semantic: `--blue #0071e3` (actions), `--red #e3342f` (critical/stop), `--orange #c76b00`
  (review/flags), `--green #248a3d` (ok/nurse), `--teal #0a8fa6` (arm), `--indigo #5856d6`
- Font: system stack (`-apple-system, SF Pro Text, …`). Buttons 30 px tall, 6 px radius; panels 10 px radius.
- Logo: rounded-square icon, gradient `#3FD0E6 → #0A7CFF`, white "h" with offered dot.
  Wordmark "Handly" SemiBold + "Care" Regular, Inter Display, outlined.

## 8. Known issues

1. **Emergency Stop button clips** at the right edge of the Arms Log table when the real-arm
   room is working at 1440 px wide. Fix: shorten the label at narrow widths, put it under the
   activity text, or give the table `table-layout: fixed` with an explicit action column.
2. Autoplay keeps firing beats every 9 s even if the nurse hasn't acted on an approval yet.
   Consider gating step 2 on the pill-box release.
3. The patient's spoken request isn't shown anywhere since the activity log was removed. Consider
   showing it in the Activity column while the arm works.
4. Static KPI numbers in body.html (7 / 28) are overwritten at startup by state (4 / 16); clean up.
5. The sidebar "Arms Log" / "Nurse Queue" items scroll within one view; there are no separate pages.
6. Not checked: screen readers, keyboard-only flow beyond focus rings, browsers other than Chromium.

## 9. Before running on hardware

- Lower the arm's speed and acceleration limits in its Viam config.
- `moveToJointPositions` skips collision checks. Teach positions with clear space around the arm.
- Dashboard Emergency Stop is software only. Keep the hardware e-stop within reach.
- The API key is in browser memory/localStorage. Use a key scoped to this one machine; never host
  the app publicly.
- First run: connect, read Live Position, teach Home, run **Test Hand Over** at low speed.

## 10. Next steps (agreed ideas, not started)

- Real-arm test and fixes (§5).
- "Shift impact" KPI strip and an ROI page with editable assumptions (nurse cost/hour,
  minutes per avoided trip, arm cost, lifespan, beds) and payback chart.
- Real thermometer integration (sensor component in Viam) instead of the hard-coded reading.
- Formal USPTO clearance for "Handly Care" (classes 7, 9, 10, 42, 44) before public use.
