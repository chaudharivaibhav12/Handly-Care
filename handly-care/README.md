# Handly Care

Nurse-station dashboard for the bedside arm demo. Five rooms are simulated; one room
(Room 12 by default) can run on your real Viam arm.

## Run it

Needs Node.js 20 or later. Use Chrome or Edge (Firefox blocks WebRTC from localhost).

```bash
npm install
npm run dev
```

Open the address Vite prints (usually http://localhost:5173).

## Connect the arm

1. In the Viam app, open your machine and go to the **CONNECT** tab. Copy the machine
   address, the API key ID and the API key.
2. In the dashboard, click **Connect Arm**, paste them in, and enter the arm's
   component name exactly as it appears in your machine config (and the gripper name,
   if you have one).
3. Pick which room the real arm stands in.

## Teach positions

Move the arm to each spot (freedrive, the Viam app's arm card, or a pendant) and press
**Save current**:

| Position | Where |
|---|---|
| Home | Folded, clear of the patient |
| Above the item | About 10 cm over the tray spot |
| At the item | Gripper around the item |
| Patient's hand | Where the patient takes it |
| Near forehead | Thermometer about 4 cm from the forehead (mount a non-contact thermometer on the arm) |

Positions are saved as joint angles in this browser. Press **Test Hand Over** to run
the full sequence once, with your hand near **Emergency Stop**.

The hand-over sequence is: open gripper → above item → at item → grip → lift →
patient's hand → hold (seconds set in setup) → release → home.

Every item uses the same tray spot, so place the next item there between demo beats.

## Safety notes

- Moves use `MoveToJointPositions`, which skips Viam's collision checks. Teach
  positions with clear space around the arm and keep the arm's speed limits low in its
  Viam config.
- **Emergency Stop** in the dashboard calls the arm's `Stop`. It is a software stop.
  Keep the arm's hardware e-stop within reach during every test and shoot.
- The API key sits in the browser. Use a key scoped to this one machine, run the page
  only on your own laptop, and do not deploy it publicly.
