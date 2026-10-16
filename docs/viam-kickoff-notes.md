# Viam kickoff notes (Fri Oct 16, 9:30 crash course)

From the "[Code That Moves] Viam Overview" slides + Vaibhav's notes. Machine: **armfarm13**
(app.viam.com → hackathons org → Arm Farm location).

## What every machine has (per the slides; confirm with `python app.py resources`)
- Components: `arm` (viam:ufactory:xArm6 or xArm850), `gripper` (viam:ufactory:gripper), `cam` (viam:camera:realsense, D435).
- Obstacles: `table` and `wall` (erh:vmodutils:obstacle). The motion service plans around them;
  **direct arm moves (MoveToPosition / MoveToJointPositions) go straight through them.** Our `RealRobot` uses the motion service. Keep it that way.

## Frames (default config)
| Frame | Parent | Translation (mm) | Orientation |
|---|---|---|---|
| arm | world | 0, 0, 0 | z=1, th=0 |
| gripper | arm | 0, 0, 105 ("probably") | z=1, th=0 |
| cam | arm | 73, -40, 18 | z=1, th=90 |
- The 105 mm gripper offset is unverified. Check the fingertip position in the 3D SCENE vs. reality before trusting `TABLE_Z`.
- The camera frame is "good enough for most pick and place". For precision there's `viam:camera-calibration:handeye`.
  We still run our own `calibrate` (last month: 2-9 cm off).
- The 3D SCENE tab is a diagnostic: the arm, gripper and camera move there live.

## Vision options they recommend
- **Shape finder** `devrel:shape-finder:detector` + `viam:vision:detections-to-segments`. Good fit for blocks.
- YOLOv8 `viam-labs:vision:yolov8` (stock model / file / Hugging Face id).
- EfficientDet-COCO via `rdk:builtin:mlmodel` + `viam:mlmodel-tflite:tflite_cpu`.
- Also: hand gestures, AprilTag pose tracker.

## Helpers
- **Manual mode** (arm Configure panel): move the arm by hand to find a pose. Use it to teach OVERVIEW / slots / zones.
- **Arm position saver** `erh:vmodutils:arm-position-saver`: saves a static pose to return to later.
- Sandbox (CONTROL tab) for quick SDK code; inline modules / registry modules for packaged logic;
  Generic API + DoCommand is their suggested module interface.
- Data manager: captures images and joint positions; use it to review failed grasps or collect training photos.
- Viam MCP server: https://app.viam.com/mcp. Discovery services: `viam:realsense:discovery`, `viam:orbbec:discovery`.
- Workflow tip from the talk: scan the workspace from the start pose first; commands run one at a time.
