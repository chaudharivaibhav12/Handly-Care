"""Everything you tune on the day lives here. Code elsewhere should not need edits.

Coordinates are millimeters in the robot's WORLD frame (origin = arm base).
Every number marked TODO must be measured on the real arm Friday:
jog the arm in the Viam app, then run `python app.py where` to print the pose.
After every edit run `python app.py check`.
"""

# ---- Names of things in your Viam machine config (check them with `python app.py resources`)
ARM = "arm"
GRIPPER = "gripper"
CAMERA = "cam"            # the wrist depth camera
MOTION = "builtin"        # the motion service every machine has

# ---- How objects are found
#  "anything": ANY object, by 3D shape (point-cloud segmenter) + Claude names it (DEC-002, the goal)
#  "vision":   known objects via labelled segmenter(s) (shape finder / YOLO / color) -> world x, y
#  "slots":    objects start in taped slots; the camera only says which slot/zone (last resort)
MODE = "anything"

# "anything" mode (anything.py). Viam service: obstacles_pointcloud on `cam`, named:
ANYTHING_SEGMENTER = "anything-3d"
# Where blobs come from (DEC-003): "cloud" = our own height map of the camera's full point cloud;
# "segmenter" = the Viam obstacles-pointcloud service above (finds edges, not tops: EXP-002).
ANYTHING_SOURCE = "cloud"
CELL_MM = 5.0                 # height-map grid cell
MIN_OBJECT_CELLS = 8          # a blob must cover at least this many cells (8 x 25 mm^2)
MAX_BLOB_HEIGHT_MM = 400.0    # higher "cells" are a hand/arm/noise
CLOUD_STEP = 2                # use every 2nd point of the 921,600 (speed)
CLOUD_MAX_RANGE_M = 1.5       # ignore points farther than this from the camera
TABLE_TOP_WORLD_Z = 15.0      # REAL table top in world z (depth camera said 10-21 mm; obstacle box says -23)
GRIPPER_MAX_OPEN_MM = 84.0    # TODO measure: xArm gripper fully open, inside of the fingers
GRASP_MARGIN_MM = 10.0        # objects must be this much narrower than the opening
MIN_GRASP_HEIGHT_MM = 12.0    # flatter than this: fingers can't get a hold
MAX_GRASP_OBJECT_HEIGHT_MM = 200.0
TOP_BAND_MM = 30.0            # "the top" of an object = its highest 30 mm (for gripping tapered things near the top)
TOP_GRIP_DEPTH_MM = 15.0      # near-top grip: fingertips close this far below the top
MIN_OBJECT_HEIGHT_MM = 15.0   # cells lower than this are depth noise (±4-20 mm specks at 63 cm, Fri 2:25). Was 8.
MIN_OBJECT_PEAK_MM = 20.0     # a blob's highest part must reach this
MIN_OBJECT_WIDTH_MM = 15.0
MIN_OBJECT_POINTS = 40        # a real object has at least this many 3D points above the table (after 1-in-3 sampling)
ANYTHING_MAX_DIM_MM = 450.0   # bigger blobs are junk; a laptop (~310) still shows up, so it can be refused
MERGE_GAP_MM = 15.0           # segmenter pieces closer than this are one object (it splits a mug into 4-5)
FLAT_FRAGMENT_MM = 8.0        # pieces thinner than this are table noise, used to fit the (tilted) table plane
TRACK_RADIUS_MM = 80.0        # same object if it's within this of where we last saw/put it
# Things on the table that are not objects: name -> (x, y, radius). A big flat blob there is ignored.
FIXTURES = {"tray": (174, 550, 150)}  # center touched 2:42 PM
FIXTURE_MIN_DIM_MM = 120.0
FIXTURE_MAX_HEIGHT_MM = 45.0
FIXTURE_RIM_MM = 25.0         # anything lower than this near a fixture is its rim (the tray edge showed up as a 10 mm strip)
NAMER_MODEL = "claude-opus-5-5"   # names new objects from image crops (one call per batch)
NAMER_TIMEOUT_S = 20.0

# Segmenter services (Viam `detections-to-segments`), each wrapping one 2D detector.
# Value None  = use the detector's labels (a YOLO/ML detector that names things), matched via OBJECTS[...]["labels"].
# Value "<id>" = everything this segmenter finds IS that object (one color detector per object).
SEGMENTERS = {"objects-3d": None}   # detections-to-segments over `shape-detector` (devrel:shape-finder: labels "<shape>-<color>")
# Color fallback, if the ML detector isn't good enough:
# SEGMENTERS = {"seg-red": "red_cube", "seg-blue": "blue_cube", "seg-cup": "cup", "seg-yellow": "marker"}

# ---- Object catalog
# description: what the AI planner reads (so it knows a marker is for writing)
# labels:      detector labels that mean this object (YOLO class names)
# color/detector: used only in "slots" mode (one Viam color_detector per object)
# size:        footprint x, footprint y, height in mm (for spacing, stacking, "next to")
# grasp_height: where the fingertips close, mm above the table (about half the height)
# Prefer objects that look the same from any angle from above (cubes, cups, round fruit):
# a two-finger gripper can't grab a marker lying the wrong way. TODO: measure all.
# stackable: True if things can be put on top of it (flat, stable top).
OBJECTS = {
    "red_cube":  {"description": "a red toy block", "labels": ["red_cube", "red block", "square-red"],
                  "color": "red", "detector": "det-red", "size": (50, 50, 50), "grasp_height": 25,
                  "stackable": True},
    "blue_cube": {"description": "a blue toy block", "labels": ["blue_cube", "blue block", "square-blue"],
                  "color": "blue", "detector": "det-blue", "size": (50, 50, 50), "grasp_height": 25,
                  "stackable": True},
    "cup":       {"description": "a small paper cup, for drinks", "labels": ["cup"],
                  "color": "white", "detector": "det-white", "size": (65, 65, 80), "grasp_height": 45},
    "marker":    {"description": "a thick yellow marker standing upright, used for writing",
                  "labels": ["marker", "pen"],
                  "color": "yellow", "detector": "det-yellow", "size": (25, 25, 120), "grasp_height": 70},
}

TOOL_THETA = 145.0   # wrist angle (deg) for every straight-down pose. theta=0 -> planner: "self-collision
                     # wrist_link vs cam_origin" (Fri 11:40). Hand-taught poses read 136 (OVERVIEW) and 154 (table touch).
TABLE_Z = 63.0       # gripper-frame z with fingertips touching the table (armfarm13, Fri 11:18, `where` = 62.9).
                     # Not the true table height (frame sits ~60 mm above the tips); consistent because moves use the same frame.
HOVER = 120.0        # how far above a spot to hover before descending (mm)
# Fri 2:53: carrying tray -> left, the arm's hanging wire knocked over two other objects (EXP-005 pick 2).
CARRY_CLEARANCE_MM = 80.0   # carried object's bottom stays this far above the tallest other object (wire slack)
OBSTACLE_MARGIN_MM = 20.0   # other seen objects go to the motion planner as boxes this much bigger, on carry moves
UNKNOWN_OBSTACLE_SIZE = (80.0, 80.0, 150.0)   # size assumed for a seen object we have no measurement for
PLACE_DROP_MM = 2.0  # release this far above the exact resting height (was 5: a standing cylinder tipped, Fri 2:23)
TALL_RATIO = 1.3     # height > 1.3 x width = tall and thin: grip higher (65% up). Was 1.5: missed the real cylinder (52x37x55 = 1.49)

# ---- Taught places (x, y in world mm). TODO: teach each one with `python app.py where`.
# Zones are named places to deliver to (mark with tape). Slots are pickup spots for
# "slots" mode AND the known positions used for camera calibration.
# All four slots touched with `where` (slot1/2 Fri 12:14, slot3/4 Fri 2:16 PM). Zones = camera estimate + offset (±10 mm).
SLOTS = {
    "slot1": (197, 164),   # touched
    "slot2": (277, 64),    # touched
    "slot3": (369, -14),   # touched
    "slot4": (456, 76),    # touched
}
ZONES = {
    "tray":  (174, 550),   # touched center 2:42 PM (z 62.4 = table). Teal tray lengthwise, ~x 20-263, y 445-614.
    "to_me": (386, 414),   # touched 2:27 PM
    "left":  (389, 225),   # touched 2:27 PM; person's left (person sits on the +y side)
    "right": (167, 347),   # touched 2:27 PM
}
# Height of a zone's floor above the table (mm), e.g. a tray. Tray floor touched at z 63.1 = TABLE_Z: 0.
# (The table itself slopes ~4 mm: z 58.7 at the slots vs 63 near the tray. TABLE_Z 63 = grasps up to 4 mm high there.)
ZONE_LIFT = {"tray": 0.0}
ALL_SPOTS = {**SLOTS, **ZONES}
ZONE_RADIUS_MM = 75.0   # 15 cm taped squares (Fri 12:30; was 90 for 18 cm). Within this of the center = "in" it.
                        # Zones are ~18 cm across so 2-3 objects fit (tape or a tray that size).

# Pose the arm goes to so the wrist camera sees the whole table. Fri 12:15: view ends before the wall (x<552), centred on the reachable table.
# History: (353,406,499) -> (275,380,560). Higher is out of reach (600-650 failed IK). Sees ~88 x 50 cm: results/table-targets.jpg.
OVERVIEW = (207, 283, 560)
OVERVIEW_THETA = 178.0   # wrist angle at OVERVIEW only: image lines up with the table edges (145 was ~33° off; Fri 11:50)

# ---- Where the robot may put things (mm, world). TODO: set from the real table.
WORKSPACE = {"x": (80.0, 480.0), "y": (-60.0, 640.0)}   # Fri 12:30: x < 480 keeps clear of wall-front (face x~558)
# Boxes (x0, x1, y0, y1) inside WORKSPACE the arm can't reach top-down. Fri 3:47 (EXP-006): at (454, 135-141)
# cam_origin hit wall-front at every carry height. Slot4 (456, 76) was taught there by hand, so the strip starts at y 100.
NO_GO = [(440.0, 480.0, 100.0, 640.0)]
MIN_REACH_MM = 200.0     # not too close to the arm's own base
MAX_REACH_MM = 650.0     # xArm6 top-down reach used by last month's teams

# ---- Precise placement
# "toward me": the world direction pointing from the table toward the person.
# Left/right/behind are derived from it. TODO: check on Friday ("+x", "-x", "+y" or "-y").
USER_DIRECTION = "+y"
NEXT_TO_GAP_MM = 30.0       # gap between objects for "next to", "left of", ...
MIN_CLEARANCE_MM = 15.0     # placed objects must not overlap anything by less than this
PLACE_TOLERANCE_MM = 20.0   # a placement within this of the target counts as done
CALIBRATION_PATH = "calibration.json"
AUTO_CALIBRATE_RADIUS_MM = 80.0   # `calibrate auto`: an object this close to a slot is "on" it (camera was ~24 mm off)

# ---- Detection filtering (lessons from last month's teams on this hardware)
MIN_CONFIDENCE = 0.5     # ignore weaker detections
OBSERVE_FRAMES = 3       # look this many times per check and vote
MIN_HITS = 2             # an object must be seen in the same place in this many looks
MATCH_RADIUS_MM = 40.0   # detections this close across looks are the same object
FRAME_GAP_S = 0.15
SETTLE_S = 0.3           # pause after reaching the overview pose before looking
MAX_OBJECT_DIM_MM = 250.0  # bigger "objects" are segmentation junk (table, wall)

# ---- "slots" mode only: pixel regions in the overview image for each spot
# (x_min, y_min, x_max, y_max). TODO: `python app.py snapshot` with objects in each spot.
REGIONS = {
    "slot1": (0, 0, 160, 120),
    "slot2": (160, 0, 320, 120),
    "slot3": (320, 0, 480, 120),
    "slot4": (480, 0, 640, 120),
    "tray":  (0, 240, 210, 480),
    "left":  (430, 240, 640, 480),
    "right": (0, 120, 210, 240),
    "to_me": (210, 240, 430, 480),
}
MIN_BOX_PX = 20          # TODO Friday: smallest real object in snapshot (px), minus ~30%
MAX_BOX_PX = 160         # largest real object, plus ~30%

# ---- AI planner
PLANNER_MODEL = "claude-sonnet-5-5"
PLANNER_TIMEOUT_S = 15.0  # never let a slow AI call freeze the demo; fall back to keywords
MAX_RETRIES = 1           # extra attempts per object after a failed check

# ---- Motion behavior
# Straight up/down near objects ("linear constraint"). If the planner refuses these
# moves on the real arm, set this to False. 10 mm is what last month's winning team
# ran on this same xArm setup.
STRAIGHT_LINE = True
LINE_TOLERANCE_MM = 10.0

# ---- Medicine drawer (Branch E, DEC-005): replayed from hand-taught JOINT poses (taught.py, drawer_joints.json).
# The planner refused tilted straight lines (EXP-007, 5:06-5:09 PM). Old Cartesian poses are in LOG 4:52.
# The winner found the gripper's "holding" flag weak. Test: grab with nothing in the
# jaws. If it still reports a grab, set this to False and rely on the camera check.
TRUST_GRAB_SENSOR = True

STATUS_PATH = "status.json"   # live state for a dashboard
TRIALS_PATH = "trials.csv"    # every test/pick attempt, for `python app.py stats`
DRAWER_JOINTS_PATH = "drawer_joints.json"   # joint poses taught by hand (taught.py)
# Joint-space replay speed caps (MoveOptions, obeyed by the arm itself). Vaibhav, 4:55 PM: half speed for the drawer.
DRAWER_MAX_VEL_DEGS = 15.0
DRAWER_MAX_ACC_DEGS = 30.0
CANCEL_ACCEPT_DEG = 3.0       # a taught joint move that comes back 'canceled' this close to its target counts as done (Sat: drawer end stop)
JOINT_MOVE_TRIES = 5          # taught joint moves: attempts on 'context canceled' (Sat 1 PM: 3 in a row killed a medicine run)
# Tissue (Fri 7:02 PM): pinch the tissue sticking out of a pop-up box and pull ONE out, then hand it over.
TISSUE_GRIP_BELOW_TOP_MM = 15.0   # fingertips this far below the measured top (the tissue puff), above the cardboard
TISSUE_PULL_MM = 250.0            # straight up this far: a pop-up tissue slides out whole
HANDOFF_HOLD_S = 0.5           # hold the item at the patient's hand this long before letting go (Sat: 3.0 -> 0.5, Vaibhav)
HANDOFF_JOINT_MATCH_DEG = 10.0  # after the tissue: within this of the taught 'handoff' joints -> the medicine's quick way back
# Thermometer (Sat): IR thermometer on a taped spot, ball on a taped spot; the light on the ball shows it works.
# Its button is one click; held down, the light goes off after ~2 s.
THERM_MODE = "press"            # "press": hold gently, click at the ball (needs the 2 positions below); "fast": the grip clicks, swing fast
THERM_HOLD_POS = 260            # measured Sat 11 AM: holds the thermometer, button not pressed
THERM_PRESS_POS = 250           # measured Sat 11 AM: clicks the button (~1 mm tighter)
THERM_CLICK_S = 0.01            # press this long, then back to HOLD: held down it doesn't measure
THERM_SHOW_S = 2.0              # keep pointing at the ball this long
THERM_GRIPPER_SETTLE_S = 0.6    # wait after a gripper position change
THERM_FAST_VEL_DEGS = 60.0      # "fast" mode swing grip -> aim; 120 after a clean run (arm config speed must allow it)
THERM_FAST_ACC_DEGS = 300.0
# Bottle cap (Sat 1:35 PM): a person holds the bottle; the arm unscrews the cap with re-grip twists of the last joint.
CAP_TWISTS = 3                  # grip-twist-release cycles (Vaibhav: 3 x 120 deg ~ one full turn)
CAP_TWIST_DEG = -120.0          # per twist; MINUS = anticlockwise seen from above? CHECK with a dry twist first
CAP_TWIST_VEL_DEGS = 90.0
CAP_TWIST_ACC_DEGS = 360.0
WRIST_LIMIT_DEG = 355.0         # xArm6 joint 6 range is +-360 deg
CAP_CUP_SPOT = "slot3"          # the small cup the cap is dropped into (Sat 2 PM)
CAP_CUP_HEIGHT_MM = 95.0        # Vaibhav: 9.5 cm tall, 7.8 cm wide
CAP_DROP_ABOVE_RIM_MM = 30.0    # fingertips this far above the rim when letting go
CAP_DROP_CLEAR_MM = 80.0        # carry height at least this far above the drop point
# Pouring (Sat 2 PM): bottle on slot 2, cup to fill on slot 1 (positions are the taught poses, taught.py).
POUR_VEL_DEGS = 10.0            # the tilt into and out of the pour: slow, no splash
POUR_ACC_DEGS = 20.0
POUR_HOLD_S = 3.0               # keep pouring this long
BOTTLE_HOLD_POS = 550           # gripper opening that holds the bottle without crushing it (Vaibhav, tools/gripper_pos.py)
POUR_APPROACH_VEL_DEGS = 30.0   # the planned move from 'high' to bottle_approach (a ~95 deg swing)
DRAWER_CLOSE_SHORT_MM = 10.0
DRAWER_EXTRA_OPEN_MM = 30.0    # pull this far past the taught 'open' (6:31 PM: wanted ~3 cm more); 0 = as taught   # close stops this short of 'grip' (5:35 PM: full close pushed the box back ~1 cm)
DRAWER_MAX_FIRST_JUMP_DEG = 45.0
# to_high: from anywhere to the drawer's 'high' pose, one joint at a time (Fri 7:15; did by hand at 6:43 and 7:13).
TO_HIGH_ORDER = (5, 0, 3, 4, 2, 1)   # wrist roll, base, forearm roll, wrist pitch, elbow, shoulder (7:19: shoulder-before-elbow dipped to z 175)
TO_HIGH_STEP_DEG = 15.0              # biggest single step
TO_HIGH_ZMIN = 170.0                 # stop if the gripper frame gets lower than this ('high' itself is z ~195)
TO_HIGH_LIFT_BELOW_Z = 260.0         # starting lower than this (and not already at high): lift j2/j3 back first
TO_HIGH_LIFT_DEG = 15.0   # refuse to start if reaching 'high' turns any joint more (a blind joint move)
