"""Every name, pose, offset and limit you tune at Open Lab.

Units: millimeters and degrees. Frames follow Viam: "world" is the arm base with +z up.
Anything marked TODO has to be measured on the real arm before you trust it.
Values marked "armfarm13" were measured at the hackathon station (sidekick/STATUS.md).
"""

# ---------------------------------------------------------------------------
# Viam resource names. `python check_setup.py` prints what your machine has.
# ---------------------------------------------------------------------------
ARM = "arm"            # armfarm13 names (same as sidekick/config.py)
GRIPPER = "gripper"
CAMERA = "cam"         # the depth camera on the wrist (RealSense D435 or Orbbec)
MOTION = "builtin"
COLOR_SOURCE = "color"
DEPTH_SOURCE = "depth"

# The hackathon code, for care.py (taught poses, .env, robot routines): Handly-Care, the parent folder.
SIDEKICK_DIR = ".."

# ---------------------------------------------------------------------------
# Depth camera
# ---------------------------------------------------------------------------
# Where per-pixel depth comes from:
#  "auto"        the point cloud if it is one point per color pixel (the RealSense module aligns it to
#                color for you), else the depth image
#  "point_cloud" only the point cloud;  "images" only the depth image (needs align_color_depth: true)
DEPTH_FROM = "auto"
DEPTH_VALID_MM = (150, 2500)   # readings outside this range count as "no data"
CAPTURE_SETTLE_S = 0.4         # wait after a move before taking a picture (the wrist wobbles)
MAX_CAPTURE_DRIFT_MM = 2.0     # if the camera moved more than this during a capture, retake it
CAMERA_CORRECTION_FILE = "camera_correction.json"   # written by `calib_check.py report --save`
CALIB_POINTS_FILE = "calib_points.json"

# ---------------------------------------------------------------------------
# The gripper's fingertips vs the gripper frame Viam moves
# ---------------------------------------------------------------------------
# How far the real fingertip center is from the gripper FRAME origin, along the gripper's own +z
# (out between the fingers). armfarm13: the frame sat ~48 mm behind the tips: TABLE_Z 63 (frame z with
# the fingertips on the table) minus the table top at world z ~15. If it's wrong, picks go too deep or
# too shallow and the thermometer stops at the wrong distance: the touch test (calib_check.py) shows it.
GRIPPER_TIP_OFFSET_MM = 48.0

# ---------------------------------------------------------------------------
# Where the arm may go. A target outside these limits is refused before moving.
# ---------------------------------------------------------------------------
ARM_BASE_XY = (0.0, 0.0)       # world origin = arm base (sidekick/config.py)
REACH_MM = (150.0, 700.0)      # horizontal distance from the base (sidekick uses 200-650 for top-down picks)
Z_LIMITS_MM = (70.0, 800.0)    # gripper FRAME z. armfarm13: 63 = fingertips on the table, so 70 never touches it

# ---------------------------------------------------------------------------
# Thermometer: find the forehead and point the thermometer at it (thermometer.py, care.py)
# ---------------------------------------------------------------------------
# Joint angles (degrees) where the wrist camera sees the person's face from 60-80 cm, thermometer
# already on the arm. With sidekick, teach it instead: `python app.py joints look_person` (care.py uses it).
LOOK_AT_PERSON_JOINTS = None   # TODO e.g. [0.0, -20.0, -30.0, 0.0, 50.0, 0.0]
LOOK_POSE_NAME = "look_person"   # the sidekick taught pose care.py goes to before looking

# How the thermometer sits on the arm, in the GRIPPER's own frame (+z = out between the fingers).
THERMO_AXIS_IN_GRIPPER = (0.0, 0.0, 1.0)      # TODO: direction the sensor points
THERMO_TIP_IN_GRIPPER_MM = (0.0, 0.0, 60.0)   # TODO: sensor tip, measured with a ruler FROM THE FINGERTIP CENTER
                                              # (GRIPPER_TIP_OFFSET_MM is added in the code)

STANDOFF_MM = 45.0         # TODO: tip-to-skin distance from your thermometer's manual (many say 3-5 cm)
PRE_APPROACH_MM = 150.0    # first stop is this much further back than STANDOFF_MM
APPROACH_STEPS = 3         # the last stretch is split into this many short straight moves
APPROACH_MAX_VEL_DEGS = 10.0   # joint speed cap asked of the motion service near the face (may be ignored)
MEASURE_SECONDS = 3.0      # hold still this long at the forehead

FACE_DISTANCE_MM = (250.0, 1100.0)   # faces closer or farther than this (from the camera) are ignored
TWO_LOOK_TOLERANCE_MM = 15.0   # two looks must agree this well, otherwise the person is moving: stop
RECHECK_TOLERANCE_MM = 30.0    # the forehead seen again from the pre-approach must be within this
MAX_AIM_ANGLE_DEG = 30.0       # thermometer axis vs straight into the forehead (measured from depth)
FOREHEAD_UP_RATIO = 0.55       # forehead pixel = eyes - 0.55 * (mouth - eyes)
FACE_MIN_SCORE = 0.75          # YuNet confidence

# Optional world box the forehead must be in (where the chair is). None = no check.
PERSON_ZONE = None             # TODO e.g. {"x": (-700, 100), "y": (150, 900), "z": (150, 800)}
USE_PERSON_OBSTACLES = True    # give the planner a head sphere and a body box to route around
HEAD_RADIUS_MM = 110.0

# ---------------------------------------------------------------------------
# Handing things to the person (care.py tissue / medicine)
# ---------------------------------------------------------------------------
# The hand-over point is in front of the person, worked out from where their face is:
HANDOVER_FORWARD_MM = 300.0    # toward the robot from the forehead
HANDOVER_BELOW_MM = 250.0      # below the forehead (about chest height)
MAX_HANDOVER_SHIFT_MM = 350.0  # never go further than this from the taught 'handoff' pose
HANDOVER_MAX_VEL_DEGS = 15.0   # joint speed cap (deg/s) for the move from 'handoff' to the person
OVERVIEW_MATCH_MM = 15.0       # remember-drawer / check-drawer: "at sidekick's overview" means this close to it

# ---------------------------------------------------------------------------
# The medicine drawer check (knob.py, care.py medicine)
# ---------------------------------------------------------------------------
KNOB_FILE = "drawer_knob.json"         # where `care.py remember-drawer` stores the knob position
KNOB_VIEW = "overview"                 # where the check looks from: sidekick's "overview", or a taught pose
                                       # name ("high", or one you teach) if the knob is out of the overview picture
KNOB_HSV_LOW = (18, 90, 90)            # yellow in OpenCV HSV (hue 0-180)
KNOB_HSV_HIGH = (38, 255, 255)         # (green drawer fronts start around hue 40)
KNOB_MIN_AREA_PX = 40
KNOB_SEARCH_RADIUS_MM = 150.0          # look for the knob within this of where it was remembered
DRAWER_MOVED_TOLERANCE_MM = 12.0       # moved more than this since the poses were taught: refuse

# ---------------------------------------------------------------------------
# Finding the bottle and the tissue with depth (locate.py)
# ---------------------------------------------------------------------------
# Joint angles where the camera looks down at the bottle/tissue area from ~40-60 cm. With sidekick,
# run `python app.py overview` first instead (its OVERVIEW pose).
OVERVIEW_JOINTS = None         # TODO e.g. [0.0, -40.0, -20.0, 0.0, 60.0, 0.0]
OVERVIEW_POSE = None           # or a gripper pose (x, y, z, o_x, o_y, o_z, theta); armfarm13: (207, 283, 560, 0, 0, -1, 178)

LOCATE_ROI = (0.05, 0.05, 0.95, 0.95)   # part of the image to search: left, top, right, bottom (fractions)
IGNORE_BOXES_PX = []           # pixel boxes to skip, e.g. the gripper fingers: [(x0, y0, x1, y1)]
PLANE_TOLERANCE_MM = 6.0       # how flat the table has to look
MIN_OBJECT_HEIGHT_MM = 12.0    # anything lower than this above the table is "table"
MAX_OBJECT_HEIGHT_MM = 350.0
MIN_BLOB_AREA_PX = 250
TOP_BAND_MM = 8.0              # the "top" of an object is everything within 8 mm of its highest point
HOVER_MM = 100.0               # hover this high above an object's top before going straight down
TWO_LOOK_TOLERANCE_OBJ_MM = 10.0

# Gripper pointing straight down. theta 145 = sidekick's TOOL_THETA: theta 0 made the planner refuse
# every pose (wrist vs camera self-collision) on armfarm13.
DOWN_ORIENTATION = (0.0, 0.0, -1.0, 145.0)

# What each object looks like to the depth camera (all sizes in mm, measured above the table).
# Run `python locate.py --list` to see the numbers for what is on the table, then tune these.
OBJECTS = {
    "bottle": {
        "height_mm": (50, 200),      # top of the cap above the table
        "length_mm": (25, 110),      # footprint, long side
        "width_mm": (20, 110),       # footprint, short side
        "grasp_below_top_mm": 10,    # fingers close this far below the top (around the cap)
        "lift_mm": 120,
    },
    "tissue": {
        "height_mm": (60, 250),      # top of the tissue that sticks out (armfarm13 box: 127 x 125, top 172-202)
        "length_mm": (90, 300),      # the box footprint
        "width_mm": (70, 200),
        "grasp_below_top_mm": 15,    # pinch this far below the tissue's tip (sidekick: TISSUE_GRIP_BELOW_TOP_MM)
        "lift_mm": 250,              # pull straight up this far (sidekick: TISSUE_PULL_MM)
        "needs_tissue_up_mm": 12,    # refuse if no tissue sticks out at least this much
    },
}
