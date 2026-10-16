# Viam 101 notes (read from the course, Oct 15)

19 pages: create machine → run viam-server → add arm + gripper → frame system → place gripper →
connect from code → scaffold → work cell → poses → move → pick → place → run → obstacles/WorldState →
avoid placed boxes → model the held box → recap.

## Key ideas
- **Component** = hardware (arm, gripper, camera); **service** = capability (motion, vision). Same API for real and simulated.
- **Frame** = origin + axes, defined relative to a parent; the tree is rooted at `world`. A gripper frame is
  parented to the arm with the flange→tool-tip offset (course: 196 mm for the vacuum gripper; last month's
  xArm finger gripper: 150 mm in config, measure it).
- **Geometry** = volume the planner avoids. A **floor** obstacle: fake generic component, box
  2000×2000×10 at z=-5, parent world.
- **Motion service:** name the frame to move (`component_name`) and a destination; it plans and executes.
  Naming the gripper puts the tool tip at the pose; naming the arm puts the flange there.
- **WorldState:** per-move obstacles. Placed objects in the `world` frame stay put; a held object in the
  gripper frame moves with it. Split placement into "over the spot with the held box modeled" then
  "straight down without it".
- **DoCommand:** extra commands beyond the standard API.

## Code patterns (Python SDK)
```python
from viam.robot.client import RobotClient
from viam.components.arm import Arm
from viam.components.gripper import Gripper
from viam.services.motion import MotionClient
from viam.proto.common import Pose, PoseInFrame, WorldState, GeometriesInFrame, Geometry, RectangularPrism, Vector3

opts = RobotClient.Options.with_api_key(api_key=KEY, api_key_id=KEY_ID)
robot = await RobotClient.at_address(ADDRESS, opts)

def down_pose(x, y, z):            # tool pointing straight down
    return Pose(x=x, y=y, z=z, o_x=0, o_y=0, o_z=-1, theta=0)

motion = MotionClient.from_robot(robot, "builtin")
await motion.move(component_name="gripper-1",
                  destination=PoseInFrame(reference_frame="world", pose=down_pose(400, -300, 400)),
                  world_state=None, extra={"timeout": 15.0}, timeout=120)

gripper = Gripper.from_robot(robot, "gripper-1")
await gripper.open(); await gripper.grab()

arm = Arm.from_robot(robot, "arm-1")
from viam.proto.component.arm import JointPositions
await arm.move_to_joint_positions(JointPositions(values=[0, -45, -30, 0, 60, 0]))   # degrees
```
Pick = home → hover → straight down → grab → back up. Place = over the spot → down → open → record.
API key + machine address come from the app's CONNECT tab (API keys, "Machine Default"; Remote address).
