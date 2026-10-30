"""A pretend robot, camera and world, so every script runs without hardware.

    python locate.py bottle --sim --execute
    python thermometer.py --sim --execute --yes

It draws color and depth pictures of a simple 3D scene (a table with a bottle and a tissue
box, or a person's head and body) from wherever the pretend camera is, and answers the
same Viam calls the real robot does. It can also have a deliberately wrong camera mount,
so the calibration check has something real to find.
"""
from __future__ import annotations

import math
from types import SimpleNamespace

import cv2
import numpy as np

import robot_io
import settings
from depth_tools import encode_pcd, encode_viam_depth
from face import Face, forehead_from_landmarks
from geometry import Intrinsics, _rz, apply_T, invert_T, make_T, matrix_to_ov, ov_to_matrix, unit

INTR = Intrinsics(615.0, 615.0, 320.0, 240.0, 640, 480)   # like a D435 at 640x480
MIN_Z_MM = 170.0                                         # depth cameras can't see closer than this

# ---------------------------------------------------------------------------
# Shapes (world frame, mm). Each returns ray distances t (inf = miss).
# ---------------------------------------------------------------------------


class Shape:
    color = (128, 128, 128)
    held = False

    def hit(self, O, D):
        raise NotImplementedError


class Rect(Shape):
    """Horizontal rectangle at height z (a table top or a floor)."""

    def __init__(self, z, xr, yr, color):
        self.z, self.xr, self.yr, self.color = z, xr, yr, color

    def hit(self, O, D):
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (self.z - O[2]) / D[:, 2]
        p = O + t[:, None] * D
        ok = (t > 0) & (p[:, 0] >= self.xr[0]) & (p[:, 0] <= self.xr[1]) & (p[:, 1] >= self.yr[0]) & (p[:, 1] <= self.yr[1])
        return np.where(ok, t, np.inf)


class Wall(Shape):
    """Vertical plane x = const (the background behind a person)."""

    def __init__(self, x, color):
        self.x, self.color = x, color

    def hit(self, O, D):
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (self.x - O[0]) / D[:, 0]
        return np.where(t > 0, t, np.inf)


class Cylinder(Shape):
    """Upright cylinder: axis at (cx, cy), radius r, from z0 to z1."""

    def __init__(self, cx, cy, r, z0, z1, color, name=""):
        self.c, self.r, self.z0, self.z1, self.color, self.name = np.array([cx, cy]), r, z0, z1, color, name

    def hit(self, O, D):
        ox, oy = O[0] - self.c[0], O[1] - self.c[1]
        a = D[:, 0] ** 2 + D[:, 1] ** 2
        b = 2 * (ox * D[:, 0] + oy * D[:, 1])
        c = ox * ox + oy * oy - self.r ** 2
        disc = b * b - 4 * a * c
        with np.errstate(invalid="ignore", divide="ignore"):
            sq = np.sqrt(np.maximum(disc, 0))
            t_side = (-b - sq) / (2 * a)
            z = O[2] + t_side * D[:, 2]
            side = np.where((disc >= 0) & (t_side > 0) & (z >= self.z0) & (z <= self.z1), t_side, np.inf)
            t_top = (self.z1 - O[2]) / D[:, 2]
        p = O + t_top[:, None] * D
        top_ok = (t_top > 0) & ((p[:, 0] - self.c[0]) ** 2 + (p[:, 1] - self.c[1]) ** 2 <= self.r ** 2)
        return np.minimum(side, np.where(top_ok, t_top, np.inf))


class Box(Shape):
    """Box standing on z0 with center (cx, cy), turned by yaw (deg): length x width, top at z1."""

    def __init__(self, cx, cy, yaw_deg, length, width, z0, z1, color, name=""):
        self.cx, self.cy, self.yaw = cx, cy, yaw_deg
        self.lo = np.array([-length / 2, -width / 2, z0])
        self.hi = np.array([length / 2, width / 2, z1])
        self.color, self.name = color, name
        self.R = _rz(math.radians(yaw_deg))

    def hit(self, O, D):
        o = self.R.T @ (O - np.array([self.cx, self.cy, 0.0]))
        d = D @ self.R
        with np.errstate(divide="ignore", invalid="ignore"):
            t1 = (self.lo - o) / d
            t2 = (self.hi - o) / d
        tmin = np.nanmax(np.minimum(t1, t2), axis=1)
        tmax = np.nanmin(np.maximum(t1, t2), axis=1)
        return np.where((tmax >= tmin) & (tmin > 0), tmin, np.inf)


class Sphere(Shape):
    def __init__(self, center, r, color, name=""):
        self.c, self.r, self.color, self.name = np.asarray(center, float), r, color, name

    def hit(self, O, D):
        oc = O - self.c
        a = np.einsum("ij,ij->i", D, D)
        b = 2 * D @ oc
        c = oc @ oc - self.r ** 2
        disc = b * b - 4 * a * c
        with np.errstate(invalid="ignore"):
            t = (-b - np.sqrt(np.maximum(disc, 0))) / (2 * a)
        return np.where((disc >= 0) & (t > 0), t, np.inf)


# ---------------------------------------------------------------------------
# The world: shapes + where the gripper and camera are
# ---------------------------------------------------------------------------


class World:
    def __init__(self, shapes, gripper_pose, cam_mount, cam_mount_error=(0, 0, 0), noise=True, seed=0):
        self.shapes = shapes
        self.T_world_gripper = gripper_pose
        self.T_gripper_cam_config = cam_mount                       # what the "frame system" believes
        self.T_gripper_cam_true = cam_mount @ make_T(np.eye(3), cam_mount_error)   # where it really is
        self.noise = noise
        self.rng = np.random.default_rng(seed)
        self.named_poses = {}
        self.moves = []          # every motion.move, for tests
        self.held = None
        self.person = None       # filled by the person scene: head center, radius, facing direction
        self.tip_offset = 0.0    # real fingertips are this far along the gripper frame's +z
        self.released = []       # (object name, fingertip position) every time the gripper let go of something

    @property
    def T_world_cam_true(self):
        return self.T_world_gripper @ self.T_gripper_cam_true

    @property
    def T_world_cam_config(self):
        return self.T_world_gripper @ self.T_gripper_cam_config

    def rays(self, T_world_cam, intr=INTR):
        us, vs = np.meshgrid(np.arange(intr.width), np.arange(intr.height))
        d_cam = np.stack([(us - intr.cx) / intr.fx, (vs - intr.cy) / intr.fy, np.ones_like(us, float)], -1).reshape(-1, 3)
        return T_world_cam[:3, 3], d_cam @ T_world_cam[:3, :3].T   # t along these rays = camera depth

    def render(self, intr=INTR):
        O, D = self.rays(self.T_world_cam_true, intr)
        best = np.full(len(D), np.inf)
        color = np.zeros((len(D), 3), np.uint8)
        for s in self.shapes:
            if s.held:
                continue
            t = s.hit(O, D)
            closer = t < best
            best[closer] = t[closer]
            color[closer] = s.color
        depth = best.reshape(intr.height, intr.width)
        img = color.reshape(intr.height, intr.width, 3).copy()
        if self.noise:
            depth = depth + self.rng.normal(0, 1.0, depth.shape) * (1 + 0.002 * np.nan_to_num(depth, posinf=0))
            img = np.clip(img.astype(np.int16) + self.rng.integers(-6, 7, img.shape), 0, 255).astype(np.uint8)
        depth = np.where(np.isfinite(depth) & (depth >= MIN_Z_MM) & (depth < 6000), depth, 0)
        if self.noise:
            depth[self.rng.random(depth.shape) < 0.005] = 0     # a few holes, like a real camera
        return img, np.round(depth).astype(np.uint16)


# ---------------------------------------------------------------------------
# Fake Viam parts (same method names the real SDK clients have)
# ---------------------------------------------------------------------------


def _pose_ns(T):
    ox, oy, oz, th = matrix_to_ov(T[:3, :3])
    x, y, z = T[:3, 3]
    return SimpleNamespace(pose=SimpleNamespace(x=x, y=y, z=z, o_x=ox, o_y=oy, o_z=oz, theta=th))


class FakeMachine:
    def __init__(self, world, camera_name, gripper_name):
        self.world, self.camera_name, self.gripper_name = world, camera_name, gripper_name
        self.resource_names = [SimpleNamespace(name=n, subtype=s) for n, s in
                               ((settings.ARM, "arm"), (gripper_name, "gripper"), (camera_name, "camera"),
                                (settings.MOTION, "motion"))]

    def frame(self, name):
        if name == "world":
            return np.eye(4)
        if name == self.camera_name:
            return self.world.T_world_cam_config
        if name == self.gripper_name:
            return self.world.T_world_gripper
        raise KeyError(f"no frame named {name}")

    async def transform_pose(self, query, destination):
        assert destination == "world"
        p = query.pose
        x, y, z = apply_T(self.frame(query.reference_frame), [p.x, p.y, p.z])
        return SimpleNamespace(pose=SimpleNamespace(x=x, y=y, z=z))

    async def close(self):
        pass


class FakeCamera:
    def __init__(self, world):
        self.world = world

    async def get_images(self, filter_source_names=None, **_):
        img, depth = self.world.render()
        self._last = (self.world.T_world_gripper.copy(), depth)   # the same frame feeds get_point_cloud
        ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 92])
        out = [SimpleNamespace(name="color", data=jpg.tobytes(), mime_type="image/jpeg"),
               SimpleNamespace(name="depth", data=encode_viam_depth(depth), mime_type="image/vnd.viam.dep")]
        if filter_source_names:
            out = [o for o in out if o.name in filter_source_names]
        return out, None

    async def get_point_cloud(self, **_):
        """One point per pixel in pixel order, meters, like the RealSense module (0,0,0 = no reading)."""
        if getattr(self.world, "no_point_cloud", False):
            raise RuntimeError("this pretend camera has no point cloud")
        last = getattr(self, "_last", None)
        if last is not None and np.allclose(last[0], self.world.T_world_gripper):
            depth = last[1]
        else:
            _, depth = self.world.render()
        self._last = None
        i = INTR
        us, vs = np.meshgrid(np.arange(i.width), np.arange(i.height))
        z = depth.astype(np.float64) / 1000.0
        pts = np.stack([(us - i.cx) * z / i.fx, (vs - i.cy) * z / i.fy, z], -1).reshape(-1, 3)
        return encode_pcd(pts), "pointcloud/pcd"

    async def get_properties(self, **_):
        i = INTR
        return SimpleNamespace(intrinsic_parameters=SimpleNamespace(
            width_px=i.width, height_px=i.height, focal_x_px=i.fx, focal_y_px=i.fy, center_x_px=i.cx, center_y_px=i.cy))


class FakeMotion:
    def __init__(self, world):
        self.world = world

    async def get_pose(self, component_name, destination_frame, **_):
        return _pose_ns(self.world.T_world_gripper)

    async def move(self, component_name, destination, world_state=None, constraints=None, **_):
        p = destination.pose
        T = make_T(ov_to_matrix(p.o_x, p.o_y, p.o_z, p.theta), [p.x, p.y, p.z])
        linear = constraints is not None and len(constraints.linear_constraint) > 0
        obstacles = 0 if world_state is None else sum(len(g.geometries) for g in world_state.obstacles)
        self.world.moves.append({"to": T, "linear": linear, "obstacles": obstacles})
        _check_person_collision(self.world, T)
        self.world.T_world_gripper = T
        return True


class FakeArm:
    def __init__(self, world):
        self.world = world

    async def move_to_joint_positions(self, positions, **_):
        key = tuple(round(v, 1) for v in positions.values)
        if key not in self.world.named_poses:
            raise RuntimeError(f"sim has no pose for joints {key}")
        _check_person_collision(self.world, self.world.named_poses[key])
        self.world.T_world_gripper = self.world.named_poses[key].copy()

    async def move_through_joint_positions(self, positions, options=None, **_):
        for p in positions:
            await self.move_to_joint_positions(p)

    async def get_joint_positions(self, **_):
        for key, T in self.world.named_poses.items():
            if np.allclose(T, self.world.T_world_gripper):
                return SimpleNamespace(values=list(key))
        return SimpleNamespace(values=[0.0] * 6)

    async def stop(self, **_):
        pass


class FakeGripper:
    def __init__(self, world):
        self.world = world

    async def open(self, **_):
        if self.world.held is not None:
            tips = apply_T(self.world.T_world_gripper, [0.0, 0.0, self.world.tip_offset])
            self.world.released.append((self.world.held.name, tips))
            self.world.held = None

    async def grab(self, **_):
        p = apply_T(self.world.T_world_gripper, [0.0, 0.0, self.world.tip_offset])   # the real fingertips
        for s in self.world.shapes:
            if isinstance(s, (Cylinder, Box)) and s.name and not s.held:
                if isinstance(s, Cylinder):
                    near = np.hypot(*(p[:2] - s.c)) <= s.r + 6
                    z0, z1 = s.z0, s.z1
                else:
                    local = s.R.T @ (p - np.array([s.cx, s.cy, 0.0]))
                    near = np.all(np.abs(local[:2]) <= (s.hi[:2] + 6))
                    z0, z1 = s.lo[2], s.hi[2]
                if near and z0 - 5 <= p[2] <= z1:
                    for part in self.world.shapes:      # the whole object comes along (body + cap)
                        if getattr(part, "name", "") == s.name:
                            part.held = True
                    self.world.held = s
                    return True
        return False

    async def stop(self, **_):
        pass


class SimFaceFinder:
    """Stands in for YuNet: projects the pretend person's eyes and mouth into the image."""

    def __init__(self, world, up_ratio=0.55):
        self.world, self.up_ratio = world, up_ratio

    def detect(self, bgr):
        person = self.world.person
        if person is None:
            return []
        c, r, f = person["center"], person["radius"], person["facing"]
        up = np.array([0.0, 0.0, 1.0])
        right = unit(np.cross(f, up))

        def on_head(lateral, vertical):
            return c + r * unit(f * r + right * lateral + up * vertical)

        # YuNet order: right eye, left eye, nose tip, right mouth corner, left mouth corner (the person's right/left)
        pts = np.array([on_head(32, 0), on_head(-32, 0), on_head(0, -30), on_head(25, -62), on_head(-25, -62)])
        T_cw = invert_T(self.world.T_world_cam_true)
        cam = apply_T(T_cw, pts)
        if np.any(cam[:, 2] < 50):
            return []
        uv = np.stack([cam[:, 0] * INTR.fx / cam[:, 2] + INTR.cx, cam[:, 1] * INTR.fy / cam[:, 2] + INTR.cy], 1)
        if np.any(uv < 0) or np.any(uv[:, 0] >= INTR.width) or np.any(uv[:, 1] >= INTR.height):
            return []
        forehead, eye_mouth = forehead_from_landmarks(uv, self.up_ratio)
        x0, y0 = uv.min(0) - eye_mouth * 0.6
        x1, y1 = uv.max(0) + eye_mouth * 0.6
        return [Face((x0, y0 - eye_mouth * 0.8, x1 - x0, y1 - y0 + eye_mouth * 0.8), 0.95, uv, forehead, eye_mouth)]


def _check_person_collision(world, T_gripper):
    """Crash the test if the gripper or the thermometer tip would go into the head."""
    if world.person is None:
        return
    fingers = apply_T(T_gripper, [0.0, 0.0, world.tip_offset])
    tip = apply_T(T_gripper, np.asarray(settings.THERMO_TIP_IN_GRIPPER_MM, float) + [0.0, 0.0, world.tip_offset])
    for p, what in ((T_gripper[:3, 3], "gripper"), (fingers, "fingertips"), (tip, "thermometer tip")):
        gap = np.linalg.norm(p - world.person["center"]) - world.person["radius"]
        if gap < 5.0:
            raise RuntimeError(f"SIM COLLISION: {what} is {gap:.0f} mm from the head surface")


# ---------------------------------------------------------------------------
# Scenes
# ---------------------------------------------------------------------------
DOWN = (0.0, 0.0, -1.0, 0.0)
CAM_MOUNT = make_T(np.eye(3), (0.0, -70.0, -120.0))   # camera 120 mm behind the fingertips, 70 mm to the side

TABLE_TRUTH = {"bottle_cap_center": np.array([470.0, 110.0, 115.0]),
               "tissue_tip_center": np.array([330.0, -60.0, 135.0])}


def table_world(cam_mount_error=(0, 0, 0), noise=True, seed=0, bottle_xy=(470.0, 110.0)) -> World:
    bx, by = bottle_xy
    shapes = [
        Rect(-750.0, (-2000, 2000), (-2000, 2000), (60, 60, 60)),          # floor
        Rect(0.0, (100, 900), (-500, 500), (175, 180, 185)),               # table
        Cylinder(bx, by, 22.0, 0.0, 95.0, (40, 120, 230), "bottle"),       # orange bottle body
        Cylinder(bx, by, 15.0, 95.0, 115.0, (245, 245, 245), "bottle"),    # white cap
        Box(330.0, -60.0, 20.0, 230.0, 120.0, 0.0, 95.0, (200, 120, 40), "tissue_box"),
        Box(330.0, -60.0, 20.0, 70.0, 6.0, 95.0, 135.0, (250, 250, 250), "tissue"),
    ]
    TABLE_TRUTH["bottle_cap_center"] = np.array([bx, by, 115.0])
    overview = make_T(ov_to_matrix(*DOWN), (380.0, 60.0, 450.0))
    w = World(shapes, overview.copy(), CAM_MOUNT, cam_mount_error, noise, seed)
    w.named_poses[(0.0, 0.0, 0.0, 0.0, 0.0, 2.0)] = overview
    return w


PERSON_LOOK_OV = (1.0, 0.0, 0.0, -90.0)   # gripper (and camera) facing +x, image upright


def person_world(noise=True, seed=0, head_center=(790.0, 30.0, 450.0)) -> World:
    hc = np.array(head_center, float)
    shapes = [
        Wall(1300.0, (90, 100, 110)),
        Rect(-750.0, (-2000, 2000), (-2000, 2000), (60, 60, 60)),
        Rect(0.0, (100, 640), (-500, 500), (175, 180, 185)),                    # table edge stops before the person
        Sphere(hc, 95.0, (120, 150, 200), "head"),
        Box(hc[0] + 40, hc[1], 0.0, 260.0, 420.0, hc[2] - 760, hc[2] - 150, (70, 60, 50), "body"),
    ]
    look = make_T(ov_to_matrix(*PERSON_LOOK_OV), (330.0, 30.0, 470.0))
    w = World(shapes, look.copy(), CAM_MOUNT, (0, 0, 0), noise, seed)
    w.person = {"center": hc, "radius": 95.0, "facing": np.array([-1.0, 0.0, 0.0])}
    w.named_poses[(0.0, 0.0, 0.0, 0.0, 0.0, 1.0)] = look
    return w


CALIB_SPOTS = [(470.0, 110.0), (300.0, 90.0), (540.0, -120.0), (560.0, 40.0)]   # where --sim-spot puts the bottle


def make_rig(scene: str = "table", cam_mount_error=None, tip_offset: float = 0.0, **kw) -> robot_io.Rig:
    """A Rig wired to the pretend world.

    The simulator keeps its calibration files apart from the real ones, and only uses a
    correction when you give it a camera mount error to correct (--sim-mount-error).
    tip_offset: put the pretend fingertips this far beyond the gripper frame (like armfarm13's 48 mm).
    """
    settings.CAMERA_CORRECTION_FILE = "sim_camera_correction.json"
    settings.CALIB_POINTS_FILE = "sim_calib_points.json"
    settings.KNOB_FILE = "sim_drawer_knob.json"
    settings.LOOK_AT_PERSON_JOINTS = [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    settings.OVERVIEW_JOINTS = [0.0, 0.0, 0.0, 0.0, 0.0, 2.0]
    settings.GRIPPER_TIP_OFFSET_MM = float(tip_offset)
    if scene == "person":
        world = person_world(**kw)
    elif scene == "care":
        world = care_world(**kw)
    else:
        world = table_world(cam_mount_error=cam_mount_error or (0, 0, 0), **kw)
    world.tip_offset = float(tip_offset)
    machine = FakeMachine(world, settings.CAMERA, settings.GRIPPER)
    correction = robot_io.load_camera_correction() if cam_mount_error is not None else None
    rig = robot_io.Rig(machine, FakeCamera(world), FakeMotion(world), FakeArm(world), FakeGripper(world),
                       correction=correction)
    rig.world = world
    return rig


def true_top_near(world: World, xy) -> np.ndarray:
    """Simulator only: the real top center of the object nearest to xy (for the touch test)."""
    best, best_d = None, 1e9
    for s in world.shapes:
        if isinstance(s, Cylinder):
            c, top = s.c, s.z1
        elif isinstance(s, Box):
            c, top = np.array([s.cx, s.cy]), s.hi[2]
        else:
            continue
        d = float(np.hypot(*(c - np.asarray(xy[:2]))))
        if d < best_d or (abs(d - best_d) < 1e-6 and top > best[2]):
            best, best_d = np.array([c[0], c[1], top]), d
    return best


# ---------------------------------------------------------------------------
# Sidekick on the simulator: care.py --sim
# ---------------------------------------------------------------------------
KNOB_BGR = (0, 220, 255)                 # the drawer's yellow knob
DRAWER_XY = (490.0, 130.0)               # drawer box center when its poses were "taught"
PILL_TIP = (490.0, 130.0, 45.0)          # fingertips around the pill box, inside the drawer
SIM_OVERVIEW = make_T(ov_to_matrix(*DOWN), (380.0, 60.0, 450.0))


def pose_T(p: dict) -> np.ndarray:
    """A sidekick taught pose (x, y, z, o_x, o_y, o_z, theta) as a 4x4."""
    return make_T(ov_to_matrix(p["o_x"], p["o_y"], p["o_z"], p["theta"]), (p["x"], p["y"], p["z"]))


def pose_dict(T) -> dict:
    ox, oy, oz, th = matrix_to_ov(T[:3, :3])
    return {"x": float(T[0, 3]), "y": float(T[1, 3]), "z": float(T[2, 3]),
            "o_x": float(ox), "o_y": float(oy), "o_z": float(oz), "theta": float(th)}


def care_world(noise=True, seed=0, head_center=(790.0, 30.0, 450.0), drawer_shift=(0.0, 0.0)) -> World:
    """The person scene plus sidekick's table: a tissue box and the medicine drawer box with its yellow knob."""
    w = person_world(noise=noise, seed=seed, head_center=head_center)
    dx, dy = DRAWER_XY[0] + drawer_shift[0], DRAWER_XY[1] + drawer_shift[1]
    w.shapes += [
        Box(330.0, -60.0, 20.0, 230.0, 120.0, 0.0, 95.0, (200, 120, 40), "tissue_box"),
        Box(330.0, -60.0, 20.0, 70.0, 6.0, 95.0, 135.0, (250, 250, 250), "tissue"),
        Cylinder(dx, dy, 18.0, 20.0, 70.0, (40, 40, 200), "pill_box"),     # inside the drawer, out of sight
        Box(dx, dy, 0.0, 120.0, 160.0, 0.0, 110.0, (60, 140, 60), "drawer_box"),
        Sphere((dx - 68.0, dy, 55.0), 12.0, KNOB_BGR, "knob"),
    ]
    return w


def care_poses(world: World, tip_offset: float) -> dict:
    """Taught poses like sidekick's drawer_joints.json (joints + gripper-frame pose), for this pretend world."""
    down = ov_to_matrix(*DOWN)

    def tips_at(tip):          # gripper frame pose that puts the fingertips at `tip`, pointing down
        return make_T(down, np.asarray(tip, float) - down @ np.array([0.0, 0.0, tip_offset]))

    poses = {"high": tips_at((450.0, 130.0, 210.0)),
             "look_person": world.T_world_gripper.copy(),           # person_world starts at its look pose
             "pill_above": tips_at((PILL_TIP[0], PILL_TIP[1], 200.0)),
             "pill_grip": tips_at(PILL_TIP),
             "handoff": tips_at((560.0, 60.0, 230.0))}
    data = {}
    world.named_poses.clear()          # only sidekick's poses, so joints always name one pose
    for i, (name, T) in enumerate(poses.items(), 1):
        joints = [10.0 * i, 0.0, 0.0, 0.0, 0.0, 0.0]
        data[name] = {"joints": joints, "pose": pose_dict(T)}
        world.named_poses[tuple(round(j, 1) for j in joints)] = pose_T(data[name]["pose"])
    return data


class SimMoveError(Exception):
    def __init__(self, msg, holding=False):
        super().__init__(msg)
        self.holding = holding


class SidekickSim:
    """Stands in for sidekick's RealRobot (same method names) on the pretend world. Every move is
    checked against the pretend person's head; .log lists what happened, in order."""

    def __init__(self, rig, data, overview_T, cfg):
        self.machine, self.camera, self.motion = rig.machine, rig.camera, rig.motion
        self.arm, self.gripper = rig.arm, rig.gripper
        self.world, self.data, self.overview, self.cfg, self.log = rig.world, data, overview_T, cfg, []

    def _go(self, T, what):
        _check_person_collision(self.world, T)
        self.world.T_world_gripper = np.array(T, dtype=np.float64)
        self.log.append(what)

    async def to_high(self, log=print):
        self._go(pose_T(self.data["high"]["pose"]), "to_high")
        return True

    async def go_overview(self):
        self._go(self.overview, "overview")

    async def move_to(self, x, y, z, straight=False, theta=None, avoid=None, must_be_straight=False, o=None,
                      max_vel=None, orient_tol=None):
        o = (0.0, 0.0, -1.0) if o is None else o
        T = make_T(ov_to_matrix(o[0], o[1], o[2], self.cfg.TOOL_THETA if theta is None else theta), (x, y, z))
        n = 0 if avoid is None else sum(len(g.geometries) for g in avoid.obstacles)
        self.world.moves.append({"to": T, "linear": bool(straight or must_be_straight), "obstacles": n, "max_vel": max_vel})
        self._go(T, ("move_to", tuple(int(round(float(v))) for v in (x, y, z))))

    async def replay(self, data, steps):
        for kind, args in steps:
            if kind == "release":
                await self.gripper.open()
            elif kind == "grab":
                await self.gripper.grab()
            elif kind == "grab?":
                if not await self.gripper.grab():
                    await self.gripper.open()
                    raise SimMoveError("I missed it: the gripper closed on nothing")
            elif kind == "move":
                for name in args:
                    self._go(pose_T(data[name]["pose"]), ("joint", name))

    async def drawer(self, action="open"):
        self.log.append(("drawer", action))
        self._go(pose_T(self.data["high"]["pose"]), ("joint", "high"))
        return 120.0

    async def stop(self):
        self.log.append("stop")

    async def close(self):
        pass


class SimPerception:
    """sidekick's perception.find on the pretend world: the boxes on the table, by name."""

    def __init__(self, world):
        self.world = world

    async def find(self, robot, frames=None, cal=None):
        await robot.go_overview()
        return {s.name: (float(s.cx), float(s.cy)) for s in self.world.shapes
                if isinstance(s, Box) and s.name in ("tissue_box", "drawer_box") and not s.held}


def _sim_world_state(others):
    from viam.proto.common import GeometriesInFrame, Geometry, Pose, RectangularPrism, Vector3, WorldState
    geoms = [Geometry(center=Pose(x=x, y=y, z=60.0, o_z=1), box=RectangularPrism(dims_mm=Vector3(x=180, y=180, z=120)),
                      label=name) for name, (x, y) in sorted(others.items())]
    return WorldState(obstacles=[GeometriesInFrame(reference_frame="world", geometries=geoms)])


async def _sim_retry(label, fn, tries=3, delay=1.0):
    return await fn()


def _sim_pills_sequence(data, hold_s=3.0):
    """The same steps as sidekick's taught.pills_sequence."""
    return [("release", []), ("move", ["pill_above"]), ("move", ["pill_grip"]), ("grab?", []),
            ("move", (["pill_lift"] if "pill_lift" in data else []) + ["pill_above"]), ("move", ["handoff"]),
            ("wait", [hold_s]), ("release", []), ("move", ["pill_above", "high"])]


def make_care(tip_offset: float = 48.0, start: str | None = "high", **kw):
    """Everything care.py needs, on the simulator: (robot, rig, face finder, taught poses, sidekick modules)."""
    rig = make_rig("care", tip_offset=tip_offset, **kw)
    world = rig.world
    settings.LOOK_AT_PERSON_JOINTS = None
    data = care_poses(world, tip_offset)
    cfg = SimpleNamespace(
        ARM=settings.ARM, GRIPPER=settings.GRIPPER, CAMERA=settings.CAMERA, MOTION=settings.MOTION,
        TABLE_Z=0.0 + tip_offset, TABLE_TOP_WORLD_Z=0.0, HOVER=120.0, TOOL_THETA=0.0,
        TISSUE_GRIP_BELOW_TOP_MM=15.0, TISSUE_PULL_MM=250.0, HANDOFF_HOLD_S=0.0,
        DRAWER_MAX_VEL_DEGS=15.0, DRAWER_MAX_ACC_DEGS=30.0, OVERVIEW=tuple(SIM_OVERVIEW[:3, 3]),
        OBJECTS={"tissue_box": {"size": (230.0, 120.0, 135.0)}, "drawer_box": {"size": (120.0, 160.0, 110.0)}})
    robot = SidekickSim(rig, data, SIM_OVERVIEW, cfg)
    if start:
        world.T_world_gripper = pose_T(data[start]["pose"])
    sk = SimpleNamespace(
        config=cfg, taught=SimpleNamespace(load=lambda: data, pills_sequence=_sim_pills_sequence),
        perception=SimPerception(world), dir=None,
        robot=SimpleNamespace(MoveError=SimMoveError, arm_halted=lambda e: "mergency" in str(e),
                              world_state=_sim_world_state, retry=_sim_retry, RealRobot=None))
    return robot, rig, SimFaceFinder(world, settings.FOREHEAD_UP_RATIO), data, sk
