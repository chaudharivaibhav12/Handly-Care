"""Talking to the robot through Viam: connect, take a 3D picture, and move with checks.

Everything else in this folder goes through the Rig class, so the same code runs on the
real arm, in the simulator (sim.py), and offline on saved pictures (snapshot.py).
"""
from __future__ import annotations

import asyncio
import json
import os
import threading
import time
from dataclasses import dataclass, field

import cv2
import numpy as np

import settings
from depth_tools import decode_color, decode_depth, depth_from_pcd
from geometry import Intrinsics, T_from_probe, apply_T, make_T, ov_to_matrix, reach_problems


class UnsafeTarget(Exception):
    """A computed target failed a safety check, so nothing moved."""


def tip_in_gripper() -> np.ndarray:
    """The fingertip center, in the gripper frame (settings.GRIPPER_TIP_OFFSET_MM along +z)."""
    return np.array([0.0, 0.0, float(settings.GRIPPER_TIP_OFFSET_MM)])


def thermo_tip_in_gripper() -> np.ndarray:
    """The thermometer's sensor tip in the gripper frame (measured from the fingertips + the frame offset)."""
    return np.asarray(settings.THERMO_TIP_IN_GRIPPER_MM, dtype=np.float64) + tip_in_gripper()


def frame_for_fingertips(tip_xyz, ov) -> np.ndarray:
    """Where to send the gripper FRAME so the fingertips end up at tip_xyz, with orientation ov."""
    return np.asarray(tip_xyz, dtype=np.float64) - ov_to_matrix(*ov[:4]) @ tip_in_gripper()


# ---------------------------------------------------------------------------
# A 3D picture: color + depth + where the camera was
# ---------------------------------------------------------------------------


@dataclass
class Capture:
    color: np.ndarray          # BGR, H x W x 3
    depth: np.ndarray          # uint16 mm, H x W (same size as color)
    intr: Intrinsics
    T_world_cam: np.ndarray    # 4x4: camera-frame point -> world point (correction applied)
    T_world_gripper: np.ndarray | None = None
    gripper_ov: tuple | None = None      # gripper orientation exactly as Viam reported it
    stamp: float = 0.0
    notes: list = field(default_factory=list)

    def to_world(self, p_cam) -> np.ndarray:
        return apply_T(self.T_world_cam, p_cam)

    @property
    def camera_position(self) -> np.ndarray:
        return self.T_world_cam[:3, 3].copy()


def save_capture(cap: Capture, folder: str) -> str:
    os.makedirs(folder, exist_ok=True)
    cv2.imwrite(os.path.join(folder, "color.png"), cap.color)
    cv2.imwrite(os.path.join(folder, "depth.png"), cap.depth.astype(np.uint16))
    meta = {"intrinsics": cap.intr.to_dict(), "T_world_cam": cap.T_world_cam.tolist(),
            "T_world_gripper": None if cap.T_world_gripper is None else cap.T_world_gripper.tolist(),
            "gripper_ov": cap.gripper_ov, "stamp": cap.stamp, "notes": cap.notes}
    with open(os.path.join(folder, "meta.json"), "w") as f:
        json.dump(meta, f, indent=2)
    return folder


def load_capture(folder: str) -> Capture:
    color = cv2.imread(os.path.join(folder, "color.png"), cv2.IMREAD_COLOR)
    depth = cv2.imread(os.path.join(folder, "depth.png"), cv2.IMREAD_UNCHANGED)
    if color is None or depth is None:
        raise FileNotFoundError(f"{folder} needs color.png and depth.png (make one with snapshot.py)")
    with open(os.path.join(folder, "meta.json")) as f:
        meta = json.load(f)
    Tg = meta.get("T_world_gripper")
    return Capture(color, depth.astype(np.uint16), Intrinsics.from_dict(meta["intrinsics"]),
                   np.array(meta["T_world_cam"]), None if Tg is None else np.array(Tg),
                   tuple(meta["gripper_ov"]) if meta.get("gripper_ov") else None,
                   meta.get("stamp", 0.0), meta.get("notes", []))


def load_camera_correction(path: str | None = None) -> np.ndarray | None:
    """Offset (camera frame, mm) measured by calib_check.py, or None."""
    path = path or settings.CAMERA_CORRECTION_FILE
    if not os.path.exists(path):
        return None
    with open(path) as f:
        return np.array(json.load(f)["offset_cam_mm"], dtype=np.float64)


def split_images(images) -> tuple[np.ndarray | None, np.ndarray | None]:
    """Pick the color and the depth image out of what get_images returned."""
    color = depth = None
    for im in images:
        mt, name = str(im.mime_type), (getattr(im, "name", "") or "").lower()
        if depth is None and (mt.startswith("image/vnd.viam.dep") or "depth" in name):
            depth = decode_depth(im.data, mt)
        elif color is None:
            color = decode_color(im.data, mt)
    return color, depth


# ---------------------------------------------------------------------------
# The robot
# ---------------------------------------------------------------------------


def _first_env(*names):
    for n in names:
        if os.environ.get(n):
            return os.environ[n]
    return None


def load_env(env_file: str | None = None):
    try:
        from dotenv import load_dotenv
    except ImportError:
        return
    load_dotenv(env_file or ".env")


class Rig:
    """The parts of the robot we use. Built by Rig.connect() (real) or sim.make_rig() (fake)."""

    def __init__(self, machine, camera, motion, arm=None, gripper=None,
                 camera_name=None, gripper_name=None, correction=None):
        self.machine, self.camera, self.motion = machine, camera, motion
        self.arm, self.gripper = arm, gripper
        self.camera_name = camera_name or settings.CAMERA
        self.gripper_name = gripper_name or settings.GRIPPER
        self.correction = correction   # camera-frame offset from calib_check.py (or None)
        self._intr = None              # camera intrinsics, read once

    # ---- connecting -------------------------------------------------------
    @classmethod
    async def connect(cls, env_file: str | None = None) -> "Rig":
        load_env(env_file)
        address = _first_env("VIAM_ADDRESS", "ROBOT_ADDRESS", "VIAM_MACHINE_ADDRESS", "MACHINE_ADDRESS")
        key = _first_env("VIAM_API_KEY", "API_KEY", "ROBOT_API_KEY")
        key_id = _first_env("VIAM_API_KEY_ID", "API_KEY_ID", "ROBOT_API_KEY_ID")
        if not (address and key and key_id):
            raise SystemExit("Put VIAM_ADDRESS, VIAM_API_KEY_ID and VIAM_API_KEY in .env "
                             "(machine page in app.viam.com -> CONNECT -> API keys). See .env.example.")
        from viam.components.arm import Arm
        from viam.components.camera import Camera
        from viam.components.gripper import Gripper
        from viam.robot.client import RobotClient
        from viam.services.motion import MotionClient

        machine = await RobotClient.at_address(address, RobotClient.Options.with_api_key(api_key=key, api_key_id=key_id))
        names = {r.name for r in machine.resource_names}

        def need(name, what):
            if name not in names:
                raise SystemExit(f"No {what} named '{name}'. This machine has: {', '.join(sorted(names))}. "
                                 f"Fix the name in settings.py.")
        need(settings.CAMERA, "camera")
        need(settings.MOTION, "motion service")
        camera = Camera.from_robot(machine, settings.CAMERA)
        motion = MotionClient.from_robot(machine, settings.MOTION)
        arm = Arm.from_robot(machine, settings.ARM) if settings.ARM in names else None
        gripper = Gripper.from_robot(machine, settings.GRIPPER) if settings.GRIPPER in names else None
        return cls(machine, camera, motion, arm, gripper, correction=load_camera_correction())

    async def close(self):
        try:
            await self.machine.close()
        except Exception:
            pass

    # ---- frames -----------------------------------------------------------
    async def probe(self, frame: str, step: float = 100.0) -> np.ndarray:
        """4x4 transform frame -> world, measured by asking Viam where 4 points of the frame are."""
        from viam.proto.common import Pose, PoseInFrame
        pts = []
        for x, y, z in ((0, 0, 0), (step, 0, 0), (0, step, 0), (0, 0, step)):
            r = await self.machine.transform_pose(
                PoseInFrame(reference_frame=frame, pose=Pose(x=x, y=y, z=z, o_x=0, o_y=0, o_z=1, theta=0)), "world")
            pts.append([r.pose.x, r.pose.y, r.pose.z])
        return T_from_probe(*pts, step)

    async def gripper_pose(self):
        """(position, orientation vector) of the gripper frame in world, as Viam reports it."""
        p = (await self.motion.get_pose(component_name=self.gripper_name, destination_frame="world")).pose
        return np.array([p.x, p.y, p.z]), (p.o_x, p.o_y, p.o_z, p.theta)

    async def joints(self):
        if self.arm is None:
            raise RuntimeError(f"no arm named '{settings.ARM}'")
        return [round(float(v), 2) for v in (await self.arm.get_joint_positions()).values]

    # ---- seeing -----------------------------------------------------------
    async def capture(self, apply_correction: bool = True, settle_s: float | None = None, retries: int = 3) -> Capture:
        """One color + depth picture, plus exactly where the camera was when it was taken."""
        await asyncio.sleep(settings.CAPTURE_SETTLE_S if settle_s is None else settle_s)
        use_cloud = settings.DEPTH_FROM in ("auto", "point_cloud")
        notes = []
        for _ in range(retries):
            T1 = await self.probe(self.camera_name)
            try:
                images, _ = await self.camera.get_images(filter_source_names=[settings.COLOR_SOURCE, settings.DEPTH_SOURCE])
            except Exception:  # noqa: BLE001  a source name this camera doesn't have
                images = []
            if not images:   # source names differ on this camera: take everything
                images, _ = await self.camera.get_images()
            cloud = None
            if use_cloud:
                try:
                    cloud, _ = await self.camera.get_point_cloud()
                except Exception as e:  # noqa: BLE001
                    notes.append(f"point cloud unavailable ({e}); using the depth image")
            T2 = await self.probe(self.camera_name)
            if np.linalg.norm(T1[:3, 3] - T2[:3, 3]) <= settings.MAX_CAPTURE_DRIFT_MM:
                break
            await asyncio.sleep(0.3)
        else:
            raise RuntimeError("the camera kept moving while taking the picture; let the arm stop first")

        color, depth_img = split_images(images)
        if color is None:
            raise RuntimeError("the camera did not return a color image. Check its 'sensors' attribute.")
        h, w = color.shape[:2]
        if self._intr is None:
            ip = (await self.camera.get_properties()).intrinsic_parameters
            if not ip.focal_x_px:
                raise RuntimeError("the camera reports no intrinsics (focal length), so pixels can't become 3D points")
            self._intr = Intrinsics(ip.focal_x_px, ip.focal_y_px, ip.center_x_px, ip.center_y_px,
                                    int(ip.width_px or w), int(ip.height_px or h))
        intr = self._intr.scaled_to(w, h)

        depth = None
        if cloud is not None:
            depth = depth_from_pcd(cloud, intr)
            if depth is not None:
                notes.append("depth from the point cloud (already aligned to color)")
            elif settings.DEPTH_FROM == "point_cloud":
                raise RuntimeError("the point cloud is not one point per color pixel, so it can't give depth per pixel")
        if depth is None:
            if depth_img is None:
                raise RuntimeError("the camera gave no depth. Check its 'sensors' attribute includes depth.")
            depth = depth_img
            if depth.shape != (h, w):
                notes.append(f"depth is {depth.shape[1]}x{depth.shape[0]} but color is {w}x{h}: depth is NOT "
                             f"aligned to color. Set align_color_depth: true on the camera (RealSense).")
                depth = cv2.resize(depth, (w, h), interpolation=cv2.INTER_NEAREST)

        T_wc = T2
        if apply_correction and self.correction is not None:
            T_wc = T2 @ make_T(np.eye(3), self.correction)
            notes.append(f"camera correction applied: {np.round(self.correction, 1).tolist()} mm")
        T_wg, ov = None, None
        try:
            T_wg = await self.probe(self.gripper_name)
            _, ov = await self.gripper_pose()
        except Exception as e:   # gripper frame missing: fine for looking, not for moving
            notes.append(f"gripper frame unavailable: {e}")
        return Capture(color, depth, intr, T_wc, T_wg, ov, time.time(), notes)

    # ---- moving -----------------------------------------------------------
    def check_target(self, xyz) -> None:
        problems = reach_problems(xyz, settings.ARM_BASE_XY, settings.REACH_MM, settings.Z_LIMITS_MM)
        if problems:
            raise UnsafeTarget("target " + str(np.round(xyz, 1).tolist()) + " is " + "; ".join(problems))

    async def move_gripper(self, xyz, ov, *, linear: bool = False, obstacles=None, timeout: float = 60.0,
                           max_vel: float | None = None):
        """Move the gripper frame to xyz (world mm) with orientation vector ov = (ox, oy, oz, theta).

        linear=True keeps the gripper on a straight line (for going down to an object or
        toward a face). obstacles = list of GeometriesInFrame the planner must avoid.
        max_vel: joint speed cap (deg/s) asked of the motion service (Sidekick passes the same extra).
        """
        from viam.proto.common import Pose, PoseInFrame, WorldState
        from viam.proto.service.motion import Constraints, LinearConstraint
        self.check_target(xyz)
        dest = PoseInFrame(reference_frame="world", pose=Pose(
            x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]),
            o_x=float(ov[0]), o_y=float(ov[1]), o_z=float(ov[2]), theta=float(ov[3])))
        constraints = (Constraints(linear_constraint=[LinearConstraint(line_tolerance_mm=10.0, orientation_tolerance_degs=5.0)])
                       if linear else None)
        world = WorldState(obstacles=list(obstacles)) if obstacles else None
        extra = {"timeout": 15.0, **({"max_vel_degs_per_sec": float(max_vel)} if max_vel else {})}
        ok = await self.motion.move(component_name=self.gripper_name, destination=dest,
                                    world_state=world, constraints=constraints, extra=extra, timeout=timeout)
        if ok is False:
            raise RuntimeError("the motion service reported the move failed")

    async def move_joints(self, joints_deg):
        from viam.proto.component.arm import JointPositions
        if self.arm is None:
            raise RuntimeError(f"no arm named '{settings.ARM}'")
        await self.arm.move_to_joint_positions(JointPositions(values=[float(v) for v in joints_deg]))

    async def open_gripper(self):
        if self.gripper is not None:
            await self.gripper.open()

    async def grab(self) -> bool:
        return bool(await self.gripper.grab()) if self.gripper is not None else False

    async def stop(self):
        for part in (self.arm, self.gripper):
            if part is not None:
                try:
                    await asyncio.wait_for(part.stop(), 3)
                except Exception:
                    pass


async def ask(prompt: str, yes: bool = False) -> None:
    """Wait for Enter before a move (skipped with --yes). Ctrl+C stops everything."""
    if yes:
        for lead in ("Press Enter to ", "Press Enter for "):
            prompt = prompt.replace(lead, "")
        print("> " + prompt.strip())
        return
    loop = asyncio.get_running_loop()
    answered = loop.create_future()

    def settle(fn, *a):
        if not answered.done():
            fn(*a)

    def wait_for_enter():   # a daemon thread, so Ctrl+C never hangs waiting for Enter
        try:
            input(prompt + " ")
            loop.call_soon_threadsafe(settle, answered.set_result, None)
        except BaseException as e:
            loop.call_soon_threadsafe(settle, answered.set_exception, RuntimeError(f"no answer: {e!r}"))

    threading.Thread(target=wait_for_enter, daemon=True).start()
    await answered


def run_main(coro) -> int:
    """asyncio.run with a friendly Ctrl+C."""
    try:
        return asyncio.run(coro)
    except KeyboardInterrupt:
        print("\nStopped (Ctrl+C). The arm was told to stop; check it in the Viam app.")
        return 130


async def make_rig(args) -> Rig:
    """--sim gives the simulator, otherwise the real machine from .env."""
    if getattr(args, "sim", False):
        import sim
        kw = {}
        if getattr(args, "sim_spot", None) is not None:
            kw["bottle_xy"] = sim.CALIB_SPOTS[args.sim_spot % len(sim.CALIB_SPOTS)]
        return sim.make_rig(getattr(args, "scene", "table"), cam_mount_error=getattr(args, "sim_mount_error", None), **kw)
    return await Rig.connect(getattr(args, "env", None))


def add_sim_args(ap):
    """Simulator options shared by every script."""
    ap.add_argument("--sim", action="store_true", help="use the simulator instead of the robot")
    ap.add_argument("--sim-mount-error", type=float, nargs=3, metavar=("X", "Y", "Z"),
                    help="simulator only: camera mounted this far (mm) from where the config says")
    ap.add_argument("--env", help="path to the .env file (default: .env here)")
