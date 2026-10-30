"""The math behind "see it, then reach it". No robot or camera needed, so it is easy to test.

Conventions (the same as Viam):
- Units are millimeters and degrees.
- Camera frame: +x to the right in the image, +y down in the image, +z out of the lens.
- An orientation is a Viam orientation vector (ox, oy, oz, theta): (ox, oy, oz) is the
  direction the frame's z axis points, and theta spins the frame around that axis.
- A transform T is a 4x4 matrix. T_world_cam turns camera-frame points into world points.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

# ---------------------------------------------------------------------------
# Camera model: pixel + depth <-> 3D point
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Intrinsics:
    """Pinhole camera model. fx, fy = focal length in pixels; cx, cy = image center."""

    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int

    def scaled_to(self, width: int, height: int) -> "Intrinsics":
        """The same camera at another image size (Viam may report intrinsics for a different size)."""
        if (width, height) == (self.width, self.height):
            return self
        sx, sy = width / self.width, height / self.height
        return Intrinsics(self.fx * sx, self.fy * sy, self.cx * sx, self.cy * sy, width, height)

    def to_dict(self) -> dict:
        return {"fx": self.fx, "fy": self.fy, "cx": self.cx, "cy": self.cy,
                "width": self.width, "height": self.height}

    @staticmethod
    def from_dict(d: dict) -> "Intrinsics":
        return Intrinsics(float(d["fx"]), float(d["fy"]), float(d["cx"]), float(d["cy"]),
                          int(d["width"]), int(d["height"]))


def deproject(u: float, v: float, depth_mm: float, intr: Intrinsics) -> np.ndarray:
    """Pixel (u = column, v = row) plus its depth -> 3D point in the camera frame (mm).

    This is the whole trick: the depth tells you how far along the pixel's ray the surface is.
        x = (u - cx) * z / fx,   y = (v - cy) * z / fy,   z = depth
    """
    z = float(depth_mm)
    return np.array([(u - intr.cx) * z / intr.fx, (v - intr.cy) * z / intr.fy, z])


def deproject_pixels(us: np.ndarray, vs: np.ndarray, depths: np.ndarray, intr: Intrinsics) -> np.ndarray:
    """Vectorized deproject: arrays of pixels and depths -> (N, 3) camera-frame points."""
    z = np.asarray(depths, dtype=np.float64)
    x = (np.asarray(us, dtype=np.float64) - intr.cx) * z / intr.fx
    y = (np.asarray(vs, dtype=np.float64) - intr.cy) * z / intr.fy
    return np.stack([x, y, z], axis=-1)


def project(points_cam: np.ndarray, intr: Intrinsics) -> np.ndarray:
    """3D camera-frame points (N, 3) -> pixels (N, 2). The inverse of deproject."""
    p = np.atleast_2d(np.asarray(points_cam, dtype=np.float64))
    z = p[:, 2]
    return np.stack([p[:, 0] * intr.fx / z + intr.cx, p[:, 1] * intr.fy / z + intr.cy], axis=-1)


# ---------------------------------------------------------------------------
# Rigid transforms
# ---------------------------------------------------------------------------


def make_T(R: np.ndarray, t) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t, dtype=np.float64)
    return T


def apply_T(T: np.ndarray, points) -> np.ndarray:
    """Apply a 4x4 transform to one point (3,) or many points (N, 3)."""
    p = np.asarray(points, dtype=np.float64)
    return p @ T[:3, :3].T + T[:3, 3]


def invert_T(T: np.ndarray) -> np.ndarray:
    R, t = T[:3, :3], T[:3, 3]
    return make_T(R.T, -R.T @ t)


def unit(v) -> np.ndarray:
    v = np.asarray(v, dtype=np.float64)
    n = np.linalg.norm(v)
    if n < 1e-12:
        raise ValueError("zero-length vector")
    return v / n


def orthonormalize(R: np.ndarray) -> np.ndarray:
    """Closest true rotation matrix to R (cleans up rounding noise)."""
    U, _, Vt = np.linalg.svd(R)
    Rn = U @ Vt
    if np.linalg.det(Rn) < 0:
        U[:, -1] *= -1
        Rn = U @ Vt
    return Rn


def T_from_probe(origin, px, py, pz, step_mm: float) -> np.ndarray:
    """Build a frame's transform from where 4 of its points landed in world.

    We ask Viam to transform the frame's origin and the points step_mm along its x, y and z
    axes. The differences are the frame's axes in world. No orientation-vector math needed,
    so there is nothing to get wrong about Viam's conventions.
    """
    o = np.asarray(origin, dtype=np.float64)
    R = np.column_stack([(np.asarray(px) - o) / step_mm,
                         (np.asarray(py) - o) / step_mm,
                         (np.asarray(pz) - o) / step_mm])
    err = float(np.abs(R.T @ R - np.eye(3)).max())
    if err > 0.02:
        raise ValueError(f"probed axes are not a rotation (error {err:.3f}); frame system looks broken")
    return make_T(orthonormalize(R), o)


# ---------------------------------------------------------------------------
# Viam orientation vectors <-> rotation matrices
# ---------------------------------------------------------------------------
_POLE_EPS = 1e-4   # Viam treats |oz| this close to 1 as "pointing straight up/down"


def _rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _ry(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def ov_to_matrix(ox: float, oy: float, oz: float, theta_deg: float) -> np.ndarray:
    """Viam orientation vector -> 3x3 rotation matrix (same math as Viam's spatialmath)."""
    n = math.sqrt(ox * ox + oy * oy + oz * oz)
    if n < 1e-12:
        raise ValueError("orientation vector is all zeros")
    ox, oy, oz = ox / n, oy / n, oz / n
    lat = math.acos(max(-1.0, min(1.0, oz)))
    lon = math.atan2(oy, ox) if 1.0 - abs(oz) > _POLE_EPS else 0.0
    return _rz(lon) @ _ry(lat) @ _rz(math.radians(theta_deg))


def matrix_to_ov(R: np.ndarray) -> tuple[float, float, float, float]:
    """3x3 rotation matrix -> Viam orientation vector (ox, oy, oz, theta_deg)."""
    R = orthonormalize(np.asarray(R, dtype=np.float64))
    ox, oy, oz = R[:, 2]
    if 1.0 - abs(oz) <= _POLE_EPS:            # straight up or down
        ox, oy, oz = 0.0, 0.0, (1.0 if oz > 0 else -1.0)
        lat, lon = (0.0 if oz > 0 else math.pi), 0.0
    else:
        lat, lon = math.acos(max(-1.0, min(1.0, oz))), math.atan2(oy, ox)
    M = _ry(lat).T @ _rz(lon).T @ R
    theta = math.degrees(math.atan2(M[1, 0], M[0, 0]))
    return float(ox), float(oy), float(oz), float(theta)


def rotation_between(a, b) -> np.ndarray:
    """The smallest rotation that turns direction a into direction b."""
    a, b = unit(a), unit(b)
    v = np.cross(a, b)
    c = float(np.dot(a, b))
    s = float(np.linalg.norm(v))
    if s < 1e-9:
        if c > 0:
            return np.eye(3)
        helper = np.array([1.0, 0.0, 0.0]) if abs(a[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        k = unit(np.cross(a, helper))          # any axis at right angles to a
        return 2.0 * np.outer(k, k) - np.eye(3)  # 180 degrees about k
    vx = np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])
    return np.eye(3) + vx + vx @ vx * ((1.0 - c) / (s * s))


def angle_deg(a, b) -> float:
    """Angle between two directions, in degrees."""
    return math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(unit(a), unit(b)))))))


# ---------------------------------------------------------------------------
# Thermometer: where should the gripper go so the tip stops in front of the forehead?
# ---------------------------------------------------------------------------


@dataclass
class ForeheadApproach:
    forehead: np.ndarray        # world mm
    axis: np.ndarray            # world direction the thermometer points (unit)
    R_gripper: np.ndarray       # gripper orientation used for the whole approach (3x3, world)
    tip_final: np.ndarray       # where the thermometer tip stops
    tip_pre: np.ndarray         # first stop, further back
    gripper_final: np.ndarray   # gripper frame origin for tip_final (this is what we command)
    gripper_pre: np.ndarray
    steps: list = field(default_factory=list)   # gripper positions for the slow last stretch

    def summary(self) -> dict:
        r = lambda v: [round(float(x), 1) for x in v]
        return {"forehead": r(self.forehead), "axis": [round(float(x), 3) for x in self.axis],
                "tip_final": r(self.tip_final), "tip_pre": r(self.tip_pre),
                "gripper_final": r(self.gripper_final), "gripper_pre": r(self.gripper_pre),
                "steps": [r(s) for s in self.steps]}


def plan_forehead_approach(forehead_w, R_world_gripper, axis_in_gripper, tip_in_gripper,
                           standoff_mm: float, pre_extra_mm: float, n_steps: int,
                           aim_direction=None) -> ForeheadApproach:
    """Gripper positions that put the thermometer tip `standoff_mm` in front of the forehead.

    By default the gripper keeps its current orientation, so the arm only slides; the tip
    is placed on the line through the forehead along the thermometer's own axis.
    With aim_direction (a world vector), the gripper first turns by the smallest rotation
    that makes the thermometer point that way.
    """
    F = np.asarray(forehead_w, dtype=np.float64)
    R = np.asarray(R_world_gripper, dtype=np.float64)
    axis_g = unit(axis_in_gripper)
    if aim_direction is not None:
        R = rotation_between(R @ axis_g, aim_direction) @ R
    a = unit(R @ axis_g)
    tip_offset_w = R @ np.asarray(tip_in_gripper, dtype=np.float64)
    tip_final = F - standoff_mm * a
    tip_pre = F - (standoff_mm + pre_extra_mm) * a
    g_final, g_pre = tip_final - tip_offset_w, tip_pre - tip_offset_w
    n = max(1, int(n_steps))
    steps = [g_pre + (g_final - g_pre) * (k / n) for k in range(1, n + 1)]
    return ForeheadApproach(F, a, R, tip_final, tip_pre, g_final, g_pre, steps)


def tip_position(gripper_pos_w, R_world_gripper, tip_in_gripper) -> np.ndarray:
    """Where the thermometer tip is for a given gripper pose."""
    return np.asarray(gripper_pos_w, dtype=np.float64) + np.asarray(R_world_gripper) @ np.asarray(tip_in_gripper, dtype=np.float64)


# ---------------------------------------------------------------------------
# Safety checks on targets
# ---------------------------------------------------------------------------


def in_box(p, box: dict | None) -> bool:
    """box = {"x": (lo, hi), "y": (lo, hi), "z": (lo, hi)}; None means no limit."""
    if box is None:
        return True
    return all(lo <= float(p[i]) <= hi for i, (lo, hi) in ((0, box["x"]), (1, box["y"]), (2, box["z"])))


def reach_problems(p, base_xy, reach_mm, z_limits) -> list[str]:
    """Reasons a gripper target is out of bounds. An empty list means it is fine."""
    problems = []
    r = math.hypot(float(p[0]) - base_xy[0], float(p[1]) - base_xy[1])
    if not reach_mm[0] <= r <= reach_mm[1]:
        problems.append(f"{r:.0f} mm from the arm base (allowed {reach_mm[0]:.0f}-{reach_mm[1]:.0f})")
    if not z_limits[0] <= float(p[2]) <= z_limits[1]:
        problems.append(f"z = {float(p[2]):.0f} mm (allowed {z_limits[0]:.0f} to {z_limits[1]:.0f})")
    return problems


# ---------------------------------------------------------------------------
# Re-anchoring a taught routine (e.g. the drawer) to where the object really is
# ---------------------------------------------------------------------------


def reanchor(points_w, taught_anchor, measured_anchor) -> np.ndarray:
    """Move taught waypoints along with their object.

    taught_anchor / measured_anchor = (x, y, z, yaw_deg) of the object (e.g. a tag on the
    drawer) when you taught the routine and now. Waypoints are turned about the vertical
    axis through the object and shifted with it, so the routine follows the object. (N, 3).
    """
    pts = np.atleast_2d(np.asarray(points_w, dtype=np.float64))
    tx, ty, tz, tyaw = taught_anchor
    mx, my, mz, myaw = measured_anchor
    Rz = _rz(math.radians(myaw - tyaw))
    rel = pts - np.array([tx, ty, tz])
    return rel @ Rz.T + np.array([mx, my, mz])


def reanchor_orientation(ov, taught_yaw_deg: float, measured_yaw_deg: float):
    """Turn a taught orientation by the same yaw the object turned."""
    R = _rz(math.radians(measured_yaw_deg - taught_yaw_deg)) @ ov_to_matrix(*ov)
    return matrix_to_ov(R)
