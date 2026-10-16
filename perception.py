"""Seeing, in world coordinates: {object_id: (x, y) or None}.

MODE "vision" (the goal): Viam 3D segmenter(s) -> each object's center in the camera
frame -> world via Viam's frame system -> camera correction (calibration.py) ->
vote across several looks. Same pipeline last month's winner used
(detector -> detections-to-segments -> transform to world), plus the filtering
ThirdArm needed (size, workspace, voting).

MODE "slots" (the fallback): vision.py says which slot/zone each object is in;
we return that spot's taught position.

The look always happens from the overview pose with the arm still: the camera
rides the wrist, so a position is only valid for the pose it was seen from.
"""
import asyncio
import math

import anything
import calibration
import config
import vision


def segmenter_for(robot, name):
    """Overridable in tests."""
    from viam.services.vision import VisionClient
    return VisionClient.from_robot(robot.machine, name)


async def to_world(robot, ref, pose):
    """A pose in frame `ref` -> world. Overridable in tests."""
    from viam.proto.common import PoseInFrame
    if ref == "world":
        return pose
    return (await robot.machine.transform_pose(PoseInFrame(reference_frame=ref, pose=pose), "world")).pose


def object_for_label(label):
    label = (label or "").strip().lower()
    for obj, info in config.OBJECTS.items():
        if label == obj.lower() or label in [lab.lower() for lab in info.get("labels", [])]:
            return obj
    return None


def in_table_area(x, y, margin=50.0):
    (x0, x1), (y0, y1) = config.WORKSPACE["x"], config.WORKSPACE["y"]
    return x0 - margin <= x <= x1 + margin and y0 - margin <= y <= y1 + margin


def plausible_size(geometry):
    if not geometry.HasField("box"):
        return True
    d = geometry.box.dims_mm
    return max(d.x, d.y, d.z) <= config.MAX_OBJECT_DIM_MM


async def detections_once(robot, cal):
    """One look through every segmenter: [(object_id, x, y), ...] in corrected world mm."""
    found = []
    for service, fixed in config.SEGMENTERS.items():
        try:
            objs = await segmenter_for(robot, service).get_object_point_clouds(config.CAMERA)
        except Exception as e:  # noqa: BLE001
            print(f"  segmenter {service} failed: {e}")
            continue
        for o in objs:
            geoms = list(o.geometries.geometries)
            if not geoms:
                continue
            g = geoms[0]
            obj = fixed or object_for_label(g.label)
            if obj is None or not plausible_size(g):
                continue
            ref = o.geometries.reference_frame or config.CAMERA
            w = await to_world(robot, ref, g.center)
            x, y = calibration.apply(cal, w.x, w.y)
            if in_table_area(x, y):
                found.append((obj, x, y))
    return found


def vote(looks, min_hits=None):
    """looks: one list of (obj, x, y) per look -> {obj: (x, y) or None}.

    Groups each object's sightings into clusters (within MATCH_RADIUS_MM), counts in
    how many LOOKS each cluster appears, and keeps the best cluster if it was seen
    often enough and clearly beats the runner-up. Two equally strong clusters for one
    object (two cups?) -> None: refusing beats guessing.
    """
    min_hits = config.MIN_HITS if min_hits is None else min_hits
    clusters = {o: [] for o in config.OBJECTS}     # obj -> [[(x, y, look_i), ...], ...]
    for i, look in enumerate(looks):
        for obj, x, y in look:
            if obj not in clusters:
                continue
            for c in clusters[obj]:
                cx = sum(p[0] for p in c) / len(c)
                cy = sum(p[1] for p in c) / len(c)
                if math.dist((x, y), (cx, cy)) <= config.MATCH_RADIUS_MM:
                    c.append((x, y, i))
                    break
            else:
                clusters[obj].append([(x, y, i)])
    scene = {}
    for obj, cs in clusters.items():
        ranked = sorted(cs, key=lambda c: len({p[2] for p in c}), reverse=True)
        hits = [len({p[2] for p in c}) for c in ranked]
        if not ranked or hits[0] < min_hits or (len(hits) > 1 and hits[1] == hits[0]):
            scene[obj] = None
            continue
        best = ranked[0]
        scene[obj] = (sum(p[0] for p in best) / len(best), sum(p[1] for p in best) / len(best))
    return scene


async def find(robot, frames=None, cal=None):
    """Go to the overview pose, look, and return {object_id: (x, y) or None}."""
    await robot.go_overview()
    if hasattr(robot, "pos"):                       # SimRobot: it knows the truth
        return robot.seen()

    if config.MODE == "slots":
        spots = await vision.observe(robot, already_there=True)
        return {o: (config.ALL_SPOTS[s] if s else None) for o, s in spots.items()}

    cal = calibration.load() if cal is None else cal
    frames = config.OBSERVE_FRAMES if frames is None else frames
    await asyncio.sleep(config.SETTLE_S)
    if config.MODE == "anything":
        return await anything.find(robot, cal, frames)
    looks = []
    for i in range(frames):
        looks.append(await detections_once(robot, cal))
        if i < frames - 1:
            await asyncio.sleep(config.FRAME_GAP_S)
    return vote(looks)


def moved(obj, xy):
    """The arm just put `obj` at xy (so "anything" mode keeps its identity)."""
    if config.MODE == "anything":
        anything.expect(obj, xy)


def zone_of(pos):
    """Name of the zone a position is in, or None."""
    if pos is None:
        return None
    for z, c in config.ZONES.items():
        if math.dist(pos, c) <= config.ZONE_RADIUS_MM:
            return z
    return None
