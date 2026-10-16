"""Seeing: from the overview pose, which object is in which spot?

Stage A (build this first): each object has its own Viam color_detector service.
We keep only detections that are object-sized and sit inside a known spot's
pixel region, then VOTE over several frames, because color detectors are noisy:
at last month's hackathon (team ThirdArm, same hardware) they returned huge
background regions (a 409x719 px "blue" blob in a corner, blue carpet on the
floor) and phantom objects that came and went between frames. So:
  1. size filter    (config.MIN_BOX_PX..MAX_BOX_PX)   - drops background blobs
  2. region filter  (center must be in a slot/zone)   - drops the floor, walls
  3. frame voting   (config.OBSERVE_FRAMES, MIN_HITS) - drops flickering phantoms
No 3D math needed.

Stage B (only if Stage A works): use a 3D segmenter to get world positions and
pick objects from anywhere. Ask a Viam engineer to help set it up.
"""
import asyncio
from collections import Counter

import config


def spot_for_pixel(cx, cy):
    for spot, (x0, y0, x1, y1) in config.REGIONS.items():
        if x0 <= cx < x1 and y0 <= cy < y1:
            return spot
    return None


def detector_for(robot, name):
    """Overridable in tests."""
    from viam.services.vision import VisionClient
    return VisionClient.from_robot(robot.machine, name)


def plausible(d):
    """Confident, and the size of a desk object as seen from the overview pose."""
    w, h = d.x_max - d.x_min, d.y_max - d.y_min
    return (d.confidence >= config.MIN_CONFIDENCE
            and config.MIN_BOX_PX <= w <= config.MAX_BOX_PX
            and config.MIN_BOX_PX <= h <= config.MAX_BOX_PX)


def spot_in_frame(dets):
    """The spot for one object in ONE frame, or None.

    Only plausible detections inside a known region count. If they point at
    two different spots, the frame is ambiguous and casts no vote.
    """
    spots = {spot_for_pixel((d.x_min + d.x_max) / 2, (d.y_min + d.y_max) / 2)
             for d in dets if plausible(d)}
    spots.discard(None)
    return spots.pop() if len(spots) == 1 else None


def vote(frame_spots, min_hits=None):
    """Most common spot across frames, if it was seen at least `min_hits` times."""
    min_hits = config.MIN_HITS if min_hits is None else min_hits
    counts = Counter(s for s in frame_spots if s is not None)
    if not counts:
        return None
    (spot, hits), *rest = counts.most_common()
    if hits < min_hits or (rest and rest[0][1] == hits):   # too rare, or a tie
        return None
    return spot


# Kept for snapshot printing and older callers.
def best_detection(dets):
    dets = [d for d in dets if plausible(d)]
    return max(dets, key=lambda d: d.confidence) if dets else None


async def observe(robot, frames=None, already_there=False):
    """Go to the overview pose and return {object_id: spot_or_None} for every object.
    ("slots" mode; perception.find calls this and turns spots into positions.)"""
    if not already_there:
        await robot.go_overview()

    frames = config.OBSERVE_FRAMES if frames is None else frames
    await asyncio.sleep(config.SETTLE_S)   # let the arm stop shaking before looking
    seen = {obj: [] for obj in config.OBJECTS}
    for i in range(frames):
        for obj, info in config.OBJECTS.items():
            try:
                dets = await detector_for(robot, info["detector"]).get_detections_from_camera(config.CAMERA)
            except Exception as e:  # noqa: BLE001
                print(f"  detector {info['detector']} failed: {e}")
                dets = []
            seen[obj].append(spot_in_frame(dets))
        if i < frames - 1:
            await asyncio.sleep(config.FRAME_GAP_S)
    return {obj: vote(spots) for obj, spots in seen.items()}


async def snapshot(robot, path="overview.jpg"):
    """Save the overview image and print every detection's pixel center and size.
    Use this Friday to fill in config.REGIONS and MIN/MAX_BOX_PX."""
    await robot.go_overview()
    try:   # color only: skips the ~1.8 MB depth image
        images, _ = await robot.camera.get_images(filter_source_names=["color"])
    except TypeError:
        images, _ = await robot.camera.get_images()
    if not images:
        images, _ = await robot.camera.get_images()
    # A depth camera returns a color image and a depth image ("image/vnd.viam.dep");
    # save the color one, a normal JPEG or PNG.
    color = next((im for im in images if str(im.mime_type) in ("image/jpeg", "image/png")), None)
    if color is None:
        print(f"no JPEG/PNG image from the camera, got: {[str(im.mime_type) for im in images]}")
        return None
    ext = "png" if "png" in str(color.mime_type) else "jpg"
    path = path.rsplit(".", 1)[0] + "." + ext
    with open(path, "wb") as f:
        f.write(color.data)
    print(f"saved {path} (camera sent: {[getattr(im, 'name', im.mime_type) for im in images]})")
    for obj, info in config.OBJECTS.items():
        for d in await detector_for(robot, info["detector"]).get_detections_from_camera(config.CAMERA):
            cx, cy = (d.x_min + d.x_max) / 2, (d.y_min + d.y_max) / 2
            w, h = d.x_max - d.x_min, d.y_max - d.y_min
            flag = "" if plausible(d) else "   <- ignored (size/confidence)"
            print(f"  {obj:11s} conf={d.confidence:.2f} center=({cx:.0f},{cy:.0f}) size={w:.0f}x{h:.0f} "
                  f"-> {spot_for_pixel(cx, cy)}{flag}")
    return path
