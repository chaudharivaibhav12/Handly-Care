"""Sanity checks for config.py. Run `python app.py check` after every edit on Friday.

Catches the mistakes that are easy to make while teaching positions in a hurry:
a missing size, two spots sharing a position, a zone out of reach, overlapping
camera regions, a segmenter pointing at an unknown object, a bad direction.
"""
import math


def _reachable(cfg, x, y):
    (x0, x1), (y0, y1) = cfg.WORKSPACE["x"], cfg.WORKSPACE["y"]
    r = math.hypot(x, y)
    return x0 <= x <= x1 and y0 <= y <= y1 and cfg.MIN_REACH_MM <= r <= cfg.MAX_REACH_MM


def config_problems(cfg):
    problems = []
    objs = cfg.OBJECTS

    if cfg.MODE not in ("anything", "vision", "slots"):
        problems.append("MODE must be 'anything', 'vision' or 'slots'")
    if cfg.MODE == "anything":
        if not getattr(cfg, "ANYTHING_SEGMENTER", ""):
            problems.append("MODE 'anything' needs ANYTHING_SEGMENTER (the obstacles_pointcloud service name)")
        if not 30 <= getattr(cfg, "GRIPPER_MAX_OPEN_MM", 0) <= 200:
            problems.append("GRIPPER_MAX_OPEN_MM looks wrong (expected 30-200 mm)")

    for obj, info in objs.items():
        if not info.get("description"):
            problems.append(f"object '{obj}' is missing 'description'")
        size = info.get("size")
        if not (isinstance(size, (tuple, list)) and len(size) == 3 and all(v > 0 for v in size)):
            problems.append(f"object '{obj}' needs size (x, y, height) in mm, all above 0")
            continue
        gh = info.get("grasp_height")
        if gh is None:
            problems.append(f"object '{obj}' has no grasp_height")
        elif not (0 < gh <= size[2]):
            problems.append(f"grasp_height for '{obj}' ({gh} mm) must be above 0 and at most its height")
        if max(size[0], size[1]) > 80:
            problems.append(f"object '{obj}' is wider than the gripper can open (~85 mm)")

    # How objects are recognized
    if cfg.MODE == "vision":
        if not cfg.SEGMENTERS:
            problems.append("SEGMENTERS is empty: nothing to find objects with")
        for service, fixed in cfg.SEGMENTERS.items():
            if fixed is not None and fixed not in objs:
                problems.append(f"segmenter '{service}' points at unknown object '{fixed}'")
        if None in cfg.SEGMENTERS.values():
            for obj, info in objs.items():
                fixed_for = [s for s, f in cfg.SEGMENTERS.items() if f == obj]
                if not info.get("labels") and not fixed_for:
                    problems.append(f"object '{obj}' has no detector labels to be found by")
        labels = [lab.lower() for i in objs.values() for lab in i.get("labels", [])]
        dup = {lab for lab in labels if labels.count(lab) > 1}
        if dup:
            problems.append(f"labels used by two objects: {sorted(dup)}")
    elif cfg.MODE == "slots":
        detectors = [i.get("detector") for i in objs.values()]
        if None in detectors or len(set(detectors)) != len(detectors):
            problems.append("in slots mode every object needs its own detector")
        colors = [i.get("color") for i in objs.values()]
        if len(set(colors)) != len(colors):
            problems.append("two objects share a color; the camera can't tell them apart")

    # Taught places
    overlap = set(cfg.SLOTS) & set(cfg.ZONES)
    if overlap:
        problems.append(f"names used as both slot and zone: {sorted(overlap)}")
    spots = {**cfg.SLOTS, **cfg.ZONES}
    names = list(spots)
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            if math.dist(spots[a], spots[b]) < 60:
                problems.append(f"spots '{a}' and '{b}' are closer than 60 mm")
    for name, (x, y) in spots.items():
        if not _reachable(cfg, x, y):
            problems.append(f"spot '{name}' ({x}, {y}) is outside WORKSPACE or reach")
    for sname, sp in cfg.SLOTS.items():
        for zname, zp in cfg.ZONES.items():
            if math.dist(sp, zp) <= cfg.ZONE_RADIUS_MM:
                problems.append(f"slot '{sname}' is inside zone '{zname}' (within ZONE_RADIUS_MM)")
    zones = list(cfg.ZONES.items())
    for i, (a, pa) in enumerate(zones):
        for b, pb in zones[i + 1:]:
            if math.dist(pa, pb) < 2 * cfg.ZONE_RADIUS_MM:
                problems.append(f"zones '{a}' and '{b}' overlap (closer than 2 x ZONE_RADIUS_MM)")

    if cfg.USER_DIRECTION not in ("+x", "-x", "+y", "-y"):
        problems.append("USER_DIRECTION must be '+x', '-x', '+y' or '-y'")
    if cfg.MIN_REACH_MM >= cfg.MAX_REACH_MM:
        problems.append("MIN_REACH_MM must be below MAX_REACH_MM")
    if cfg.PLACE_TOLERANCE_MM <= 0 or cfg.ZONE_RADIUS_MM <= 0:
        problems.append("PLACE_TOLERANCE_MM and ZONE_RADIUS_MM must be above 0")

    # "slots" mode camera regions
    if cfg.MODE == "slots":
        for spot in spots:
            if spot not in cfg.REGIONS:
                problems.append(f"spot '{spot}' has no camera REGION")
        for r in cfg.REGIONS:
            if r not in spots:
                problems.append(f"REGION '{r}' doesn't match any slot or zone")
    regs = list(cfg.REGIONS.items())
    for name, (x0, y0, x1, y1) in regs:
        if not (x0 < x1 and y0 < y1):
            problems.append(f"REGION '{name}' has min >= max")
    for i, (a, ra) in enumerate(regs):
        for b, rb in regs[i + 1:]:
            if ra[0] < rb[2] and rb[0] < ra[2] and ra[1] < rb[3] and rb[1] < ra[3]:
                problems.append(f"REGIONS '{a}' and '{b}' overlap")

    if not (0 < cfg.MIN_BOX_PX < cfg.MAX_BOX_PX):
        problems.append("MIN_BOX_PX must be above 0 and below MAX_BOX_PX")
    if not (1 <= cfg.MIN_HITS <= cfg.OBSERVE_FRAMES):
        problems.append("MIN_HITS must be between 1 and OBSERVE_FRAMES")
    if cfg.HOVER < 50:
        problems.append("HOVER under 50 mm risks hitting other objects")
    if cfg.MAX_RETRIES < 0:
        problems.append("MAX_RETRIES can't be negative")
    return problems
