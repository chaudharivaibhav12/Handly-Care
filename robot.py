"""Moving things: one tested routine, move_object(obj, src, dst).

The planner never sends coordinates. It names an object and a destination,
and this file turns that into motions using the poses in config.py.

RealRobot follows the patterns from Viam 101: motion.move on the gripper frame,
destinations as PoseInFrame in "world", the tool pointing straight down.
SimRobot fakes all of it so you can test the rest of the program without hardware.
"""
import asyncio
import math
import os
import random
import time

import config


def tool_pose(x, y, z, o, theta):
    """Fingertips at (x, y, z) with the tool pointing along o = (o_x, o_y, o_z) (Viam orientation vector)."""
    from viam.proto.common import Pose
    return Pose(x=x, y=y, z=z, o_x=o[0], o_y=o[1], o_z=o[2], theta=theta)


def down_pose(x, y, z, theta=None):
    from viam.proto.common import Pose
    # Tool z-axis pointing at the table (Viam 101's down_pose), wrist turned to TOOL_THETA:
    # theta=0 makes the planner reject every solution (wrist_link vs cam self-collision).
    return Pose(x=x, y=y, z=z, o_x=0, o_y=0, o_z=-1, theta=config.TOOL_THETA if theta is None else theta)


def straight_line(orient_tol=None):
    from viam.proto.service.motion import Constraints, LinearConstraint
    kw = {"orientation_tolerance_degs": orient_tol} if orient_tol else {}
    return Constraints(linear_constraint=[LinearConstraint(line_tolerance_mm=config.LINE_TOLERANCE_MM, **kw)])


def _why(e):
    """The useful part of a motion-planner refusal (which parts would collide, and how often)."""
    text = str(e)
    i = text.find("Failures:")
    return (text[i:] if i >= 0 else text)[:500]


class MoveError(Exception):
    """The motion planner couldn't do a move. holding=True: the object may still be in the gripper."""
    def __init__(self, msg, holding=False):
        super().__init__(msg)
        self.holding = holding


def obstacle_size(name):
    return config.OBJECTS.get(name, {}).get("size") or config.UNKNOWN_OBSTACLE_SIZE


def world_state(others):
    """Every other seen object as a box the planner must keep the arm out of (+ margin, square because we
    don't know its turn). Fri 2:53: a free carry move swept the hanging wire through two objects."""
    from viam.proto.common import GeometriesInFrame, Geometry, Pose, RectangularPrism, Vector3, WorldState
    m = config.OBSTACLE_MARGIN_MM
    geoms = []
    for name, (x, y) in sorted(others.items()):
        w, d, h = obstacle_size(name)
        side = max(w, d) + 2 * m
        geoms.append(Geometry(center=Pose(x=x, y=y, z=config.TABLE_TOP_WORLD_Z + (h + m) / 2, o_z=1),
                              box=RectangularPrism(dims_mm=Vector3(x=side, y=side, z=h + m)), label=name))
    return WorldState(obstacles=[GeometriesInFrame(reference_frame="world", geometries=geoms)])


def is_tilted(p):
    """Is the tool NOT pointing straight down? (An all-zero orientation = unknown = treated as down.)"""
    ox, oy, oz = (getattr(p, k, 0.0) for k in ("o_x", "o_y", "o_z"))
    if ox == oy == oz == 0:
        return False
    return abs(oz + 1.0) > 0.02 or abs(ox) > 0.05 or abs(oy) > 0.05


def arm_halted(e):
    """The arm itself stopped (E-stop pressed, servo fault): never retry, never try another path.
    Fri 4:23 PM: after the E-stop, a refused straight lift fell back to a free move and was retried twice."""
    text = str(e)
    return any(k in text for k in ("Emergency Stop", "Servo motor", "emergency stop"))


async def retry(label, fn, tries=3, delay=1.0):
    """Ride out a brief connection blip (Viam 101's helpers.retry does the same job)."""
    for attempt in range(tries):
        try:
            return await fn()
        except Exception as e:  # noqa: BLE001
            # A planner refusal ("all IK solutions failed constraints") is not a blip: asking again just
            # burned ~30 s with the same answer (Fri 3:39). Only connection trouble is retried.
            if attempt == tries - 1 or "IK solutions failed" in str(e) or arm_halted(e):
                raise
            print(f"  [{label}] failed ({e}); retrying")
            await asyncio.sleep(delay)


class RealRobot:
    def __init__(self, machine, arm, gripper, motion, camera):
        self.machine, self.arm, self.gripper, self.motion, self.camera = machine, arm, gripper, motion, camera
        self.retry_delay = 1.0

    @staticmethod
    async def dial():
        """Just the connection, no resource lookups (so `resources` works even with wrong names in config)."""
        from viam.robot.client import RobotClient
        # The SDK only builds clients for resource types imported BEFORE connecting: without these,
        # VisionClient.from_robot raised ResourceNotFoundError for a READY service (Fri 1:10 PM).
        import viam.components.arm, viam.components.camera, viam.components.gripper  # noqa: F401,E401
        import viam.services.motion, viam.services.vision  # noqa: F401,E401
        opts = RobotClient.Options.with_api_key(
            api_key=os.environ["VIAM_API_KEY"],
            api_key_id=os.environ["VIAM_API_KEY_ID"],
        )
        return await RobotClient.at_address(os.environ["VIAM_ADDRESS"], opts)

    @classmethod
    async def connect(cls):
        from viam.components.arm import Arm
        from viam.components.camera import Camera
        from viam.components.gripper import Gripper
        from viam.services.motion import MotionClient
        m = await cls.dial()
        return cls(m, Arm.from_robot(m, config.ARM), Gripper.from_robot(m, config.GRIPPER),
                   MotionClient.from_robot(m, config.MOTION), Camera.from_robot(m, config.CAMERA))

    async def close(self):
        await self.machine.close()

    async def move_to(self, x, y, z, straight=False, theta=None, avoid=None, must_be_straight=False, o=None,
                      max_vel=None, orient_tol=None):
        """Move the gripper's fingertips to (x, y, z), pointing down.
        straight=True asks for a straight-line path (used for going down/up).
        must_be_straight: never fall back to a free move (e.g. while holding a drawer knob).
        theta: wrist angle, default config.TOOL_THETA. avoid: WorldState of obstacles for the planner.
        o: tool direction instead of straight down (the drawer knob is gripped tilted).
        max_vel: joint speed cap (deg/s) passed to the planner as an extra (drawer moves).
        orient_tol: degrees the tool may turn along a straight line (drawer: a strict tilted line had no solution)."""
        from viam.proto.common import PoseInFrame
        pose = down_pose(x, y, z, theta) if o is None else tool_pose(x, y, z, o, theta)
        dest = PoseInFrame(reference_frame="world", pose=pose)

        async def go(constraints):
            return await self.motion.move(
                component_name=config.GRIPPER,
                destination=dest,
                constraints=constraints,
                extra={"timeout": 15.0, **({"max_vel_degs_per_sec": max_vel} if max_vel else {})},
                timeout=120,
                **({"world_state": avoid} if avoid is not None else {}),
            )

        if must_be_straight:
            return await go(straight_line(orient_tol))
        if straight and config.STRAIGHT_LINE:
            try:
                return await go(straight_line(orient_tol))
            except Exception as e:  # noqa: BLE001
                if arm_halted(e):
                    raise
                # The planner couldn't find a straight path; fall back to a normal move.
                print(f"  straight-line move refused ({e}); using a normal move")
        return await retry("move", lambda: go(None), delay=self.retry_delay)

    async def go_overview(self):
        # From low (e.g. left touching the table after teaching a spot), lift straight up first so the
        # planned path can't sweep across the objects on the table.
        p = None
        try:
            p = await self.where()
            if is_tilted(p) and p.z < config.TO_HIGH_LIFT_BELOW_Z:
                # Low and tilted (the drawer/pill poses): the planner won't plan a tilted straight lift (EXP-007),
                # so lean shoulder and elbow back in joint space first, like by hand at 7:02 PM.
                await self.lean_back()
            elif p.z < config.TABLE_Z + config.HOVER:
                await self.lift(p, config.HOVER + 30)
        except Exception as e:  # noqa: BLE001
            if arm_halted(e) or (p is not None and is_tilted(p)):
                # Low AND tilted (e.g. at the drawer): a free move from here swung the arm across the table.
                raise MoveError("I'm low and tilted and couldn't lift straight up; move me clear by hand",
                                holding=False) from e
            print(f"  couldn't lift first ({e}); going straight to the overview")
        await self.move_to(*config.OVERVIEW, theta=config.OVERVIEW_THETA)

    async def lean_back(self):
        """Shoulder and elbow back TO_HIGH_LIFT_DEG each (raises the gripper ~25 cm on the xArm6), slowly,
        joint space. A 'context canceled' reply is checked against where the arm actually is."""
        from viam.proto.component.arm import JointPositions, MoveOptions
        j = list((await self.arm.get_joint_positions()).values)
        j[1] -= config.TO_HIGH_LIFT_DEG
        j[2] -= config.TO_HIGH_LIFT_DEG
        opts = MoveOptions(max_vel_degs_per_sec=10.0, max_acc_degs_per_sec2=20.0)
        for attempt in range(3):
            try:
                await self.arm.move_through_joint_positions([JointPositions(values=j)], options=opts)
                return
            except Exception as e:  # noqa: BLE001
                if arm_halted(e) or "anceled" not in str(e) or attempt == 2:
                    raise
                now = list((await self.arm.get_joint_positions()).values)
                if max(abs(a - b) for a, b in zip(now, j)) < 0.5:
                    return

    async def lift(self, p, dz):
        """Straight up by dz from pose p, KEEPING p's tool direction and wrist angle. Fri 4:59: lifting from the
        tilted drawer pose as 'pointing down' spun the wrist ~240 deg and swung the arm low across the table."""
        tilted = is_tilted(p)
        ox, oy, oz = (getattr(p, k, 0.0) for k in ("o_x", "o_y", "o_z"))
        await self.move_to(p.x, p.y, p.z + dz, straight=True, theta=getattr(p, "theta", None),
                           o=(ox, oy, oz) if tilted else None, must_be_straight=tilted)
        return tilted

    async def where(self):
        """Current gripper pose in world. Use this to teach slots, zones and OVERVIEW."""
        p = await self.motion.get_pose(component_name=config.GRIPPER, destination_frame="world")
        return p.pose

    async def move_object(self, obj, src, dst, lift=0.0, others=None):
        """Pick `obj` at position `src` (x, y) and set it down at `dst` (x, y).
        lift: height of what it lands on (e.g. a cube when stacking), mm.
        others: {name: (x, y)} of the other objects on the table; carry moves go above and around them.
        Returns False (and carries nothing) if the grab missed."""
        sx, sy = src
        dx, dy = dst
        grip = config.OBJECTS[obj]["grasp_height"]
        grasp_z = config.TABLE_Z + grip
        place_z = config.TABLE_Z + lift + grip + config.PLACE_DROP_MM
        travel_z = max(grasp_z, place_z) + config.HOVER   # carried object clears the table
        others = {o: p for o, p in (others or {}).items() if o != obj and p is not None}
        avoid = None
        if others:
            # Carried object's bottom (travel_z - grip) clears the tallest other object by CARRY_CLEARANCE_MM.
            tallest = max(obstacle_size(o)[2] for o in others)
            travel_z = max(travel_z, config.TABLE_Z + lift + tallest + grip + config.CARRY_CLEARANCE_MM)
            avoid = world_state(others)

        low_z = max(grasp_z, place_z) + config.HOVER      # the plain carry height (worked all day)

        async def carry_to(x, y):
            # High carry first; near the walls it can be out of reach (Fri 3:39: cam_origin vs wall-front at
            # z ~325 next to slot4). Then the plain height, still planned around the other objects.
            try:
                await self.move_to(x, y, travel_z, avoid=avoid)
                return travel_z
            except Exception as e:  # noqa: BLE001
                if travel_z <= low_z:
                    raise
                print(f"  high carry refused ({_why(e)}); carrying at the normal height")
                await self.move_to(x, y, low_z, avoid=avoid)
                return low_z

        await self.gripper.open()
        try:
            z = await carry_to(sx, sy)                       # above it
        except Exception as e:  # noqa: BLE001
            raise MoveError(f"no safe path to the {obj}", holding=False) from e
        await self.move_to(sx, sy, grasp_z, straight=True)   # straight down
        held = await self.gripper.grab()                     # fingers close
        await self.move_to(sx, sy, z, straight=True)         # straight up
        if config.TRUST_GRAB_SENSOR and not held:
            await self.gripper.open()
            return False
        try:
            z = await carry_to(dx, dy)                       # over the destination
        except Exception as e:  # noqa: BLE001
            # Fri 3:39: this used to crash the app with the object still in the gripper. Put it back instead.
            print(f"  carry refused ({_why(e)}); putting the {obj} back")
            try:
                await self.move_to(sx, sy, z)
                await self.move_to(sx, sy, grasp_z + config.PLACE_DROP_MM, straight=True)
                await self.gripper.open()
                await self.move_to(sx, sy, z, straight=True)
            except Exception:  # noqa: BLE001
                raise MoveError(f"no safe path to put the {obj} down, and I couldn't put it back",
                                holding=True) from e
            raise MoveError(f"no safe path to there, so I put the {obj} back", holding=False) from e
        await self.move_to(dx, dy, place_z, straight=True)   # straight down
        await self.gripper.open()
        await self.move_to(dx, dy, z, straight=True)         # straight up, clear of it
        return True

    async def replay(self, data, steps, start="high"):
        """Run taught.py steps: joint moves through named poses at the drawer speed cap, gripper open/grab, waits.
        No motion planner (blind): poses must be taught with clear space. Starts only near the `start` pose."""
        from viam.proto.component.arm import JointPositions, MoveOptions
        opts = MoveOptions(max_vel_degs_per_sec=config.DRAWER_MAX_VEL_DEGS,
                           max_acc_degs_per_sec2=config.DRAWER_MAX_ACC_DEGS)
        # The first move (from wherever the arm is to the first pose) is not taught, and joint moves are blind.
        now = (await self.arm.get_joint_positions()).values
        jump = max(abs(a - b) for a, b in zip(now, data[start]["joints"]))
        if jump > config.DRAWER_MAX_FIRST_JUMP_DEG:
            raise MoveError(f"I'm too far from the start pose ({jump:.0f} deg on one joint); "
                            f"move me near 'high' first", holding=False)

        fast = MoveOptions(max_vel_degs_per_sec=config.THERM_FAST_VEL_DEGS,
                           max_acc_degs_per_sec2=config.THERM_FAST_ACC_DEGS)

        async def through(names, o=opts):
            # Fri 5:38: 'context canceled' (connection drop) stopped a close halfway. Same target = safe to resend.
            target = data[names[-1]]["joints"]
            left = list(names)            # waypoints still to go: a resend never goes back to ones already passed

            async def once():
                nonlocal left
                try:
                    await self.arm.move_through_joint_positions(
                        [JointPositions(values=data[n]["joints"]) for n in left], options=o)
                except Exception as e:  # noqa: BLE001
                    if arm_halted(e) or "anceled" not in str(e):
                        raise
                    # Fri 10:3x PM + Sat 12:21: the drawer pull came back 'canceled' 1.7 deg short of open_far, 3 times
                    # in a row (the drawer's end stop, most likely), killing the run while gripping the knob.
                    # Essentially there = carry on; resending only pushes into the stop again.
                    now = list((await self.arm.get_joint_positions()).values)
                    gap = max(abs(a - b) for a, b in zip(now, target))
                    if gap > config.CANCEL_ACCEPT_DEG:
                        # Sat 1:00 PM: cancels hit free moves too (retreat -> high, pill_above), not only the pull.
                        # Resend from where the arm is: drop the waypoints it has already reached or passed, so a
                        # multi-point move (the pull, gripping the knob) never goes backwards.
                        d = [max(abs(a - b) for a, b in zip(now, data[n]["joints"])) for n in left]
                        k = min(range(len(left)), key=d.__getitem__)
                        if d[k] < config.CANCEL_ACCEPT_DEG and k < len(left) - 1:
                            k += 1                                   # already at that waypoint: head for the next
                        left = left[k:]
                        raise
                    print(f"  {names[-1]}: move reported canceled {gap:.1f} deg short; close enough, going on")
            await retry("joint move", once, tries=config.JOINT_MOVE_TRIES, delay=self.retry_delay)

        t0 = time.time()
        for kind, args in steps:
            if kind == "release":
                await self.gripper.open()
            elif kind == "grab":
                await self.gripper.grab()
            elif kind == "set":
                # xArm gripper position, 0 (closed) .. 850 (open), as last month's winner used it
                await self.gripper.do_command({"set": int(args[0])})
                await asyncio.sleep(config.THERM_GRIPPER_SETTLE_S)
                print(f"  gripper -> {int(args[0])}")
            elif kind == "click":
                # Press a button held in the fingers for a moment, then let it go (no settle wait in between).
                press, hold, secs = args
                c0 = time.time()
                await self.gripper.do_command({"set": int(press)})
                c1 = time.time()
                await asyncio.sleep(secs)
                await self.gripper.do_command({"set": int(hold)})
                c2 = time.time()
                # Timings show whether 'set' waits for the fingers (then the press lasts longer than secs).
                print(f"  click: gripper {int(press)} (set took {c1 - c0:.2f} s) + {secs} s, back to {int(hold)} "
                      f"(set took {c2 - c1 - secs:.2f} s)")
            elif kind == "fast":
                await through(args, fast)
                print(f"  FAST {' -> '.join(args)}: {time.time() - t0:.1f} s")
                t0 = time.time()
            elif kind == "slow":
                # the pour tilt: slower than the drawer cap so the water doesn't slosh out
                await through(args, MoveOptions(max_vel_degs_per_sec=config.POUR_VEL_DEGS,
                                                max_acc_degs_per_sec2=config.POUR_ACC_DEGS))
                print(f"  slow {' -> '.join(args)}: {time.time() - t0:.1f} s")
                t0 = time.time()
            elif kind == "turned":
                # A taught pose with the last wrist joint turned by `deg` (the cap twist). xArm6 joint 6: +-360 deg.
                name, deg = args
                j = list(data[name]["joints"])
                j[5] += deg
                if abs(j[5]) > config.WRIST_LIMIT_DEG:
                    raise MoveError(f"twisting to {j[5]:.0f} deg would pass the wrist limit; re-teach "
                                    f"'{name}' with the wrist turned the other way", holding=False)
                twist = MoveOptions(max_vel_degs_per_sec=config.CAP_TWIST_VEL_DEGS,
                                    max_acc_degs_per_sec2=config.CAP_TWIST_ACC_DEGS)
                await retry("twist", lambda: self.arm.move_through_joint_positions(
                    [JointPositions(values=j)], options=twist), delay=self.retry_delay)
                print(f"  {name} wrist {deg:+.0f}: {time.time() - t0:.1f} s")
                t0 = time.time()
            elif kind == "grab?":
                held = await self.gripper.grab()
                if config.TRUST_GRAB_SENSOR and not held:
                    # Verify every step: an empty grab goes back up and says so, never "hands over" nothing.
                    await self.gripper.open()
                    await through(["pill_above", "high"])
                    raise MoveError("I missed it: the gripper closed on nothing", holding=False)
            elif kind == "wait":
                await asyncio.sleep(args[0])
            else:
                await through(args)
                print(f"  {' -> '.join(args)}: {time.time() - t0:.1f} s")
                t0 = time.time()

    async def drawer(self, action="open"):
        """Open (or close) the medicine drawer by replaying the joint poses taught by hand (taught.py), at the
        DRAWER_MAX_VEL_DEGS cap. No motion planner: it planned none of the tilted straight lines (EXP-007).
        Returns the knob's travel (mm) from the taught poses."""
        import taught
        data = taught.load()
        if "open" in data:
            taught.add_open_far(data, config.DRAWER_EXTRA_OPEN_MM)
        if action != "open" and "grip" in data:
            taught.add_close_stop(data, config.DRAWER_CLOSE_SHORT_MM)
        await self.replay(data, taught.sequence(data, action))
        slide = (["grip" if action == "open" or "close_stop" not in data else "close_stop"] + taught.pulls(data)
                 + ["open"] + (["open_far"] if "open_far" in data else []))
        pts = [tuple(data[n]["pose"][k] for k in ("x", "y", "z")) for n in slide]
        return sum(math.dist(p, q) for p, q in zip(pts, pts[1:]))

    async def to_high(self, log=print):
        """Bring the arm to the drawer's 'high' pose one joint at a time (TO_HIGH_ORDER), in small slow steps,
        checking the gripper height after every step. If it starts low, shoulder and elbow lean back first.
        A 'context canceled' step is re-read and resumed (it often arrives after the move finished, LOG 6:36)."""
        import taught
        from viam.proto.component.arm import JointPositions, MoveOptions
        target = taught.load()["high"]["joints"]
        opts = MoveOptions(max_vel_degs_per_sec=10.0, max_acc_degs_per_sec2=20.0)

        async def joints():
            return list((await self.arm.get_joint_positions()).values)

        async def step(j, label):
            for attempt in range(3):
                try:
                    await self.arm.move_through_joint_positions([JointPositions(values=j)], options=opts)
                    break
                except Exception as e:  # noqa: BLE001
                    if arm_halted(e) or "anceled" not in str(e) or attempt == 2:
                        raise
                    now = await joints()
                    if max(abs(a - b) for a, b in zip(now, j)) < 0.5:
                        break                                   # it got there; only the reply was cancelled
                    log(f"  {label}: cancelled partway, resuming")
            p = await self.where()
            log(f"  {label}: gripper z {p.z:.0f}")
            if p.z < config.TO_HIGH_ZMIN:
                raise MoveError(f"stopped: the gripper got low (z {p.z:.0f}) on the way to 'high'", holding=False)

        j = await joints()
        p = await self.where()
        if p.z < config.TO_HIGH_LIFT_BELOW_Z and max(abs(a - b) for a, b in zip(j, target)) > 5.0:
            j[1] -= config.TO_HIGH_LIFT_DEG
            j[2] -= config.TO_HIGH_LIFT_DEG
            await step(list(j), "lean back first")
        for i in config.TO_HIGH_ORDER:
            while abs(j[i] - target[i]) > 0.3:
                d = max(-config.TO_HIGH_STEP_DEG, min(config.TO_HIGH_STEP_DEG, target[i] - j[i]))
                j[i] += d
                await step(list(j), f"joint {i + 1} -> {j[i]:.1f}")
        return True

    async def tissue(self, box_xy, top_h, others=None, hold_s=3.0):
        """Pinch the tissue sticking out of a pop-up box seen at box_xy (calibrated), top_h mm above the table,
        pull one tissue straight up, carry it to the taught 'handoff' pose (planned around the other seen
        objects), hold, let go, then to 'high' (joint by joint) so the drawer routines can follow. Top-down Cartesian moves: the box is where the camera saw it."""
        import taught
        data = taught.load()
        if "handoff" not in data:
            raise KeyError("teach 'handoff' first with `python app.py joints handoff`")
        x, y = box_xy
        grip_z = config.TABLE_Z + top_h - config.TISSUE_GRIP_BELOW_TOP_MM
        above = config.TABLE_Z + top_h + config.HOVER
        others = {o: p for o, p in (others or {}).items() if p is not None}
        avoid = world_state(others) if others else None
        await self.gripper.open()
        try:
            await self.move_to(x, y, above, avoid=avoid)                            # above the tissue
        except Exception as e:  # noqa: BLE001
            if arm_halted(e):
                raise
            raise MoveError("no safe path to the tissues", holding=False) from e
        await self.move_to(x, y, grip_z, straight=True)                             # down into the puff
        await self.gripper.grab()          # a tissue is too thin for the "holding" flag: don't trust it here
        await self.move_to(x, y, grip_z + config.TISSUE_PULL_MM, straight=True)    # pull one out, straight up
        h = data["handoff"]["pose"]
        try:
            await self.move_to(h["x"], h["y"], h["z"], theta=h["theta"],
                               o=(h["o_x"], h["o_y"], h["o_z"]), avoid=avoid)       # to the patient's hand
        except Exception as e:  # noqa: BLE001
            if arm_halted(e):
                raise
            print(f"  hand-over refused ({_why(e)}); keeping the tissue up here")
            raise MoveError("no safe path to your hand; I'm holding the tissue up high", holding=True) from e
        print("  handing over")             # the dashboard says "Here's your tissue" on this line
        await asyncio.sleep(hold_s)
        await self.gripper.open()
        await self.back_from_handoff(data)   # end at 'high' so `medicine` can follow directly (7:15 PM)

    async def back_from_handoff(self, data):
        """From the hand-over spot to 'high'. If the planner left the arm in the taught 'handoff' joint configuration,
        take the medicine's own way back (one joint move through pill_above, proven every medicine run); else the
        careful joint-by-joint to_high. Sat 11:45: to_high after the tissue took ~30+ s for a short distance."""
        from viam.proto.component.arm import JointPositions, MoveOptions
        now = list((await self.arm.get_joint_positions()).values)
        gap = max(abs(a - b) for a, b in zip(now, data["handoff"]["joints"]))
        if gap > config.HANDOFF_JOINT_MATCH_DEG or "pill_above" not in data or "high" not in data:
            print(f"  not in the taught handoff configuration ({gap:.0f} deg off): careful way back")
            await self.to_high()
            return
        opts = MoveOptions(max_vel_degs_per_sec=config.DRAWER_MAX_VEL_DEGS,
                           max_acc_degs_per_sec2=config.DRAWER_MAX_ACC_DEGS)
        t0 = time.time()
        await retry("joint move", lambda: self.arm.move_through_joint_positions(
            [JointPositions(values=data[n]["joints"]) for n in ("pill_above", "high")], options=opts),
            delay=self.retry_delay)
        print(f"  pill_above -> high: {time.time() - t0:.1f} s")

    async def pills(self, hold_s=3.0):
        """Drawer open, arm at 'high': take the medicine out of the drawer and hand it to the patient."""
        import taught
        data = taught.load()
        await self.replay(data, taught.pills_sequence(data, hold_s))

    async def cap(self):
        """Arm at 'high': unscrew a bottle cap (a person holds the bottle) by CAP_TWISTS re-grip twists of
        CAP_TWIST_DEG each, then lift the cap off and keep holding it above the bottle (Sat 1:35 PM)."""
        import taught
        data = taught.load()
        await self.replay(data, taught.cap_sequence(data, config.CAP_TWISTS, config.CAP_TWIST_DEG))

    async def drop_cap(self, xy, rim_h, drop_z=None):
        """Holding the cap above the bottle: carry it at the current height to above the cup at xy (calibrated
        slot), straight down to CAP_DROP_ABOVE_RIM_MM over the rim (rim_h mm above the table), let go, back up.
        Same-height carry first so it never sweeps low past the bottle (Sat 2 PM)."""
        x, y = xy
        here = await self.where()
        if drop_z is None:
            drop_z = config.TABLE_Z + rim_h + config.CAP_DROP_ABOVE_RIM_MM
        travel = max(here.z, drop_z + config.CAP_DROP_CLEAR_MM)
        await self.move_to(x, y, travel)                                   # over the cup, pointing down
        await self.move_to(x, y, drop_z, straight=True)                    # down to just above the rim
        await self.gripper.open()
        print("  cap dropped")
        await self.move_to(x, y, travel, straight=True)                    # back up, clear of the cup

    async def pour(self):
        """Arm at 'high', bottle (cap off) on slot 2: pick it up, pour into the slot 1 cup, stand it back on slot 2.
        'high' -> bottle_approach is a ~95 deg jump on several joints, too big to replay blind across the table, so
        the PLANNER takes the arm there (around the table and walls), and the taught replay continues only if it
        arrived in the taught arm shape. Back to 'high' the careful way (to_high)."""
        import taught
        data = taught.load()
        steps = taught.pour_sequence(data, config.POUR_HOLD_S, config.BOTTLE_HOLD_POS)    # checks the poses first
        if "bottle_via" in data:
            # Sat 2:19 PM: the planner refused bottle_approach (forearm inside the configured wall-front box, though
            # the real wall is clear). Taught waypoint instead: high -> bottle_via -> bottle_approach, and back.
            await self.replay(data, [("release", []), ("move", ["bottle_via", "bottle_approach"])] + steps
                              + [("move", ["bottle_via", "high"])])
            return
        await self.gripper.open()
        a = data["bottle_approach"]["pose"]
        await self.move_to(a["x"], a["y"], a["z"], theta=a["theta"], o=(a["o_x"], a["o_y"], a["o_z"]),
                           max_vel=config.POUR_APPROACH_VEL_DEGS)       # a big planned swing: not at full speed
        now = list((await self.arm.get_joint_positions()).values)
        gap = max(abs(x - y) for x, y in zip(now, data["bottle_approach"]["joints"]))
        if gap > config.HANDOFF_JOINT_MATCH_DEG:
            raise MoveError(f"reached the bottle in a different arm shape ({gap:.0f} deg off the taught one); "
                            f"not replaying from here", holding=False)
        await self.replay(data, steps, start="bottle_approach")
        await self.to_high()

    async def thermometer(self):
        """Arm at 'high': pick the IR thermometer off its taped spot, point it at the ball so its light shows,
        put it back. Mode "press" (THERM_MODE): hold it gently, click the button only at the ball.
        Mode "fast": the grip itself clicks the button (light goes off ~2 s later), so swing fast to the ball."""
        import taught
        data = taught.load()
        await self.replay(data, taught.therm_sequence(data, config.THERM_MODE, config.THERM_SHOW_S,
                                                      config.THERM_HOLD_POS, config.THERM_PRESS_POS,
                                                      config.THERM_CLICK_S))

    async def stop(self):
        """Emergency-style stop for Ctrl+C. Each call is best effort."""
        for part in (self.arm, self.gripper):
            try:
                await part.stop()
            except Exception:  # noqa: BLE001
                pass


class SimRobot:
    """No hardware. Knows where each object is (x, y) and pretends to move it.

    flaky:       chance a grab misses (tests retries)
    place_error: (dx, dy) mm added to every placement, or a function
                 (obj, attempt) -> (dx, dy); tests the "nudge" when a placement lands off
    on_move:     hook called after each move (tests use it to simulate a person
                 typing, or moving things, at a checkpoint)
    """

    def __init__(self, layout, flaky=0.0, on_move=None, seed=None, move_seconds=0.0,
                 place_error=(0.0, 0.0)):
        # layout values may be spot names ("slot1") or (x, y)
        self.pos = {o: (tuple(config.ALL_SPOTS[p]) if isinstance(p, str) else p)
                    for o, p in layout.items()}
        self.flaky = flaky
        self.on_move = on_move
        self.place_error = place_error
        self.moves = []             # (obj, src, dst, succeeded), for tests
        self.rng = random.Random(seed)
        self.overview_visits = 0
        self.move_seconds = move_seconds   # >0 makes timing feel like a real arm
        self.lifts = []

    @classmethod
    async def connect(cls, layout, flaky=0.0, seed=None, move_seconds=0.0):
        return cls(layout, flaky, seed=seed, move_seconds=move_seconds)

    def seen(self):
        return {o: self.pos.get(o) for o in config.OBJECTS}

    def spot(self, obj):
        """Name of the taught spot the object sits on (for readable tests), else None."""
        p = self.pos.get(obj)
        for name, xy in config.ALL_SPOTS.items():
            if p is not None and math.dist(p, xy) < 1.0:
                return name
        return None

    async def close(self):
        pass

    async def go_overview(self):
        self.overview_visits += 1

    async def where(self):
        return "sim: no real pose"

    async def move_object(self, obj, src, dst, lift=0.0, others=None):
        await asyncio.sleep(self.move_seconds)
        here = self.pos.get(obj)
        ok = here is not None and math.dist(here, src) < 1.0 and self.rng.random() >= self.flaky
        if ok:
            err = self.place_error(obj, len(self.moves)) if callable(self.place_error) else self.place_error
            self.pos[obj] = (dst[0] + err[0], dst[1] + err[1])
            self.lifts.append(lift)
        self.moves.append((obj, tuple(src), tuple(dst), ok))
        if self.on_move:
            self.on_move(self, obj, src, dst)
        return ok

    async def drawer(self, action="open"):
        self.drawer_moves = getattr(self, "drawer_moves", []) + [action]
        return 0.0

    async def pills(self, hold_s=3.0):
        self.drawer_moves = getattr(self, "drawer_moves", []) + ["pills"]

    async def tissue(self, box_xy, top_h, others=None, hold_s=3.0):
        self.drawer_moves = getattr(self, "drawer_moves", []) + ["tissue"]

    async def thermometer(self):
        self.drawer_moves = getattr(self, "drawer_moves", []) + ["thermometer"]

    async def cap(self):
        self.drawer_moves = getattr(self, "drawer_moves", []) + ["cap"]

    async def drop_cap(self, xy, rim_h, drop_z=None):
        self.drawer_moves = getattr(self, "drawer_moves", []) + ["drop_cap"]

    async def pour(self):
        self.drawer_moves = getattr(self, "drawer_moves", []) + ["pour"]

    async def to_high(self, log=print):
        self.drawer_moves = getattr(self, "drawer_moves", []) + ["to_high"]
        return True

    async def stop(self):
        pass
