"""RealRobot with fake Viam parts: checks the exact motion sequence the real arm would get.

These use the real Viam proto types (Pose, PoseInFrame, Constraints), so a
mistake in how we build motion requests shows up here, not on the robot.
"""
import pytest

import config
from conftest import run
from robot import RealRobot, SimRobot


class FakeMotion:
    def __init__(self, fail_straight=False, blips=0):
        self.calls, self.fail_straight, self.blips = [], fail_straight, blips

    async def move(self, component_name, destination, constraints=None, extra=None, timeout=None, **kw):
        if self.blips > 0:
            self.blips -= 1
            raise ConnectionError("blip")
        straight = constraints is not None and len(constraints.linear_constraint) > 0
        if straight and self.fail_straight:
            raise RuntimeError("no straight path")
        p = destination.pose
        self.calls.append({"component": component_name, "frame": destination.reference_frame,
                           "xyz": (p.x, p.y, p.z), "o": (p.o_x, p.o_y, p.o_z), "straight": straight,
                           "constraints": constraints, "world_state": kw.get("world_state")})
        return True

    async def get_pose(self, component_name, destination_frame, **kw):
        from viam.proto.common import Pose, PoseInFrame
        assert component_name == config.GRIPPER and destination_frame == "world"
        return PoseInFrame(reference_frame="world", pose=Pose(x=1, y=2, z=3))


class FakeGripper:
    def __init__(self, grabs=True, log=None):
        self.grabs, self.log = grabs, log if log is not None else []

    async def open(self, **kw):
        self.log.append("open")

    async def grab(self, **kw):
        self.log.append("grab")
        return self.grabs

    async def stop(self, **kw):
        self.log.append("stop")


class FakeArm:
    def __init__(self, fail=False):
        self.stopped, self.fail = False, fail

    async def stop(self, **kw):
        if self.fail:
            raise RuntimeError("arm offline")
        self.stopped = True


def make(grabs=True, **motion_kw):
    log = []
    motion = FakeMotion(**motion_kw)
    gripper = FakeGripper(grabs, log)
    orig = motion.move

    async def move(*a, **kw):
        r = await orig(*a, **kw)
        log.append(("move",) + motion.calls[-1]["xyz"])
        return r
    motion.move = move
    robot = RealRobot(machine=None, arm=FakeArm(), gripper=gripper, motion=motion, camera=None)
    robot.retry_delay = 0
    return robot, motion, gripper, log


SRC, DST = (452.5, -147.0), (300.0, -300.0)    # anywhere, not just taught spots


def test_move_object_full_sequence():
    robot, motion, _, log = make()
    assert run(robot.move_object("red_cube", SRC, DST)) is True
    sx, sy = SRC
    dx, dy = DST
    g = config.TABLE_Z + config.OBJECTS["red_cube"]["grasp_height"]
    p = g + config.PLACE_DROP_MM
    h = max(g, p) + config.HOVER
    assert log == [
        "open",
        ("move", sx, sy, h),        # above the object
        ("move", sx, sy, g),        # straight down
        "grab",
        ("move", sx, sy, h),        # straight up
        ("move", dx, dy, h),        # over destination
        ("move", dx, dy, p),        # down, released just above resting height
        "open",
        ("move", dx, dy, h),        # back up
    ]


def test_stacking_places_higher_and_travels_higher():
    robot, motion, _, log = make()
    lift = config.OBJECTS["blue_cube"]["size"][2]
    run(robot.move_object("red_cube", SRC, DST, lift))
    g = config.TABLE_Z + config.OBJECTS["red_cube"]["grasp_height"]
    p = g + lift + config.PLACE_DROP_MM
    assert ("move", DST[0], DST[1], p) in log
    assert log[1][3] == p + config.HOVER          # carried high enough to clear the lower cube


def test_grasp_height_comes_from_the_catalog(monkeypatch):
    monkeypatch.setattr(config, "TABLE_Z", -23.0)    # e.g. the winner's table top
    robot, motion, _, log = make()
    run(robot.move_object("cup", SRC, DST))
    assert log[2] == ("move", SRC[0], SRC[1], -23.0 + config.OBJECTS["cup"]["grasp_height"])


def test_every_move_targets_the_gripper_in_world_pointing_down():
    robot, motion, *_ = make()
    run(robot.move_object("marker", SRC, DST))
    for c in motion.calls:
        assert c["component"] == config.GRIPPER
        assert c["frame"] == "world"
        assert c["o"] == (0, 0, -1)


def test_only_vertical_moves_are_straight_line(monkeypatch):
    monkeypatch.setattr(config, "STRAIGHT_LINE", True)
    robot, motion, *_ = make()
    run(robot.move_object("red_cube", SRC, DST))
    assert [c["straight"] for c in motion.calls] == [False, True, True, False, True, True]
    lc = motion.calls[1]["constraints"].linear_constraint[0]
    assert lc.line_tolerance_mm == config.LINE_TOLERANCE_MM


def test_straight_line_can_be_turned_off(monkeypatch):
    monkeypatch.setattr(config, "STRAIGHT_LINE", False)
    robot, motion, *_ = make()
    run(robot.move_object("red_cube", SRC, DST))
    assert not any(c["straight"] for c in motion.calls)


def test_straight_line_refused_falls_back_to_normal_move(monkeypatch):
    monkeypatch.setattr(config, "STRAIGHT_LINE", True)
    robot, motion, *_ = make(fail_straight=True)
    assert run(robot.move_object("red_cube", SRC, DST)) is True
    assert len(motion.calls) == 6 and not any(c["straight"] for c in motion.calls)


def test_missed_grab_carries_nothing_and_opens_gripper(monkeypatch):
    monkeypatch.setattr(config, "TRUST_GRAB_SENSOR", True)
    robot, motion, gripper, log = make(grabs=False)
    assert run(robot.move_object("red_cube", SRC, DST)) is False
    assert not any(c["xyz"][:2] == DST for c in motion.calls)   # never went to the destination
    assert log[-1] == "open"


def test_untrusted_grab_sensor_is_ignored(monkeypatch):
    monkeypatch.setattr(config, "TRUST_GRAB_SENSOR", False)
    robot, motion, *_ = make(grabs=False)
    assert run(robot.move_object("red_cube", SRC, DST)) is True
    assert len(motion.calls) == 6


def test_connection_blip_is_retried():
    robot, motion, *_ = make(blips=1)
    robot.retry_delay = 0
    run(robot.go_overview())
    assert motion.calls[-1]["xyz"] == config.OVERVIEW


def test_persistent_failure_is_raised_not_swallowed():
    robot, motion, *_ = make(blips=99)
    with pytest.raises(ConnectionError):
        run(robot.go_overview())


def test_unknown_object_is_an_error_before_moving():
    robot, motion, *_ = make()
    with pytest.raises(KeyError):
        run(robot.move_object("laptop", SRC, DST))
    assert motion.calls == []


def test_where_returns_world_pose():
    robot, *_ = make()
    p = run(robot.where())
    assert (p.x, p.y, p.z) == (1, 2, 3)


def test_stop_stops_arm_and_gripper_even_if_one_fails():
    robot, _, gripper, log = make()
    run(robot.stop())
    assert robot.arm.stopped and "stop" in log
    robot.arm = FakeArm(fail=True)
    run(robot.stop())                       # must not raise
    assert log.count("stop") == 2


# ---------------- SimRobot itself ----------------

def test_sim_accepts_spot_names_or_positions():
    r = SimRobot({"red_cube": "slot1", "cup": (420.0, 200.0)})
    assert r.pos["red_cube"] == tuple(config.SLOTS["slot1"]) and r.pos["cup"] == (420.0, 200.0)
    assert r.spot("red_cube") == "slot1" and r.spot("cup") is None


def test_sim_refuses_to_move_object_from_the_wrong_place():
    r = SimRobot({"red_cube": "slot1"})
    assert run(r.move_object("red_cube", config.SLOTS["slot2"], DST)) is False
    assert r.spot("red_cube") == "slot1"


def test_sim_place_error_is_applied():
    r = SimRobot({"red_cube": "slot1"}, place_error=(5.0, -2.0))
    run(r.move_object("red_cube", config.SLOTS["slot1"], DST))
    assert r.pos["red_cube"] == (DST[0] + 5.0, DST[1] - 2.0)


def test_sim_flaky_is_reproducible_with_seed():
    a = SimRobot({"red_cube": "slot1"}, flaky=0.5, seed=7)
    b = SimRobot({"red_cube": "slot1"}, flaky=0.5, seed=7)
    ra = [run(a.move_object("red_cube", a.pos["red_cube"], DST)) for _ in range(5)]
    rb = [run(b.move_object("red_cube", b.pos["red_cube"], DST)) for _ in range(5)]
    assert ra == rb


def test_down_pose_uses_taught_wrist_angle(monkeypatch):
    """Hardware bug Fri 11:40: theta=0 made every IK solution collide wrist_link with the camera."""
    from robot import down_pose
    monkeypatch.setattr(config, "TOOL_THETA", 145.0)
    p = down_pose(353, 406, 499)
    assert (p.o_x, p.o_y, p.o_z, p.theta) == (0, 0, -1, 145.0)


def test_overview_uses_its_own_wrist_angle(monkeypatch):
    """OVERVIEW turns the wrist so the image lines up with the table; picks keep TOOL_THETA."""
    calls = []

    class FakeMotion:
        async def move(self, component_name, destination, constraints, extra, timeout):
            calls.append(destination.pose.theta)

    monkeypatch.setattr(config, "TOOL_THETA", 145.0)
    monkeypatch.setattr(config, "OVERVIEW_THETA", 178.0)
    r = RealRobot(None, None, None, FakeMotion(), None)
    run(r.go_overview())
    run(r.move_to(400, 450, 150))
    assert calls == [178.0, 145.0]


def test_dial_registers_vision_before_connecting():
    """Hardware bug Fri 1:10 PM: the SDK skips resource types not imported before connecting, so a
    READY vision service raised ResourceNotFoundError. dial() must import vision first."""
    import inspect
    src = inspect.getsource(RealRobot.dial)
    assert src.index("viam.services.vision") < src.index("RobotClient.at_address")


def test_overview_lifts_straight_up_first_when_the_arm_is_low(monkeypatch):
    """Fri 2:16 PM: the arm was left touching the table at slot3; going to the overview from there
    must not plan a path across the objects. Lift straight up first, then go."""
    import types
    moves = []

    class FakeMotion:
        async def get_pose(self, component_name, destination_frame):
            return types.SimpleNamespace(pose=types.SimpleNamespace(x=369, y=-14, z=58))

        async def move(self, component_name, destination, constraints, extra, timeout):
            p = destination.pose
            moves.append(((p.x, p.y, p.z), constraints is not None))

    monkeypatch.setattr(config, "TABLE_Z", 63.0)
    monkeypatch.setattr(config, "HOVER", 120.0)
    run(RealRobot(None, None, None, FakeMotion(), None).go_overview())
    assert moves[0] == ((369, -14, 58 + 150), True)           # straight up, same x/y
    assert moves[-1][0] == tuple(config.OVERVIEW)


def test_carry_goes_above_and_around_the_other_objects():
    """EXP-005 pick 2 (Fri 2:53): carrying tray -> left, the hanging wire knocked over a 14 cm toy and a
    cylinder. Carry moves now give the planner every other object as a box, and fly above the tallest one."""
    robot, motion, _, log = make()
    toy = config.UNKNOWN_OBSTACLE_SIZE[2]                       # no measurement -> assumed size
    run(robot.move_object("red_cube", SRC, DST, others={"red_cube": SRC, "mystery_toy": (369.0, -14.0),
                                                       "blue_cube": (277.0, 64.0), "gone": None}))
    grip = config.OBJECTS["red_cube"]["grasp_height"]
    h = config.TABLE_Z + toy + grip + config.CARRY_CLEARANCE_MM
    assert log[1] == ("move", SRC[0], SRC[1], h) and log[5] == ("move", DST[0], DST[1], h)
    carry = [motion.calls[0], motion.calls[3]]                  # above the source, over the destination
    for c in carry:
        geoms = c["world_state"].obstacles[0].geometries
        assert c["world_state"].obstacles[0].reference_frame == "world"
        assert sorted(g.label for g in geoms) == ["blue_cube", "mystery_toy"]   # not itself, not unseen
        cube = [g for g in geoms if g.label == "blue_cube"][0]
        assert cube.box.dims_mm.x == 50 + 2 * config.OBSTACLE_MARGIN_MM
        assert cube.center.z - cube.box.dims_mm.z / 2 == pytest.approx(config.TABLE_TOP_WORLD_Z)
    # Vertical moves keep no obstacles: the destination's neighbors are already checked by places.
    assert all(c["world_state"] is None for i, c in enumerate(motion.calls) if i not in (0, 3))


def test_carry_height_never_drops_below_the_normal_hover():
    robot, motion, _, log = make()
    run(robot.move_object("red_cube", SRC, DST, others={"flat": (0.0, 0.0)}))
    g = config.TABLE_Z + config.OBJECTS["red_cube"]["grasp_height"]
    assert log[1][3] >= g + config.PLACE_DROP_MM + config.HOVER


def test_no_other_objects_means_no_obstacles():
    robot, motion, *_ = make()
    run(robot.move_object("red_cube", SRC, DST))
    assert all(c["world_state"] is None for c in motion.calls)


class WallMotion(FakeMotion):
    """Refuses any pose above `max_z` within 60 mm of `near` (like cam_origin vs wall-front, Fri 3:39)."""
    def __init__(self, near, max_z, everywhere=False):
        super().__init__()
        self.near, self.max_z, self.everywhere = near, max_z, everywhere

    async def move(self, component_name, destination, constraints=None, **kw):
        p = destination.pose
        close = abs(p.x - self.near[0]) < 60 and abs(p.y - self.near[1]) < 60
        if close and (p.z > self.max_z or self.everywhere) and p.z > config.TABLE_Z + 100:
            raise RuntimeError("all IK solutions failed constraints: cam_origin vs wall-front")
        return await super().move(component_name, destination, constraints, **kw)


def make_wall(near, max_z, everywhere=False):
    robot, motion, gripper, log = make()
    wall = WallMotion(near, max_z, everywhere)
    robot.motion, robot.gripper = wall, gripper
    robot.retry_delay = 0
    return robot, wall, gripper


TALL = {"mystery_toy": (369.0, -14.0)}       # unknown size -> 150 mm tall -> high carry


def test_high_carry_out_of_reach_falls_back_to_the_normal_height():
    """EXP-006 req 2 (Fri 3:39): 'over the destination' at the high carry z hit wall-front and crashed."""
    robot, wall, gripper = make_wall(DST, max_z=config.TABLE_Z + 200)
    assert run(robot.move_object("red_cube", SRC, DST, others=TALL)) is True
    g = config.TABLE_Z + config.OBJECTS["red_cube"]["grasp_height"]
    low = g + config.PLACE_DROP_MM + config.HOVER
    at_dst = [c for c in wall.calls if c["xyz"][:2] == DST]
    assert at_dst[0]["xyz"][2] == low and at_dst[0]["world_state"] is not None   # still around obstacles


def test_unreachable_destination_puts_the_object_back_instead_of_crashing():
    robot, wall, gripper = make_wall(DST, max_z=0, everywhere=True)
    with pytest.raises(Exception) as e:
        run(robot.move_object("red_cube", SRC, DST, others=TALL))
    assert type(e.value).__name__ == "MoveError" and e.value.holding is False
    g = config.TABLE_Z + config.OBJECTS["red_cube"]["grasp_height"]
    assert ("move", SRC[0], SRC[1], g + config.PLACE_DROP_MM) in [("move",) + c["xyz"] for c in wall.calls]
    assert gripper.log[-1] == "open"                                  # let go of it, back where it was


def test_planner_refusal_is_not_retried():
    robot, wall, _ = make_wall(DST, max_z=0, everywhere=True)
    tries = []
    real = wall.move

    async def counting(component_name, destination, constraints=None, **kw):
        tries.append((destination.pose.x, destination.pose.y))
        return await real(component_name, destination, constraints, **kw)
    wall.move = counting
    with pytest.raises(Exception):
        run(robot.move_object("red_cube", SRC, DST))          # no others: one carry height only
    assert tries.count(DST) == 1                                 # refused once, not asked 3 times


class EStopMotion(FakeMotion):
    async def move(self, component_name, destination, constraints=None, **kw):
        self.calls.append({"xyz": (destination.pose.x, destination.pose.y, destination.pose.z)})
        raise RuntimeError("rpc error: code = Unknown desc = xArm: Emergency Stop Button Pushed In; ")


def test_estop_is_never_retried_or_replanned(monkeypatch):
    """EXP-007 try 1 (Fri 4:23): with the E-stop in, a straight lift fell back to a free move and was retried twice."""
    monkeypatch.setattr(config, "STRAIGHT_LINE", True)
    robot, _, gripper, _ = make()
    robot.motion = EStopMotion()
    with pytest.raises(RuntimeError, match="Emergency Stop"):
        run(robot.move_to(100.0, 200.0, 300.0, straight=True))
    assert len(robot.motion.calls) == 1


class PoseAt(FakeMotion):
    def __init__(self, pose):
        super().__init__()
        self.pose = pose

    async def get_pose(self, component_name, destination_frame, **kw):
        from viam.proto.common import PoseInFrame
        return PoseInFrame(reference_frame="world", pose=self.pose)


def test_overview_from_a_low_tilted_pose_leans_back_in_joints_first():
    """Fri 4:59 PM a 'pointing down' lift from the tilted drawer pose swung the arm low; 5:06 the planner refused tilted
    straight lifts. 7:02 the joint lean-back worked: shoulder and elbow back, then the normal move to the overview."""
    from viam.proto.common import Pose
    robot, _, gripper, _ = make()
    robot.motion = PoseAt(Pose(x=65, y=446, z=195, o_x=0.44, o_y=0.02, o_z=-0.90, theta=173))
    robot.arm = JointArm([89.9, 1.1, -50.6, -33.6, 53.1, 101.7])
    run(robot.go_overview())
    assert robot.arm.moves[0] == pytest.approx([89.9, 1.1 - config.TO_HIGH_LIFT_DEG, -50.6 - config.TO_HIGH_LIFT_DEG,
                                                -33.6, 53.1, 101.7])
    assert robot.motion.calls[-1]["xyz"] == tuple(config.OVERVIEW)


def test_overview_from_low_tilted_stops_if_the_lean_back_fails():
    from viam.proto.common import Pose
    from robot import MoveError
    robot, _, _, _ = make()
    m = PoseAt(Pose(x=126.6, y=427.9, z=82.8, o_x=0.578, o_y=0.122, o_z=-0.807, theta=-178.4))
    robot.motion = m

    class Broken(JointArm):
        async def move_through_joint_positions(self, positions, options=None, **kw):
            raise RuntimeError("all IK solutions failed constraints")
    robot.arm = Broken([0.0] * 6)
    with pytest.raises(MoveError):
        run(robot.go_overview())                     # never a free move from down there
    assert m.calls == []


def test_lift_from_straight_down_is_unchanged():
    from viam.proto.common import Pose
    robot, _, _, _ = make()
    robot.motion = PoseAt(Pose(x=200, y=300, z=70, o_x=0, o_y=0, o_z=-1, theta=145))
    run(robot.go_overview())
    assert robot.motion.calls[0]["o"] == (0, 0, -1)



def test_tissue_pinches_the_puff_pulls_straight_up_and_hands_over(monkeypatch):
    """Fri 7:02 PM: one tissue from a pop-up box seen by the camera, to the taught hand-over spot."""
    import asyncio, taught
    async def no_sleep(*a, **k):
        return None
    monkeypatch.setattr(asyncio, "sleep", no_sleep)
    taught.save("handoff", [0.0] * 6, {"x": -245.0, "y": 359.0, "z": 241.0, "o_x": 0.4, "o_y": 0.0, "o_z": -0.9, "theta": 170.0})
    robot, motion, gripper, log = make()
    robot.back_from_handoff = lambda data: asyncio.sleep(0)
    run(robot.tissue((341.0, 283.0), 173.0, others={"drawer_box": (297.0, 482.0), "gone": None}))
    g = config.TABLE_Z + 173.0 - config.TISSUE_GRIP_BELOW_TOP_MM
    xyz = [c["xyz"] for c in motion.calls]
    assert xyz[0] == (341.0, 283.0, config.TABLE_Z + 173.0 + config.HOVER)
    assert xyz[1] == (341.0, 283.0, g) and motion.calls[1]["straight"]
    assert xyz[2] == (341.0, 283.0, g + config.TISSUE_PULL_MM) and motion.calls[2]["straight"]
    assert xyz[3] == (-245.0, 359.0, 241.0) and motion.calls[3]["o"] == pytest.approx((0.4, 0.0, -0.9))
    for i in (0, 3):                                              # carry moves avoid the drawer box
        assert [gm.label for gm in motion.calls[i]["world_state"].obstacles[0].geometries] == ["drawer_box"]
    assert [x for x in gripper.log if isinstance(x, str)] == ["open", "grab", "open"]


def test_tissue_never_dips_below_the_puff():
    """Fingers must stay above the cardboard: at most TISSUE_GRIP_BELOW_TOP_MM into the measured top."""
    assert 0 < config.TISSUE_GRIP_BELOW_TOP_MM <= 25



class JointArm:
    """Fake arm that tracks joints; fails a move with 'context canceled' when told to."""
    def __init__(self, joints, cancel_at=(), cancel_after_move=False):
        self.j, self.moves, self.cancel_at, self.after, self.n = list(joints), [], set(cancel_at), cancel_after_move, 0

    async def get_joint_positions(self, **kw):
        import types
        return types.SimpleNamespace(values=list(self.j))

    async def move_through_joint_positions(self, positions, options=None, **kw):
        self.n += 1
        target = list(positions[-1].values)
        if self.n in self.cancel_at:
            if self.after:
                self.j = target
            else:
                self.j = [a + (b - a) / 2 for a, b in zip(self.j, target)]
            raise RuntimeError("(<Status.CANCELLED: 1>, 'context canceled', None)")
        self.j = target
        self.moves.append(target)


def high_robot(start, z=400.0, **arm_kw):
    import taught
    from viam.proto.common import Pose
    taught.save("high", [89.9, 1.1, -50.6, -33.6, 53.1, 101.7], {"x": 65, "y": 446, "z": 195})
    robot, _, _, _ = make()
    robot.motion = PoseAt(Pose(x=0, y=0, z=z, o_x=0, o_y=0, o_z=-1, theta=0))
    robot.arm = JointArm(start, **arm_kw)
    return robot


def test_to_high_goes_one_joint_at_a_time_in_order():
    """Fri 7:15: what we did by hand at 6:43 / 7:13, as one command."""
    start = [53.8, -12.6, -100.4, 0.0, 113.1, 51.8]          # the overview
    robot = high_robot(start)
    run(robot.to_high(log=lambda *_: None))
    arm = robot.arm
    assert arm.j == pytest.approx([89.9, 1.1, -50.6, -33.6, 53.1, 101.7])
    order = []
    prev = start
    for m in arm.moves:
        changed = [i for i in range(6) if abs(m[i] - prev[i]) > 1e-9]
        assert len(changed) == 1                               # one joint per move
        assert abs(m[changed[0]] - prev[changed[0]]) <= config.TO_HIGH_STEP_DEG + 1e-9
        if not order or order[-1] != changed[0]:
            order.append(changed[0])
        prev = m
    assert order == list(config.TO_HIGH_ORDER)                 # shoulder (it lowers most) last


def test_to_high_resumes_after_a_cancelled_step():
    robot = high_robot([53.8, -12.6, -100.4, 0.0, 113.1, 51.8], cancel_at={2})
    run(robot.to_high(log=lambda *_: None))
    assert robot.arm.j == pytest.approx([89.9, 1.1, -50.6, -33.6, 53.1, 101.7])


def test_to_high_stops_if_the_gripper_gets_low():
    from robot import MoveError
    robot = high_robot([53.8, -12.6, -100.4, 0.0, 113.1, 51.8], z=150.0)
    with pytest.raises(MoveError, match="low"):
        run(robot.to_high(log=lambda *_: None))
    assert len(robot.arm.moves) == 1                           # stopped after the first step


def test_to_high_leans_back_first_when_starting_low():
    robot = high_robot([122.7, -16.7, -32.6, 44.8, 25.3, 101.7], z=241.0)   # the hand-over pose
    run(robot.to_high(log=lambda *_: None))
    first = robot.arm.moves[0]
    assert first[1] == pytest.approx(-16.7 - config.TO_HIGH_LIFT_DEG) and first[2] == pytest.approx(-32.6 - config.TO_HIGH_LIFT_DEG)
    assert first[0] == 122.7                                   # nothing else moves in the lift


def _taught_for_return():
    import taught
    for n, j in (("high", [90.0, 1, -51, -34, 53, 102]), ("pill_above", [66.0, -8, -36, 48, 25, 102]),
                 ("handoff", [123.0, -17, -33, 45, 25, 102])):
        taught.save(n, j, {"x": 0, "y": 0, "z": 300})
    return taught.load()


def test_after_the_tissue_the_arm_goes_back_the_medicines_quick_way():
    """Sat 11:45: to_high (joint by joint, 10 deg/s) after the tissue took ~30+ s; the medicine's own return
    from the same taught 'handoff' (one joint move through pill_above) is proven and much quicker."""
    data = _taught_for_return()
    robot, _, _, _ = make()
    robot.arm = JointArm([125.0, -15, -33, 45, 25, 102])            # planner landed within 2 deg of 'handoff'
    seen = []

    async def through(positions, options=None, **kw):
        seen.append(([list(p.values) for p in positions], options))
    robot.arm.move_through_joint_positions = through
    called = []
    robot.to_high = lambda *a, **k: called.append(1)
    run(robot.back_from_handoff(data))
    assert called == []
    assert seen[0][0] == [data["pill_above"]["joints"], data["high"]["joints"]]
    assert seen[0][1].max_vel_degs_per_sec == config.DRAWER_MAX_VEL_DEGS


def test_after_the_tissue_an_unexpected_arm_configuration_goes_the_careful_way():
    import asyncio
    data = _taught_for_return()
    robot, _, _, _ = make()
    robot.arm = JointArm([123.0, -17, -33, 45 + 180, 25, 102])      # same spot, wrist flipped: not the taught way
    called = []

    async def careful(*a, **k):
        called.append(1)
    robot.to_high = careful
    run(robot.back_from_handoff(data))
    assert called == [1] and robot.arm.moves == []
