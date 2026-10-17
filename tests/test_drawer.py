"""Drawer by joint-space teach-and-replay (EXP-007, Fri 5:10 PM: the planner refused the tilted straight lines)."""
import pytest

import config
import taught
from conftest import run
from robot import RealRobot

NAMES = ["high", "approach", "grip", "pull2", "pull1", "open", "retreat"]


def teach_all():
    for i, n in enumerate(NAMES):
        taught.save(n, [float(i)] * 6, {"x": 240.0 - 40 * (n.startswith("pull") or n == "open") * (1 + (n == "open")),
                                        "y": 420.0, "z": 73.0, "o_x": 0.6, "o_y": 0.0, "o_z": -0.8, "theta": -178.0})


class FakeArm:
    def __init__(self, at=None):
        self.moves, self.at = [], at

    async def get_joint_positions(self, **kw):
        import types
        return types.SimpleNamespace(values=self.at if self.at is not None else taught.load()["high"]["joints"])

    async def move_through_joint_positions(self, positions, options=None, **kw):
        self.moves.append(([list(p.values) for p in positions], options))


class FakeGripper:
    def __init__(self, log):
        self.log = log

    async def open(self, **kw):
        self.log.append("release")

    async def grab(self, **kw):
        self.log.append("grab")
        return True


def robot():
    log = []
    arm = FakeArm()
    r = RealRobot(machine=None, arm=arm, gripper=FakeGripper(log), motion=None, camera=None)
    r.retry_delay = 0
    return r, arm, log


def names_of(arm):
    by_joints = {tuple(v["joints"]): n for n, v in taught.load().items()}
    return [[by_joints[tuple(j)] for j in seg] for seg, _ in arm.moves]


def test_open_replays_the_taught_poses_in_order_gripping_only_for_the_pull(monkeypatch):
    monkeypatch.setattr(config, "DRAWER_EXTRA_OPEN_MM", 0.0)
    teach_all()
    r, arm, log = robot()
    run(r.drawer("open"))
    assert names_of(arm) == [["high", "approach"], ["grip"], ["pull1", "pull2", "open"], ["retreat", "high"]]
    assert log == ["release", "grab", "release"]


def test_close_runs_the_pull_backwards_and_stops_short_of_the_grip(monkeypatch):
    """Fri 5:35 PM: closing all the way to 'grip' pushed the drawer too far and nudged the box."""
    monkeypatch.setattr(config, "DRAWER_EXTRA_OPEN_MM", 0.0)
    teach_all()
    r, arm, log = robot()
    run(r.drawer("close"))
    segs = [[list(j) for j in seg] for seg, _ in arm.moves]
    data = taught.load()
    J = lambda n: data[n]["joints"]
    assert segs[0] == [J("high"), J("retreat")] and segs[1] == [J("open")]
    assert segs[2][:2] == [J("pull2"), J("pull1")]
    stop = segs[2][2]
    assert stop != J("grip")                                         # not all the way in
    g, p1 = J("grip")[0], J("pull1")[0]
    assert min(g, p1) < stop[0] < max(g, p1)                         # between grip and pull1
    assert segs[3] == [J("approach"), J("high")]


def test_close_stop_is_short_mm_before_grip():
    teach_all()
    d = taught.add_close_stop(taught.load(), 10.0)
    import math
    xyz = lambda n: tuple(d[n]["pose"][k] for k in ("x", "y", "z"))
    assert math.dist(xyz("close_stop"), xyz("grip")) == pytest.approx(10.0)


def test_every_drawer_move_carries_the_speed_cap():
    """Vaibhav, 4:55 PM: half speed for the drawer. The arm obeys MoveOptions itself (no planner involved)."""
    teach_all()
    r, arm, _ = robot()
    run(r.drawer("open"))
    for _, opts in arm.moves:
        assert opts.max_vel_degs_per_sec == config.DRAWER_MAX_VEL_DEGS
        assert opts.max_acc_degs_per_sec2 == config.DRAWER_MAX_ACC_DEGS


def test_refuses_to_move_until_every_pose_is_taught():
    taught.save("high", [0.0] * 6, {"x": 0, "y": 0, "z": 0})
    r, arm, log = robot()
    with pytest.raises(KeyError, match="approach"):
        run(r.drawer("open"))
    assert arm.moves == [] and log == []


def test_reteaching_a_name_replaces_it():
    taught.save("grip", [1.0] * 6, {"x": 1, "y": 2, "z": 3})
    taught.save("grip", [2.0] * 6, {"x": 1, "y": 2, "z": 3})
    assert taught.load()["grip"]["joints"] == [2.0] * 6


def test_refuses_a_big_blind_first_jump_to_high():
    """The move to 'high' is the one joint move nobody taught: from far away it could swing across the table."""
    teach_all()
    r, arm, log = robot()
    arm.at = [90.0] * 6
    from robot import MoveError
    with pytest.raises(MoveError, match="near 'high'"):
        run(r.drawer("open"))
    assert arm.moves == [] and log == []


def test_a_dropped_connection_mid_drawer_is_retried(monkeypatch):
    """EXP-007 close 2 (Fri 5:38): 'context canceled' after the first move left the arm waiting by the drawer."""
    monkeypatch.setattr(config, "DRAWER_EXTRA_OPEN_MM", 0.0)
    monkeypatch.setattr(config, "CANCEL_ACCEPT_DEG", 0.5)   # test poses are only 1 deg apart: the cancel is "far" here
    teach_all()
    r, arm, log = robot()
    real, state = arm.move_through_joint_positions, {"n": 0}

    async def flaky(positions, options=None, **kw):
        state["n"] += 1
        if state["n"] == 2:
            raise ConnectionError("(<Status.CANCELLED: 1>, 'context canceled', None)")
        return await real(positions, options, **kw)
    arm.move_through_joint_positions = flaky
    run(r.drawer("open"))
    assert names_of(arm) == [["high", "approach"], ["grip"], ["pull1", "pull2", "open"], ["retreat", "high"]]


def teach_pills():
    teach_all()
    for i, n in enumerate(["pill_above", "pill_grip", "handoff"]):
        taught.save(n, [10.0 + i] * 6, {"x": 100.0, "y": 450.0, "z": 150.0 - 50 * (n == "pill_grip")})


class Gripper2(FakeGripper):
    def __init__(self, log, holds=True):
        super().__init__(log)
        self.holds = holds

    async def grab(self, **kw):
        self.log.append("grab")
        return self.holds


def test_pills_takes_the_item_out_and_hands_it_over(monkeypatch):
    """Fri 5:58 PM: medicine (red cylinder for now) from a fixed spot in the open drawer to the patient's hand."""
    import asyncio
    monkeypatch.setattr(asyncio, "sleep", _no_sleep)
    teach_pills()
    r, arm, log = robot()
    run(r.pills(hold_s=3.0))
    assert names_of(arm) == [["pill_above"], ["pill_grip"], ["pill_above"], ["handoff"], ["pill_above", "high"]]
    assert log == ["release", "grab", "release"]


async def _no_sleep(*a, **k):
    return None


def test_pills_stops_and_says_so_when_the_grab_is_empty(monkeypatch):
    import asyncio
    monkeypatch.setattr(asyncio, "sleep", _no_sleep)
    monkeypatch.setattr(config, "TRUST_GRAB_SENSOR", True)
    teach_pills()
    r, arm, log = robot()
    r.gripper = Gripper2(log, holds=False)
    from robot import MoveError
    with pytest.raises(MoveError, match="missed"):
        run(r.pills())
    assert ["handoff"] not in names_of(arm)                        # never "hands over" nothing
    assert names_of(arm)[-1] == ["pill_above", "high"]


def test_pills_needs_its_poses_taught():
    teach_all()
    r, arm, log = robot()
    with pytest.raises(KeyError, match="pill_above"):
        run(r.pills())
    assert arm.moves == []


def test_open_far_pulls_extra_mm_past_open_along_the_last_segment(monkeypatch):
    """Fri 6:31 PM: open ~3 cm more than taught, without re-teaching."""
    import math
    monkeypatch.setattr(config, "DRAWER_EXTRA_OPEN_MM", 30.0)
    teach_all()
    r, arm, log = robot()
    run(r.drawer("open"))
    d = taught.add_open_far(taught.load(), 30.0)
    segs = [[list(j) for j in seg] for seg, _ in arm.moves]
    assert segs[2][-1] == d["open_far"]["joints"]                 # the pull ends past 'open'
    xyz = lambda n: tuple(d[n]["pose"][k] for k in ("x", "y", "z"))
    assert math.dist(xyz("open_far"), xyz("open")) == pytest.approx(30.0)
    assert segs[3][0] == taught.load()["retreat"]["joints"]       # then lets go and backs off as taught


def test_close_starts_from_open_far_when_extended(monkeypatch):
    monkeypatch.setattr(config, "DRAWER_EXTRA_OPEN_MM", 30.0)
    teach_all()
    r, arm, log = robot()
    run(r.drawer("close"))
    d = taught.add_open_far(taught.load(), 30.0)
    assert [list(j) for j in arm.moves[1][0]] == [d["open_far"]["joints"]]


def test_pills_lifts_straight_out_of_the_drawer_when_taught(monkeypatch):
    """Fri 6:37 PM: the item is gripped tilted; pill_lift raises it clear of the drawer walls before moving on."""
    import asyncio
    monkeypatch.setattr(asyncio, "sleep", _no_sleep)
    teach_pills()
    taught.save("pill_lift", [20.0] * 6, {"x": 100.0, "y": 450.0, "z": 200.0})
    r, arm, log = robot()
    run(r.pills())
    assert names_of(arm)[2] == ["pill_lift", "pill_above"]


def test_a_pull_canceled_just_short_of_the_end_counts_as_done(monkeypatch):
    """Fri night + Sat 12:21: the pull came back 'context canceled' 1.7 deg short of open_far three times in a row
    (the drawer's end stop), and the run died gripping the knob. Within CANCEL_ACCEPT_DEG: let go and carry on."""
    monkeypatch.setattr(config, "DRAWER_EXTRA_OPEN_MM", 0.0)
    teach_all()
    r, arm, log = robot()
    real = arm.move_through_joint_positions

    async def stalls(positions, options=None, **kw):
        target = list(positions[-1].values)
        if target == taught.load()["open"]["joints"]:
            arm.at = [v - 1.7 for v in target]                        # stopped 1.7 deg short of the end
            raise RuntimeError("(<Status.CANCELLED: 1>, 'context canceled', None)")
        arm.at = target
        return await real(positions, options, **kw)
    arm.move_through_joint_positions = stalls
    run(r.drawer("open"))
    assert names_of(arm)[-1] == ["retreat", "high"]                   # finished the routine
    assert log == ["release", "grab", "release"]                      # let go of the knob


def test_a_cancel_far_from_the_target_still_fails():
    teach_all()
    r, arm, log = robot()

    async def cancels(positions, options=None, **kw):
        raise RuntimeError("(<Status.CANCELLED: 1>, 'context canceled', None)")
    arm.move_through_joint_positions = cancels
    with pytest.raises(RuntimeError, match="canceled"):
        run(r.drawer("open"))
