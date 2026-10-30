"""care.py on the simulator: sidekick's routines with the person, the tissue box and the drawer knob."""
import os
import sys
import textwrap

import numpy as np
import pytest

import care
import knob
import person
import settings
import sim
from conftest import run


def setup(**kw):
    """(robot, rig, finder, taught poses, sidekick stand-in), starting at 'high' like sidekick's routines."""
    return sim.make_care(**kw)


def fingertips(T, tip=48.0):
    return T[:3, 3] + T[:3, :3] @ np.array([0.0, 0.0, tip])


async def remembered_drawer(rig, robot):
    await robot.go_overview()
    knob.remember(await rig.capture())
    robot.log.clear()


# ---- the person --------------------------------------------------------------------

def test_handover_point_is_in_front_of_the_face_toward_the_robot():
    hand = person.handover_point([700.0, 30.0, 480.0], [210.0, -40.0, 470.0], forward_mm=300, below_mm=250)
    d = np.array([-490.0, -70.0]) / np.hypot(490.0, 70.0)                 # forehead -> camera, flat
    assert hand == pytest.approx([700.0 + 300 * d[0], 30.0 + 300 * d[1], 230.0], abs=0.1)
    # the camera's height doesn't tilt it: the 300 mm is horizontal
    assert person.toward_robot([700.0, 30.0, 480.0], [210.0, 30.0, 900.0]) == pytest.approx([-1.0, 0.0, 0.0])


def test_dry_runs_move_nothing():
    robot, rig, finder, data, sk = setup(start="look_person")
    start = rig.world.T_world_gripper.copy()
    assert run(care.tissue_flow(robot, rig, finder, data, sk)) == "dry run"
    assert run(care.medicine_flow(robot, rig, finder, data, sk)) == "dry run"
    assert run(care.thermometer_flow(robot, rig, finder, data, sk))["result"] == "dry run"
    assert rig.world.moves == [] and robot.log == []
    assert np.allclose(rig.world.T_world_gripper, start)


def test_go_refuses_the_drawer_poses():
    robot, rig, finder, data, sk = setup()
    data["grip"] = data["pill_grip"]                      # a drawer pose: only sidekick's routine goes there
    with pytest.raises(care.Refused, match="reached only by sidekick's routines"):
        run(care.go(robot, sk, data, "grip", yes=True))
    assert robot.log == [] and rig.world.moves == []
    run(care.go(robot, sk, data, "look_person", yes=True))
    run(care.go(robot, sk, data, "overview", yes=True))   # from a spoke: back to 'high' the same way first
    assert robot.log == ["overview"]
    assert np.allclose(rig.world.T_world_gripper, sim.SIM_OVERVIEW)


def test_missing_look_pose_is_a_clear_stop():
    robot, rig, finder, data, sk = setup()
    del data["look_person"]
    with pytest.raises(care.Refused, match="python app.py joints look_person"):
        run(care.tissue_flow(robot, rig, finder, data, sk, execute=True, yes=True))
    assert robot.log == [] and rig.world.moves == []


# ---- the drawer knob ----------------------------------------------------------------

def test_knob_check_measures_how_far_the_drawer_moved():
    robot, rig, finder, data, sk = setup()
    run(robot.go_overview())
    saved = knob.remember(run(rig.capture()))
    moved, _ = knob.check(run(rig.capture()), saved)
    assert moved < 3.0
    robot2, rig2, *_ = setup(drawer_shift=(30.0, 0.0))
    run(robot2.go_overview())
    moved, _ = knob.check(run(rig2.capture()), saved)
    assert moved == pytest.approx(30.0, abs=4.0)


def test_knob_out_of_view_is_not_found():
    robot, rig, finder, data, sk = setup()
    run(robot.go_overview())
    saved = knob.remember(run(rig.capture()))
    run(care.go(robot, sk, data, "look_person", yes=True))      # looking at the person: no knob in view
    with pytest.raises(knob.KnobNotFound):
        knob.check(run(rig.capture()), saved)


def test_drawer_check_can_look_from_a_taught_pose():
    settings.KNOB_VIEW = "high"                     # if the knob is out of the overview picture
    robot, rig, finder, data, sk = setup(start="look_person")
    run(care.drawer_command(robot, rig, data, sk, "remember-drawer", execute=True, yes=True))
    assert np.allclose(rig.world.T_world_gripper, sim.pose_T(data["high"]["pose"]))
    assert run(care.drawer_check(rig)) < 3.0


def test_medicine_refuses_if_the_drawer_box_moved():
    robot, rig, finder, data, sk = setup()
    run(remembered_drawer(rig, robot))
    robot2, rig2, finder2, data2, sk2 = setup(drawer_shift=(0.0, -25.0))     # someone bumped the box
    with pytest.raises(care.Refused, match="mm from where its poses were taught"):
        run(care.medicine_flow(robot2, rig2, finder2, data2, sk2, execute=True, yes=True))
    assert ("drawer", "open") not in robot2.log
    assert robot2.log == ["overview"]          # it only went to look


def test_medicine_needs_the_drawer_remembered_before_moving():
    robot, rig, finder, data, sk = setup()
    with pytest.raises(knob.KnobNotFound, match="remember-drawer"):
        run(care.medicine_flow(robot, rig, finder, data, sk, execute=True, yes=True))
    assert robot.log == [] and rig.world.moves == []


# ---- the routines -------------------------------------------------------------------

def test_medicine_is_handed_over_in_front_of_the_person():
    robot, rig, finder, data, sk = setup()
    run(remembered_drawer(rig, robot))
    assert run(care.medicine_flow(robot, rig, finder, data, sk, execute=True, yes=True)) == "handed over"
    log = robot.log
    assert log.index("overview") < log.index(("drawer", "open")) < log.index(("drawer", "close"))
    assert ("joint", "pill_grip") in log and ("joint", "handoff") in log
    (name, where), = rig.world.released
    assert name == "pill_box"
    who_hand = expected_hand()
    assert np.linalg.norm(where - who_hand) < 15.0                        # let go in front of the person
    taught = fingertips(sim.pose_T(data["handoff"]["pose"]))
    assert np.linalg.norm(where - taught) > 100.0                         # not at the taught spot
    to_person = [m for m in rig.world.moves if m.get("max_vel")]
    assert len(to_person) == 1 and to_person[0]["obstacles"] == 2         # slowly, around head and body
    assert np.allclose(rig.world.T_world_gripper, sim.pose_T(data["high"]["pose"]))   # back at 'high'


def expected_hand():
    """The hand-over point as seen from the look pose of an identical world: ~300 mm in front of the
    face (the head's front is at x = 695) and 250 mm below the forehead."""
    robot, rig, finder, data, sk = setup(start="look_person")
    who = run(care.find_person(rig, finder))
    assert who.hand == pytest.approx([695.0 - 300.0, 30.0, who.fix.point_w[2] - 250.0], abs=8.0)
    return who.hand


def test_tissue_is_pulled_and_handed_over_in_front_of_the_person():
    robot, rig, finder, data, sk = setup()
    assert run(care.tissue_flow(robot, rig, finder, data, sk, execute=True, yes=True)) == "handed over"
    (name, where), = rig.world.released
    assert name == "tissue"
    who_hand = expected_hand()
    assert np.linalg.norm(where - who_hand) < 15.0
    assert robot.log[-1] == "to_high"
    # the pick used sidekick's heights: fingertips 15 mm under the measured top, then 250 mm straight up
    grab = [m for m in rig.world.moves if m["linear"]]
    assert fingertips(grab[0]["to"])[2] == pytest.approx(135.0 - 15.0, abs=1.0)
    assert grab[1]["to"][2, 3] - grab[0]["to"][2, 3] == pytest.approx(250.0)


def test_person_too_far_from_the_taught_spot_hands_over_there():
    settings.MAX_HANDOVER_SHIFT_MM = 50.0
    robot, rig, finder, data, sk = setup()
    assert run(care.tissue_flow(robot, rig, finder, data, sk, execute=True, yes=True)) == "handed over"
    (_, where), = rig.world.released
    assert np.linalg.norm(where - fingertips(sim.pose_T(data["handoff"]["pose"]))) < 1.0


def test_no_safe_path_to_the_person_hands_over_at_the_taught_spot(capsys):
    robot, rig, finder, data, sk = setup()
    run(remembered_drawer(rig, robot))
    real_move = robot.move_to

    async def planner_refuses_slow_moves(*a, max_vel=None, **kw):
        if max_vel:
            raise RuntimeError("all IK solutions failed constraints")
        return await real_move(*a, **kw)

    robot.move_to = planner_refuses_slow_moves
    assert run(care.medicine_flow(robot, rig, finder, data, sk, execute=True, yes=True)) == "handed over"
    (_, where), = rig.world.released
    assert np.linalg.norm(where - fingertips(sim.pose_T(data["handoff"]["pose"]))) < 1.0
    assert "handing over at the taught spot" in capsys.readouterr().out
    assert robot.log[-1] == ("joint", "high")          # drawer closed, arm at 'high'


def test_thermometer_measures_and_comes_back_the_way_it_went():
    robot, rig, finder, data, sk = setup()
    log = run(care.thermometer_flow(robot, rig, finder, data, sk, execute=True, yes=True))
    assert log["result"] == "measured"
    look = sim.pose_T(data["look_person"]["pose"])
    assert np.allclose(rig.world.moves[-1]["to"], look)                  # planned back to the look pose
    assert rig.world.moves[-1]["obstacles"] == 2
    assert np.allclose(rig.world.T_world_gripper, sim.pose_T(data["high"]["pose"]))


def test_pills_sequence_is_split_at_the_handoff():
    *_, data, sk = setup()
    to_handoff, back = care.pills_steps(sk, data, 3.0)
    assert to_handoff[-1] == ("move", ["handoff"]) and ("wait", [3.0]) not in to_handoff
    assert back == ["pill_above", "high"]


def test_cli_on_the_simulator(capsys):
    assert care.main(["remember-drawer", "--sim"]) == 1                  # not at the overview, no --execute
    assert "isn't at 'overview'" in capsys.readouterr().out
    assert care.main(["remember-drawer", "--sim", "--execute", "--yes"]) == 0
    assert care.main(["check-drawer", "--sim", "--execute", "--yes"]) == 0
    assert "OK." in capsys.readouterr().out
    assert care.main(["medicine", "--sim", "--execute", "--yes"]) == 0
    assert "medicine: handed over" in capsys.readouterr().out
    assert care.main(["look", "--sim"]) == 1                             # at 'high': no face in view
    assert "python care.py go look_person" in capsys.readouterr().out
    assert care.main(["look", "--sim", "--execute", "--yes"]) == 0
    assert "Hand-over point" in capsys.readouterr().out


# ---- loading sidekick ---------------------------------------------------------------

def test_sidekick_is_loaded_from_the_parent_folder_without_changing_it(tmp_path, monkeypatch):
    sk_dir = tmp_path / "handly-care"   # vision-upgrade lives inside Handly-Care (SIDEKICK_DIR = "..")
    sk_dir.mkdir()
    files = {
        "config.py": 'CAMERA = "cam"\nGRIPPER = "gripper"\nDRAWER_JOINTS_PATH = "drawer_joints.json"\n',
        "robot.py": "class MoveError(Exception):\n    pass\n",
        "taught.py": "def load():\n    return {}\n",
        "perception.py": "async def find(robot):\n    return {}\n",
        ".env": "VIAM_ADDRESS=test-machine.viam.cloud\n",
    }
    for name, text in files.items():
        (sk_dir / name).write_text(textwrap.dedent(text))
    before = {p.name: p.read_text() for p in sk_dir.iterdir()}
    monkeypatch.setattr(care, "HERE", str(sk_dir / "vision-upgrade"))
    monkeypatch.delenv("VIAM_ADDRESS", raising=False)
    for mod in ("config", "robot", "taught", "perception"):
        monkeypatch.delitem(sys.modules, mod, raising=False)
    monkeypatch.setattr(sys, "path", list(sys.path))
    try:
        sk = care.load_sidekick()
        assert sk.config.DRAWER_JOINTS_PATH == os.path.join(str(sk_dir), "drawer_joints.json")
        assert os.environ["VIAM_ADDRESS"] == "test-machine.viam.cloud"
        assert str(sk_dir) in sys.path
        assert {p.name: p.read_text() for p in sk_dir.iterdir() if p.suffix != ".pyc" and p.is_file()} == before
    finally:
        for mod in ("config", "robot", "taught", "perception"):
            sys.modules.pop(mod, None)
