"""Planner: keyword rules, relations, validation (the safety layer), ordering, and the AI path."""
import json
import types

import pytest

import config
import planner
from conftest import START, scene


def acts(plan):
    return {a["object"]: a["destination"] for a in plan["actions"]}


def rules(*requests, s=None):
    s = scene() if s is None else s
    return planner.validate(planner.plan_with_rules(list(requests), s), s)


# ---------------- keyword planner (the offline fallback) ----------------

def test_something_to_write_with_gives_marker():
    assert acts(rules("Give me something to write with")) == {"marker": "to_me"}


def test_zone_sorting_with_everything_else():
    assert acts(rules("Put the cup in the tray, everything else to the left")) == {
        "cup": "tray", "red_cube": "left", "blue_cube": "left", "marker": "left"}


def test_everything_skips_items_already_in_a_zone():
    s = scene(marker="to_me")
    assert "marker" not in acts(rules("everything to the left", s=s))


def test_specific_object_beats_everything_regardless_of_order():
    assert acts(rules("everything to the left, and the cup in the tray"))["cup"] == "tray"


def test_stack_on_top_of():
    assert acts(rules("put the red cube on top of the blue cube")) == {"red_cube": "on:blue_cube"}


@pytest.mark.parametrize("text, dest", [
    ("put the red cube next to the cup", "next_to:cup"),
    ("put the red cube beside the cup", "next_to:cup"),
    ("put the marker left of the cup", "left_of:cup"),
    ("put the marker to the right of the cup", "right_of:cup"),
    ("put the marker in front of the cup", "in_front_of:cup"),
    ("put the marker behind the cup", "behind:cup"),
])
def test_relations(text, dest):
    mover = "red_cube" if "red" in text else "marker"
    assert planner.plan_with_rules([text], scene())["actions"] == [{"object": mover, "destination": dest}]


def test_on_the_left_is_a_zone_not_a_relation():
    assert acts(rules("put the cup on the left")) == {"cup": "left"}


def test_relations_and_zones_in_one_request_with_anchor_moved_first():
    plan = rules("put the red cube next to the cup, and the cup in the tray")
    assert [a["object"] for a in plan["actions"]] == ["cup", "red_cube"]   # anchor first
    assert acts(plan) == {"cup": "tray", "red_cube": "next_to:cup"}


def test_change_of_mind_replaces_the_destination():
    a = acts(rules("put the red cube on top of the blue cube", "actually put the red cube next to the cup instead"))
    assert a == {"red_cube": "next_to:cup"}


def test_leave_removes_object():
    a = acts(rules("Put the cup in the tray, everything else to the left", "Actually, leave the cup"))
    assert "cup" not in a and a["red_cube"] == "left"


def test_give_me_object_goes_to_me():
    assert acts(rules("give me the blue cube")) == {"blue_cube": "to_me"}


def test_unknown_request_is_refused_with_no_actions():
    p = rules("Pick up my laptop")
    assert p["actions"] == [] and "can't" in p["say"].lower()


@pytest.mark.parametrize("text", ["open the window", "what happened", "I ordered pizza", "I'm bored", "cupboard"])
def test_words_inside_other_words_do_not_match(text):
    assert planner._objects_in(text.lower()) == set()


def test_all_does_not_mean_everything():
    assert acts(rules("put all the blue things in the tray")) == {"blue_cube": "tray"}


def test_words_for_objects_not_in_the_catalog_are_ignored(monkeypatch):
    objs = dict(config.OBJECTS)
    objs.pop("cup")
    monkeypatch.setattr(config, "OBJECTS", objs)
    assert "mug" not in planner.words() and "cup" not in planner.words()


# ---------------- validation: the safety layer ----------------

def v(actions, s=None, say="ok"):
    return planner.validate({"say": say, "actions": actions}, scene() if s is None else s)


def test_valid_move_kept():
    assert acts(v([{"object": "red_cube", "destination": "tray"}])) == {"red_cube": "tray"}


def test_unknown_object_rejected():
    p = v([{"object": "laptop", "destination": "tray"}])
    assert p["actions"] == [] and "laptop" in p["problems"][0]


@pytest.mark.parametrize("dst", ["slot2", "floor", "(100, 200)", None, "under:cup", "next_to:laptop", 7])
def test_bad_destinations_never_reach_the_robot(dst):
    p = v([{"object": "red_cube", "destination": dst}])
    assert p["actions"] == [] and p["already"] == [] and p["problems"]


def test_invisible_object_rejected():
    p = v([{"object": "red_cube", "destination": "tray"}], s=scene(red_cube=None))
    assert p["actions"] == [] and "can't see" in p["problems"][0]


def test_relation_to_itself_rejected():
    assert v([{"object": "cup", "destination": "next_to:cup"}])["actions"] == []


def test_stack_on_non_stackable_rejected_with_reason():
    p = v([{"object": "red_cube", "destination": "on:cup"}])
    assert p["actions"] == [] and "stack" in p["problems"][0]


def test_impossible_relation_rejected_now_with_reason():
    p = v([{"object": "red_cube", "destination": "next_to:cup"}], s=scene(cup=None))
    assert p["actions"] == [] and "can't see the cup" in p["problems"][0]


def test_relation_to_an_anchor_that_moves_first_is_kept():
    # "behind" the cup is off the table where it is now, but fine after it moves
    s = scene(cup=(210, 0))
    p = v([{"object": "cup", "destination": "to_me"},
           {"object": "red_cube", "destination": "behind:cup"}], s=s)
    assert acts(p) == {"cup": "to_me", "red_cube": "behind:cup"}


def test_already_true_goes_to_already_not_actions():
    p = v([{"object": "red_cube", "destination": "tray"}], s=scene(red_cube="tray"))
    assert p["actions"] == [] and p["already"] == [{"object": "red_cube", "destination": "tray"}]


def test_first_instruction_for_an_object_wins():
    p = v([{"object": "red_cube", "destination": "tray"}, {"object": "red_cube", "destination": "left"}])
    assert acts(p) == {"red_cube": "tray"}


def test_slots_mode_only_allows_zones(monkeypatch):
    monkeypatch.setattr(config, "MODE", "slots")
    p = v([{"object": "red_cube", "destination": "next_to:cup"}, {"object": "cup", "destination": "tray"}])
    assert acts(p) == {"cup": "tray"} and "marked zones" in p["problems"][0]


@pytest.mark.parametrize("raw", [None, [], "nonsense", {"actions": "go"}, {"actions": [5, "x"]}, {}])
def test_garbage_plans_never_crash_and_never_move(raw):
    assert planner.validate(raw, scene())["actions"] == []


def test_say_is_always_a_string():
    assert planner.validate({"say": 42, "actions": []}, scene())["say"] == "42"


def test_order_puts_anchors_first_and_survives_cycles():
    a = [{"object": "red_cube", "destination": "on:blue_cube"}, {"object": "blue_cube", "destination": "tray"}]
    assert [x["object"] for x in planner.order(a)] == ["blue_cube", "red_cube"]
    cyc = [{"object": "red_cube", "destination": "next_to:cup"}, {"object": "cup", "destination": "next_to:red_cube"}]
    assert len(planner.order(cyc)) == 2


# ---------------- AI path, with a fake client ----------------

class FakeClient:
    def __init__(self, reply=None, error=None, blocks=None):
        self.reply, self.error, self.blocks, self.calls = reply, error, blocks, []
        self.messages = self

    def create(self, **kw):
        self.calls.append(kw)
        if self.error:
            raise self.error
        blocks = self.blocks or [types.SimpleNamespace(text=self.reply)]
        return types.SimpleNamespace(content=blocks)


def test_ai_plan_is_parsed_and_validated():
    reply = json.dumps({"say": "Sure!", "actions": [
        {"object": "marker", "destination": "to_me"},
        {"object": "red_cube", "destination": "on:blue_cube"},
        {"object": "laptop", "destination": "tray"}]})
    p = planner.make_plan(["I need to take notes, and stack the red block"], scene(), client=FakeClient(reply))
    assert acts(p) == {"marker": "to_me", "red_cube": "on:blue_cube"} and p["say"] == "Sure!"
    assert any("laptop" in x for x in p["problems"])


def test_ai_reply_with_extra_text_around_json_still_works():
    reply = 'Here you go:\n{"say": "ok", "actions": [{"object": "red_cube", "destination": "tray"}]}\nThanks'
    assert acts(planner.make_plan(["red in tray"], scene(), client=FakeClient(reply))) == {"red_cube": "tray"}


def test_ai_reply_split_across_blocks_is_joined():
    blocks = [types.SimpleNamespace(text='{"say": "ok", "actions": '),
              types.SimpleNamespace(text='[{"object": "red_cube", "destination": "tray"}]}')]
    assert acts(planner.make_plan(["x"], scene(), client=FakeClient(blocks=blocks))) == {"red_cube": "tray"}


def test_bad_ai_json_falls_back_to_keywords_and_says_so():
    p = planner.make_plan(["Give me something to write with"], scene(), client=FakeClient("not json {"))
    assert acts(p) == {"marker": "to_me"} and "keyword planner" in p["problems"][0]


def test_ai_error_falls_back_to_keywords_and_says_so():
    p = planner.make_plan(["Give me something to write with"], scene(),
                          client=FakeClient(error=ConnectionError("offline")))
    assert acts(p) == {"marker": "to_me"} and "unavailable" in p["problems"][0]


def test_ai_prompt_has_catalog_positions_zones_stackable_and_requests_in_order():
    fake = FakeClient('{"say": "", "actions": []}')
    planner.make_plan(["first thing", "second thing"], scene(marker="to_me", cup=None), client=fake)
    call = fake.calls[0]
    text = call["messages"][0]["content"]
    assert call["model"] == config.PLANNER_MODEL and call["system"] == planner.SYSTEM
    assert "temperature" not in call            # Sonnet 5.5 400s on sampling params (found Fri 12:45)
    assert call["output_config"] == {"effort": "low"}
    assert "used for writing" in text and "in zone to_me" in text
    assert "cup: a small paper cup, for drinks (not visible)" in text
    assert "blue_cube: a blue toy block, stackable (at x=500 y=-60 mm)" in text
    assert "Zones: tray, left, right, to_me" in text
    assert text.index("1. first thing") < text.index("2. second thing")
    for word in ("left_of:", "next_to:", "on:", "in front of = closer to the person"):
        assert word in planner.SYSTEM


def test_no_api_key_means_no_ai_call(monkeypatch):
    called = []
    monkeypatch.setattr(planner, "plan_with_ai", lambda *a, **k: called.append(1))
    planner.make_plan(["give me the red cube"], scene())
    assert called == []


def test_real_client_has_short_timeout_and_no_silent_retries(monkeypatch):
    """A hung AI call must not freeze the demo (Duet used timeouts and a fallback)."""
    import anthropic
    made = {}

    class Spy:
        def __init__(self, **kw):
            made.update(kw)
            raise ConnectionError("offline")
    monkeypatch.setattr(anthropic, "Anthropic", Spy)
    monkeypatch.setenv("ANTHROPIC_API_KEY", "test")
    p = planner.make_plan(["Give me something to write with"], scene())
    assert made == {"timeout": config.PLANNER_TIMEOUT_S, "max_retries": 0}
    assert acts(p) == {"marker": "to_me"}


def test_start_layout_is_what_the_tests_assume():
    assert set(START) == set(config.OBJECTS)
