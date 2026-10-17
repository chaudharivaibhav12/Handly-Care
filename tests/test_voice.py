"""Voice requests: one sentence -> one allowed routine. Stop is instant and local; the AI can't invent routines."""
import sys

import pytest

import bridge
import voice


class FakeClient:
    """Stands in for the Claude client: returns a fixed reply, or raises."""
    def __init__(self, reply=None, error=None):
        self.reply, self.error, self.calls = reply, error, 0
        self.messages = self

    def create(self, **kw):
        self.calls += 1
        if self.error:
            raise self.error
        return type("Msg", (), {"content": [type("Block", (), {"text": self.reply})()]})()


@pytest.mark.parametrize("text,action", [
    ("Can I have my medicine please", "medicine"),
    ("I need my pills", "medicine"),
    ("could you get me a tissue", "tissue"),
    ("my nose is running", "tissue"),
    ("go back to your start pose", "to_high"),
    ("I want a glass of water", "none"),
    ("help me get out of bed", "none"),
])
def test_keywords(text, action):
    assert voice.interpret(text, use_ai=False)["action"] == action


@pytest.mark.parametrize("text", ["stop", "STOP!", "please stop now", "wait", "no, wait", "freeze"])
def test_stop_is_local_and_never_asks_the_ai(text):
    client = FakeClient('{"action": "medicine", "say": "x"}')
    out = voice.interpret(text, client=client)
    assert out["action"] == "stop" and out["source"] == "local" and client.calls == 0


def test_no_in_a_long_sentence_is_not_a_stop():
    assert voice.interpret("can I have my medicine, no rush", use_ai=False)["action"] == "medicine"


def test_ai_answer_used_when_valid():
    out = voice.interpret("I feel a sneeze coming", client=FakeClient('{"action": "tissue", "say": "Here comes a tissue."}'))
    assert out == {"action": "tissue", "say": "Here comes a tissue.", "source": "ai"}


def test_ai_cannot_invent_a_routine():
    out = voice.interpret("open the window", client=FakeClient('{"action": "open_window", "say": "Sure!"}'))
    assert out["action"] == "none" and out["source"] == "keywords"


def test_ai_failure_falls_back_to_keywords():
    out = voice.interpret("my medicine please", client=FakeClient(error=TimeoutError("slow")))
    assert out == {"action": "medicine", "say": voice.REPLIES["medicine"], "source": "keywords"}


def test_ai_garbage_falls_back_to_keywords():
    out = voice.interpret("a tissue please", client=FakeClient("I think a tissue"))
    assert out["action"] == "tissue" and out["source"] == "keywords"


def test_bridge_say_stops_even_while_busy():
    r = bridge.Runner(routines={"slow": (["slow"], 60, [("", "Waiting")])},
                      command=lambda a: [sys.executable, "-c", "import time; time.sleep(30)"])
    r.start("slow")
    asked = []
    out = bridge.say(r, "Stop!", interpret=lambda t: asked.append(t))
    assert out["action"] == "stop" and asked == []
    assert r.status()["busy"] is False or r.result is not None


def test_bridge_say_refuses_new_request_while_busy():
    r = bridge.Runner(routines={"slow": (["slow"], 60, [("", "Waiting")])},
                      command=lambda a: [sys.executable, "-c", "import time; time.sleep(30)"])
    r.start("slow")
    try:
        asked = []
        out = bridge.say(r, "my medicine please", interpret=lambda t: asked.append(t))
        assert out["action"] == "busy" and asked == []
    finally:
        r.stop(grace=3)


def test_bridge_say_never_starts_the_arm():
    r = bridge.Runner(routines={"slow": (["slow"], 60, [("", "Waiting")])}, command=lambda a: ["unused"])
    out = bridge.say(r, "my medicine please", interpret=lambda t: {"action": "medicine", "say": "ok", "source": "ai"})
    assert out["action"] == "medicine" and out["text"] == "my medicine please"
    assert r.status()["busy"] is False and r.run_id == 0      # the nurse's Start click runs it, not the voice
