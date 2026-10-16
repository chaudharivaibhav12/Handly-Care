"""ai.py: Anthropic directly or through OpenRouter (Fri 3:20 PM: Anthropic account had no credit)."""
import ai
import planner


def test_openrouter_model_names(monkeypatch):
    monkeypatch.setenv("OPENROUTER_API_KEY", "sk-or-test")
    assert ai.model("claude-sonnet-5-5") == "anthropic/claude-sonnet-5.5"
    assert ai.model("claude-opus-5-5") == "anthropic/claude-opus-5.5"
    assert ai.model("claude-fable-5-1") == "anthropic/claude-fable-5.1"


def test_anthropic_model_names_unchanged(monkeypatch):
    monkeypatch.setenv("ANTHROPIC_API_KEY", "sk-ant-test")
    assert ai.model("claude-sonnet-5-5") == "claude-sonnet-5-5"


def test_openrouter_client_never_sends_the_anthropic_key(monkeypatch):
    monkeypatch.setenv("ANTHROPIC_API_KEY", "sk-ant-secret")
    monkeypatch.setenv("OPENROUTER_API_KEY", "sk-or-test")
    c = ai.client(5.0)
    assert str(c.base_url).startswith(ai.OPENROUTER_URL)
    assert c.api_key == "sk-or-test" and c.auth_token == "sk-or-test"


def test_openrouter_key_alone_turns_the_ai_planner_on(monkeypatch):
    assert not ai.available()
    monkeypatch.setenv("OPENROUTER_API_KEY", "sk-or-test")
    assert ai.available()
    called = {}

    def fake(requests, scene, client=None):
        called["yes"] = True
        raise RuntimeError("offline")
    monkeypatch.setattr(planner, "plan_with_ai", fake)
    planner.make_plan(["give me something to write with"], {"marker": (300.0, 100.0)})
    assert called                                   # tried the AI, then fell back to keywords
