"""Voice requests for the dashboard: one sentence -> ONE allowed routine and a short reply to speak.

"Stop" (wait / halt / no) is matched here, with no AI, so it always works at once. Everything else goes to Claude
(ai.py, short timeout), whose answer is checked against ACTIONS in code: the AI can never invent a routine.
No AI key, or any AI failure -> keywords. The dashboard still asks the nurse to click Start before the arm moves.
"""
import re

import config
import planner

ACTIONS = ("medicine", "tissue", "thermometer", "to_high", "none")
VOICE_TIMEOUT_S = 8.0           # the person is waiting; keywords answer if Claude is slow
STOP_RE = re.compile(r"\b(stop|halt|freeze)\b", re.I)                 # anywhere in the sentence
SHORT_STOP_RE = re.compile(r"\b(wait|no|don'?t)\b", re.I)            # only in a short one ("no, wait!"), not "no rush"
KEYWORDS = [   # (pattern, action): first match wins
    (re.compile(r"\b(pills?|medicines?|medications?|meds|tablets?|pill ?box|drugs?)\b", re.I), "medicine"),
    (re.compile(r"\b(tissues?|kleenex|nose|sneez\w*|napkin)\b", re.I), "tissue"),
    (re.compile(r"\b(temperature|thermometer|fever|feel (hot|warm))\b", re.I), "thermometer"),
    (re.compile(r"\b(start(ing)? (pose|position)|reset|go back)\b", re.I), "to_high"),
]
REPLIES = {
    "medicine": "Getting your medicine now.",
    "tissue": "Getting you a tissue.",
    "thermometer": "Let me check your temperature.",
    "to_high": "Moving back to my start pose.",
    "none": "Sorry, I can only bring your medicine or a tissue, or check your temperature. I'll let your nurse know.",
}

SYSTEM = """You are the voice of a bedside robot arm helping a patient with limited hand mobility.
It can do exactly these routines and nothing else:
- medicine: open the drawer, take out the nurse-prepared pill box, hand it to the patient
- tissue: pull one tissue from the box and hand it to the patient
- thermometer: take a temperature reading with the no-contact thermometer (e.g. 'I feel hot', 'check my temperature')
- to_high: move the arm back to its start pose
- none: anything else (water, food, getting up, pain, emergencies, unclear requests) -> no movement
Reply with ONLY a JSON object: {"action": "<medicine|tissue|thermometer|to_high|none>", "say": "<one short, warm sentence to speak, max 15 words>"}
For "none", say briefly that you can't do it and that the nurse will be told. Never promise anything outside the list."""


def is_stop(text):
    text = text or ""
    return bool(STOP_RE.search(text)) or (len(text.split()) <= 3 and bool(SHORT_STOP_RE.search(text)))


def by_keywords(text):
    for pattern, action in KEYWORDS:
        if pattern.search(text or ""):
            return action
    return "none"


def by_ai(text, client=None):
    """-> (action, say), or None when the reply isn't usable. Raises on network/API errors (caller falls back)."""
    if client is None:
        import ai
        client = ai.client(VOICE_TIMEOUT_S)
    msg = client.messages.create(
        model=planner._model(config.PLANNER_MODEL), max_tokens=200, output_config={"effort": "low"},
        system=SYSTEM, messages=[{"role": "user", "content": f'The patient said: "{text}"'}])
    data = planner.parse_reply("".join(getattr(b, "text", "") for b in msg.content))
    if not data or data.get("action") not in ACTIONS:
        return None
    say = str(data.get("say") or "").strip()[:160] or REPLIES[data["action"]]
    return data["action"], say


def interpret(text, client=None, use_ai=None):
    """-> {"action": stop|medicine|tissue|to_high|none, "say": str, "source": local|ai|keywords}."""
    text = (text or "").strip()[:300]
    if is_stop(text):
        return {"action": "stop", "say": "Stopping.", "source": "local"}
    if not text:
        return {"action": "none", "say": "Sorry, I didn't catch that.", "source": "local"}
    if use_ai is None:
        import ai
        use_ai = client is not None or ai.available()
    if use_ai:
        try:
            got = by_ai(text, client)
            if got:
                return {"action": got[0], "say": got[1], "source": "ai"}
        except Exception as e:  # noqa: BLE001 -- any AI failure: keywords answer instead
            print(f"[voice] AI failed ({type(e).__name__}: {e}); using keywords")
    action = by_keywords(text)
    return {"action": action, "say": REPLIES[action], "source": "keywords"}
