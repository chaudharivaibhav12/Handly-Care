"""The one place that builds the Claude client: Anthropic directly, or through OpenRouter.

Fri 3:20 PM: the Anthropic account had no credit, but Vaibhav's OpenRouter key did. OpenRouter speaks
the Anthropic Messages API at OPENROUTER_URL, so the same anthropic SDK works with a different base URL,
the OpenRouter key and OpenRouter's model names. OPENROUTER_API_KEY wins when both keys are set.
"""
import os
import re

OPENROUTER_URL = "https://openrouter.ai/api"


def via_openrouter():
    return bool(os.environ.get("OPENROUTER_API_KEY"))


def available():
    """Is any AI backend configured? (Otherwise: keyword planner, plain color names.)"""
    return via_openrouter() or bool(os.environ.get("ANTHROPIC_API_KEY"))


def model(name):
    """Our model id -> the backend's: claude-sonnet-5-5 -> anthropic/claude-sonnet-5.5 on OpenRouter."""
    if not via_openrouter():
        return name
    return "anthropic/" + re.sub(r"-(\d+)-(\d+)$", r"-\1.\2", name)


def client(timeout):
    """Short timeout, no silent retries: on any failure the caller falls back (keywords / plain names)."""
    import anthropic
    if via_openrouter():
        key = os.environ["OPENROUTER_API_KEY"]
        # Both headers carry the OpenRouter key, so the Anthropic key from .env is never sent to OpenRouter.
        return anthropic.Anthropic(base_url=OPENROUTER_URL, api_key=key, auth_token=key,
                                   timeout=timeout, max_retries=0)
    return anthropic.Anthropic(timeout=timeout, max_retries=0)
