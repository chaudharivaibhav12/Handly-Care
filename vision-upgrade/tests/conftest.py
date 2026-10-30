import asyncio
import copy
import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import settings  # noqa: E402


@pytest.fixture(autouse=True)
def clean_settings_and_cwd(tmp_path, monkeypatch):
    """Every test gets a fresh copy of settings and its own folder for out/ and json files."""
    saved = {k: copy.deepcopy(v) for k, v in vars(settings).items() if k.isupper()}
    monkeypatch.chdir(tmp_path)
    yield
    for k, v in saved.items():
        setattr(settings, k, v)


def run(coro):
    return asyncio.run(coro)
