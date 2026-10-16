"""research/check.py: each rule is shown to fire by breaking it on purpose."""
import importlib.util
import os

import pytest

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
spec = importlib.util.spec_from_file_location("research_check", os.path.join(HERE, "research", "check.py"))
rc = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rc)

MAP = """# STATUS
### Branch A — Core — ACTIVE
### Branch C — Slots — PARKED
"""
DEC = """---
id: DEC-001
date: 2026-10-16 11:20
evidence: []
branch_change: initial
---
## Decision
Park Branch C.
## Reason
Plan.
"""
EXP = """---
id: EXP-001
title: "x"
date: 2026-10-16 11:20
status: {status}
branch: A
parent: null
motivated_by: [goal]
hypothesis: "5/5"
rules_out: []
superseded_by: null
commit: ""
commit_verified: false
config_hash: ""
decision: {decision}
---
## Why
{why}
## Setup
cmd
## Results
{results}
## Interpretation
ok
## Consequence
ok
"""


@pytest.fixture
def tree(tmp_path, monkeypatch):
    (tmp_path / "research" / "experiments").mkdir(parents=True)
    (tmp_path / "research" / "decisions").mkdir()
    monkeypatch.setattr(rc, "ROOT", str(tmp_path))
    monkeypatch.setattr(rc, "RESEARCH", str(tmp_path / "research"))
    monkeypatch.setattr(rc, "MAP", str(tmp_path / "STATUS.md"))
    monkeypatch.setattr(rc, "DEC_000", str(tmp_path / "DECISIONS.md"))
    (tmp_path / "STATUS.md").write_text(MAP, encoding="utf-8")
    (tmp_path / "DECISIONS.md").write_text("pre-event", encoding="utf-8")
    (tmp_path / "research" / "decisions" / "DEC-001.md").write_text(DEC, encoding="utf-8")

    def put(status="completed", decision="CONTINUE", why="Goal: move. DEC-001.", results="5/5"):
        (tmp_path / "research" / "experiments" / "EXP-001.md").write_text(
            EXP.format(status=status, decision=decision, why=why, results=results), encoding="utf-8")
    return tmp_path, put


def test_clean_tree_passes(tree):
    _, put = tree
    put()
    assert rc.check() == []


def test_completed_without_results_or_decision_fails(tree):
    _, put = tree
    put(results="<!-- todo -->", decision='""')
    errs = rc.check()
    assert any("'Results' is empty" in e for e in errs)
    assert any("decision is not one of" in e for e in errs)


def test_why_must_cite_something(tree):
    _, put = tree
    put(why="because")
    assert any("Why must cite" in e for e in rc.check())


def test_unknown_id_is_caught(tree):
    _, put = tree
    put(why="Follows EXP-099.")
    assert any("EXP-099" in e for e in rc.check())


def test_parked_branch_needs_a_dec(tree):
    root, put = tree
    put()
    (root / "research" / "decisions" / "DEC-001.md").write_text(DEC.replace("Branch C", "it"), encoding="utf-8")
    assert any("Branch C is PARKED" in e for e in rc.check())


def test_non_initial_dec_needs_evidence(tree):
    root, put = tree
    put()
    (root / "research" / "decisions" / "DEC-002.md").write_text(
        DEC.replace("DEC-001", "DEC-002").replace("initial", "Branch C PARKED -> ACTIVE"), encoding="utf-8")
    assert any("needs evidence" in e for e in rc.check())


def test_branch_must_exist(tree):
    root, put = tree
    put()
    p = root / "research" / "experiments" / "EXP-001.md"
    p.write_text(p.read_text(encoding="utf-8").replace("branch: A", "branch: Z"), encoding="utf-8")
    assert any("branch 'Z'" in e for e in rc.check())


def test_setup_never_prints_secret_values(monkeypatch):
    monkeypatch.setenv("VIAM_API_KEY", "super-secret-value")
    out = rc.setup()
    assert "super-secret-value" not in out
    assert "VIAM_API_KEY=yes" in out
