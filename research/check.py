"""Validate the research records. Standard library only.

    python research/check.py           # 0 errors -> exit 0
    python research/check.py --setup   # print the operating point to paste into an EXP's Setup

Checks: required front matter, every cited EXP/DEC ID exists, completed EXPs have Why /
Results / Interpretation / Consequence filled and a decision, Why cites an ID or "goal",
every EXP's branch exists in STATUS.md, and every PARKED/ABANDONED branch is named by a DEC.
DECISIONS.md (the pre-event planning record) counts as DEC-000.
"""
import hashlib
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RESEARCH = os.path.join(ROOT, "research")
MAP = os.path.join(ROOT, "STATUS.md")
DEC_000 = os.path.join(ROOT, "DECISIONS.md")

ID_RE = re.compile(r"\b(EXP|DEC)-\d{3}\b")
BRANCH_RE = re.compile(r"^### Branch ([A-Z]) — (.+?) — (ACTIVE|PARKED|ABANDONED)\s*$", re.M)
EXP_FIELDS = ["id", "title", "date", "status", "branch", "parent", "motivated_by", "hypothesis",
              "rules_out", "superseded_by", "commit", "commit_verified", "config_hash", "decision"]
DEC_FIELDS = ["id", "date", "evidence", "branch_change"]
STATUSES = {"planned", "running", "completed"}
DECISIONS = {"CONTINUE", "PIVOT", "PARK", "ABANDON"}
MUST_FILL = ["Why", "Results", "Interpretation", "Consequence"]


def parse(text):
    """-> (front matter dict, {section heading: body}). Values: str, list, or None."""
    meta, body = {}, text
    m = re.match(r"^---\s*\n(.*?)\n---\s*\n?", text, re.S)
    if m:
        body = text[m.end():]
        for line in m.group(1).splitlines():
            line = re.sub(r"\s+#.*$", "", line).strip()
            if not line or ":" not in line:
                continue
            key, _, val = line.partition(":")
            val = val.strip()
            if val.startswith("[") and val.endswith("]"):
                val = [v.strip().strip("\"'") for v in val[1:-1].split(",") if v.strip()]
            elif val in ("null", "~", ""):
                val = None if val != "" else ""
            else:
                val = val.strip("\"'")
            meta[key.strip()] = val
    sections, current = {}, None
    for line in body.splitlines():
        h = re.match(r"^##\s+(.+?)\s*$", line)
        if h:
            current = h.group(1)
            sections[current] = ""
        elif current:
            sections[current] += line + "\n"
    return meta, sections


def filled(s):
    return bool(re.sub(r"<!--.*?-->", "", s or "", flags=re.S).strip())


def records(kind):
    folder = os.path.join(RESEARCH, "experiments" if kind == "EXP" else "decisions")
    out = []
    if os.path.isdir(folder):
        for name in sorted(os.listdir(folder)):
            if name.endswith(".md"):
                path = os.path.join(folder, name)
                with open(path, encoding="utf-8") as f:
                    text = f.read()
                out.append((name, text) + parse(text))
    return out


def as_list(v):
    if v is None or v == "":
        return []
    return v if isinstance(v, list) else [v]


def check():
    errors = []
    exps, decs = records("EXP"), records("DEC")
    known = {"DEC-000"} if os.path.exists(DEC_000) else set()
    for name, _, meta, _ in exps + decs:
        rid = meta.get("id", "")
        if not re.fullmatch(r"(EXP|DEC)-\d{3}", rid or ""):
            errors.append(f"{name}: bad or missing id '{rid}'")
            continue
        if name != rid + ".md":
            errors.append(f"{name}: file name should be {rid}.md")
        if rid in known:
            errors.append(f"{name}: duplicate id {rid}")
        known.add(rid)

    map_text = open(MAP, encoding="utf-8").read() if os.path.exists(MAP) else ""
    branches = {b: state for b, _, state in BRANCH_RE.findall(map_text)}
    if not branches:
        errors.append("STATUS.md: no '### Branch X — name — ACTIVE|PARKED|ABANDONED' headings")

    for name, text, meta, sec in exps:
        missing = [k for k in EXP_FIELDS if k not in meta]
        if missing:
            errors.append(f"{name}: missing front matter {missing}")
        if meta.get("status") not in STATUSES:
            errors.append(f"{name}: status must be one of {sorted(STATUSES)}")
        if meta.get("branch") not in branches:
            errors.append(f"{name}: branch '{meta.get('branch')}' is not in STATUS.md")
        if not ID_RE.search(sec.get("Why", "")) and "goal" not in sec.get("Why", "").lower() \
                and meta.get("status") != "planned":
            errors.append(f"{name}: Why must cite an EXP/DEC ID or the goal")
        if meta.get("status") == "completed":
            for s in MUST_FILL:
                if not filled(sec.get(s)):
                    errors.append(f"{name}: completed but '{s}' is empty")
            if meta.get("decision") not in DECISIONS:
                errors.append(f"{name}: completed but decision is not one of {sorted(DECISIONS)}")

    for name, text, meta, sec in decs:
        missing = [k for k in DEC_FIELDS if k not in meta]
        if missing:
            errors.append(f"{name}: missing front matter {missing}")
        if not as_list(meta.get("evidence")) and meta.get("branch_change") != "initial":
            errors.append(f"{name}: needs evidence EXP IDs (only an 'initial' DEC may have none)")
        for s in ("Decision", "Reason"):
            if not filled(sec.get(s)):
                errors.append(f"{name}: '{s}' is empty")

    # Every cited ID must exist (records, LOG.md, STATUS.md).
    cited_in = [(n, t) for n, t, _, _ in exps + decs]
    for extra in (os.path.join(RESEARCH, "LOG.md"), MAP):
        if os.path.exists(extra):
            cited_in.append((os.path.relpath(extra, ROOT), open(extra, encoding="utf-8").read()))
    for name, text in cited_in:
        for m in ID_RE.finditer(text):
            if m.group(0) not in known:
                errors.append(f"{name}: cites {m.group(0)}, which doesn't exist")

    dec_texts = [t for _, t, _, _ in decs]
    for b, state in branches.items():
        if state in ("PARKED", "ABANDONED") and not any(f"Branch {b}" in t for t in dec_texts):
            errors.append(f"STATUS.md: Branch {b} is {state} but no DEC names 'Branch {b}'")
    return errors


def setup():
    """Operating point: every simple config value + hashes. Never prints secret values."""
    sys.path.insert(0, ROOT)
    import config
    h = hashlib.sha1()
    for f in ("config.py", "calibration.json"):
        p = os.path.join(ROOT, f)
        if os.path.exists(p):
            h.update(open(p, "rb").read())
    lines = [f"config_hash: {h.hexdigest()[:10]}"
             f"   (calibration.json {'present' if os.path.exists(os.path.join(ROOT, 'calibration.json')) else 'MISSING'})"]
    for k in sorted(dir(config)):
        v = getattr(config, k)
        if k.isupper() and isinstance(v, (int, float, str, bool, tuple, list, dict, type(None))):
            r = repr(v)
            lines.append(f"- {k} = {r if len(r) < 160 else r[:157] + '...'}")
    env = ("VIAM_ADDRESS", "VIAM_API_KEY_ID", "VIAM_API_KEY", "ANTHROPIC_API_KEY", "OPENROUTER_API_KEY")
    lines.append("- env set: " + ", ".join(f"{e}={'yes' if os.environ.get(e) else 'no'}" for e in env)
                 + "  (values never recorded)")
    return "\n".join(lines)


if __name__ == "__main__":
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    if "--setup" in sys.argv:
        try:
            from dotenv import load_dotenv
            load_dotenv(os.path.join(ROOT, ".env"))
        except ImportError:
            pass
        print(setup())
        sys.exit(0)
    errs = check()
    for e in errs:
        print("✗", e)
    print(f"{len(errs)} error(s)")
    sys.exit(1 if errs else 0)
