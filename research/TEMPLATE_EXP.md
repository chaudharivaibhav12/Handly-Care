---
id: EXP-NNN
title: ""
date: YYYY-MM-DD HH:MM
status: planned          # planned | running | completed
branch: A                # a branch letter from STATUS.md
parent: null             # latest EXP on this branch, or null
motivated_by: []         # EXP/DEC IDs, or goal for the very first ones
hypothesis: ""           # one falsifiable sentence with a threshold, e.g. "test passes 5/5"
rules_out: []            # filled at close: what this result now excludes
superseded_by: null
commit: ""               # git commit that actually ran ("" while there is no git)
commit_verified: false
config_hash: ""          # from `python research/check.py --setup` (config.py + calibration.json)
decision: ""             # CONTINUE | PIVOT | PARK | ABANDON, set at close
---

## Why
<!-- One or two lines. Must cite an EXP/DEC ID, or "goal". -->

## Setup
<!-- Paste `python research/check.py --setup`, then add what it can't see:
     objects on the table, lighting, who ran the arm, camera moved since calibrate?
     Exact command(s), e.g. `python app.py test red_cube slot1 slot2`.
     Env vars by NAME only, never values. -->

## Results
<!-- Raw numbers only. `python app.py stats` output, trials.csv time window (HH:MM-HH:MM),
     raw files in results/EXP-NNN*. No interpretation here. -->

## Interpretation
<!-- What it means + confidence (high/medium/low) and why. -->

## Consequence
<!-- What we do differently now. Bug found on hardware -> name the test added. -->

## Hacks left in place
<!-- Hard-coded values, bypasses, sim-only shortcuts to remove before the demo. "none" is fine. -->
