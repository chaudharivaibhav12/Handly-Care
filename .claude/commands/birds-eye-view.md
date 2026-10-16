---
description: Move armfarm13 to the bird's-eye OVERVIEW pose (camera sees the whole table)
allowed-tools: Bash(.venv/Scripts/python app.py where), Bash(.venv/Scripts/python app.py overview)
---

Vaibhav typed this command, which is his go-ahead to move the arm now. Don't ask again.

1. In one line, say: "Moving to bird's-eye view (OVERVIEW from config.py). Hand on the E-stop."
2. Run `.venv/Scripts/python app.py where` (from the project root) and note the start pose.
3. Run `.venv/Scripts/python app.py overview` (timeout 180 s). This uses the Viam motion service, so it
   plans around the table, wall and ceiling obstacles. It does not use a direct joint move.
4. Run `.venv/Scripts/python app.py where` again. Report in 2-3 lines: the start pose, the end pose, and the
   distance in mm from config.OVERVIEW. If the error is under 10 mm, say it's fine.
5. If a step fails, show the error line and stop. **Don't retry the move.** Common causes:
   - Manual mode is still ON (Configure panel → turn it off).
   - The E-stop is pressed.
   - The connection dropped (`deadline has elapsed`).
   - The planner found no path: say so. Never edit the obstacles or OVERVIEW to force it through.
6. Add a line to `research/LOG.md`: `- Fri HH:MM note: /birds-eye-view ok|failed, end pose off by N mm`.
