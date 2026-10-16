# DECISIONS: how we got here (planning session, Thu Oct 15 evening → Fri 3 AM)

1. **Accepted** as the final attendee (email from Camila). Viam 101 course recommended beforehand.
2. **First idea:** "talk to the robot + it checks its own work" (natural language → plan → pick/place → look again).
3. **Research on past events** (LeRobot hackathons, Viam's Sept 2026 "Fine Motor Skills" at the same venue):
   winners combine a human story with reliability; LLM+arm is common now; avoid speed, liquids,
   high-precision insertion; interactive demos beat watch-only.
4. **ChatGPT brainstorm reviewed** (Vaibhav shared it): Rock-Paper-Scissors (five-finger hand isn't
   available; tokens look basic), Robot Escape Room, SHIFT adaptive warehouse, Robot Artist, Chess.
   Claude's critique: warehouse/sorting is the most crowded category (recycle sorter + benchmark task last
   month), a drawing robot (Duet) and a game (Jenga arcade) were done last month. Kept the
   re-planning/recovery engine, wanted a human story.
5. **Vaibhav picked the "Physical AI Personal Assistant"** → named **Sidekick**: desk helper for people
   with limited hand mobility. Simplified: 4 objects, fixed zones, constrained actions.
6. **ChatGPT reviewed the Sidekick plan; changes adopted:** start with 4 objects; treat camera
   localization as its own milestone (slots first); interruptions only at safe checkpoints (nobody
   reaches in while moving); one `move_object` action validated in code; an object info table so the AI
   knows a marker is for writing; show real state, not "thoughts".
7. **Starter code written** from the Viam 101 course (read in the browser): motion service on the
   gripper frame, world-frame poses pointing down, retries. Then reviewed line by line, bugs fixed, and
   tests written; every safety behavior checked by breaking it deliberately.
8. **Code of last month's teams analyzed** (Gaze Arm winner, Duet, recycle sorter, ThirdArm, juggling,
   arm-5, ball catch, keyboard arm, Viam's reference). Applied: detection size/region filters + 3-look
   voting (the old "biggest blob wins" was exactly wrong); AI 15 s timeout + fallback; straight-line
   tolerance 10 mm.
9. **Don't copy Gaze** (same judges; gaze calibration ate their two days). Take its engine instead.
10. **Vaibhav's choices (Fri 2:20 AM):**
    - Pick-and-place **precisely**: find objects anywhere and place at exact spots, relative to others.
    - Input: **talk, with pointing as a stretch goal**.
    - Objects: **try everyday objects first, switch if they don't work** (detector ladder).
    - Team: **mostly 1 teammate**.
11. **Sidekick v2 built:** world-coordinate perception via segmenters, camera calibration from taught
    spots, relational placement (`left_of`/`next_to`/`on`...), mm error measured each time, nudge,
    anchor-follow repair, slots mode as fallback. Zones widened to ~18 cm (a cup + cube must fit).
12. **Roles:** Vaibhav leads and owns everything; the teammate is the **test pilot** (arm, E-stop,
    logged trials, breaking things). Added trial logging (`trials.csv`, `stats`) and `TEAMMATE.md`.
13. **Open choice:** Saturday stretch goal. Claude's recommendation: stacking or a precision
    dashboard; pointing ("put it there") is ~2-3 h and only worth it if M1-M4 are solid by Friday night.

### Rejected and why
| Option | Why not |
|---|---|
| Gaze control | Last month's winner; same judges; heavy calibration |
| Warehouse / sorting (SHIFT) | Most crowded category; reads as "another sorter" |
| Drawing robot | Done last month (Duet) |
| Game (RPS, chess, Jenga) | Games done last month; chess is high risk; RPS with tokens looks basic |
| Juggling / fast motion | Last month's team hit speed and motor limits |
| Five-finger gestures | Hardware is a two-finger gripper |
