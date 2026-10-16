# Test pilot guide

You run the arm and the tests; the lead owns the code and decides what changes.
Your numbers decide what goes in the demo. No coding needed.

## Your jobs
1. **Safety officer.** You're next to the E-stop whenever the arm moves. Nobody reaches in
   while it moves. Stop it if anything looks wrong; there's no penalty for a false alarm.
2. **Teach positions** (Friday 11:00). Jog the arm in the Viam app so the gripper is centered
   just above each taped spot, run `python app.py where`, and read out the x and y. Lead types them into `config.py`.
3. **Run the trials.** Every run is logged to `trials.csv` automatically.
   - `python app.py test red_cube slot1 tray`: answer the "did it land right?" prompt
   - `python app.py pick cup tray`: prints "placed, 7 mm from target"
   - Put the object back somewhere new between runs (for `pick`, anywhere on the table)
   - `python app.py stats` shows success rate and accuracy per object
4. **Break it on purpose** (Saturday morning). Scatter objects, try odd requests,
   move the cup while the arm is at the overview pose, say "actually..." mid-task. Write down
   anything that surprises you, word for word.
5. **Demo partner.** In the demo you're the "judge" who scatters objects and moves the cup.

## Rules for good trials
- 5 tries minimum per object before calling it good. Demo-ready = 4 of 5 on target.
- Change one thing at a time. If the lead edits `config.py`, start a fresh batch of 5.
- Note anything odd in the prompt: "slipped", "hit the tray edge", "grabbed the wrong side".
- Same lighting as the demo; don't leave anything extra on the table.

## Tell the lead immediately if
- the arm moves somewhere unexpected, or the planner refuses a move it did before
- `objects` gives different answers on 3 runs without anything moving (more than 1 cm)
- a grab "succeeds" with nothing in the gripper
