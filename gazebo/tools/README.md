# gazebo/tools — the measurement harnesses that survived 2026-09-03

Kept from the OPEN-24 investigation (ISSUES.md, CLOSED) because the
*method* is the deliverable; the trotting "regression" it chased was a
block artefact.

- `bisect_point.sh <sha> [N]` — one bisect point: checks `<sha>` out in a
  persistent worktree (`/tmp/bisect_wt`), incremental-builds
  `mit_ctrl_sim` with the **documented configure** (empty build type —
  see SKILL.md rule 5d), deploys it through `deploy_host.sh DEPLOY_SRC=`
  (fresh inode, re-sign, proves the control loop starts), runs N
  `trotting@2.5` flat dashes through the conductor, tallies from
  `mission_runner`'s own `PASS=/FELL=` summary line (the archive is one
  run behind), marks a controller that dies at startup **INVALID** rather
  than counting it, and restores HEAD's binary on exit whatever happens.
  `VARIANT=variants/x.sh` applies a one-line change on top of `<sha>` and
  labels the rows `sha:x`.
- `ab_interleaved.sh [N]` — the comparison that can carry a causal claim:
  two saved binaries (`/tmp/bin_head`, `/tmp/bin_<other>`) alternated
  **every run** through `DEPLOY_SRC`, so both arms share every minute of
  host state. Copy and edit the arm names / gait / speed. SKILL.md rule
  5e: two blocks are not a comparison.
- `variants/` — the four one-line variants tried that night, as examples
  of the mechanism: none was a fix (`noinit` even proved *harmful* on
  walking, 3/6 vs 6/6 interleaved), which is the point.

All of them assume the repo at
`/Users/kfinisterre/Desktop/Cheetah/Cheetah-Software` and the conductor on
`:8420`, like every other script in `gazebo/`.
