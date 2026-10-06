# Potential issues – ENABLE_JERK_ACCELERATION (found while fixing terminal crawl)

Reproduction harness: `tools/jerk_sim/jerk_sim.py`. It is a float32 port of the planner profile math and the `st_prep_buffer()` segment loop.
- `--moves d1,d2,.. feed acc jerk steps_mm ticks [tail_mm]` prints a per-segment log
- `--fixed` runs the patched decel
- `--recalc` injects `st_update_plan_block_parameters()` mid block
- `--nojerk` runs with jerk disabled
- Sweep (no `--moves`) runs in parallel: `--steps=40,80,400,800` (default), `--jerk-only`, `--snapcfg` (list worst snaps). It writes `result_orig.json` / `result_fixed.json` including per-run snaps.
- `snap_compare.py` compares the two result files.

Units inside grbl are mm, mm/min, mm/min², mm/min³ and min.

## Speed-snap characterization (simulator, 4,860 jerk-on runs per variant)
**Metric:** jump in the *executed* step rate between consecutive segments that exceeds `Amax × segment spacing + one step of rate quantization`. With jerk off this flags 0 of 4,860 runs, so it does not trigger on normal quantization.

| | Original | Fixed |
|---|---|---|
| Runs with snaps | 1,064 | 1,498 |
| Block-end snaps (drop to exit speed) | 2,137 | 2,862 |
| Accel-end snaps (`acc_end`, #2, code unchanged) | ~550 | ~550 |
| "Rise" snaps (stepper ahead of plan) | 6 | 6 |
| Typical snap size | p50 2–3 mm/s, p95 ≈ 8 mm/s, max 14 mm/s | p50 2–3 mm/s, p95 ≈ 8 mm/s, max 13 mm/s |
| Snap size vs. feed | p95 ≈ 19 %, max 38 % | p95 ≈ 19 %, max 38 % |

- **Size vs. the per-segment acceleration limit:** worst snap per run in the fixed code is p50 5.6×, p95 64× what one segment of max acceleration allows. Only 7 % are within 1×.
- **Not discrete-step noise:** these snaps are 2–50× larger than the one-segment acceleration limit, so they are worse than plain constant acceleration.
- **Direction:** almost all are drops — the stepper reaches block end too fast. That is #1: the planner budgets too little distance for the S-curve when Δv < programmed rate (triangle profiles, short blocks, collinear deceleration).
- **The fix moved results around:**
  - 308 crawl runs became snap runs.
  - 359 clean runs became snap runs. Checked cases: both versions are late at full deceleration, and the residual speed just crosses the quantization threshold. Example: 40 steps/mm, 8.15 vs 8.5 mm/s.
  - The crawl was hiding many snaps: the old code ran out of speed early instead of arriving late.
- **Rejected tweak:** planning at full jerk when behind, instead of 0.9·J. Snap runs only fell 1,498 → 1,443, while crawls rose 41 → 353 (worst 26 s). Keep `JERK_PLAN_FACTOR` 0.9.
- **Fix direction:** #1 (planner uses the exact S-curve distance) removes the cause of the block-end snaps. #2 removes the accel-end snaps. Re-run `snap_compare.py` afterwards; the target is "rise/drop beyond 1× Amax·dt ≈ 0".

## Low steps/mm (40, 80) results
- No new failure mode; step errors 0.
- **Crawl flag:** 41 of 4,860 fixed runs exceed the >100 ms final-segment threshold. Most (34) are at 40 steps/mm with jerk 50 mm/s³. These are physically correct jerk-limited tails: covering the last 1–2 steps (0.025–0.05 mm) from rest at J = 50 takes `(6·d/J)^(1/3)` ≈ 140–180 ms. Example `-20,-5 1200 100 50 40 1000`: 149 ms for 2 steps. The rest are the #10 REV residual (J = 10000 at 100 ticks/s).
- **Snaps:** 40 steps/mm has fewer snaps (169 of 1,215 runs) than 800 steps/mm (614 of 1,215). Coarse steps raise the quantization threshold and hide smaller snaps.
- The crawl metric should be made jerk-aware (compare the final segment with `(6·d/J)^(1/3)`) before tightening it further.

## FIXED – terminal crawl (for reference)
`grbl/stepper.c`, `Ramp_Decel` jerk branch, now `jerk_decel_target()`.
- **Old behaviour:** deceleration was ramped up or down based on speed alone (`v > exit + a²/2J`) and never checked the remaining distance. The ramp-down step had a floor of `max(a - J·dt, J·dt)`.
- **What went wrong:** whenever the S-curve used less distance than the planner's `(v²-ve²)/(2·a_eff)`, speed reached ~0 while mm remained. The terminal branch `time_var = 2·mm/(v+ve)` then produced a single segment lasting seconds to hours.
- **Typical trigger:** a block that starts in decel with `last_accel` already non-zero, for example the final block of a collinear pair with exit speed 0.
- **Also fixed:** the old first decel segment computed `time_to_jerk = accel_var` (an acceleration, not a time). The value was harmless but dimensionally wrong.

## 1. Planner effective acceleration does not match the stepper S-curve for partial speed changes
`grbl/planner.c` `plan_buffer_line()`, "Calculate effective acceleration over block".
- **What happens:** `a_eff` is derived for a full 0 → `programmed_rate` S-curve. For any Δv < programmed_rate, a real S-curve needs MORE distance than `Δv(v+ve)/(2·a_eff)`, because T(x)/x decreases with x.
- **Effect:** the stepper falls behind the plan. It reaches block end above the planned exit speed, and the terminal branch then snaps `current_speed = exit_speed`, which is a velocity step.
- **Observed:** 40.9 → 33.4 mm/s at the Z-20/Z-5 junction (F3000, 500 mm/s², 1000 mm/s³).
- **Also wrong when** nominal ≠ programmed: feed override >100 % or a clamp to `rapid_rate`. In that case `a_eff` still uses `programmed_rate`.
- **Fix direction:** compute the decel/accel distance from the actual S-curve (Δv, entry accel) in the planner, or pass a per-block distance budget.

## 2. Velocity snap at end of the jerk accel ramp
`stepper.c` `Ramp_Accel`, at `mm_remaining < prep.accelerate_until`.
- **What happens:** `current_speed = maximum_speed` and `last_accel = 0` are forced. Because of #1, the S-curve usually has not reached `maximum_speed` yet, so this is a velocity jump and an instantaneous accel drop. That violates jerk.
- **Related:** the ramp-down floor `max(a - J·dt, J·dt)` keeps accelerating, and nothing checks `current_speed` against `maximum_speed`. Speed can overshoot `maximum_speed` before the ramp ends and then snap down.
- **Fix direction:** use the same distance-solved approach as decel. Solve for the plateau accel that reaches `maximum_speed` with a = 0 exactly at `accelerate_until`.

## 3. Acceleration reset across collinear accel blocks
- **What happens:** the same `last_accel = 0` reset fires at every block end during acceleration (accel-only blocks). Acceleration drops to 0 at each junction and has to ramp again.
- **Effect:** jerk discontinuity, and slower acceleration across short segments (arcs, G64 chains).
- **Fix direction:** carry `last_accel` across when the next block continues accelerating.

## 4. Stale `last_accel` after a mid-block replan (sign confusion)
`st_update_plan_block_parameters()` → profile recomputed with `ramp_type` reset; `prep.last_accel` is kept.
- `last_accel` stores a magnitude whose meaning depends on the ramp:
  - Replanned mid-decel into `Ramp_Accel` (triangle): the deceleration magnitude is reused as a positive acceleration. That is an instant sign flip of up to 2·Amax.
  - Replanned mid-accel into `Ramp_Decel` (decel-only): the acceleration is reused as deceleration.
- **When it happens:** depends on when the next line arrives. The decel side is now distance-corrected, but the jerk step at the switch remains.
- **Fix direction:** store signed acceleration, or reset/negate on ramp-type change. When the replan flips direction, first ramp `a` through 0.

## 5. `last_accel` not cleared at end of block
`stepper.c` terminal branch: `prep.last_accel = 0.0f` is commented out.
- **Effect:** the value leaks into the next block. After a stop (exit 0, e.g. a reversal), the next block starts `Ramp_Accel` with a non-zero `a`, which is a jerk violation at start.
- With the fix, decel landing makes `a` small at exit 0, but it is not guaranteed to be 0.
- **Fix direction:** clear `last_accel` when `exit_speed == 0`; keep it for continuing decel.

## 6. `Ramp_DecelOverride` is not jerk-limited
- **What happens:** it uses constant `pl_block->acceleration` and sets `last_accel = 0` every segment. A feed-override reduction therefore produces second-order decel and then an accel step into cruise/decel.

## 7. Feed hold with jerk
- **What happens:** `decel_dist` and the hold `exit_speed` use `a_eff` (`inv_2_accel`).
  - Hold now stops at `mm_complete` exactly (distance-tracked).
  - A hold issued mid-accel starts decel with `last_accel` = the accelerating magnitude (see #4).
  - In the "decel through entire block" case, the exit speed handed to the next block is from `a_eff`, so it may not match the S-curve (see #1).
- The experimental `fast_hold` path picks `current_speed` from a buffered segment but leaves `last_accel` unchanged.

## 8. Stale profile fields in the profile classification
`st_prep_buffer()` profile section.
- **What happens:** for Decel-only and Accel-only types, `prep.decelerate_after` and `prep.maximum_speed` are not set and keep the previous block's values.
- **Effect:** `Ramp_Accel` end uses `mm_remaining == prep.decelerate_after` (float equality on a stale value) to choose Decel vs Cruise.

## 9. Second-order (jerk OFF) terminal residual – pre-existing
- **Repro:** `--nojerk --moves -20,-5 1200 10 50 400 250`
- **What happens:** the final segment enters the terminal branch with v ≈ 0 and about 5e-6 mm left. That is 1 step taking 15 s (`2·mm/(v+0)`).
- **Cause:** float residual between `mm_remaining` and the step grid when speed reaches exactly 0 one segment early.
- Not caused by jerk; it is present in the original code.

## 10. Coarse segment rate vs. high jerk
- **What happens:** with J = 10000 mm/s³ at 100 segments/s, the whole jerk-out spans only about 4 segments. The fixed controller can then leave a ~0.008 mm (about 3 steps) residual and a ~150 ms final segment.
- **Fix direction:** raise `ACCELERATION_TICKS_PER_SECOND` when using high jerk, or sub-step the final jerk-out.

## 11. Path blending arcs and jerk
`planner.c`: for arcs, `max_acceleration` is capped to `min_accel·0.5` but `jerk` is not scaled.
- `a_eff` is computed from the capped value, so `time_to_max_accel` shrinks and the profile becomes steeper than intended.

## 12. CPU cost of the fix
- `jerk_decel_target()` runs up to 20 bisection iterations (≈ 40 flops each) per decel segment. That is fine on H7, but worth checking on FPU-less targets or at very high `ACCELERATION_TICKS_PER_SECOND`.
- A closed-form or Newton solve would cut this.

# Potential issues – other

## 13. `G64 P!` stops motion and the controller stops replying (no hard fault)
- **Observed:** sending `G64 P!` immediately stopped motion. After that, the controller no longer replied to commands. It did not hard fault.
- **What the parser sees:** `!` is the legacy feed-hold realtime character. `protocol.c` removes it and raises `EXEC_FEED_HOLD`, so the parser receives `G64P`. `P` has no value, so the generic word reader rejects the block (`ngc_read_real_value()` → `Status_ExpressionSyntaxError`) before any `G64` code runs.
- **Motion stopping is expected**, since a feed hold was requested. Losing replies is not.
- **Suspects:**
  - **Flush during hold:** the rejected block's error cleanup (`gc_at_exit()`) flushes a held `G64` move through `mc_line()`. With the controller in HOLD and the planner full, `mc_line()` may wait for buffer space that never frees, so the parser stays blocked.
  - **Hold never released:** HOLD waits for cycle start (`~`). Check whether `~` recovers it and whether `?` still gets a status reply.
  - **Error state blocks later lines:** with `COMPATIBILITY_LEVEL 0`, `protocol.c` only runs later g-code while `gc_state.last_error` is OK (or tool-change pending). Check how a parse error interacts with this.
- **Isolation tests:**
  - `G64 P` with no `!`
  - `G4 P`
  - `!` alone while idle
  - `G64 P!` right after reset, with no prior moves
  - `G64 P!` during a `G64` move stream
- **Note:** this is likely not specific to `G64`. Any line containing `!` triggers a feed hold, and any word with no value takes the same error path.
