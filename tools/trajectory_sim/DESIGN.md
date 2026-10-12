# Trajectory Simulator Design Notes

Living design record for `tools/trajectory_sim/`. Read this before changing any layer.
Project plan and milestone gates: [Trajectory_Project_Plan.md](../../Trajectory_Project_Plan.md).
Architecture rationale: [grblHAL_G64_Jerk_Limited_Trajectory_Architecture.md](../../grblHAL_G64_Jerk_Limited_Trajectory_Architecture.md).

## 0. HANDOFF STATUS (2026-10-10) - read first when resuming

M1-M4 are done (M4 gate passed 2026-10-10; evidence in the plan). M4 = section 3C: L3 `geometry.py`,
L4 `constraints.py`, per-block limits in L5, `revise` in L6, double-precision validators with
`validate_envelope` / `validate_axes`, machine spectrum in `tests/machines.py`. Next: M5 (axis/event model,
(v, a)-dependent envelopes, markers' lifecycle, executor) or the open items in section 5. Requirements
added by the user this session: section 1A (check mode), no Decimal in validation.

STATE (2026-10-11) - the interruption items are resolved:
- Full suite re-verified: 101 tests pass, 793 s (plan evidence line updated).
- `tests/optimal_reference.py` validated (section 3D). `tests/bench_optimal.py` benchmarks simple corners
  against it; results and the gap decomposition in section 3D. Next: step 4 design (discuss with the user
  before implementing): best blend size <= P from a nested ladder, signed junction accelerations, and the
  L4 envelope, which the benchmark shows is the largest loss.
- User expectations recorded 2026-10-10, status:
  * Time non-increasing in P for simple corners and strictly below G61: non-increasing holds on all six
    simple corners on all five machines (bench_optimal); "strictly below G61" fails wherever we keep the stop
    (e.g. standard 90 deg P <= 1). Physics: on S-curve machines at P <= 0.2 and >= 90 deg even the optimal
    traversal of our full-size blend is 0.5-3% SLOWER than G61, so the stop is right there; strictly below
    G61 needs a different blend size or shape, not only better timing. Program-level violation still open:
    bench_ruckig "random 0" P1.0 1.191 s > P0.2 1.188 s.
  * Too many exact-stop fallbacks at small P (streamed programs: 126 stops vs 36 blends).
- Firmware characterisation (2026-10-11, user: one code base, no firmware changes yet): test-only
  tests/firmware_model.py (current H743 planner, $11 = 0, jerk off) and tests/bench_firmware.py; proposals in
  Firmware_Planner_Modifications.md (FW-1 per-axis arc limits, FW-2 acceleration-circle arc law). Finding: on
  constant-acceleration machines the firmware with FW-1 + FW-2 adaptive (k_max 1) beats our current pipeline on
  every benchmark program (P1 vs exact stop: 90 deg -6.7% vs ours -5.8%, zigzag -11.4% vs -8.6%, square -8.7% vs
  -7.1%, chords -80.4% vs -74.2%): tangent arcs + the exact circle bound beat clothoids + our sum-bound L4 there.
- FIRMWARE CHANGED (2026-10-11, user-approved for machine testing): FW-1 + FW-2 adaptive implemented in
  grbl/planner.c, planner.h, stepper.c, config.h (PATH_BLEND_ARC_KMAX 0.9); Waveshare build has
  ENABLE_JERK_ACCELERATION=0. Details and status: Firmware_Planner_Modifications.md "Implementation". Host checks:
  tests/test_firmware_model.py, tests/test_firmware_stepper_law.py; reference check tests/test_optimal_reference.py.
- Open decisions (user to confirm): (1) coordinated XYZ axes symmetric acceleration (A = D) to simplify
  L4/validation/M5 envelopes and Ruckig phase sync, keeping asymmetric A/D only in the 1-DoF law for
  independent (pump) axes; policy if a config sets D != A on a coordinated axis (min / reject / use A).
  (2) Pump axes planned fully independently (own 1-DoF queue, synchronized to path events only)?
- Performance finding: one 29-move G64 chord program (0.88 s of motion) took ~57 s wall: 85 full-queue L5
  plans (29 revise + 56 front-end previews), 14,446 `best_peak` golden-section searches, ~1M profile
  evaluations. Fixes proposed (not started): incremental lookahead (re-plan only the tail whose backward
  bounds change), previews only over the differing suffix, analytic best_peak candidates, fast/long test tiers.
- Ruckig (0.19.4) cannot do G64-style blends (no path following/tolerance); it is the reference only for L2,
  straight lines and the G61 baseline. LinuxCNC (WSL/VM sim) and the legacy grblHAL sim (tools/jerk_sim) are
  candidate external G64 references, not started.
- Throwaway diagnostic scripts live in %TEMP%\traj_diag (not in the repo).

M4 handoff facts:
- Full suite: 98 tests, ~760 s. The L8 front end previews two full-queue plans per blendable corner
  (O(n^2) planning per program); test_pipeline dominates. Optimisation is an open item (section 5).
- G64 <= G61 is pinned by `test_g64_never_slower_than_g61` (worst ratio 1.000 on all five machines) and
  `test_axis_aligned_right_angle_g64_beats_g61` (P 3 mm: 0.555 vs 0.660 s on the standard machine).
- Pipeline harness (`tests/test_pipeline.Pipeline`): propose -> project -> `runtime.revise`; on rejection
  re-propose with blend=False (exact stop, always accepted). In firmware this glue belongs to the layer
  that owns the queue; keep the accept-only-if-a-witness-survives rule.
- Stops are only ever added at a revision boundary (a cleared G61 stop was the worst bug found).
- Diagnostic approach that worked: reproduce a failing seed in a temp script, print pieces/blocks and the
  elements around the worst window, then decide whether planner or oracle is wrong (both happened).

M3 resolution (seed 6006 case 3 in tests/test_runtime.py), kept for reference:

- Root cause 1 (L5): blocks 5-6 arrived while the tail was decelerating toward the old queue end. Their new
  cap boundary forces a = 0 there, and sampled need(w) over [0, b[1]] was everywhere above region 0's
  length (hump-shaped, not V-shaped), so no a = 0 exit existed. A direct monotone stop to the new queue end
  was feasible. Fix: coarsening fallback (3B item 4a). `_first_exit` now uses the analytic basins
  {bound, v*, 0} instead of a ternary search that assumed unimodality.
- Root cause 2 (L5): an override of 0.469 then arrived where reaching the new cap before the queue end was
  impossible. Fix: the transient falls back to a stop and plans from rest (3B item 7).
- Root cause 3 (L6): `override_log` recorded the request position even when that replan failed, so the old
  suffix seemed to break the new caps. It now logs where a plan honouring the override is installed.
- Not the ternary search missing an interval, and no later-region bound error.

Other facts not recorded elsewhere:
- Full suite takes ~125 s (M4); the slow parts are the pipeline and lookahead/runtime random suites
  (bisection over `minimum_distance`, which runs a golden-section search). Optimise only after correctness.
- Ruckig tests (test_scalar_time_law.py) are structured as: `ruckig_optimum()` helper (exact speed-extrema
  filter); generic contract = never faster than Ruckig; analytic-specific `test_peak_side_solutions_are_time_optimal`
  (173 peak-side cases match Ruckig to 4.4e-16; 1 valley-side case is feasible but 32% slower - expected,
  that family is not optimal); the analytic `test_known_family_gaps` pins each case exactly (0: OK valley
  side, 1.01502x Ruckig; 1: INSUFFICIENT_DISTANCE, family minimum 5.03504 mm > 3.5709 mm; 2: OK valley
  side, 1.17935x). The contract keeps only the generic checks (`test_known_family_gap_cases_are_consistent`).
- Lookahead: replans from moving states 300/300 OK, after queue extension (+ override) 300/300 OK.
- L5 tolerance rule: accept exact fits within SLACK (1e-9 mm), but bisection searches use the strict predicate
  need(w) <= length, otherwise they converge onto L2's acceptance edge and fail by rounding.
- Hold is validated by the test harness with jerk = inf for `hold` policy elements only; normal elements must
  be Phases with |j| <= J (test_runtime.certify).

Carried forward from M3: thousands-of-cycles soak belongs to M6 (current soak: 40 collinear Z cycles per
order); the L5 completeness of the a = 0 boundary family with coarsening is measured, not proven.

## 1. Working rules

- Units everywhere: mm, s, mm/s, mm/s^2, mm/s^3. grblHAL minute units convert only via `State.from_grbl` / `Limits.from_grbl`.
- Reference arithmetic is float64. float32 is a separate study path (`numerics.py`), never silently mixed in.
- Never clamp, snap, or reset state to make a case pass. Infeasibility is returned as an explicit status with a witness.
- Validation (`validation.py`) is independent of production code: it imports only `model`. It runs in double
  with explicit forward rounding bounds (no Decimal: firmware has none), so the same checks can be ported.
- Tests must not be bent to fit an implementation. Fixtures come from hand derivation, Decimal (test-side
  oracle only), brute force, independent integration, or Ruckig. Oracle tolerances must come from a stated
  error model (truncation, rounding, declared solver slack), never from observed failures.
- Run pipeline scenarios against the whole machine spectrum (`tests/machines.py`), not one machine.

## 1A. Firmware requirements (design inputs, from the user)

- **Check mode** (2026-10-10). The target has little flash, RAM and output bandwidth, so extensive on-device
  validation or post-test analysis is not possible. The firmware must provide a check mode that runs G-code
  test lines through every layer (parser, geometry, constraint projection, lookahead, runtime/commit) except
  step generation and output, and streams a series of lines and arcs with enough metadata for client-side
  test code to visualize the planned geometry and analyze the planned kinematics for correctness.
  Implications for every layer contract:
  - Layer outputs must be compact, serializable records, emitted incrementally in commit order (no whole-job
    buffering); the executor (L7) is the only layer bypassed. Output must be deterministic for a given input.
  - Per geometry piece: kind, endpoints/centre/plane or blend parameters, s range, source id, feed, stop flag,
    corner decision with certified deviation, derivative bounds and the projected path limits.
  - Per committed time-law element: (j, duration) plus boundary states (s, v, a), policy tag (normal/hold),
    override in force and transient windows, so the client can re-integrate and re-check limits.
  - Open: G64 blends are clothoid pairs, not lines or arcs. Either the record set gains a clothoid record
    (origin, t0, n0, sharpness, length, direction; 9-10 numbers) or blends are reported as a certified
    line/arc approximation with an error bound. Decide before M5/M7; the client checker can then reuse
    this repository's validators (`validate`, `validate_envelope`, `validate_axes`).
  - On-device checks are limited to the cheap invariants already in the runtime (stop witness, status codes);
    the heavy certification runs on the client.
- **G64 never slower than G61** (2026-10-10): with the same acceleration and jerk limits, a run with G64 P
  (any P) must not take longer than the same program with exact stops. P is a permission to deviate, not a
  mandate to blend: a stop at the vertex has zero deviation, so a corner may always fall back to it.
  Status: met since the throughput fixes (section 3C); pinned by `test_g64_never_slower_than_g61`.

## 2. Layers and contracts

Source of truth: `LAYERS` in [contracts.py](contracts.py). [tests/test_architecture.py](tests/test_architecture.py) enforces it:
every module is registered in exactly one layer, imports only the sibling modules its layer lists, and
imports only its listed third-party packages. Lower layers never import higher ones.

| Layer | Modules | May import | Contract (inputs -> outputs) | Status |
|---|---|---|---|---|
| L0 data model | `model` | - | Immutable records: `State(s,v,a)`, `Limits(speed,acceleration,jerk,deceleration=None->acceleration)` (`jerk=inf` = no jerk limit), `Phase(jerk,duration)`, `AccelerationStep(acceleration)` (instantaneous a change), `Element = Phase \| AccelerationStep`, `Segment`, `Marker`, `Trajectory`; `PathBlock(source_id, length, speed_cap, stop_after, limits=None)`; source moves `LineMove`, `ArcMove` (`Move`), `Vector`. Rejects non-finite values and negative durations; no kinematic feasibility checks. | M1 done; asymmetric decel + steps M2; blocks M3; moves + block limits M4 |
| C contracts | `contracts` | model | `LAYERS` registry, `SolveStatus`, `SolveResult`, Protocols `ScalarTimeLaw` (L2), `Lookahead` + `PlanResult` (L5), `PathPiece` / `PathGeometry` + `DerivativeBounds`, `Corner`, `Revision` (L3), `ConstraintProjection` (L4). Higher layers depend on these types, not on concrete implementations. | M2-M4 |
| V validation | `validation` | model | Double precision with explicit rounding bounds (no Decimal). `validate(trajectory, limits, expected_final, tolerance)`: continuity, declared dynamics, endpoint, full-interval 0<=v<=V, -D<=a<=A, abs(j)<=J; steps are `jerk_limit` issues unless jerk is inf. `validate_caps` (speed caps over s intervals), `validate_envelope` (per-interval path limits, hold steps waivable), `validate_axes` (per-axis limits from derivative bounds, section 3C). | M1; double + envelope/axes M4 |
| L1 kinematics | `phases`, `numerics` | model (+numpy for numerics) | `advance`, `trajectory`, `critical_times`, `extrema` (float64, exact; steps keep s and v, extrema include the pre-step state). `numerics`: float32 propagation mirror for precision studies. | M1 done; numerics M2 |
| L2 scalar time law | `scalar_solver` | model, contracts, phases | `AnalyticTimeLaw` implements `ScalarTimeLaw` (section 3). | M2 |
| L3 path geometry | `geometry` | model, contracts | `GeometryStream` (PathGeometry): lines, planar arcs, clothoid-pair G64 blends in arc length with q', q'', q''' and certified bounds; corner decisions with deviation certificates; markers; streaming revisions (section 3C). | M4 |
| L4 constraint projection | `constraints` | model, contracts, phases, geometry | `ConservativeProjection` (ConstraintProjection): constant path limits per piece from derivative bounds and axis limits; `blocks()` -> PathBlocks. (v,a)-dependent envelopes later. | M4 |
| L5 lookahead | `lookahead` | model, contracts, phases, scalar_solver, constraints | `ConservativeLookahead`: regions with per-block limits; commit only plans ending at rest (section 3B). | M3; block limits M4 |
| L6 runtime | `runtime` | model, contracts, phases, lookahead | `StreamingRuntime`: commit horizon, override, hold/resume, geometry `revise`. Markers' lifecycle M5. | M3; revise M4 |
| L7 executor | `executor` | model, contracts, phases, geometry (+numpy) | Committed phases -> axis steps (ideal + firmware-like timer/AMASS). | planned M5 |

To add a layer implementation: register it in `LAYERS`, implement the layer's Protocol (add the Protocol to
`contracts.py` when the layer starts), and run the shared contract test suite for that layer against it.
Alternative implementations (float32 port, Ruckig-backed oracle, future C port via bindings) must pass the same suite.

### `SolveResult` semantics (L2 output)

- `status`: `OK`, `INVALID_INPUT`, `SPEED_LIMIT`, `REVERSE_PROGRESS`, `INSUFFICIENT_DISTANCE`, `NO_CONVERGENCE`.
- `phases` start at `start`. On `SPEED_LIMIT`, `REVERSE_PROGRESS`, `INSUFFICIENT_DISTANCE` the phases are a
  witness profile that demonstrates the failure, so callers and tests can verify the claim independently.
- `iterations`: bisection steps used (bounded by `MAX_BISECTIONS`).
- `proven_infeasible`: True only when no admissible profile exists (section 3.2 boundary conditions);
  False on a failure means only this implementation's family failed. L5 must treat the two differently:
  a proven failure is a planning error upstream; an unproven one may be retried with another method.
- Postconditions on `OK` (verified by `validation.validate`): target reached within slack; 0 <= v <= V,
  -D <= a <= A, abs(j) <= J over every full phase; phases are continuous by construction (propagated).

## 3. L2 scalar time law design (M2)

### 3.1 `acceleration_profile(a0, a1, dv, limits)` - minimum-time speed change

Changes speed by `dv` while moving acceleration from `a0` to `a1`. Profile: ramp (|j| = J), optional
plateau (j = 0, a = +A or -D), ramp (|j| = J). This one-switch family is the time-optimal solution for this
problem with jerk and (asymmetric) acceleration limits.

- Speed change of the direct ramp a0 -> a1: `dv_direct = (a0 + a1) * |a1 - a0| / (2J)`.
- `dv > dv_direct`: peak above both endpoints (sign +1). `dv < dv_direct`: peak below both (sign -1).
  `dv == dv_direct`: single direct ramp.
- Area of a ramp from x to p is `(p^2 - x^2) / (2J)` (sign-correct for any x, p). So without a plateau
  `dv = (2p^2 - a0^2 - a1^2) / (2J)` and `p = sign * sqrt(sign*J*dv + (a0^2 + a1^2)/2)`.
- Plateau cap: `cap = A` for sign +1, `D` for sign -1. If `|p| > cap`: `p = sign*cap`, plateau
  `t2 = (dv - sign*(2cap^2 - a0^2 - a1^2)/(2J)) / p`.
- Durations: `|p - a0|/J`, `t2`, `|p - a1|/J`; zero-length phases are dropped.
- `|dv - dv_direct| <= 64*eps*(V + |dv_direct| + 1)` (pure float rounding, ~1e-12 mm/s at 100 mm/s) is
  treated as the direct ramp: ramp durations grow like sqrt(dv/J), so rounding in a propagated boundary
  speed would otherwise become a ~30 ns detour. Do NOT widen this to a tolerance like SPEED_SLACK: an
  absorbed dv is an unmodelled speed step, and `best_peak`'s optimiser found and exploited a 1e-9 mm/s
  slack (2.6e-8 mm of "free" distance that the region solve could not reproduce).
- Discontinuity to expect: with a0, a1 both negative and dv just above dv_direct, the optimal profile
  detours through a positive peak near |a1| (duration jumps). This is physically correct, not a bug.
- Unlimited jerk (`J = inf`, second-order/trapezoidal motion): the same formulas reduce exactly -
  dv_direct = 0, the peak clamps to the cap, plateau = dv/peak, ramps have zero duration. Ramps are
  emitted as `AccelerationStep`s; `_compact` drops empty phases and merges adjacent steps (only the
  last value of back-to-back steps is ever observed).

### 3.2 `velocity_transition(start, v1, a1, limits)` and `stop(start, limits)`

- Rejects (INVALID_INPUT) boundaries outside 0 <= v <= V or -D <= a <= A.
- Speed bounds are evaluated over the full profile with `phases.extrema`; violations return
  `REVERSE_PROGRESS` / `SPEED_LIMIT` with the profile as witness. Nothing is clamped.
- `stop` = transition to (v=0, a=0). It is minimum time and is used as the braking reserve.
  Analytic feasibility boundaries (independent test oracles, and the basis of `proven_infeasible`).
  Acceleration needs at least a^2/(2J) of speed change to reach zero at jerk <= J, so for any target:
  - REVERSE_PROGRESS is forced if `a0 < 0 and v0 < a0^2/(2J)`, or (goal side) `a1 > 0 and v1 < a1^2/(2J)`.
  - SPEED_LIMIT is forced if `a0 > 0 and v0 + a0^2/(2J) > V`, or (goal side) `a1 < 0 and v1 + a1^2/(2J) > V`.
  For `stop` these are exact (iff); for other targets they are sufficient, not necessary.
- Inherent overshoot: ending with a1 < 0 at v1 implies v > v1 + a1^2/(2J) just before the end, so some
  velocity targets are infeasible near V. The status reports it; no slower profile can avoid it.

### 3.3 `distance_transition(start, goal, limits)` - conservative position-constrained move

Family ("cruise at zero acceleration"): `acceleration_profile(a0 -> 0, v0 to c)`, cruise at `c`,
`acceleration_profile(0 -> a1, c to v1)`. Exposed as `peak_profile(...)` for later layers. `c` is a cruise
speed, not necessarily a peak: below v* = v0 + a0|a0|/(2J) the start half takes its monotone (no
interior a = 0) branch, which replanning from mid-deceleration needs.

- Domain: `c in [c_low, V]` with `c_low = max(0, v1 - a1|a1|/(2J))` (goal half must be a single ramp).
  Travelled distance is NOT monotone in c (the M2 claim was wrong once the domain went below v*; tests
  found it). `best_peak` minimises it: golden-section search plus candidates {c_low, V, v*, v0, v1}.
  Unimodality is an assumption, checked by `test_best_peak_is_family_minimum` (201 samples per case).
- `minimum_distance` = distance at `best_peak` (the L5 feasibility oracle).
- History: the original bound also had v0 + a0|a0|/(2J) and forced a back to 0 first; that wasted
  distance and failed 23% of replans from moving states.
- If `c_low > V`: SPEED_LIMIT, proven only when section 3.2's forced conditions hold. If distance at
  `best_peak` exceeds the request: INSUFFICIENT_DISTANCE (witness = that profile; never proven).
- If distance at `best_peak` is within `DISTANCE_SLACK` of the target, c = `best_peak` (durations grow
  like sqrt(dc) there, so a rounding-level surplus would otherwise create microsecond detours). If
  distance at V fits: c = V plus cruise. Otherwise bisection on [best_peak, V] keeping
  `travelled(low) <= D < travelled(high)`; correctness needs only continuity (intermediate value
  theorem), run to machine precision (bounded by `MAX_BISECTIONS`). The residual distance becomes a
  cruise only if it exceeds `DISTANCE_SLACK`; otherwise it is rounding and no picosecond phase is emitted.
- Complete only for v0 == v1 with zero boundary accelerations (any D > 0, including rest-to-rest).
  Otherwise conservative: it may report INSUFFICIENT_DISTANCE where another topology fits (for example
  an optimal profile that dips below v1 and recovers uses less distance). Not time-optimal (acceleration
  is forced through zero at the peak). The Ruckig comparison test measures both gaps; M3 decides whether
  an optimal family is needed.
- Measured (M2, seed 7117, 600 random transitions): wherever this family succeeds its duration equals
  Ruckig's time-optimal duration (max relative gap 5.3e-12). Reason: every unimodal speed profile passes
  through a = 0 at its maximum, so forcing a = 0 at the peak costs nothing there. Coverage gap: 3/600 cases
  where Ruckig has a valid 0 <= v <= V profile and this family reports INSUFFICIENT_DISTANCE - all with
  nonzero goal acceleration, whose optimum is a valley profile (speed dips between v0 and v1) or a
  monotone profile with no a = 0 point. Pinned in `test_known_family_gaps_are_reported`; closing the gap
  needs a "valley" family (minimum speed with a = 0) and a no-interior-zero family.
- Re-measured with asymmetric limits (D/A from 0.05 to 3): max gap 7.3e-12 over 156 common cases,
  2/600 coverage gaps, and `proven_infeasible` never contradicts an admissible Ruckig profile.
- Re-measured with 20% unlimited-jerk cases and machine-precision bisection: max gap 4.4e-16 over 173
  common cases, 3/600 coverage gaps. Ruckig supports `max_jerk = inf`, so it is the oracle for both orders.

### 3.4 Tolerances

`SPEED_SLACK = 1e-9` (mm/s and mm/s^2) and `DISTANCE_SLACK = 1e-9` mm in `scalar_solver.py`. They only
absorb float64 rounding; they never move a state.

## 3B. M3 straight-path lookahead (L5) and streaming runtime (L6)

### Inputs and contracts

- L0 `PathBlock(source_id, length, speed_cap, stop_after=False)`: one source move on a straight path
  (path coordinate s is monotone; reversals, exact stops and dwells are `stop_after=True`, forcing v = a = 0
  at its end). Blocks passed to a planner begin at `start.s`; the runtime trims a partly executed block.
- `ScalarTimeLaw.minimum_distance(start, v1, a1, limits) -> float | None`: distance the implementation
  needs to reach (v1, a1); None when its family cannot. Lets L5 stay implementation-agnostic.
- `Lookahead.plan(start, blocks, limits, override) -> PlanResult(status, start, elements,
  boundary_speeds, transient_end, detail)`. On OK the elements: start at `start`, end at rest exactly at
  the end of the last block, respect each block's cap (min(speed_cap*override, limits.speed)) except
  inside an override transient ending at `transient_end`, and obey A/D/J. Source boundaries inside a
  region of equal cap are not visible to the profile (no resets).

### L5 `ConservativeLookahead` algorithm

1. Regions: merge consecutive blocks with equal effective cap and no stop between them.
2. Boundary speed caps `bmax[k] = min(cap[k-1], cap[k])` (0 after a stop and at the queue end).
3. Backward pass: `b[k] = min(bmax[k], largest u <= cap[k] with D(u, b[k+1]) <= L[k])`.
4. Forward pass: region 0 starts from the committed state (v0, a0 possibly nonzero); its exit speed is
   the largest w <= b[1] with `law.minimum_distance(start, w, 0) <= L[0]`. need(w) is NOT V-shaped: it
   rises above v* = v0 + a0|a0|/(2J), and on [0, v*] it is hump-shaped (decelerating further first costs
   distance, then w -> 0 approaches the stopping distance), so its local minima are w = v* and w = 0.
   `_first_exit` tries bound, then the v* basin, then the w = 0 basin, each followed by bisection (each basin
   is a feasible prefix once the higher ones failed). Checked against 81-point brute-force sampling
   (`test_first_exit_is_largest_sampled_feasible_speed`). The earlier ternary search assumed unimodality.
   Later regions: `e[k+1] = min(b[k+1], largest w <= cap[k] with D(e[k], w) <= L[k])`.
4a. Coarsening fallback: if region 0 has no admissible a = 0 exit, merge it with region 1 at the lower cap
   (never across an exact stop, never to a cap below the state) and repeat. Needed because a = 0 at cap
   boundaries is not preserved under queue extension: a state decelerating toward the old queue end may
   have no a = 0 exit once new blocks with different caps add boundaries, although a monotone stop at the
   new queue end exists (root cause of seed 6006 case 3, see section 0).
5. Each region: `law.distance_transition((s_k, e[k], 0) -> (s_k+1, e[k+1], 0))` with the region's cap;
   region 0 starts from the propagated actual state. Boundaries are crossed with a = 0 (conservative,
   not time-optimal at cap changes; equal-cap blocks are merged so source boundaries cost nothing).
6. `D(u, w)`: zero-end-acceleration speed change distance = (u + w)/2 * T(|w - u|), with
   T = 2 sqrt(dv/J) if dv <= C^2/J else dv/C + C/J, C = A when speeding up, D when slowing; T = dv/C when
   J = inf. Physics of any symmetric time-optimal profile; tests cross-check it against L2 propagation.
7. Override transient: if the start speed (or v0 + a0^2/(2J) when a0 > 0) exceeds region 0's cap, L2
   first decelerates to the lowest cap met during the transient (speed cap lifted only for that
   transient), and `transient_end` reports where caps apply again. If the rest of the queue is then too
   short, the transient is a stop instead (L2 minimum-time `stop`; not proven to be the shortest stop) and
   planning resumes from rest; `transient_end` is the stop position.
8. Correctness never relies on the monotonicity/V-shape arguments alone: every region solve is checked by
   L2 status, and tests certify plans with the independent validator plus `validate_caps`.

### L6 `StreamingRuntime`

- Owns: received blocks, committed elements (immutable, with policy tags `normal` / `hold`), the planned
  suffix from the committed tail, override, hold flag. Time base: execution time t; the commit horizon is
  t + `commit_latency` (models segment buffer latency). Idle at rest is committed as zero-speed phases.
- Invariant checked after every commit/replan: the suffix from the tail ends at rest at or before the end
  of the known queue. Since every plan ends with a stop, any prefix of it keeps a valid stop witness.
- Replan on events (block arrival, override change, resume). A failed replan keeps the previous suffix
  (still a valid stop), is recorded in `failures`, and sets `replan_pending`, which retries after every
  commit step until a plan succeeds or nothing remains.
- Override: requested at any time; it takes effect at the tail where a plan honouring it is installed, and
  `override_log` records that position (not the request position). Until then the old suffix, planned
  under the old caps, is the committed motion. Fixed 2026-10-10: the log used to claim the request tail
  even when that replan failed, so the old suffix appeared to violate the new caps.
- Hold: from the committed tail (latency accounted), acceleration-limited stop = L2 `stop` with
  `jerk = inf` (step to -D, plateau, step to 0). Jerk continuity is waived only for these `hold` elements.
  Resume replans from the tail.

## 3C. M4 geometry (L3), constraint projection (L4) and their integration

Decisions (user, 2026-10-10): blend family = clothoid pair; joins with a curvature jump (line-arc,
arc-line, arc-arc not on one circle) are certified exact stops - transition curves belong above the
velocity layer (CAM / client); per-axis dynamics in M4 = conservative constant limits per piece.

### Pieces (all parameterised by arc length u, |q'| = 1)

- `LinePiece`: q' = d; q'' = q''' = 0. Bounds |d_i|.
- `ArcPiece` (G17/18/19 plane = normal axis k; in-plane axes (k+1, k+2) mod 3, ccw about +k): angle
  theta = phi0 + sign u / r; q'' = -radial / r, q''' = -q' / r^2. Bounds are exact sinusoid maxima over the
  swept angle. Inconsistent radii: the centre is refitted onto the chord bisector if it moves <= 0.005 mm
  (grbl's arc tolerance), else INVALID_INPUT. Helical arcs: INVALID_INPUT (not yet supported).
- `ClothoidPiece`: heading psi = c sigma^2 / 2, curvature c sigma; q' = T, q'' = c sigma N, q''' = c N - (c sigma)^2 T.
  Position (X, Y) = sigma * sum_n (i x)^n / (n! (2n+1)), x = c sigma^2 / 2 <= pi/2: power series, truncated
  at 1e-18 relative. `reverse` traverses it from sigma = L back to the origin. Bounds: exact sinusoid maxima
  of T_i and N_i over psi in [0, turn]; |q''_i| <= kmax max|N_i|, |q'''_i| <= c max|N_i| + kmax^2 max|T_i|.

### G64 corner blend (line-line only)

Deflection D = angle(t_in, t_out) in (0, pi). Two mirrored clothoids of half length L each turn D/2
(c = D / L^2); half 1 starts at p_in = V - T t_in, half 2 is the reflection of half 1 across the bisector,
traversed backward from p_out = V + T t_out. With Cs + i Sn = integral_0^1 exp(i D w^2 / 2) dw:
  trim T = L (Cs + Sn tan(D/2)),   Hausdorff deviation <= L Sn / cos(D/2).
Proof of the deviation bound (the certificate, not a radius formula): headings stay in [0, D] with D < pi,
so the blend is convex and on half 1 the distance to the incoming line Y(sigma) increases monotonically to
L Sn at the midpoint, whose foot lies on the incoming segment (x = -L Sn tan(D/2) < 0); by symmetry the
same holds for half 2. Polyline points: those projecting onto half 1 are within Y <= L Sn of it; the rest
of the replaced segment lies between that point and V, so (convexity of distance along a segment) within
max(L Sn, |V - mid|) of the midpoint; |V - mid| = L Sn / cos(D/2). Series error (1e-13 relative, generous)
is added to Sn. L = min(P / deviation_ratio, max_trim / trim_ratio) with
max_trim = min(incoming line remaining, outgoing line / 2, vertex_s - committed_s).
Exact stop when D >= pi - 1e-9 (reversal), L < 1e-9 mm, P is None (G61), or the runtime rejects the blend.
"Continuous" (no blend, no stop) when tangent and curvature vectors match within 1e-9 (collinear lines,
arcs continuing one circle). Zero-length moves add no geometry; a G61 among them stays in force.
P semantics: Hausdorff distance between the blend and the polyline it replaces (two-sided; stricter than
LinuxCNC's one-sided "path within P" by 1/cos(D/2)).

### Streaming and markers

`propose(move, committed_s, blend)` is pure and returns a `Revision(from_s, stop_at_from, pieces, stops,
corner)`; `accept` applies it. Only path after `committed_s` may change: a blend's p_in must lie ahead of
the committed tail. The runtime's `revise(from_s, blocks, stop_at_from)` accepts a revision only if the
current stop witness ends by from_s (then it simply replans) or a plan through the new geometry succeeds;
otherwise the harness re-proposes with blend=False (exact stop at the vertex; always accepted because the
old witness stops there). Source markers: each move's marker is its end vertex, moved to the blend midpoint
when the corner is blended (zero-length moves share the vertex marker).

### L4 conservative projection

Axis i on a piece with B1 >= |q_i'|, B2 >= |q_i''|, B3 >= |q_i'''|: velocity <= B1 v; acceleration
<= B1 |a| + B2 v^2; jerk <= B1 |j| + 3 B2 v |a| + B3 v^3 (from q' a + q'' v^2 and its time derivative).
- Straight pieces (B2 = B3 = 0): exact: V = min V_i/B1_i, A = min A_i/B1_i, D = min D_i/B1_i, J = min J_i/B1_i
  (axes with jerk = inf, profile 0, impose no jerk limit; all such -> path jerk inf).
- Curved pieces: axis acceleration bounded by min(A_i, D_i) both ways (the sign of the axis speed change is
  not tracked). V = largest speed (bisection; all constraints monotone in V) keeping A(V) and J(V) >=
  CURVED_SHARE (0.5, a throughput policy, not a safety parameter) of the straight-line values, where
  A(V) = min (min(A_i,D_i) - B2 V^2)/B1 and J(V) = min (J_i - 3 B2 V A_line - B3 V^3)/B1.
- Feed is the block's `speed_cap` (scaled by override); `limits.speed` is the hard axis/curvature cap,
  which L5 keeps as the region `ceiling` even inside override transients. L5 regions now carry the block's
  limits (min with the planner's), merge only when all limits match, and transients and the coarsening
  fallback use the lowest limits crossed. Hold uses the lowest deceleration of the blocks its stop crosses
  (stepping |a| down first if already braking harder).

### Validation of the integration (tests/test_pipeline.py, all machines in tests/machines.py)

Path validator (continuity, endpoint), `validate_envelope` per block, `validate_caps` for feed x override
outside transients, `validate_axes` per piece from derivative bounds, deviation <= P per blended corner,
and an independent check without any derivative bound: divided differences of sampled axis positions
q(s(t)) are weighted means of axis velocity/acceleration/jerk (q(s(t)) is C^2 off hold), so they must
respect the axis limits, up to rounding plus L2's declared DISTANCE_SLACK (1e-9 mm).

Boundary attribution in `validate_envelope` / `validate_axes` / `validate_caps`: a phase overlapping a piece
by <= `tolerance.position` is not attributed to it (speed and acceleration are continuous, so the piece's
own phases check the same boundary values); a step is a jerk discontinuity of a piece only strictly inside
it; intervals no longer than the tolerance take everything. Pinned by `test_envelope_boundary_attribution`.

Defects the pipeline found (2026-10-10), all fixed with regression tests:
- `accept` / `revise` overwrote the stop flag at from_s: a blend consuming the whole line after a G61 vertex
  silently removed that exact stop (curvature jump at speed). Stops are now only ever added.
- A rounding-level remainder (7e-15 mm) of a consumed line survived as a micro piece; now consumed.
- Validators attributed a neighbouring phase's jerk / boundary speed to a piece it touched by < 1e-9 mm.
- A G61 stop may land up to DISTANCE_SLACK before the vertex and leave under the next piece's limits; the
  finite-difference oracle now carries that declared position uncertainty.

Known throughput limitation (not a safety issue): on the example machine (X/Y J = 21 mm/s^3, D = 40), a
0.05 mm G64 corner is crossed at ~0.006 mm/s. The constant-per-piece jerk bound charges 3 B2 v |a| with
the straight-line |a| even where a ~ 0 at the apex. M5's (v, a)-dependent envelope (or a lower A share on
curved pieces for such machines) addresses it.

### Throughput finding (2026-10-10): G64 is slower than G61 everywhere

Zigzag in XY, 20 mm legs, feed 100 mm/s, corner deflections 30/60/90/120/150 deg, all moves queued (no
starvation). Total time (and apex speed at the 90 deg corner):

| Machine | G61 | G64 P0.01 | P0.05 | P0.2 | P1.0 |
|---|---|---|---|---|---|
| standard (S-curve) | 1.92 s | 5.78 s (0.07 mm/s) | 5.76 s (0.34) | 5.67 s (1.34) | 5.26 s (6.6) |
| standard_trapezoidal | 1.66 s | 1.70 s (3.1) | 1.74 s (6.8) | 1.79 s (13.6) | 1.82 s (30.5) |
| high_jerk | 1.67 s | 1.83 s | 1.82 s | 1.82 s | 1.83 s |
| example (J = 21) | 18.1 s | 167 s | 167 s | 166 s | 163 s |

Root causes, all in the conservative constant-per-piece projection (L4) plus L5's region rules:
1. Acceleration-jerk coupling: the jerk bound 3 B2 v |a| uses the straight-line |a| (>= 1000 mm/s^2) on
   every curved piece, although |a| ~ 0 near the apex. For P = 0.05 at 90 deg (kappa_max ~ 11 /mm) this caps
   v at ~0.35 mm/s; the physical limits (v^2 kappa <= A, v^3 c <= J) allow ~5-9 mm/s.
2. Constant cap over the whole clothoid: the cap is set by kappa_max at the apex but applies from the blend
   start (kappa = 0), so the machine must already be at apex speed at p_in and creep through the blend.
   Also hurts trapezoidal machines (no coupling term), which is why they are 2-10% slower.
3. No fallback on cost: even a physically optimal tight blend can cost more than a stop (slow traversal of
   the blend length vs braking to zero). The planner must pick the faster of blend and stop per corner.

Proposed fixes (not yet implemented): (a) per curved sub-piece choose the acceleration share that
maximises the reachable speed up to the feed, instead of a fixed CURVED_SHARE; (b) split each clothoid half
into sub-pieces so caps follow kappa(s); (c) choose blend vs exact stop per corner by an estimated
local time cost, with a test asserting G64 total time <= G61 on every machine and program.

### Throughput fixes (implemented 2026-10-10)

(b) Each blend half is emitted as BLEND_SEGMENTS (4) `ClothoidPiece` sub-ranges [sigma_from, sigma_to] of
its clothoid, so each sub-piece's bounds use its own kappa range instead of kappa_max.

(a) `path_limits(bounds, axes, share)`: on curved pieces |a| <= share A_line and |j| <= share J_line (A_line,
J_line = straight-line values for the same tangent bounds); the cap is the largest v satisfying every axis
constraint with that |a| and |j| (the coupling term now uses share A_line, not A_line). `blocks()` picks a
share per curved piece from SHARES = 2^-8 .. 2^-1 (share 1 is excluded: it leaves no budget for the curve,
cap 0): a backward pass gives the highest speed each piece can carry and still brake to what follows, a
forward pass the highest speed it can reach from what precedes (a = 0 at ends, like L5), and each piece
takes the share maximising the smaller of the two. First attempt (escalate share while a piece could not
change speed between its own cap and the next) was wrong: it pushed low-curvature sub-pieces to share 1 and
cap 0 - kept here as a warning.
Result at the axis-aligned 90 deg corner, standard machine, F100: apex 0.34 -> 5.3 mm/s at P 0.05 and
6.6 -> 39 mm/s at P 1.0, i.e. the physical limit of the geometry.

(c) L8 `frontend.MotionFrontEnd`: P permits deviation, it does not mandate a blend. For each blendable
corner it asks the runtime for a non-mutating `preview` plan from the committed tail to the end of the
known queue with the blend and with an exact stop, and keeps the faster; a blend the runtime cannot accept
also becomes a stop. (While holding there is no plan from the moving tail; a rest-to-rest window over
[incoming line start, end of new move] is compared instead.) First attempt used that rest-to-rest window
always: in chains of short segments speed never builds up inside the window, so stops looked as good as
blends and the run degenerated to G61 (chords test failed; streamed programs 106 stops vs 29 blends).

Hold after (a): blend pieces now carry a small *path* deceleration (small share), and the old hold
(lowest deceleration crossed, one constant rate) overshot the queue end - the stop-witness invariant caught
it. With jerk waived, a piece's real braking budget is its axis acceleration limit at its cap:
`PathBlock.hold_deceleration` = max(planned D, min over axes (min(A_i, D_i) - B2 v_cap^2) / B1) (straight:
planned D). `hold()` brakes on each block at that block's own rate (steps between blocks, waived). This is
the shortest stop under those position-dependent limits, and the planned witness obeys the same or
stricter limits on every block, so v_hold(s) <= v_witness(s) and the hold ends no later than the witness.
Hold motion is certified per axis (`validate_axes`), not against the planned path envelope.
Hold edge cases found by `standard_high_accel` (fixed, `test_hold_at_rest_on_a_stop_vertex`): at rest the
hold emits no braking step (a step to the finished block's rate would act on the next, possibly weaker,
block); a block the tail has reached within rounding is finished, so braking uses the block ahead; past
the last block the last block's rate applies.

Decision rule has no threshold: a blend is kept only if its preview is strictly faster (ties keep the
stop). Example (90 deg, F100, P 0.2, same apex 19.9 mm/s on both machines): trapezoidal blend 0.5632 s vs
stop 0.5667 s (kept); high_jerk (J = 500000) blend 0.5722 s vs stop 0.5716 s (declined, 0.6 ms slower).
The difference is planner conservatism, not physics: L5 crosses each of the 9 sub-piece boundaries with
a = 0, which costs a jerk ramp down and up at every boundary when J is finite (free steps when J = inf),
and the apex sub-pieces carry tiny shares (A 5, J 2160). Signed junction accelerations (section 5) remove it.

Why small P still often keeps the stop (physics, not conservatism): a corner within P has radius ~2.4 P
at 90 deg, so its apex is jerk-limited to v ~ (J R^2)^(1/3) - about 7 mm/s at P = 0.05 on the standard
machine. At F100 nearly all of the stop's deceleration remains; the possible gain is milliseconds and the
a = 0 hand-overs at sub-piece boundaries cost about as much. Forced-blend/stop time ratios, standard machine
(90 deg, 20 mm legs): P 0.01-0.2 -> 0.99-1.08; P 1 -> 0.93-1.05; P 3 -> 0.85-0.92. Trapezoidal and
high-jerk machines: never worse than +1%, down to 0.84. The decision rule makes G64 <= G61 hold anyway.

Machine spectrum: `standard` (synthetic, fixtures/machine_standard_xyz.json: full speed in 0.1-0.2 s,
D within 25% of A), its all-profile-0 and Z-profile-0 variants, `standard_high_accel` (user request
2026-10-10: same jerk, A = D = sqrt(V J) per axis - the fastest ramp the jerk limit allows; X/Y/Z reach
full speed in 163/179/100 ms vs 183/215/103 ms standard and 133/167/63 ms trapezoidal. Matching the
trapezoidal time exactly is impossible at this jerk: it needs (V/A)^2 >= 4 V/J), a near-trapezoidal
high-jerk variant. The real `example` machine (D = A/10, X/Y J = 21 mm/s^3) is out of the default spectrum
since 2026-10-10 (user: too slow while iterating basic behaviour; J likely mis-entered); `machines.extreme()`
keeps it for opt-in runs.

### Speed benchmark vs Ruckig (tests/bench_ruckig.py)

Run `.\.venv\Scripts\python.exe -m tools.trajectory_sim.tests.bench_ruckig`. Ruckig is a valid time-optimal
reference for: (1) L2 1-DoF state-to-state moves; (2) single straight moves through the whole stack
(phase-synchronized multi-DoF Ruckig moves all axes proportionally; where Ruckig cannot phase-synchronize,
a 1-DoF Ruckig along the line with the exact projected limits; both agree to 1e-9 where both exist);
(3) the G61 baseline of a program (sum of per-segment rest-to-rest optima). It cannot follow polylines with
corners, so G64 is reported against that G61 optimum. Asymmetric deceleration maps per axis by direction.

Results (2026-10-10, pinned by tests/test_ruckig_benchmark.py):
- L2 1-DoF (174 cases where Ruckig stays in 0 <= v <= V): time ratio median 1.000000, p95 1.000000, max
  1.32 (one valley-side case); 2 unsolved (known family gaps, section 3.3).
- Straight lines through the whole stack, 60 per machine, all five machines: ratio exactly 1.000000
  (max |ratio - 1| < 1e-6). Phase-synchronized and 1-DoF Ruckig references agree to 1e-9.
- Programs (feed 100): our G61 equals the Ruckig G61 optimum to 1e-6 on every machine. G64 vs that optimum:
  P0.05 0 to -10%, P0.2 -2 to -14%, P1.0 -2 to -18% (largest on trapezoidal / high-jerk machines). Not
  monotone in P (random 0: P1.0 slightly slower than P0.2) - the per-corner greedy decision is local.

## 3D. Optimal reference and G64 corner benchmark (2026-10-11)

`tests/optimal_reference.py` (test-only, CasADi/IPOPT): time-optimal traversal of a FIXED path, split into
rest-to-rest sections at stop flags; variables v, a per node on an arc-length grid; per-axis V, A/D (D only
while the axis speed magnitude falls), J at interval midpoints with j = v_mean da/ds. Optional `envelope`
mode uses constant path limits per piece instead (the planner's own L4 output). Three fixes were needed to
validate it: (1) grid graded toward both ends of every piece with I_x(5,5) (a uniform grid converges like
N^-1/3 near rest, where s ~ t^3; grading only section ends left limit jumps at blend boundaries unresolved,
which put `env` 2% ABOVE the planner's own plan); (2) jerk from the interval's own time; (3) IPOPT gradient
scaling off (it scaled the objective by ~1e-8 and stopped ~0.1% short of bounds).
Validation vs Ruckig, straight rest-to-rest lines, all five machines (4 lines each): always above Ruckig,
excess 6e-4 / 1.5e-4 / 3.5e-5 / 1.1e-5 at count 50 / 100 / 200 / 400 (second order). Corners: converges from
above with refinement (e.g. 30 deg P1: 0.51127 / 0.51121 / 0.51119 s at 200 / 400 / 800 vs planner
0.511184 s), so at count 200 the reference overstates by ~0.02 points.

`tests/bench_optimal.py`: two 20 mm legs at F100, axis-aligned 90 deg and the five zigzag corners, P 0.01-3.
Columns (% vs the G61 optimum): plan (front end), blend (planner, forced blend), env (optimum under the
planner's L4 limits), opt (optimum of our full-size blend under axis limits), best (min of opt over P' <= P
and the stop). Findings:
- Our time is non-increasing in P and never above G61 on every corner and machine.
- Physics: on S-curve machines with P <= 0.2 and corners >= 90 deg, even the optimally timed full-size blend
  is 0.5-3% slower than the stop (apex 1-15 mm/s), so best = G61; gains appear at P >= 1 (3-16%) and P3
  (8-24%); corners <= 60 deg gain 1-4% already at small P. Trapezoidal / high-jerk: 0.3-5% at small P.
- Total loss (plan - best), worst per machine: standard and standard_mixed 11.4 points (120 deg P3),
  standard_high_accel 12.0, trapezoidal 7.0, high_jerk 7.7. Typical 90 deg P1-P3: 6-9 points.
- Where the loss is: blend - env = L5 scheduling (a = 0 at the 9 sub-piece boundaries etc.), env - opt = L4
  projection (constant per-piece limits, min(A, D) both ways on curves, coupling term), plan - blend = front-
  end decision (never positive). Standard machine, axis-aligned 90 deg: L5 1.5 / 2.4 / 2.6 points and L4
  4.4 / 5.9 / 5.9 at P 0.2 / 1 / 3 (P0.2: blend +6.6, env +5.1, opt +0.7); 120 deg P3: L5 3.1, L4 8.3.
  Trapezoidal machine: L5 is ~0 (steps at boundaries are free), all loss is L4 (90 deg P1: 7.0 points).
  high_jerk: L5 0.1-1.0, L4 3-7.
- Missed blends: we keep the stop where the optimum of our blend is faster: 7.7% (standard 90 deg P1, 150 deg
  P3), ~3% (60 deg P0.2, 120 deg P1), because the planner's blend time exceeds the stop there.
- Full-size blend is not always best within P: opt is not monotone at small P (standard 90 deg: +1.7, +2.0,
  +0.7 at P 0.01 / 0.05 / 0.2), supporting the nested ladder (best size <= P).

## 4. Numerics (float32) notes

- `numerics.propagate_f32(start, phases, relative)` mirrors `phases.advance` operation order in numpy
  float32 after rounding each phase's jerk and duration to float32 (as firmware would store them).
- Firmware design input to verify in M2 evidence: propagate positions relative to the segment start
  (`relative=True`) rather than in absolute machine coordinates; float32 has ~7 significant digits.
- Measured (M2, 1000 rest-to-rest solver outputs up to ~500 mm): all endpoint errors within
  16*eps32*(sum of term magnitudes). Worst at s0 = 1000 mm: 8.4e-5 mm segment-relative vs 1.9e-4 mm
  absolute. Decision input for M7: store/propagate segment-relative s; 8e-5 mm is ~0.03 step at 400
  steps/mm, so step-exact endpoints still need integer step targets in the executor (M5), not float s.
- Not yet studied: solving in float32 (only propagation of float64-solved phases is covered).

## 5. Open items carried forward

- Machine-settings semantics: resolved 2026-10-09 (section 7); asymmetric deceleration implemented.
- M0 baseline capture (pinned core revision, legacy fixtures, real machine settings) is still pending.
- L2 coverage gap (section 3.3): valley / monotone families for nonzero goal acceleration. Decide in M3
  whether lookahead ever requests such goals; if junction accelerations are planned by L5, it will.
- Float32 solving (not just propagation) before M7.
- Signed junction accelerations across source boundaries (L5 crosses boundaries at a = 0; conservative).
- M5: (v, a)-dependent constraint envelopes (tight corners on low-jerk machines are very slow with the
  constant per-piece bounds, section 3C), marker lifecycle, executor, check-mode record format including
  clothoid blends (section 1A).
- Helical arcs (INVALID_INPUT today); blends involving arcs are exact stops by decision (section 3C).
- CURVED_SHARE (0.5) is a throughput policy; tune per machine class once real jobs are available.
- L4 uses the sum bound B1|a| + B2 v^2 per axis (like the firmware's 50/50 arc split). On arcs the exact
  per-axis bound is the acceleration circle amp_i sqrt(a^2 + (v^2/R)^2) (Firmware_Planner_Modifications.md
  FW-2); a circle-based projection for arcs (and a tighter one for clothoids) is a candidate for the L4 loss
  measured in 3D. Test-only firmware model and comparison: tests/firmware_model.py, tests/bench_firmware.py.
- Front-end cost: two full-queue preview plans per blendable corner. Bound the preview horizon (plan only
  to the first point where both alternatives coincide in state) before M7; the firmware budget forbids O(n^2).
- Blend benefit is small at small P on S-curve machines (physics, section 3C); larger gains need corners
  that keep a != 0 across boundaries (signed junction accelerations) and (v, a)-dependent envelopes.

## 6. Environment notes

- Run tests from the workspace root: `.\.venv\Scripts\python.exe -m unittest discover -s tools/trajectory_sim/tests -t . -v`
- venv has numpy and ruckig (test-only oracle; never a firmware dependency).
- Visualisation (test-only, matplotlib in the venv): `.\.venv\Scripts\python.exe -m tools.trajectory_sim.tests.view_paths`
  - 3D exact-stop vs G64 paths (firmware current / FW-1 / FW-1+2 adaptive / ours) and speed vs time per
  program, P, machine; buttons flip program / P / corner zoom / machine, check boxes toggle runs.
- Ruckig quirks: `calculate` raises `RuckigError` (not a status) for some targets that inevitably exceed
  a limit, and otherwise returns profiles that overshoot V or go below 0 when the start state forces it.
  Tests locate its acceleration zero crossings (exact speed extrema) and compare only profiles inside
  0 <= v <= V. Sparse sampling missed a 16 ms overshoot once; do not rely on sampling alone.
- Writing files with PowerShell `Set-Content -Encoding utf8` adds a BOM, which breaks the AST-based
  architecture test. Use the editor tools instead.
- If the shared terminal prints `SyntaxError: invalid syntax` for shell commands, it is stuck in a Python
  REPL; send `exit()` first.

## 7. Machine settings input (one example machine) and contract implications

Raw host settings: [fixtures/machine_example_xyz.json](fixtures/machine_example_xyz.json). One machine of a
general-purpose platform: use it as a scenario/regression input, never as defaults. Units are unverified.

Literal interpretation (jerk mm/s^3, decel = planned-motion limit), computed with `AnalyticTimeLaw`:

| Axis | V | A | D | J | 0->V | stop from V | peak accel reached |
|---|---|---|---|---|---|---|---|
| X/Y | 169.33 | 400 | 40.2 | 21.17 | 5.66 s, 479 mm | 479 mm (60% of 798 mm travel) | 60 of 400 |
| Z | 100 | 1000 | 1000 | 423.33 | 0.97 s, 48.6 mm | 48.6 mm (54% of 89.9 mm travel) | 206 of 1000 |

Settings confirmed 2026-10-09 (controller `$$`: [fixtures/machine_example_settings.txt](fixtures/machine_example_settings.txt)):

- Host values equal the firmware's: $110-$112 max rate (mm/min), $120-$122 accel, $800-$802 jerk
  (grblHAL mm/s^3), $810-$812 deceleration (fork setting, 0 = track accel). So the firmware really runs
  X/Y with J = 21.167 mm/s^3 and the table above is what it executes. The inch-derived numbers are just
  how the values were entered. User (2026-10-10): 21 mm/s^3 is unreasonably small, likely 10x too low;
  treat this machine as an extreme case, not a representative one.
- Deceleration is deliberately asymmetric (the 10x X/Y value was an extreme test of the algorithm). In this
  fork `settings_get_deceleration()` has no callers: $81x is stored but the planner/stepper ignore it.
- Host `accelerationProfile`: 0 = constant acceleration (trapezoidal velocity), 2 = trapezoidal
  acceleration (S-curve velocity), 1 = sine velocity (out of scope). grblHAL has no per-axis profile:
  with ENABLE_JERK_ACCELERATION every axis is jerk-limited by $80x, Z included ($802 = 423.33).
- Host has separate accelerationJerk/decelerationJerk (equal here); firmware has one $80x per axis.

Implemented: L0 `Limits.deceleration` (None tracks acceleration, like $81x = 0); L1/V/L2 use
`-D <= a <= A`; jerk stays symmetric. The derivation in section 3.1 holds with `cap = A` or `D`.

Still to decide (before M3/L4):

- Per-axis profile 0 - DECIDED 2026-10-10 (Option A): a constant-acceleration axis imposes no jerk
  constraint. Mapping for L4: profile 0 -> axis jerk = inf; profile 2 -> axis jerk = $80x; profile 1
  (sine) -> unsupported, reject. Path jerk limit = min over moving jerk-limited axes (scaled by the path
  geometry); if every moving axis is profile 0 the path limit is inf and L2 emits `AccelerationStep`s.
  Implemented in L0/L1/V/L2 (M2). Validation treats a step under a finite path jerk as a violation; the
  acceleration-limited hold/cancel handoff (confirmed scope) must be certified by L6 with its own policy,
  not by relaxing the validator.
- Separate deceleration jerk: needs a rule for which phases use it (sign of a vs speeding up/slowing down).
  Not needed while firmware has one jerk per axis.
- Unequal axis limits (Z faster accel, slower speed than X/Y): on a straight multi-axis line the path
  limit is min over moving axes of limit_i / |u_i|; deceleration likewise uses D_i. On curves this is
  L4's (v, a)-dependent envelope.
- 100 steps/mm and travel up to ~800 mm: absolute float32 error at 800 mm is ~1.5e-4 mm (~0.015 step);
  keep segment-relative storage. Scenario generators should cover these ranges.
- Regression scenario for M3 (not the design focus): two collinear Z moves where the second is too short
  to reach max acceleration or velocity, ending at rest; expect no boundary reset and no terminal crawl.
