# G64 Trajectory Overhaul: Simulator-First Project Plan

Status: M1 complete (2026-10-06); M2 scalar time law implemented and tested with layer contracts (2026-10-09). M3 straight-path lookahead and streaming runtime complete (2026-10-10). M4 certified G64 XYZ geometry complete (2026-10-10), including the throughput requirement "G64 never slower than G61" (DESIGN.md section 3C). No new firmware implementation authorized by these milestones.

Design basis: [trajectory architecture](grblHAL_G64_Jerk_Limited_Trajectory_Architecture.md).
Historical evidence: [potential issues](potentialissues.md) and [legacy simulator](tools/jerk_sim/jerk_sim.py).
Living design record (layers, contracts, math, tolerances, open items): [tools/trajectory_sim/DESIGN.md](tools/trajectory_sim/DESIGN.md). Read it first when resuming.

## Confirmed Scope

- First release: coordinated Cartesian XYZ. Preserve extension contracts for rotary and process axes, but do not assume their units or feed semantics.
- Normal motion: bounded per-axis velocity, acceleration, and jerk, including through G64 blends.
- Hold and jog cancel: acceleration-limited controlled stopping. Acceleration discontinuities are explicitly allowed at this policy handoff, not during ordinary motion.
- Hardware E-stop, power loss, and driver disable are separate machine policies. Do not model them as guaranteed position-preserving controlled stops.
- Source G-code boundaries are markers, not mandatory velocity or acceleration resets.
- Retain the parser and machine framework. Replace trajectory decisions rather than adding another correction downstream of trapezoidal planning.
- Do not change existing firmware or legacy simulator behavior during initial development. Keep historical fixtures reproducible.
- Check mode (requirement, 2026-10-10): the target lacks flash, RAM and output bandwidth for extensive on-device validation, so firmware provides a check mode that runs G-code test lines through all layers except step generation and output, and streams lines and arcs with enough metadata for client-side test code to visualize the planned geometry and analyze the planned kinematics. Layer outputs must therefore be compact, serializable and streamed in commit order (DESIGN.md section 1A).

## Corrections To Earlier Conclusions

- The existing Python simulator estimates segment step rates; it does not reproduce H743 timer quantization, AMASS/Bresenham pulse placement, driver timing, or finite segment queues. Calling those rates fully executed is too strong.
- The current snap metric is a screening tool, not a proof of physical acceleration or jerk compliance. Synthetic start/stop samples, segment averages, small step counts, and reversals need separate treatment.
- The formula `t = (6*d/J)^(1/3)` describes motion from zero speed and acceleration under constant positive jerk. It does not establish the correctness of an arbitrary terminal braking tail. Use actual incoming speed, signed acceleration, distance, and terminal conditions.
- Removing the long crawl in selected fixtures did not prove jerk compliance, feasible braking at every junction, or safety of the current firmware patch. Preserve it as a comparison baseline, not as the reference solver.
- Bounded changes to a stored acceleration variable do not alone prove correct jerk integration. Validate position, speed, and acceleration from the same phase polynomial.
- Neither an AI-generated implementation nor a claimed host timing result is an acceptance criterion. Benchmark the eventual implementation on the H743.

## Units And State

The new simulator uses seconds, millimeters, mm/s, mm/s^2, and mm/s^3. Convert grblHAL minute-based values only at explicit boundaries.

Reference arithmetic uses float64. A separate float32 backend follows target operations and tolerances. Neither backend silently clamps infeasible states into valid ones.

Scalar state: `(s, v, a)`, with signed acceleration and nonnegative path speed. Reversal is a new path direction after a stop, not negative path progress.

Geometry state: `q(s)` and derivatives `q_s`, `q_ss`, `q_sss`, plus primitive identity, source mapping, and tolerance certificate.

Axis quantities:

```
velocity     = q_s * v
acceleration = q_s * a + q_ss * v^2
jerk         = q_s * j + 3*q_ss*v*a + q_sss*v^3
```

Queue state separates executed position, committed trajectory tail, and uncommitted lookahead. Replanning starts from the committed tail state; it cannot rewrite pulses already scheduled.

## Simulator Modules

Proposed location: `tools/trajectory_sim/`, developed incrementally in the existing Python environment.

Layer levels, allowed dependencies and per-layer contracts are defined in code by `LAYERS` in `tools/trajectory_sim/contracts.py` and enforced by `tests/test_architecture.py`; DESIGN.md section 2 summarizes them. Each layer's Protocol lives in `contracts.py`, and every implementation of a layer must pass that layer's shared contract test suite.

| Module | Contract | Independent check |
|---|---|---|
| `model.py` | Typed states, limits, signed constant-jerk phases, markers, immutable committed prefix | Serialization, units, nonnegative durations, explicit ownership |
| `phases.py` | Exact polynomial propagation and phase extrema | Hand-derived solutions, split/compose invariance, derivative checks |
| `scalar_solver.py` | State-to-state transitions, reduced phases, feasibility and stopping queries | Independent analytic cases; optional local Ruckig comparisons |
| `geometry.py` | Lines/arcs, collinear merging, smooth blends, physical arc-length parameterization | Derivatives, endpoint/tangent/curvature matching, certified deviation |
| `constraints.py` | Per-axis constraint envelope for a geometry interval and `(v,a)` | Dense independent sampling plus conservative interval/extrema bounds |
| `lookahead.py` | Finite-horizon feasible scheduling with signed junction acceleration | Full resimulation and supported terminal-stop certificate |
| `runtime.py` | Streaming, commit horizon, override, hold/cancel, resume and marker lifecycle | Deterministic event schedules, queue ownership, no stale callbacks |
| `executor.py` | Ideal step crossing reference and firmware-like segment/pulse model | Pulse timestamps, step counts, direction timing, timer/AMASS model |
| `validation.py` | Independent limit, continuity, deviation, event and pulse checks | Known-bad trajectories must fail, not just known-good tests pass |
| `scenarios.py` / `tests/` | Fixtures, seeded randomized cases, reports | Reproducible failure reduction and machine-readable outcomes |

Keep validation independent of the production solver helpers wherever practical. A transcription of the same formula is not an independent oracle.

## Core Design Decisions

### Scalar Solver

Start with exact constant-jerk phase integration and zero-boundary-acceleration transitions, including triangular acceleration and missing cruise/plateau phases. This supplies a small independently verifiable foundation.

Extend to nonzero signed boundary acceleration before queue lookahead is accepted. Reject unreachable states, negative phase times, speed overshoot, reverse progress, NaN, and infinity explicitly. Do not reset acceleration to make a case solvable.

Use bracketed monotone solves where their domain is proved. Newton steps may accelerate a bracketed solve but cannot replace feasibility checks or bounded iteration counts. Do not assume an arbitrary-acceleration distance function is globally monotone without proof.

Ruckig, if approved, is an optional offline local single-DoF reference for compatible boundary conditions. No cloud waypoint feature and no firmware dependency are assumed. It does not validate our path geometry or finite-queue lookahead.

### Geometry And Strict Jerk

Start with exact stop and collinear line merging. A tangent line/arc blend with a curvature jump cannot satisfy strict axis jerk at nonzero speed. It is not an accepted production shortcut for the confirmed requirements.

Evaluate curvature-continuous blends, initially a quintic parametric family with specified endpoint derivatives. Polynomial degree alone does not guarantee a feasible blend: verify joins, curvature/third derivatives, maximum deviation, no cusps, and nonzero parameter derivative.

Use physical arc length for `s`; a polynomial parameter is not automatically distance. Bound error in arc-length inversion and derivative conversion. Curvature derivatives must remain bounded; finite jerk jumps are permissible, acceleration jumps are not.

If a blend cannot be certified within tolerance, trim budget, or computation budget, reduce speed or use an exact stop. Preserve a valid path instead of exceeding tolerance or jerk.

Define P as a documented maximum geometric deviation from the source path, with a clear original-path-to-blend correspondence and certified bound. Q simplification is deferred until its error budget and semantics are explicitly approved.

### Dynamic Constraints And Lookahead

Curvature consumes acceleration and jerk budget. Therefore local scalar acceleration/jerk limits depend on both `v` and `a`; one fixed acceleration or jerk value per block is insufficient.

Stage the lookahead problem: straight path with constant limits first, then scheduled local speed limits, then variable geometry with projected per-axis constraints. Backward feasibility operates in `(v,a)` state space, not speed alone. A terminal-stoppable witness trajectory is required for each committed prefix.

Keep the first planner conservative and feasible rather than globally time-optimal. Establish a correct fallback schedule before optimizing throughput. Queue exhaustion must produce a supported stop, never a late discovery that braking is infeasible.

An override reduction is a new requested cap, not an instantaneous speed assignment. If already above it, produce a limit-respecting transition and report the transient. An increase must not invalidate a previously committed braking reserve.

### Hold, Cancel, And Events

Acceleration-limited stopping means jerk continuity is waived at the policy switch. Curved-path centripetal acceleration still counts against each axis limit; `a = -Amax` cannot be assumed valid on a curve.

Model committed pulse latency explicitly. Calculate the stopping trajectory from the state after immutable queued output, not only the state at receipt of the hold command. Resume from the actual stopped state.

Each marker fires exactly once when executed. Define how markers within a replaced corner map to the blended path; exact-position IO may require an exact stop or an unblended event location. Abort invalidates pending markers and advanced events. Replanning revises predictions, not completed events.

### Pulse Execution

Compare two executors: ideal axis step-crossing timestamps obtained from `q(s(t))`, and a segment/Bresenham/timer/AMASS implementation matching the driver configuration.

Check monotonic axis pieces, direction reversals, pulse width, direction setup/hold, integer endpoint counts, accumulated timing drift, and maximum sustainable step rate. Define rounding at half-step crossings explicitly.

Do not infer infinite physical jerk from every discrete pulse. Compare pulse timing and reconstructed velocity windows against the continuous reference with a declared quantization/error budget. Separately detect true profile velocity/acceleration jumps at joins.

## Milestones And Pass Gates

| Stage | Deliverable | Gate before proceeding |
|---|---|---|
| M0 Requirements and baseline | This plan, pinned core/build configuration, immutable legacy fixtures, measured/unknown budgets | XYZ scope, strict jerk, hold/cancel policy recorded; no fabricated machine settings |
| M1 Independent scalar foundation | Model, exact phase propagation, independent validator, analytic reduced profiles | Endpoint and continuity checks; negative tests catch injected faults; units verified |
| M2 General scalar solver | Signed arbitrary-acceleration transitions and stopping feasibility; float64/float32 comparisons | Extremum checks over full phases; finite bounded solves; reproducible random suite; optional oracle agreement |
| M3 Streaming straight-path lookahead | Forward/backward feasible scheduling, commit horizon, starvation, overrides and stop/resume | Every committed prefix has a valid stop; no source-boundary resets; queue latency accounted for |
| M4 Certified G64 XYZ geometry | Smooth line-line blends, arc joins, tolerance and derivative certificates | Bounded deviation and per-axis dynamics across joins; uncertifiable cases stop safely |
| M5 Integrated runtime/executor | Axis constraints, event lifecycle, ideal and firmware-like pulses | Geometry/time-law/pulse agreement; timing error budget; no late/stale markers; exact integer endpoint |
| M6 Simulation release qualification | Regressions, randomized adversarial cases, soak, target-like arithmetic and resources | All hard invariants pass; failures minimized; signed-off numerical and runtime budgets |
| M7 Firmware shadow implementation | Feature-gated C/C++ port, matching Python fixtures, no new motor output | Differential host tests and H743 measured worst-case time/RAM; no missed service deadlines |
| M8 Controlled cutover | Bench hardware then representative process jobs | Jog/homing/probe/hold/alarm policies tested; rollback verified; explicit user authorization |

Firmware creation starts at M7, not while M1-M6 remain incomplete. There is no date-based bypass of a failed gate.

## Regression Matrix

### Mathematics And State

- Rest-to-rest, rest-to-motion, motion-to-rest, nonzero entry/exit speed, positive/negative boundary acceleration.
- Every reduced phase topology: no cruise, no acceleration plateau, acceleration-only/deceleration-only, very short distance and near-zero delta speed.
- Cases initially accelerating when a stop is requested; acceleration passes through zero according to the selected policy.
- Full phase extrema, not just endpoints: speed extrema where acceleration is zero, acceleration extrema, finite jerk, nonnegative progress.
- Split and recombine identical phases; subdividing a collinear source move must not introduce resets or significant profile changes.
- Float64 versus target float32; tiny residuals, large coordinates, unit conversion, failed root bracket, iteration exhaustion, non-finite inputs.

### Geometry And XYZ

- Straight continuations, reversals, nearly straight corners, angles near 180 degrees, tiny/zero-length inputs, unequal adjacent lengths.
- Alternating tiny blocks, long-short-long, consecutive blends competing for trim distance, narrow P, large P, ill-conditioned polynomials.
- XY/XZ/YZ and genuine 3D paths; unequal axis limits; curvature and curvature derivative extrema.
- Line-line, line-arc, arc-line, arc-arc, full arcs, helical paths when supported; unsupported geometry has an explicit stop/rejection policy.
- Translation/rotation where axis limits permit, subdivision invariance, source-boundary marker mapping, geometric tolerance certificates.

### Queue, Runtime, And Safety

- Empty/single/short/full queue, delayed refills, burst arrivals, starvation at each phase and on a blend, alternating horizon size.
- Feed reductions/increases during every phase; repeated hold/resume; cancel at a corner and during reversal; abort with pending callbacks.
- Solver failure after commitment, too-short braking horizon, backlog and output-buffer latency, bounded memory and CPU work.
- Dedicated legacy/special-motion paths remain separately testable: homing, probing and synchronized motion are not silently routed into the new normal-motion solver.
- Source completion, position IO, advance IO, dwell/exact stop, event ordering and cancellation.

### Sampling And Pulses

- At least 40/80/400/800 steps/mm and 100/250/1000 nominal sample rates, plus adaptive sampling by error bound.
- Low/high jerk, acceleration and feed; short segments with fewer than one step; high rates, timer rollover/saturation and AMASS transitions.
- Axis direction changes within a path primitive; per-axis and total step counts; timing reconstruction and quantized endpoint.
- Repeated legacy final-Z patterns and multi-axis jobs for thousands of cycles, with deterministic event schedules and seeds.

## Acceptance Rules

- No arbitrary minimum speed, maximum terminal-time clamp, endpoint snap, skipped distance, or limit inflation to conceal infeasibility.
- Continuous reference `(s,v,a)` across ordinary joins; zero velocity and acceleration at a completed rest target. Terminal deceleration is monotone where intended; hold from positive acceleration has an explicitly allowed transient.
- Full-interval axis limits pass certified bounds. Dense samples supplement certificates but do not replace them.
- G64 deviation stays within the documented P budget including arc-length and numerical errors.
- Planner prediction and propagated trajectory agree within a specified numerical budget; pulse endpoint equals the commanded quantized position without final correction pulses.
- Long final segments are judged against a boundary-state reference, not a universal 100 ms cutoff.
- Profile snaps are tested directly at joins; pulse rate variations are judged against the executor quantization budget rather than an arbitrary one-step allowance.
- Tests cover at least several thousand repeated cycles before simulation release; random suite size and machine-specific budgets are recorded per release.

## Pending Inputs And Decisions

These do not block M1:

- Exact failing G-code and actual controller settings. Example machine's host settings and controller `$$` received 2026-10-09 ([host](tools/trajectory_sim/fixtures/machine_example_xyz.json), [$$](tools/trajectory_sim/fixtures/machine_example_settings.txt); semantics confirmed, DESIGN.md section 7). Still needed: the failing G-code (low priority; general correctness is the goal). Never treat one machine's settings as platform defaults.
- Confirmed 2026-10-09: deceleration is a deliberate, independent limit (now in `Limits`); per-axis profile 0 = constant acceleration, 2 = S-curve. Decided 2026-10-10 (Option A): profile 0 axes impose no jerk constraint; L0 has `AccelerationStep` and `Limits.jerk = inf`; L2 handles both orders. Open: whether deceleration jerk differs from acceleration jerk.
- Ruckig: approved and installed in the venv as a test-only reference (2026-10-09). Not a firmware dependency.
- Physical acceptable path error, G64 P semantics, and whether fixed-location IO in a corner requires stopping.
- Actual timer clock, step/pulse/direction constraints, output buffer depth, maximum step rates and measured service latency.
- Meaning of controlled alarm stopping versus immediate output disable; documented separately from hold/cancel.
- Required IO timing and production throughput. Derive simulator error budgets first; confirm machine budgets before M6/M7.
- Future rotary/pump semantics. Keep typed axis units and synchronization interfaces extensible without adding unrequested eight-axis behavior now.

## Immediate Next Work

1. Complete the pending M0 baseline capture: immutable legacy fixtures, core revision, build configuration, and real machine settings. Do not revert existing work.
2. M3 design: straight-path lookahead on `ScalarTimeLaw` (braking reserve from `stop`, commit horizon, override caps, acceleration-limited hold). Decide whether junction states with nonzero acceleration are needed; if so, first close the L2 family gap (DESIGN.md section 3.3).
3. Float32 solving study (not only propagation) before M7.

## M1 Implementation Evidence

Implemented under `tools/trajectory_sim/`:

- `model.py`: immutable state, limits, constant-jerk phases, explicit segment boundaries, source markers, and trajectory records. Signed acceleration is retained; non-finite inputs and negative phase durations are rejected. State feasibility belongs to validation, so invalid trajectories can be represented and tested without clamping.
- `phases.py`: closed-form propagation and sampling, phase-chain composition, and complete scalar position/velocity/acceleration extrema. Units are seconds and millimeters; explicit grblHAL conversions are tested.
- `validation.py`: an independent Decimal (60-digit arithmetic) evaluator, without importing production propagation. Checks declared dynamics, initial/interphase continuity, final state, nonnegative speed, and full-interval scalar speed/acceleration/jerk limits. Tolerances are explicit and do not mutate trajectories.
- `tests/`: 20 standard-library unittest tests. No new third-party dependency installed.

Evidence:

- Hand-derived and exact rational endpoint checks, derivative consistency, and interior stationary-point tests.
- 3,000 seeded phase split/composition cases and 1,000 seeded independent Decimal endpoint cases.
- 200 seeded extrema comparisons against 101 independently evaluated samples per phase. Sampling is supplemental to analytic extrema, not the limit proof.
- Analytic fixtures cover triangular acceleration, seven phases, plateau without cruise, acceleration/deceleration-only, nonzero signed acceleration, cruise, and a tiny rest-to-rest move. These are fixtures, not a general solver.
- Injected position, velocity, and acceleration snaps, incorrect propagated endpoints, bad final acceleration, hidden interior overspeed/reversal, and limit violations are rejected.
- 2,000 repeated rest-to-rest profiles (8,000 phases) propagate without resetting state; every cycle is checked at rest and the expected final position is verified. This is scalar foundation testing, not an integrated runtime/pulse soak.

Run from the workspace root:

```powershell
& ./.venv/Scripts/python.exe -m unittest discover -s tools/trajectory_sim/tests -t . -v
```

M1 covers reference float64 propagation and independent scalar validation only. It does not implement arbitrary-state trajectory solving, curvature/axis projection, lookahead, markers firing, float32 execution, or pulse timing. Firmware and the legacy simulator are unchanged. M0's exact machine fixtures remain pending.

M1 re-validated 2026-10-09: all 20 M1 tests pass unchanged.

## M2 Implementation Evidence

Implemented under `tools/trajectory_sim/` (details and derivations in DESIGN.md section 3):

- `contracts.py`: layer registry (`LAYERS`), `SolveStatus`, `SolveResult` (failure results carry a witness profile), and the `ScalarTimeLaw` Protocol.
- `scalar_solver.py` (`AnalyticTimeLaw`): minimum-time `velocity_transition` and `stop` for arbitrary signed boundary accelerations; conservative position-exact `distance_transition` (peak at zero acceleration, bounded bisection). Speed bounds are checked over full phases and reported, never clamped.
- `numerics.py`: float32 propagation mirror (segment-relative or absolute).
- Tests: `test_architecture.py` (layer import rules), `test_scalar_time_law.py` (contract suite reusable by any implementation), `test_numerics.py`. The earlier trial `test_scalar_solver.py` was removed because its fixtures had been adjusted to fit the code.

Evidence (43 tests pass, about 6 s):

- Architecture: every module registered once; imports follow layer rules; validation imports only `model`.
- Hand-derived triangle, plateau, and seven-phase profiles match exactly.
- 3,000 seeded velocity transitions: OK results certified by the independent Decimal validator; every failure's witness independently shows the claimed violation. No INVALID_INPUT or NO_CONVERGENCE on valid inputs.
- Minimum time: never slower than a brute-force enumeration of the profile family (300 cases), and matches Ruckig's velocity-interface duration (500 cases).
- Stop: status matches the analytic reverse/overspeed boundaries on 3,000 cases; stopping distance is never beaten by weaker limits.
- 2,000 seeded distance transitions: OK results certified (endpoint and full-interval limits), failures verified, bisection within its bound. Constant-speed boundaries (including rest-to-rest) are always solved (1,000 cases). Splitting a move at a nonzero-acceleration point reproduces the whole move with no reset.
- Ruckig position comparison (600 cases): duration equal to Ruckig's time-optimum wherever both solve (max gap 5.3e-12 relative); 3 cases (0.5%) are a documented family gap, pinned as fixtures.
- float32: errors within 16·eps32 times term magnitudes; worst 8.4e-5 mm segment-relative vs 1.9e-4 mm absolute at s0 = 1000 mm.

M2 gate status: extremum checks, bounded finite solves, reproducible random suite, and oracle agreement are met. Open: the family gap for nonzero goal accelerations, and float32 solving (propagation only so far).

## M3 Implementation Evidence

Implemented under `tools/trajectory_sim/` (DESIGN.md section 3B):

- `contracts.py`: `Lookahead` Protocol, `PlanResult`, `ScalarTimeLaw.minimum_distance`; L0 `PathBlock`.
- `lookahead.py` (`ConservativeLookahead`, L5): regions of equal cap, backward/forward passes with a = 0 at cap boundaries, region-0 exit search over the analytic basins of need(w), coarsening fallback when no a = 0 exit exists, override transient (to the lowest cap, or a stop when the queue is too short).
- `runtime.py` (`StreamingRuntime`, L6): commit horizon with latency, event replans with retry, stop-witness invariant after every commit/replan, override logged where it takes effect, acceleration-limited hold and resume.

Evidence (full suite: 68 tests pass, about 71 s, two identical runs):

- Contract suite: 400 random plans from rest, 300 replans from moving states and 300 replans after queue extension (30% with an override) all OK and certified by the Decimal validator plus `validate_caps`. Equal-cap source splits give identical elements.
- Region-0 exit search matches 81-point brute-force sampling (60 cases); `ramp_distance` matches L2 propagation (1,000 cases).
- Streaming: 25 seeded scenarios (arrivals, starvation, overrides, hold/resume) plus starvation, override, hold and collinear short-Z soak (40 cycles, both orders): all executed motion certified; 244 replans, 0 fell back to the previous witness. Low-speed crawl bounded per interval.
- Regression fixtures from seed 6006 case 3 (queue extension mid-deceleration; override with no room for the transient) plan OK and certified.
- Latency: hold and override requested at T leave every element committed before T + latency unchanged; new motion starts at the committed tail.
- L2 known gaps pinned exactly (2 valley-side solves at 1.015x and 1.179x Ruckig; 1 INSUFFICIENT_DISTANCE).

M3 gate:

- [x] Every committed prefix has a valid stop (runtime invariant checked after every commit and replan; hold certified).
- [x] No source-boundary resets (equal-cap blocks merged; identical elements; collinear Z regression).
- [x] Queue latency accounted for (`test_requests_take_effect_at_committed_tail`, `test_hold_and_resume_with_latency`).

Carried forward: the a = 0 cap-boundary family with coarsening is conservative and its completeness is measured, not proven; signed junction accelerations and the L2 valley/monotone families remain open; the thousands-of-cycles soak belongs to M6.

## M4 Implementation Evidence

Decisions (2026-10-10): clothoid-pair blends (not quintic: native arc length, closed-form derivative bounds); joins with a curvature jump are certified exact stops (transition curves belong above the velocity layer); per-axis dynamics as conservative constant limits per piece; validation in double with explicit rounding bounds (no Decimal: firmware has none). Design and proofs: DESIGN.md section 3C.

Implemented under `tools/trajectory_sim/`:

- `geometry.py` (L3, `GeometryStream`): lines, planar G17/18/19 arcs (full circles, centre refit <= 0.005 mm), symmetric clothoid-pair G64 blends with an analytic Hausdorff deviation certificate, streaming revisions that never change committed path, source markers.
- `constraints.py` (L4, `ConservativeProjection`): constant path V/A/D/J per piece from certified |q'|, |q''|, |q'''| bounds; exact on lines; profile-0 axes impose no jerk limit.
- L5 per-block limits and hard axis ceilings (override transients never lift them); L6 `revise` (accept a geometry revision only if nothing committed changes and a stop witness survives), hold under the lowest crossed deceleration.
- `validation.py`: double precision throughout; new `validate_envelope` and `validate_axes`.
- Fixtures: `machine_standard_xyz.json` (synthetic, full speed in 0.1-0.2 s) and a test spectrum of five machines (`tests/machines.py`).

Evidence (full suite re-verified 2026-10-10 after the hold fixes: 101 tests pass, 793 s; `test_g64_never_slower_than_g61` 348 s, `test_tiny_segment_arc_approximation` 223 s and `test_random_corners_certified` 146 s take 90%; the front end's preview plans dominate, see DESIGN.md 5):

- Pieces: hand-derived arcs in all three planes (incl. full circle and cw), clothoid positions match an independent Simpson integration of the heading to 1e-10, derivatives match central differences within their own truncation bounds, derivative bounds hold on 400-sample sweeps.
- Corners: 300 random 3D corners (deflection 1e-6 rad to pi - 1e-6, P 1e-3..1 mm, lengths 0.01..50 mm): every blend's certified deviation <= P, independently confirmed by dense two-sided Hausdorff sampling with Lipschitz slack; joins continuous in q, q', q''; trims within limits. 0 deg continues without a stop; 180 deg stops; 150-segment chains of 0.02-0.08 mm chords stay certified.
- Arc joins: line -> tangent arc, arc -> line, non-tangent arcs and G61 stop; same-circle continuation is continuous.
- Projection: on 300+ curved pieces, every axis velocity/acceleration/jerk at the extreme admissible path states stays within axis limits; straight lines match hand values exactly.
- Pipeline on all five machines: G64 square, 15 streamed random 3D programs with starvation, override and hold, 0.05 mm-chord arc approximations, arcs and joins, late moves (blend shrinks or exact stop). Executed motion certified by path, envelope, feed-cap and axis validators, deviation <= P per corner, and an independent finite-difference check of sampled axis positions. Corner kinds over streamed programs: 92 blends, 43 stops.
- Regressions found by the pipeline and fixed: an exact stop cleared by a later blend, a 7e-15 mm micro piece, validator boundary attribution.

M4 gate:

- [x] Bounded deviation: certified Hausdorff deviation <= P for every blend, independently confirmed by sampling.
- [x] Per-axis dynamics across joins: G2-continuous blends; per-axis limits certified per piece and by finite differences across every join.
- [x] Uncertifiable cases stop safely: reversals, tiny blends, curvature jumps, G61, and blends the runtime cannot accept become exact stops; starvation shrinks or stops corners without touching committed motion.
- [x] Throughput (requirement added 2026-10-10): a G64 run is never slower than the same program with exact stops. Fixes in DESIGN.md 3C: curvature-following blend sub-pieces, reachability-based acceleration share per piece, and an L8 front end that keeps a blend only where the planned time to finish is shorter. Proof tests on all five machines: `test_g64_never_slower_than_g61` (zigzag 30-150 deg, three random 3D polylines, chords; P 0.001-1 mm; worst G64/G61 ratio 1.000) and `test_axis_aligned_right_angle_g64_beats_g61` (never slower; at P 3 mm strictly faster, 0.555 vs 0.660 s on the standard machine, apex 77 mm/s).

Carried forward: conservative constant per-piece limits make tight corners very slow on low-jerk machines (example machine: ~0.006 mm/s through a 0.05 mm corner); (v, a)-dependent envelopes in M5. Check-mode record format for clothoid blends (DESIGN.md 1A). Helical arcs unsupported (INVALID_INPUT).