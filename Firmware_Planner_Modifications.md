# Firmware Planner Modifications (proposals)

Proposed changes to the current grblHAL firmware path planner (`grbl/planner.c`, `grbl/motion_control.c`),
derived from the trajectory simulator. **Status: FW-1 and FW-2 (adaptive) implemented 2026-10-11 for machine
testing (user approval); FW-3 still a proposal.**

## Implementation (2026-10-11, branch dev/octowrapper, Waveshare OpenH743I-C build)
- `platformio.ini` (waveshare_openh743i_c): `ENABLE_JERK_ACCELERATION=0`.
- `config.h`: `PATH_BLEND_ARC_KMAX` (default 0.9): cruise centripetal share k_max (compile time).
- `planner.h`: `PATH_BLEND_ARC_LAW` = path blending without jerk; `plan_block_t.arc_accel` (A of the circle law,
  0 for lines).
- `planner.c`:
  - arc block limits: FW-1 (plane share sqrt(1 - n_i^2) per XYZ axis). Law builds: rate = min(V_i / share,
    sqrt(k_max A R)), `arc_accel` = A, `acceleration` = A sqrt(1 - k_max^2) (constant fallback used by feed
    hold). Jerk builds: FW-1 with the old 50/50 split.
  - `plan_reach_speed_sqr()` replaces `v^2 + 2 a L` in the three places of `planner_recalculate()`.
- `stepper.c` (`st_prep_buffer`): the velocity profile is written with `profile_distance(v^2)` (v^2/(2a), or
  (R/2) asin(v^2/(A R)) on arcs) and its inverse; for constant acceleration this is the same algebra as before.
  On arc blocks in normal operation: accelerating segments take the speed from the law at the new position;
  braking segments use the circle's tangential budget at the current speed for the original near-zero-speed /
  end-of-block logic and take the speed from the law at the distance still to go (so the ramp lands on the
  exit speed); override decelerations step along the law. Feed hold on arcs brakes at the constant fallback rate.
- `$11` (junction deviation) is a setting: set `$11=0` on the machine (FW-0).
- Host port of the new segment logic: `tools/trajectory_sim/tests/test_firmware_stepper_law.py` (10 ms segments;
  speeds stay on/under both law curves, blocks end exactly at the exit speed and length, time within 2% of the
  model). It caught a stall in the first version (braking to rest on an arc: segment distance -> 0 as v -> 0).
- Not yet tested on hardware; feed override and feed hold on arcs are only reasoned about, not simulated.
- Hardware test 1 (2026-10-11, user): at F57.15 mm/s blended moves ran at ~8.7 mm/s; some moves that start or end
  a blended path ran at the right speed. Program in `tests/test_firmware_hw_regressions.py`. NOT reproduced: the
  firmware model plans full feed on every long straight block, and the ported C segment generator executes it
  (P 0.01 / 0.1 / 1, current and FW-1+2 arc limits, machine_example_xyz settings). Open: the cause lies in
  something neither models (streaming / pending-move timeouts, step quantisation, float32, settings on the
  machine, or a C-level difference from the port).
  Resolved: the client rejected `$800`-`$807` (jerk settings, refused by a jerk-off build) and stopped sending
  the rest, so only X was configured. With all axes set the pocket matched the simulator.
- Hardware test 2 (2026-10-11): the x_chain (short, nearly collinear moves) slowed or stopped at interior points
  with `$11 = 0` but not with `$11 = 0.01`. Cause and fix: FW-4 above.

Scope: the H743 build with `ENABLE_PATH_BLENDING=1` and `ENABLE_JERK_ACCELERATION=0` (constant-acceleration
blocks; the current jerk implementation can produce instantaneous velocity steps and is out of scope).

Evidence tools (test-only, in `tools/trajectory_sim/tests/`):
- `firmware_model.py`: model of the current planner (G61 / G61.1 / G64 arc blends, block limits, junction rule,
  backward/forward planner with a 100-block window, trapezoids). Checked by `test_firmware_model.py` against
  hand derivations and an independent phase-plane integration. Options model the proposals below.
- `bench_firmware.py`: current firmware vs proposals vs the new architecture, same machine and programs.

## Settings (no code change)

### FW-0: junction deviation `$11 = 0`
- `$11 > 0` lets G61 corners (and G64 corners that fail to blend) pass the vertex at
  v^2 = A_j `$11` sin(theta/2) / (1 - sin(theta/2)). That speed is derived for an imaginary arc the machine never
  follows: at the vertex the axis velocities change instantaneously, which no machine can do (it is absorbed by
  following error, mechanical compliance or lost steps).
- With `$11 = 0` the junction speed is 0: G61 becomes an exact stop (identical to G61.1).
- All simulator comparisons use `$11 = 0`. Measured effect of the default 0.01 mm: 0.2 mm chord program 0.134 s
  instead of 0.728 s, i.e. a 5x "speed-up" that is not physically executable.
- Caveat (hardware test 2): with `$11 = 0` EVERY junction that is not exactly straight stops, including joins the
  firmware itself makes nearly tangent (G64 blend arcs before FW-4; G2/G3 arcs, which `mc_arc` splits into
  chords). G2/G3 at `$11 = 0` therefore stop at every chord. Until G2/G3 get the same treatment as blends,
  use a small non-zero `$11` (e.g. 0.002-0.005 mm) where G2/G3 or G61 polylines matter.

### FW-4: joins into and out of a blend arc are tangent (implemented 2026-10-11)
Where: `plan_buffer_line()`, junction speed. Line blocks take direction and length from integer step targets
(`pl.position`), blend arcs from floats. On the short trimmed lines of a chain of short moves (1-2 mm at 100
steps/mm) the rounding tilts the line by a few mrad, -cos(angle) exceeds the planner's "straight" threshold
(0.999999) and junction deviation applies: with `$11 = 0` that is a full stop at every line <-> arc join.
The joins are tangent by construction, so the junction speed is left unlimited when the block or the previous
block is a blend arc (exact stops still win). The residual direction step equals step rounding, which every
line already has. Model: `firmware_model` options `steps_per_mm` and `tangent_blend_junctions`; reproduced 4-8
stops on the hardware x_chain (+38% to +73% time) and none with FW-4
(`test_firmware_hw_regressions.test_blended_chain_never_stops_inside`).

## Code changes

### FW-1: per-axis contribution for blend-arc limits (a planar arc ignores axes outside its plane)
Where: `plan_buffer_line()`, `#if ENABLE_PATH_BLENDING` block after `block->rapid_rate = limit_max_rate_by_axis_maximum(unit_vec)`.

Current code (arc blocks):
```c
float min_rate  = min(X.max_rate, min(Y.max_rate, Z.max_rate));
float min_accel = min(X.acceleration, min(Y.acceleration, Z.acceleration));
float centripetal_rate = sqrtf(min_accel * block->arc.radius * 0.5f);
block->rapid_rate   = min(block->rapid_rate, min(min_rate, centripetal_rate));
block->acceleration = min(block->acceleration, min_accel * 0.5f);
```
Problems:
- X, Y and Z always limit the arc, even when it lies in the XY plane. On a machine with Z max rate 50 mm/s,
  every XY blend arc is capped at 50 mm/s, whatever its radius (example: a 30 deg corner at P1 has a 28 mm
  arc that acceleration alone would allow at 106 mm/s; G64 is then 16% slower than an exact stop).
- Likewise Z's (usually lower) acceleration limits both the tangential acceleration and the centripetal cap.
- `limit_*_by_axis_maximum(unit_vec)` uses the tangent at the arc START only, which is not valid along the arc
  (it is only safe because of the global minima above).

Proposal: limit the arc by each axis' actual share of the arc plane. With unit radial r and tangent t at the
arc start (they span the plane), amp_i = sqrt(r_i^2 + t_i^2) (0 for axes perpendicular to the plane, 1 for an
axis in the plane). Along the whole arc |axis velocity| <= amp_i v and
|axis acceleration| <= amp_i (|a_t| + v^2 / R). Keeping the current 50/50 split:
```c
// for each axis i with amp_i > 0:
rate_limit  = min_i(max_rate_i / amp_i);
accel_plane = min_i(acceleration_i / amp_i);
block->rapid_rate   = min(rate_limit, sqrtf(0.5f * accel_plane * radius));
block->acceleration = 0.5f * accel_plane;
```
This bounds every axis by its own limit at every point of the arc (proof above: each axis' velocity and
acceleration components are sinusoids of amplitude <= amp_i). Straight lines are unaffected (they already use
the per-axis contribution through the unit vector). Cost: a few multiplies and a sqrt per arc block.
Status: modelled (`arc_limits="planar"`), unit-tested, benchmarked - results below.

Results (`bench_firmware.py`, symmetric trapezoidal standard machine, `$11 = 0`, F100): fixes the worst case
(30 deg corner at P1: 0.642 -> 0.474 s; exact stop 0.556 s) and the zigzag at P1 (1.896 -> 1.665 s, -12%);
elsewhere 0.3-3% faster. With Z acceleration raised to Y's, gains appear only where the Z max rate (50 mm/s)
was the cap. FW-1 alone still leaves G64 slower than an exact stop on most corners: the 50/50 split remains.

### FW-2: exact acceleration bound on the arc (acceleration circle) and the centripetal share
On an arc, axis i's acceleration is a_t t_i(phi) - (v^2/R) r_i(phi): two sinusoids in quadrature with amplitude
amp_i each, so its exact maximum is amp_i sqrt(a_t^2 + (v^2/R)^2). With A_p = min_i A_i / amp_i (FW-1) every axis
stays within its limit iff the (tangential, centripetal) pair lies inside the circle
a_t^2 + (v^2/R)^2 <= A_p^2. The current code uses the sum a_t + v^2/R <= A_min, i.e. the point (0.5, 0.5), which
is well inside the circle (0.707). A constant block acceleration needs a fixed point on the circle:
```c
// k = centripetal share, 0 < k < 1 (proposed setting, default 0.9)
block->rapid_rate   = min(rate_limit, sqrtf(k * accel_plane * radius));
block->acceleration = sqrtf(1.0f - k * k) * accel_plane;
```
k = 0.707 dominates the current split on both terms. Larger k = faster constant-speed cornering
(cap sqrt(k A R): 95% of the physical maximum at k = 0.9) but less ability to change speed on the arc.

Measured (firmware model, `arc_limits="circle"`, time vs exact stop; worst axis acceleration / limit = 1.00 in
every run, and `test_circle_arc_limits_keep_every_axis_within_limits` checks it by finite differences):

| P1 | current | FW-1 | k 0.707 | k 0.9 | k 0.95 | k 0.99 |
|---|---|---|---|---|---|---|
| 90 deg axis | +4.4% | +1.4% | -2.8% | -5.6% | -6.1% | -6.6% |
| zigzag | +11.5% | -2.1% | -7.0% | -10.1% | -10.8% | -11.3% |
| random 1 | +3.8% | +2.4% | -5.0% | -9.6% | -10.6% | -11.2% |
| square | +3.8% | +1.1% | -4.1% | -7.3% | -8.0% | -8.6% |
| chords (0.2 mm) | -76.4% | -77.7% | -79.1% | -77.5% | -76.5% | -74.8% |

At P0.05 the same ordering with smaller gains (90 deg axis: +2.2% current, -0.4% at k 0.9). Chords (chains of
arcs with no lines between them, where the speed must change ON arcs) prefer more tangential budget; isolated
corners prefer k -> 1. Why k -> 1 suits an isolated corner: a constant-radius arc is capped at sqrt(A R) at
every point, including its end, so its exit speed can never exceed its cruise cap; the time-optimal profile is
decelerate on the line, cross the arc at constant speed at the centripetal limit, accelerate on the next line.

Options: (a) fixed k as a setting (default 0.9, consistent corner speeds); (b) per-arc k from the speed change the
plan needs on that arc (k high when cruising, lower in arc chains) - faster for chains but needs an extra
planner iteration and makes identical corners run differently depending on their neighbours. Trade-off for the
user: k also sets the lateral acceleration step at the line -> arc tangent point (0 -> k A instantly with
jerk off; current 0.5 A).

### FW-2 adaptive (recommended): the acceleration circle as the arc's speed-change law
Instead of a fixed point on the circle, let the arc use the whole circle: at speed v the tangential budget is
a_t = A_p sqrt(1 - w^2), w = v^2 / (A_p R) (so the effective k at each point is w itself). Then
d(v^2)/ds = 2 a_t gives dw/ds = (2/R) sqrt(1 - w^2), whose solution is closed form:

    asin(w1) = asin(w0) + 2 L / R        (w capped at 1; braking is the same law backward)

So the look-ahead / look-behind planner needs no iteration: in `planner_recalculate()` both passes replace
`v_next^2 + 2 a L` by `reach(block, v^2)`: unchanged for lines, `A_p R sin(min(pi/2, asin(v^2/(A_p R)) + 2L/R))`
for arc blocks (one asinf + sinf per arc block per pass). It is monotone in v^2, so the backward/forward
optimality and the `planned` pointer logic are unaffected. One setting remains: k_max, the cruise cap
v^2 <= k_max A_p R, i.e. the lateral acceleration step allowed where the line meets the arc (jerk off).

Behaviour: an isolated corner cruises the arc at its cap (identical to fixed k = k_max); arcs that must change
speed (chains of arcs, arcs entered from a lower speed) automatically shift budget to the tangential term.
Identical corners behave identically; differences appear only where the physics requires them.

Measured (firmware model `arc_limits="adaptive"`, time vs exact stop; `test_adaptive_arc_law_matches_direct_integration`
checks the closed form against an RK4 integration):

| | fixed k 0.9 | adaptive k_max 0.9 | adaptive k_max 0.99 | adaptive k_max 1 |
|---|---|---|---|---|
| 90 deg axis P1 | -5.6% | -5.6% | -6.6% | -6.7% |
| zigzag P1 | -10.1% | -10.1% | -11.3% | -11.4% |
| random 1 P1 | -9.6% | -9.6% | -11.2% | -11.4% |
| square P1 | -7.3% | -7.3% | -8.6% | -8.7% |
| chords (any P) | -77.5% | -80.4% | -80.4% | -80.4% |

(best fixed k on chords is 0.707: -79.1%.) Still slower than an exact stop at small P on one program
(random 2: +2.2% at P0.05 with k_max 0.9): FW-3 remains relevant.

Stepper impact (`st_prep_buffer`): ramp distances on arc blocks come from the same closed form
(d = (R/2)(asin(w_b) - asin(w_a))); within a ramp, compute the speed from the position on the curve
(v^2 = A_p R sin(asin(w_start) + 2 s / R)) rather than integrating a constant acceleration, so the executed
speed never leaves the circle (segment timing is approximated, the acceleration bound is not). The model has
not yet characterised that segment discretisation.

## Candidates (to quantify next)
- FW-2b: superseded by FW-2 adaptive.
- FW-3: blend-or-stop decision: skip the arc where it is slower than an exact stop (cheap local estimate).
  Re-evaluate after FW-1/FW-2; it may become unnecessary.
