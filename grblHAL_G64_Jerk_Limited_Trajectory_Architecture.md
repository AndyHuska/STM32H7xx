# grblHAL G64 and Jerk Limited Trajectory Architecture

Engineering plan \| STM32H743 eight axis controller \| 6 October 2026

Recommendation. Retain grblHAL as the machine control framework and replace the trajectory decision layer with a path geometry stage and a state continuous, jerk constrained time law. Develop the solver in the existing Python simulator before embedding it. LinuxCNC is a reference for G64 geometry and lookahead; Ruckig is a reference and candidate component for state to state jerk constrained motion. Neither repository is a drop in solution for the combined path following problem. [1–7]

## Problem and design target

The current grblHAL jerk option computes a block effective acceleration for its established lookahead, then executes a jerk shaped profile downstream. The planner block retains entry speed and a scalar acceleration estimate rather than a complete boundary state. This can create infeasible braking decisions and visible step rate changes at source block boundaries; discussion 384 reports slow jogging and other shortcomings. The reported rate snaps and final crawl are also reproducible in the project Python simulator. Treat these observations as regression inputs, not as proof of one particular code defect. [1–3]

The target is a continuous path with an executable time law. Its source G-code boundaries are metadata for events and reporting, not mandatory acceleration resets. The committed trajectory must remain feasible under feed override, hold, alarm, and finite queue depth. The existing machine interface, parsing, settings, Ethernet, IO and H743 pulse hardware remain useful.

## Proposed trajectory pipeline

| **Stage**                | **Input and output**                                           | **Responsibility**                                                                                                                  |
|--------------------------|----------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------|
| 1 Path geometry          | Parsed lines and arcs → q(s), path tags                        | Apply exact stop or G64 P blending; handle short moves; retain mapping to source commands and IO markers.                           |
| 2 Constraint projection  | q(s), axis limits → local speed and dynamic limits             | Evaluate axis velocity, acceleration and jerk along the actual path, including curvature terms.                                     |
| 3 Time law and lookahead | Path limits, queue horizon, current (s,v,a) → committed phases | Find reachable boundary states and feasible jerk phases; reserve enough future distance for stopping.                               |
| 4 Sample and step        | s(t) → axis positions/steps                                    | Convert path position to 8 axis targets and feed the existing step buffer; timestamp command completion at execution, not planning. |

Interface contract. Geometry supplies a continuous path coordinate s, a bounded deviation certificate for each blend, a mapping q(s) and derivatives where defined, a constraint envelope, and source command markers. The time law supplies ordered constant jerk phases and a resumable (s,v,a) state. The executor consumes only committed phases and returns actual executed position and marker completion.

## Specific algorithm choices

### Geometry and G64 tolerance

Start with exact stop and merged collinear lines. For a noncollinear G64 corner, trim the incoming and outgoing primitives and construct a tangent blend with a measured maximum deviation ≤ P from the programmed polyline. Cap trim length to avoid consuming adjacent short blocks. Recheck full deviation by analytic extrema or conservative certified subdivision; a corner radius formula alone is not enough. Compare LinuxCNC trajectory and blend geometry code and its G64 P Q semantics, but define this controller’s own documented P behavior and test it. [4, 5]

A line to circular arc connection is tangent continuous but its curvature changes abruptly, so Cartesian acceleration can jump at nonzero speed. For the first G64 release, explicitly bound the resulting acceleration change and corner speed; do not advertise strict axis jerk continuity across these transitions. A later curvature continuous blend, such as a quintic transition or clothoid family with verified q′, q″ and q‴, is required before claiming bounded Cartesian jerk everywhere. This is an engineering deduction from the derivative relations below.

### State continuous time law

Use one dimensional path progress with state (s,v,a). On constant jerk phase j for duration Δt: a₁=a₀+jΔt; v₁=v₀+a₀Δt+½jΔt²; s₁=s₀+v₀Δt+½a₀Δt²+(jΔt³)/6. Enforce phase duration ≥ 0 and continuity of s, v and a across source boundaries. A seven phase jerk limited state transition solver, including reduced phase cases, supplies a proven mathematical starting point; compare its solutions against Ruckig’s single DoF state to state trajectories and paper. [6, 7]

Do not solve each source block independently. Carry the full state through the queue. For a lookahead horizon, obtain conservative backward reachable states from the terminal stop condition and forward feasible states from the executed state; reduce peak speed or replan until both sets meet. Use a numerical feasibility oracle based on the exact state transition solver for short segments and changing local speed limits. Commit only the prefix whose stopping distance is supported by the available queue. Replan the uncommitted suffix for feed override and incoming blocks. Specify a deterministic bounded computation budget on the H743; stop safely on solver failure.

### Axis projection and curvature

For q=q(s), axis velocity is q′v; acceleration is q′a+q″v²; jerk is q′j+3q″va+q‴v³. Enforce each axis limit on these values across each path primitive and blend. Include mixed linear and rotary axis unit and feed semantics explicitly; do not apply a single Euclidean mm norm to XYZABCUV by default. Pump axes may need separate acceleration, deceleration and process constraints. Sampling checks must also bound values between sample instants.

## grblHAL integration boundaries

- Keep the G-code parser, runtime and driver interfaces. Introduce a feature gated trajectory module at the planner to segment boundary; avoid making the new time law a postprocessor of the old effective acceleration calculation. Audit planner.c, planner.h and stepper.c, especially st_prep_buffer, against a pinned core commit. [2, 3, 8]

- Retain the low level step timer, direction and pulse generation if measured rate changes and axis positions match the trajectory. The current step segment buffering approximates each interval at constant velocity; choose segment duration from an error bound and validate high rate quantization before claiming acceleration continuity in physical pulses. [8]

- Define dedicated paths for jog, homing, probe, synchronized spindle and emergency motion. The present jerk flag excludes some of these motions. Preserve hold and alarm stopping behavior before using the new planner in production. [2]

- Associate IO events with path coordinate or executed command markers. Fire completion callbacks when the executor crosses the marker; for an advance event, schedule against predicted trajectory time and revise it on replanning. Report timing error and prohibit a queued marker from firing after abort.

## Development milestones

| **Milestone**           | **Small deliverable and pass condition**                                                                                                                                                                       |
|-------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| M1 Capture baseline     | Freeze a core commit and simulator inputs. Export s, v, a, j, axis step rates, block IDs and queue depth for the known snap and 1.2 mm crawl cases. Reproduce both with deterministic fixtures.                |
| M2 State solver         | Implement scalar analytic phases in Python. Check exact endpoints, constraints, reduced phase cases, and continuity; cross check randomized feasible cases with Ruckig.                                        |
| M3 Queue lookahead      | Add forward/backward feasibility and finite horizon stopping. Long short long sequences at varied starting accelerations show no crawl or boundary reset.                                                      |
| M4 G64 geometry         | Add line line blends and short segment handling. Verify maximum deviation ≤ P and safe behavior at 0°, near 180°, tiny segments and queue starvation.                                                          |
| M5 Axis and event model | Project eight axis limits; test XYZ plus independent pump axes, completion markers, early IO events, override and hold replanning.                                                                             |
| M6 H743 shadow run      | Run the trajectory module beside the current planner without driving outputs. Compare planned versus executed positions, worst execution time, RAM, queue latency and step rate quantization.                  |
| M7 Controlled cutover   | Enable new execution behind a build or setting gate. Bench test homing, jog, alarm, feed hold and power loss; then run representative solder paths and soak tests. Retain a verified legacy mode for rollback. |

## Decision points and limits

At M2 decide whether a small embedded solver can meet the numerical and runtime budget or whether to integrate Ruckig for single state transitions. Ruckig Community uses a cloud API for intermediate waypoints, so do not base the offline embedded path planner on that feature. Ruckig is MIT licensed and uses C++20 in its current public code; confirm toolchain size and exceptions policy before integration. [6, 9]

At M4 decide whether a speed limited tangent arc satisfies current process needs. A guarantee of continuous Cartesian acceleration or jerk needs smoother curvature transitions and stronger proof. At M6 decide whether grblHAL segment generation can meet the measured pulse rate and event timing constraints; if it cannot, replace that narrow interface while preserving the broader controller.

Licensing is a design constraint: grblHAL core identifies GPL version 3 or later, and LinuxCNC tp.c identifies GPL version 2. Treat LinuxCNC as an algorithm and design reference until code reuse has a deliberate license review. This document is an engineering plan, not a license determination. [2, 4]

## Primary references

[1] grblHAL discussion 384, S Curve Jerk for Acceleration settings [https://github.com/grblHAL/core/discussions/384](https://github.com/grblHAL/core/discussions/384)

[2] grblHAL core planner.c, effective acceleration and jerk eligibility [https://github.com/grblHAL/core/blob/master/planner.c](https://github.com/grblHAL/core/blob/master/planner.c)

[3] grblHAL core planner.h, plan block state [https://github.com/grblHAL/core/blob/master/planner.h](https://github.com/grblHAL/core/blob/master/planner.h)

[4] LinuxCNC trajectory planner tp.c, blend and S curve calculations [https://github.com/LinuxCNC/linuxcnc/blob/master/src/emc/tp/tp.c](https://github.com/LinuxCNC/linuxcnc/blob/master/src/emc/tp/tp.c)

[5] LinuxCNC G code documentation, G64 P Q [https://github.com/LinuxCNC/linuxcnc/blob/master/docs/src/gcode/g-code.adoc](https://github.com/LinuxCNC/linuxcnc/blob/master/docs/src/gcode/g-code.adoc)

[6] Ruckig repository and README, API and waypoint limits [https://github.com/pantor/ruckig](https://github.com/pantor/ruckig)

[7] Berscheid and Kröger, RSS 2021 paper [https://roboticsproceedings.org/rss17/p015.html](https://roboticsproceedings.org/rss17/p015.html)

[8] grblHAL core stepper.h and config.h, step segment buffer [https://github.com/grblHAL/core/blob/master/stepper.h](https://github.com/grblHAL/core/blob/master/stepper.h)

[9] Ruckig license [https://github.com/pantor/ruckig/blob/main/LICENSE](https://github.com/pantor/ruckig/blob/main/LICENSE)

[10] LinuxCNC configuration, S curve planner settings [https://github.com/LinuxCNC/linuxcnc/blob/master/docs/src/config/ini-config.adoc](https://github.com/LinuxCNC/linuxcnc/blob/master/docs/src/config/ini-config.adoc)
