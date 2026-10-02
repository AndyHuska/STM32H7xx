# G64 Path Blending Progress

## Status

Phase 1 implementation is in progress. The current branch contains the parser, settings, planner metadata, deferred G1 resolution, native-arc scaffolding, execution-ID ownership, and synchronized-IO boundary work needed to continue development. It is not yet a hardware-validated production feature.

The implementation is limited to consecutive XYZ-only `G1` moves in `G64`. Unsupported geometry falls back to exact motion. Rotary axes, arc-to-line blending, line-to-arc blending, arc-to-arc blending, and Naive CAM `Q` simplification remain out of scope.

## Implemented

### Feature and parser foundations

- `ENABLE_PATH_BLENDING` is now overridable by the build environment instead of being unconditionally forced off by `grbl/config.h`.
- `G61` and `G61.1` parsing is enabled when the feature is compiled in.
- The G61 mantissa validation defect was corrected so both `G61` and `G61.1` are accepted.
- `G61.1` is carried into planner metadata as an explicit exact-stop boundary and forces zero junction speed at the affected programmed boundary.
- `G64 P` accepts a nonnegative tolerance, rejects unsupported `Q`, converts active inch units and scaling into internal millimeter tolerance, and uses a configurable default when P is omitted.
- A `$780` G64 default tolerance setting was added using reserved persistent bytes, preserving `settings_t` size and the existing settings version. Existing NVS data without the marker falls back to `DEFAULT_PATH_TOLERANCE`, currently `0.01 mm`.
- `$11` remains the legacy junction-deviation setting and is not reused as the G64 geometric tolerance.

### Deferred G1 corner resolution

A single eligible G1 is held before planner insertion. When the next eligible G1 arrives:

1. The incoming line is shortened to the first tangent point.
2. A native arc primitive covers the first half of the tangent fillet.
3. A second native arc primitive covers the second half of the fillet.
4. The outgoing line continues from the second tangent point.

The fillet is bounded by the active tolerance and available adjacent-line lengths. Degenerate, collinear, reversing, too-short, mixed-axis, and otherwise unsupported cases flush the pending line unchanged.

Pending candidates flush before non-linear motion, G64 mode changes, dwell, synchronization, parser errors, reset, abort, and cancellation. Arc and canned-cycle internal `mc_line()` calls are excluded from top-level G1 deferral.

### Planner and stepper support

- Planner blocks can carry native arc metadata: center, radial vector, normal, radius, and sweep.
- Planner arc blocks use arc length and endpoint tangent vectors for lookahead input.
- Conservative XYZ velocity and acceleration limits are applied to native arcs.
- A planner batch API preflights capacity and commits all corner primitives before one lookahead recalculation.
- Stepper preparation interpolates native arc points per segment and derives per-segment XYZ Bresenham deltas and direction.
- Arc segments refresh their direction, event count, counters, and step vectors at each segment boundary; ordinary line segments retain the existing path.

### Execution IDs and synchronized IO

- The preceding host command owns the incoming shortened line and first half-arc.
- The following host command owns the second half-arc and outgoing line.
- The angular midpoint is the execution ownership handoff.
- M62/M63/M67 output associated with the following source line is attached to the midpoint-owned second half-arc, so it executes at the handoff and is not duplicated on the first half.
- Existing execution ledger/FIFO behavior remains the accounting mechanism; generated primitives do not create new host-visible IDs.
- Deferred source execution effects are pre-counted before parser closure so `[EXEC:N]` cannot be emitted before the deferred geometry is committed.

## Current Build Validation

The feature-enabled Waveshare environment was built successfully in an isolated PlatformIO output directory:

```text
platformio run -c platformio.netcheck.ini -j 8 -s -e waveshare_openh743i_c
PIO_EXIT=0
```

The normal workspace build can fail when an active debug session locks `.pio/build/waveshare_openh743i_c/firmware.elf`. The isolated build avoids that artifact lock. The temporary validation config is removed after each build.

Editor diagnostics and diff checks passed for the current modified core files. The project still emits its existing selected-spindle warnings.

A non-blending target build was attempted but stalled during PlatformIO dependency installation and did not produce a reliable compatibility result. It still needs a clean validation run.

## Required Testing

No automated geometry or planner test harness was found in the repository. Testing currently requires a host protocol harness, debugger instrumentation, and hardware observations.

### Parser and mode tests

- `G61` accepts exact path mode.
- `G61.1` accepts exact stop mode and forces zero-speed programmed boundaries.
- `G64` without P selects the configured `$780` tolerance.
- `G64 P0` disables geometric blending.
- Positive `G64 P` overrides the default.
- Negative, non-finite, and out-of-range P values are rejected.
- Inch-mode P is converted to millimeters.
- G64 mode transitions flush pending candidates.
- `$11` changes G61 junction behavior independently of G64 P.
- G64 Q is rejected until Naive CAM is implemented.

### Geometry tests

- 90-degree X-to-Y corner.
- Shallow-angle corner.
- Nearly collinear moves.
- Reversal and near-reversal.
- Zero-length line.
- Very short adjacent lines.
- Asymmetric incoming/outgoing lengths.
- Consecutive corners where trims would overlap.
- Verify tangent continuity at both tangent points and the midpoint.
- Verify maximum geometric deviation does not exceed active P.
- Verify unsupported rotary or mixed XYZ/rotary motion falls back to exact lines.

### Planner and stepper tests

- Planner nearly full when a four-primitive corner is resolved.
- No partial corner batch becomes visible.
- Lookahead remains active across generated lines and arcs.
- Axis velocity and acceleration limits are respected over the native arc.
- Hold during the incoming line, first half-arc, second half-arc, and outgoing line.
- Resume does not complete an unfinished primitive early.
- Reset, E-stop, alarm, and planner flush discard unretired internal geometry.
- Existing ordinary lines and existing G2/G3 arc behavior remain unchanged.

### Execution and synchronized IO tests

- Two tagged G1 lines around a corner produce exactly one `[EXEC:N]` per source line.
- The preceding ID completes at the midpoint after the first half-arc retires.
- The following ID completes only after the second half-arc and outgoing line retire.
- Untagged lines do not produce execution events.
- File-generated line numbers do not become execution IDs.
- M62/M63/M67 on the following source line apply once at the midpoint.
- Same-line synchronized output plus motion still produces one source completion.
- Aborted or discarded internal geometry produces no false completion.
- `Exec:<active>,<last_completed>` remains consistent throughout the transition.

## Follow-On Phases

### Phase 2: More geometric combinations

- Line-to-arc blending.
- Arc-to-line blending.
- Arc-to-arc blending.
- Better overlap optimization for multiple short consecutive segments.
- Explicit handling for supported helical arcs.
- Formal policy for mixed XYZ and rotary axes, or continued exact fallback.
- More complete native-arc constraint analysis over changing axis rates.

### Phase 3: Naive CAM and advanced trajectory control

- Implement G64 Q Naive CAM simplification for many short, nearly-collinear moves.
- Add configurable simplification/error policies distinct from geometric corner fillets.
- Integrate jerk-limited or S-curve acceleration where supported.
- Improve blend batching and memory/capacity behavior for large generated primitive runs.
- Add deterministic planner/geometry unit tests and a repeatable host protocol test harness.

## Known Risks

- Native arc execution has been added to the current stepper path but still requires hardware validation for step quantization, arc endpoint accuracy, acceleration timing, direction changes, and hold/resume.
- The planner’s existing block accounting is line-oriented. Every generated primitive must continue to contribute exactly one retirement effect to its owning source ID.
- Synchronized IO timing depends on preserving one output-list owner at the midpoint. Any future block reordering must retain that invariant.
- The current feature has no automated geometric regression suite.
- The implementation should not be considered complete until the hardware success criterion is demonstrated: a 90-degree G64 corner shows simultaneous X/Y motion, smooth tangent transition, deviation within P, correct midpoint IO timing, and correct execution callbacks.
