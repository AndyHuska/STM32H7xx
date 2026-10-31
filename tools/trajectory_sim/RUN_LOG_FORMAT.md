# Unified Run Log CSV Format

Version 1 of the unified run log records the commanded test program and normalized controller observations in one interleaved CSV file. The trajectory viewer uses the move and event records to reconstruct and simulate the program, then compares the simulation with the status samples.

The format is firmware-independent. The client writes normalized values; the viewer does not parse raw status-message strings from either firmware.

## File Rules

- One file represents one run from one firmware build.
- The file is UTF-8 CSV with a single header row, a comma delimiter, a period as the decimal separator, and standard CSV quoting. UTF-8 BOM is discouraged.
- Every record uses the same columns below. Columns not used by that record type are empty, not zero.
- Physical values use canonical units: positions and distances in millimeters, feed and speed in millimeters per second, acceleration in millimeters per second squared, jerk in millimeters per second cubed, and times in milliseconds. Convert inches and per-minute rates in the client before writing.
- Coordinates are absolute in the frame named by the `run` record. All move and status coordinates must use that same frame. The frame should be a stable identifier such as `machine` or `work`; do not mix frames within a file.
- `sequence` is an integer starting at zero and increasing by one for every record, including metadata and events. It preserves write order when timestamps are equal.
- `elapsed_ms` is a nonnegative, monotonic duration from the client's run clock origin. It may repeat when events occur together. Use a monotonic clock, not wall-clock time. Record the time the client observes or emits each row; the semantics for moves and completion events are specified below.
- Numeric values use invariant decimal notation. `NaN` and infinities are not valid data, except `max_jerk_mm_s3` may be the literal `inf` to mean that the axis has no configured jerk limit.
- Empty fields mean unavailable or not applicable. Readers must not infer zero from an empty field.
- Records may be streamed and interleaved after the preamble. The `run` record, all `setting` records, and all `axis_limit` records must appear before the first `path_start`, `move`, `event`, or `status` record.
- A log may end without `run_end` if a run is interrupted. Such a file is incomplete but can still be inspected; the viewer should indicate that it ended without a normal close.

## Header

The header is exact and case-sensitive. New writers should keep all columns, even when a particular run does not use them. `blend_k` is an optional trailing extension to version 1; readers also accept the original header without this final column. Existing column positions are unchanged.

```csv
record_type,sequence,elapsed_ms,run_id,path_id,path_name,move_id,event,move_kind,frame,schema_version,firmware,build_id,machine_id,setting_name,setting_value,axis,max_speed_mm_s,max_accel_mm_s2,max_decel_mm_s2,max_jerk_mm_s3,start_x_mm,start_y_mm,start_z_mm,end_x_mm,end_y_mm,end_z_mm,feed_mm_s,blend_mode,blend_tolerance_mm,stop_after,dwell_ms,x_mm,y_mm,z_mm,path_speed_mm_s,blend_k
```

## Record Types

### `run`

Exactly one `run` record is required, and it must be the first data row. It identifies the run and declares its coordinate frame and schema version.

Required fields:

- `record_type`: `run`
- `sequence`: `0`
- `elapsed_ms`: `0`
- `run_id`: unique identifier for this run
- `frame`: coordinate frame used by every position in this file
- `schema_version`: `1`
- `firmware`: firmware family/name, for example `grblHAL`
- `build_id`: version, commit, or other build identifier

`machine_id` is optional because the client may not know a simulator profile. If the client knows a stable machine identity, include it. Other columns are empty on this row.

### `setting`

Zero or more `setting` records follow the `run` record and precede runtime records. They preserve relevant firmware and planner settings as client-normalized key/value pairs.

Required fields: `record_type=setting`, `sequence`, `elapsed_ms=0`, `run_id`, `setting_name`, and `setting_value`.

Use stable setting names where possible. Include units in the name when a setting is numeric and not self-evident, for example `junction_deviation_mm`, `steps_per_mm_x`, or `path_blending_enabled`. Values are text and must not contain unescaped CSV delimiters. These records are informational unless the viewer explicitly recognizes a setting.

### `axis_limit`

One record per coordinated axis, before runtime records. This lets the viewer construct a matching machine model without requiring the client to know the viewer's machine-profile name.

Required fields: `record_type=axis_limit`, `sequence`, `elapsed_ms=0`, `run_id`, `axis`, and the applicable limits:

- `max_speed_mm_s`
- `max_accel_mm_s2`
- `max_decel_mm_s2`
- `max_jerk_mm_s3`

Acceleration, deceleration, and speed limits must be positive. Use the same acceleration value for acceleration and deceleration if the firmware has a symmetric limit. Use `inf` for jerk only when jerk is unlimited. Axis names are case-sensitive identifiers such as `X`, `Y`, and `Z`.

The viewer may use manual machine selection when axis limits are absent or incomplete, but it must make that fallback visible; it must not silently substitute a profile.

### `path_start` and `path_end` events

Every named test path has a `path_start` event and, on normal completion, a `path_end` event. Paths are ordered by their records in the file and may have descriptive names such as `L01` and `L02`.

Required for both: `record_type=event`, `sequence`, `elapsed_ms`, `run_id`, `path_id`, `path_name`, and `event` (`path_start` or `path_end`). `path_id` must be stable and unique within the run. Other fields are empty.

A path can contain several moves and other events. These events identify the client's logical test-path/queue grouping; their timestamps do not identify when the path begins or ends physical execution. A following path may begin executing before the preceding path's final completion callback when motion is blended. `path_end` does not imply that the entire run has ended.

### `move`

One `move` record represents one commanded straight-line XYZ move. Record both endpoints in the declared coordinate frame, including the start point, so the reader can detect discontinuities instead of silently joining unrelated moves.

Record every commanded move in queue order, including the approach to the first test path, travel between paths, and exit/retract moves. Do not omit commute moves just because they are outside the test geometry; they are part of the simulated run. Use `path_id` to group a move with the logical path it serves, while remembering that path boundaries do not imply execution stops.

Required fields: `record_type=move`, `sequence`, `elapsed_ms`, `run_id`, `path_id`, `move_id`, `move_kind`, `start_x_mm`, `start_y_mm`, `start_z_mm`, `end_x_mm`, `end_y_mm`, and `end_z_mm`.

- `move_id` is unique within the run and is referenced by its `move_complete` event.
- `move_kind` is `linear` or `rapid` in version 1. Version 1 models straight XYZ moves only.
- For `linear`, `feed_mm_s` is required and positive.
- For `rapid`, leave `feed_mm_s` empty. The viewer uses the selected or recorded axis speed limits to model the rapid; if no suitable limits are available, it must report that the rapid cannot be simulated rather than inventing a feed.
- `blend_mode` is `exact` or `g64`. For `g64`, `blend_tolerance_mm` is required and positive. For `exact`, leave `blend_tolerance_mm` empty.
- `blend_k` is the optional, dimensionless G64 K arc-sizing share, a finite number from `0` to `1` inclusive. Record the effective active value on each G64 move, including the firmware default used when the G64 command omits K. Leave it empty for exact moves, non-move records, or when the value is unavailable. Empty means unknown, not zero.
- `blend_k=0` selects the largest blend arc allowed by P and the adjacent move lengths. A positive K allows the firmware to choose a smaller arc sufficient for the programmed feed at that centripetal acceleration share, still bounded by P and move lengths. The firmware caps the sizing share at its arc cruise share. This is distinct from `PATH_BLEND_ARC_KMAX`, which limits arc speed; do not substitute one for the other. For example, `G64 P2 K0.8` is recorded as `blend_mode=g64`, `blend_tolerance_mm=2`, `blend_k=0.8` (with P converted to mm if needed).
- The Blended Constant Accel simulator applies each move's `blend_k` to circular arc sizing, using the incoming command's value at a junction as the firmware does. K=0 retains the largest arc allowed by P and adjacent lengths; a missing value retains the model default (currently zero for logged runs). Blended S-Curve is a separate clothoid algorithm and does not apply this circular-arc sizing rule. The exact-stop baseline remains unchanged.
- `stop_after` is `true` or `false`; use `true` when the controller must come to a stop after this move. A dwell or an explicit stop should be represented as an event as well.
- `elapsed_ms` is when the client queues/records the command. It does not identify the move's physical start time because the controller may queue commands ahead of execution.

A zero-length move is valid and represents a queued no-motion command; retain its `move_id` because it may still produce a completion callback. Model it as zero-duration geometry without discarding the event identity. A repeated `move_id`, a start point that does not match the preceding move endpoint, or a move referring to an inactive `path_id` is an invalid record sequence. The viewer should report the offending sequence and ID.

### `event`

Events express execution boundaries and non-move timing that are needed to interpret the run.

Supported `event` values in version 1:

- `run_start` and `run_end`
- `path_start` and `path_end` (described above)
- `move_complete`
- `dwell_start` and `dwell_end`
- `stop`

For a `move_complete`, `move_id` is required. Its `elapsed_ms` is when the client receives the controller's completion notification for that command. This is an execution synchronization point, unlike the `move` row's issue time. A move must not have more than one `move_complete` event.

For `dwell_start`, `dwell_ms` is required and nonnegative. It applies at the position after the preceding move in the active path, or at the path's initial position if no move has yet occurred; the reader treats it as an exact stop even if `stop_after` was false. `dwell_end` is optional when the controller reports completion separately; if emitted, its timestamp should reflect the observed end. `stop` marks an explicit stop boundary not already represented by `stop_after` or a dwell and likewise forces an exact stop after the preceding move.

`run_start` and `run_end` have no required fields beyond the common `record_type`, `sequence`, `elapsed_ms`, and `run_id`. A file may omit `run_end` if interrupted.

### `status`

One `status` record represents one normalized controller sample. Status samples are sparse observations; they are not expected to arrive at a fixed interval.

Required fields: `record_type=status`, `sequence`, `elapsed_ms`, `run_id`, `x_mm`, `y_mm`, and `z_mm`. `path_speed_mm_s` is recommended and may be empty if unavailable. It is the scalar path speed, not an individual axis velocity. The client must express positions in the run's declared frame and speed in mm/s, regardless of the source firmware's raw status format.

Status rows do not require a `path_id`: samples can occur between paths or while the controller is transitioning. Do not fabricate a path association when the client cannot determine one.

## Timing And Comparison

The run clock must be shared by all record types. In particular, command queue/issue times, completion notifications, and status sample times must all use the same monotonic elapsed-time origin. `path_start`, `path_end`, and `move` timestamps describe record/queue order only; do not use them as motion start/end times or to clip observed status samples.

The viewer initializes the simulated position from the first status sample and preserves any initial idle interval. It detects motion start from the first sustained status change: nonzero scalar path speed or an XYZ change larger than the available position-resolution threshold in consecutive samples. It delays the simulated trace with a stationary, zero-speed prefix to that elapsed time; it must not trim the real samples or use queued move/path timestamps as an execution start.

The viewer must not treat a move's issue timestamp as its physical start time. The controller's `move_complete` event is the only per-move execution boundary in version 1: it means the referenced move has fully executed, while the following queued move may already have begun due to blending. Completion callbacks remain useful for validating later simulated move boundaries, but do not replace status-based motion-start detection. If a move has no completion event, its execution is unconfirmed by this log; do not reject the partial capture or infer a completion time. Retain unshifted simulated durations and observed elapsed times; alignment must not alter either series' measured durations.

Position comparison uses the absolute XYZ values directly because move endpoints and status samples share the same frame. Speed comparison uses `path_speed_mm_s` and the simulated scalar path speed. Preserve original sample times, including irregular intervals and repeated timestamps; do not resample the recorded data merely to make it look like the simulation's denser trace.

## Example

This abbreviated sequence shows metadata, two sequential test paths, a move-completion synchronization event, and interleaved status samples. It lists named field values rather than repeating empty CSV cells; the fixed header above remains the exact serialization contract.

| Sequence | Record | Example values |
| ---: | --- | --- |
| 0 | `run` | `run_id=run-20261013-01`, `frame=work`, `schema_version=1`, `firmware=grblHAL`, `build_id=commit-abc123` |
| 1-3 | `axis_limit` | X/Y: speed 200, acceleration/deceleration 1000, jerk `inf`; Z: speed 80, acceleration/deceleration 500, jerk `inf` (mm-based units) |
| 4 | `setting` | `setting_name=junction_deviation_mm`, `setting_value=0.01` |
| 5-6 | `event` | `run_start`, then `path_start` for `path_id=L01`, `path_name=Test L01` |
| 7 | `move` | `move_id=M001`, `path_id=L01`, linear from (0, 0, 0) to (10, 0, 0) mm, feed 169.333333 mm/s, G64 tolerance 0.254 mm |
| 8-9 | `status` | At 20 ms: (0.2, 0, 0) mm, 5 mm/s; at 53 ms: (1.1, 0, 0) mm, 80 mm/s |
| 10-11 | `event` | `move_complete` for `M001` at 72 ms, then `path_end` for `L01` |
| 12 | `event` | `path_start` for `path_id=L02`, `path_name=Test L02` |
| 13 | `move` | `move_id=M002`, `path_id=L02`, rapid from (10, 0, 0) to (12, 2, 0) mm, exact blend mode, `stop_after=true` |
| 14 | `status` | At 140 ms: (11.8, 1.8, 0) mm, 0 mm/s |

These values are illustrative. A production file should include the actual axis limits, coordinates, and closing events for normally completed paths and runs. Serialize records using the exact header and column ordering above, leaving unused columns empty.

## Version 1 Scope

- Supported commanded geometry: straight XYZ moves only (`linear` and `rapid`). Arcs and non-Cartesian axes require a later schema revision.
- Supported comparison channels: absolute XYZ position and scalar path speed versus elapsed time.
- Firmware-specific raw status messages, per-axis velocity, temperatures, digital I/O, and other status fields are outside this format version. Add normalized columns or a versioned extension only when the viewer needs them.
- A status-only CSV without commanded move rows is not a unified run log and cannot be used to reconstruct the simulation path.
