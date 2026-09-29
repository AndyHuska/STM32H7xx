Implement host-visible execution tracking in this grblHAL firmware.

Goal: let the client distinguish:

- command accepted/queued
- command currently executing
- command actually completed
The client needs this to show the active execution line in the UI and synchronize external operations with real controller execution.

Use the existing G-code `N` number as the host execution ID.

Example:
`N1001 G1 X100 Y100 F5000`

Keep normal `ok` behavior unchanged

## Protocol
Add realtime status field:

`|Exec:<active_id>,<last_completed_id>`

Example:
`<Run|MPos:...|F:5080|AV:...|Exec:1002,1001>`

Meaning:
- active execution ID = 1002
- last completed ID = 1001
When nothing tagged is executing:
`|Exec:0,1001`

Also emit asynchronous completion:

`[EXEC:<id>]`

Example:
`[EXEC:1001]`

Also include abort reporting:
`[EXEC_ABORT:<first_aborted_id>]`

## Critical execution semantics
Do NOT report completion when:

- G-code is parsed
- planner accepts the block
- planner discards the block
- stepper preparation consumes the planner block
grblHAL buffers planner blocks into stepper blocks and segments ahead of actual motion.

A motion command is complete only when the final stepper segment associated with that logical host command has actually finished execution.

Do not use `plan_discard_current_block()` as the completion point.

## Reuse existing line-number plumbing
Inspect the current codebase first.

grblHAL already carries `N` / line-number information through parts of the parser/planner.

Reuse that existing field/type where possible.

Trace the execution ID through:

parser
→ `plan_line_data_t`
→ `plan_block_t`
→ stepper execution block
→ stepper segment(s)

Extend structures only where needed so the ID survives to actual execution.

## Active execution ID
Track the ID of the logical command currently being executed by the stepper/output engine.

Do not derive this from the planner tail.

Conceptually:

`active_exec_id`

When no tagged command is actively executing:
`active_exec_id = 0`

During feed hold, keep the same active ID. Hold must not count as completion.

## Motion completion
A planner/stepper block can generate multiple segments.

Add or reuse metadata so the execution layer knows when a segment is the final segment of its block, for example:

`last_segment_of_block`

Only when that final segment actually finishes should the corresponding block become eligible for completion.

Do not print from the step ISR.

Instead, enqueue the completed ID into a small ISR-safe FIFO/ring buffer.

Example concept:

`exec_completion_fifo[16]`

ISR work should be minimal:
`push_completed_id(id)`

Foreground/protocol code drains the FIFO and emits:
`[EXEC:id]`

Use existing grblHAL atomic/critical-section conventions. No heap, blocking, logging, or string formatting in the ISR.

## Logical command vs internal blocks
A single host command may create multiple internal planner blocks.

Example:
`N500 G2 ...`

An arc may be expanded into several internal blocks/chords.

The host must receive exactly one:
`[EXEC:500]`

and only after the entire logical command finishes.

Carry metadata indicating whether an internal block is the final block belonging to that execution ID.

Conceptually:

`execution_id`
`execution_id_final`

Example:

`N500 -> block A final=false`
`block B final=false`
`block C final=true`

Only the final block can generate `[EXEC:500]`.

Apply the same principle to canned cycles or any command that expands internally.

## IO completion
Support both synchronized and immediate IO.

### Synchronized IO
If an IO command is carried through planner/stepper execution, preserve its execution ID and report completion when the IO operation actually executes.

Example:
`N700 M62 P3`

Emit:
`[EXEC:700]`

at the synchronized execution point, not parse time.

### Immediate IO
For intentionally immediate IO commands, completion may be emitted immediately after the hardware output change succeeds.

Still preserve:

- `ok` = accepted
- `[EXEC:id]` = applied/executed
If one logical tagged line contains both motion and synchronized IO, emit only one completion event, after all effects of that line are complete.

## Reset / abort / alarm behavior
Queued or interrupted commands must never be falsely reported complete.

On reset, abort, E-stop, alarm flush, planner cancellation, etc.:

- clear active execution state
- clear pending completion state as appropriate
- do not emit EXEC for discarded work
- optionally emit `[EXEC_ABORT:<first_aborted_id>]`
- preserve `last_completed_id` for diagnostics unless existing reset behavior strongly suggests otherwise
Feed hold is different:

- keep current active ID
- do not complete it
- resume continues the same ID

## Untagged commands
Commands without `N` must continue to work exactly as they do now.

Do not emit `[EXEC:0]`.

Untagged commands must not overwrite `last_completed_id`.

Do not generate execution events for informational commands such as:
`?`
`$G`
`$$`
`$I`

Execution tracking is primarily for:

- motion
- dwell
- synchronized IO
- immediate IO where completion matters
- spindle/coolant/output commands where applicable

## ID type
Use the existing grblHAL line-number type if appropriate, preferably at least 32 bits.

The host controls ID generation.

Do not depend on simple greater-than comparisons for rollover handling.

## Preserve streaming/lookahead
This feature must be observational only.

Do not:

- wait for host acknowledgment
- serialize execution
- disable planner lookahead
- disable blending
- wait for planner empty
- infer completion from machine position or Idle
The controller should still accept multiple tagged commands ahead of execution.  All may receive `ok` before any of them complete.  Execution events should later arrive in actual execution order.

Feed hold:
- active ID remains current command
- no completion while held
- completion only after resume and finish
Abort:
- unfinished IDs do not emit EXEC
- optional EXEC_ABORT reports where execution was invalidated
Arc:
`N500 G2 ...`

Expected:

- exactly one `[EXEC:500]`
- only after the complete arc finishes
Dwell:
`N600 G4 ...`

Expected:

- completion after dwell time expires
Synchronized IO:
`N700 ...`

Expected:

- completion at the actual IO execution point
Very short blocks:

- ensure FIFO preserves multiple rapid completions without losing IDs
Realtime recovery:

- even if the host misses `[EXEC:id]`, the next status report must expose:
`Exec:<active>,<last_completed>`

## Implementation guidance
Before editing, identify:

- where `N` is parsed
- `plan_line_data_t`
- `plan_block_t`
- `st_block_t`
- stepper segment type
- planner-to-stepper conversion
- where segments are generated
- where segment execution actually finishes
- synchronized output-command structures
- realtime status assembly
- foreground protocol/feedback output
- reset/alarm/planner-flush paths
- existing ring-buffer helpers
Implement the smallest maintainable change.

Realtime status independently reports:

`Exec:<active>,<last_completed>`

