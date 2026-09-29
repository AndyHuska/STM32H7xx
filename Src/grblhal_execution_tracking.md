# Execution Completion Tracking

## Purpose

Execution tracking lets a host distinguish a G-code line that has been accepted from one whose tracked effects have actually executed. It reuses the host-supplied G-code `N` value as the execution ID and does not change the normal `ok` response, planner lookahead, blending, or streaming behavior.

The existing design proposal is in [`grblhal_execution_tracking_design.md`](grblhal_execution_tracking_design.md). This document records the implemented contract and architecture.

## Protocol

Every realtime status report includes:

```text
|Exec:<active_id>,<last_completed_id>
```

For example:

```text
<Run|MPos:...|Exec:1002,1001>
```

`active_id` is the tagged command currently executing in the stepper/output path. It is `0` when no tagged command is active. `last_completed_id` is the most recently completed tagged command, in actual completion order. The value is retained across soft abort/reset handling for diagnostics. Status processing drains pending completion records before taking the execution-state snapshot.

Successful completion is also reported asynchronously:

```text
[EXEC:<id>]
```

Cancellation reports the oldest unfinished tagged command in acceptance order:

```text
[EXEC_ABORT:<id>]
```

A bounded tracking-queue or ledger overflow is made visible with:

```text
[EXEC_OVERFLOW]
```

## IDs and Scope

- Only an explicit, positive G-code `N` word is an execution ID. The original parsed value is captured before existing line-number callbacks can replace or synthesize line numbers.
- `N0` and commands without an explicit `N` are untagged. They do not emit execution events or change `last_completed_id`.
- File-stream-generated line numbers remain available to existing `Ln` reporting but are not treated as host execution IDs.
- Tracking covers planner-backed motion, dwell, synchronized and immediate IO, and successful spindle/coolant changes where those effects are handled by the parser.
- Informational and realtime requests do not create execution events.

The ID uses the existing `line_number_t` type (`uint32_t`). IDs are not ordered numerically; rollover and non-monotonic host IDs are handled using acceptance order where ordering is needed.

## Completion Semantics

Parsing, planner acceptance, planner discard, and stepper preparation do not complete a command. A planner block contributes a pending effect when it is accepted. Segment generation marks `block_end` only when the planner block reaches its normal end; a partial feed-hold termination does not receive that marker. The stepper ISR queues a completion only when the marked segment is actually retired.

A logical command can create multiple planner blocks, as with arcs, splines, canned cycles, threading, probing, or backlash compensation. The tracking ledger counts accepted blocks and other tracked effects against the command's ID. The parser closes the logical command after dispatch; one `[EXEC:id]` is emitted only when the command is closed and every counted effect has completed. This avoids requiring each motion generator to predict its final internal block.

Completion IDs are queued by the ISR into a 16-entry FIFO. The ISR does not format or write protocol messages. Foreground processing drains the FIFO, updates `last_completed_id`, and emits completion messages in actual completion order.

The acceptance-order ledger has 1,024 entries, exceeding the configured maximum of 1,000 planner blocks. Both the completion FIFO and ledger are bounded; exhaustion is reported rather than silently treated as successful completion.

## IO and Dwell

Synchronized M62/M63/M67 output commands retain their source execution ID and preserve the existing behavior of waiting for a later motion block. The ID completes when that output command is applied at the synchronized stepper execution point. If the same tagged line also creates motion, both effects are counted and produce one event after the motion finishes.

Stepper preparation transfers ownership of synchronized output-command lists to the stepper block so planner cleanup cannot free them before ISR application. Lists are reclaimed in foreground processing or reset cleanup.

Immediate M64/M65/M68 completion is queued after the corresponding output API call reports success. Spindle and coolant changes are similarly tracked after their synchronized setters succeed. G4 dwell remains active through the wait and completes after the delay; internal canned-cycle dwells retain the enclosing command's ID rather than creating separate events.

## Hold, Abort, and Reset

Feed hold does not clear the active ID or complete a partially executed block. The ID remains active through the hold and is completed only after resumed motion reaches its normal final segment.

Stepper reset and protocol abort paths clear active and unfinished tracking state. Already-confirmed completion records are drained first, unfinished work does not emit `[EXEC:id]`, and `last_completed_id` is preserved. `[EXEC_ABORT:id]` identifies the earliest unfinished ledger entry by acceptance order, not by numeric ID comparison.

## Implementation Map

- [`../grbl/gcode.c`](../grbl/gcode.c) captures explicit N values, opens/closes logical tracking entries, and tracks IO, dwell, spindle, and coolant effects.
- [`../grbl/gcode.h`](../grbl/gcode.h) carries execution IDs on synchronized output commands.
- [`../grbl/planner.h`](../grbl/planner.h) and [`../grbl/planner.c`](../grbl/planner.c) propagate IDs and count accepted planner blocks.
- [`../grbl/motion_control.h`](../grbl/motion_control.h) and [`../grbl/motion_control.c`](../grbl/motion_control.c) keep dwell IDs active at the actual wait boundary.
- [`../grbl/stepper.h`](../grbl/stepper.h) and [`../grbl/stepper.c`](../grbl/stepper.c) own the active state, acceptance ledger, ISR-safe FIFOs, block-end retirement, foreground event output, and reset cleanup.
- [`../grbl/grbllib.c`](../grbl/grbllib.c) drains pending completions from the foreground realtime task.
- [`../grbl/report.c`](../grbl/report.c) adds the unconditional realtime `Exec` field and drains pending events before the status snapshot.
- [`../grbl/protocol.c`](../grbl/protocol.c) integrates abort reporting with reset handling.

## Implementation Notes and Limitations

The tracker is implemented in the existing stepper module rather than a separate source module. Logical multi-block completion uses per-ID effect counts plus parser closure rather than an explicit `execution_id_final` flag on planner blocks; the per-segment `block_end` flag still identifies actual planner-block retirement.

The digital output API returns whether a HAL callback is installed, not a hardware acknowledgment. Therefore `[EXEC:id]` for digital output confirms the callback was invoked, not an independently acknowledged physical output transition. The current HAL API provides no later acknowledgment to wait for.

## Validation

The configured Waveshare target was built successfully with:

```text
platformio run -j 16 -e waveshare_openh743i_c
```

The final build reported 100,060 bytes RAM and 457,156 bytes flash used. Editor diagnostics and `git diff --check` were clean. Runtime protocol tests for rapid short blocks, arc/cycle completion, IO timing, feed hold/resume, and reset/abort still require a host-stream harness and hardware run; they were not performed as part of this implementation.