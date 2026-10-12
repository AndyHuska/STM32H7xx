"""L6 streaming runtime over L5 plans: commit horizon, block arrival, feed override, hold and resume.

Execution is simulated in time. Invariants and policies: DESIGN.md section 3B.
"""

from __future__ import annotations

import math
from dataclasses import replace
from typing import Callable, Sequence

from .contracts import Lookahead, PlanResult, ScalarTimeLaw
from .model import AccelerationStep, Element, Limits, PathBlock, Phase, State
from .phases import split_at, trajectory

NORMAL, HOLD = "normal", "hold"
SLACK = 1e-9
REST_TOLERANCE = 1e-6


class InvariantError(RuntimeError):
    pass


class StreamingRuntime:
    def __init__(self, lookahead: Lookahead, law: ScalarTimeLaw, limits: Limits, commit_latency: float) -> None:
        self.lookahead, self.law, self.limits = lookahead, law, limits
        self.commit_latency = commit_latency
        self.blocks: list[PathBlock] = []
        self.queue_end = 0.0
        self.override = 1.0
        self.holding = False
        self.time = 0.0
        self.tail, self.tail_time = State(0.0, 0.0, 0.0), 0.0
        self.committed: list[Element] = []
        self.policies: list[str] = []
        self.suffix: tuple[Element, ...] = ()
        self.suffix_policy = NORMAL
        self.failures: list[PlanResult] = []
        self.replan_pending = False
        self.transients: list[tuple[float, float]] = []
        self.override_log: list[tuple[float, float]] = [(0.0, 1.0)]
        self.plans = 0
        self.witness_checks = 0
        self.on_witness: Callable[[State, tuple[Element, ...], str], None] | None = None

    @property
    def idle(self) -> bool:
        return not self.suffix and not self.replan_pending

    def append(self, block: PathBlock) -> None:
        self.blocks.append(block)
        self.queue_end += block.length
        if not self.holding:
            self._replan()

    def set_override(self, factor: float) -> None:
        """Takes effect from the committed tail once a plan honouring it is installed; `override_log` records
        that position. Until then the previous suffix (old caps) remains the committed stop witness."""
        self.override = factor
        if not self.holding:
            self._replan()

    def hold(self) -> None:
        """Acceleration-limited stop from the committed tail; jerk continuity is waived for it. Brakes on each
        block at that block's hold deceleration (curved pieces have less axis budget left). It is the
        shortest stop under those per-block limits; the planned witness obeys the same or stricter limits, so
        the hold ends no later than the witness, inside the queue."""
        self.holding = True
        elements: list[Element] = []
        state, s, index, ends = self.tail, 0.0, 0, []
        for block in self.blocks:
            s += block.length
            ends.append((s, self._hold_deceleration(block)))
        # A block reached within rounding is finished: braking starting on its end acts in the next block.
        while index < len(ends) and ends[index][0] <= state.s + SLACK:
            index += 1
        # A rounding-level speed is rest: no braking step (it would act on whatever block comes next).
        while state.v > SLACK:
            # Past the last block (tail on its end within rounding) the last block's rate still applies.
            end_s, deceleration = ends[min(index, len(ends) - 1)] if ends else (math.inf, self.limits.deceleration)
            elements.append(AccelerationStep(-deceleration))
            distance = max(0.0, end_s - state.s) if index < len(ends) else math.inf
            if state.v * state.v <= 2.0 * deceleration * distance * (1.0 + 1e-12):
                elements.append(Phase(0.0, state.v / deceleration))
                break
            elements.append(Phase(0.0, (state.v - math.sqrt(state.v * state.v - 2.0 * deceleration * distance)) / deceleration))
            state = trajectory(state, elements[-2:]).final
            index += 1
        elements.append(AccelerationStep(0.0))
        self.suffix, self.suffix_policy = tuple(elements), HOLD
        self._check_witness()

    def _hold_deceleration(self, block: PathBlock) -> float:
        if block.hold_deceleration is not None:
            return block.hold_deceleration
        return self.limits.deceleration if block.limits is None else block.limits.deceleration

    def resume(self) -> None:
        self.holding = False
        self._replan()

    def revise(self, from_s: float, blocks: Sequence[PathBlock], stop_at_from: bool) -> bool:
        """Replace the path from `from_s` on (geometry revision). Accepted only if nothing committed changes
        and either the current stop witness ends by from_s or a new plan succeeds; otherwise nothing changes."""
        if from_s < self.tail.s - SLACK:
            return False
        old = self.blocks, self.queue_end
        self.blocks = self._revised(from_s, blocks, stop_at_from)
        self.queue_end = sum(block.length for block in self.blocks)
        witness_end = trajectory(self.tail, self.suffix).final.s
        if witness_end <= from_s + REST_TOLERANCE:
            if not self.holding:
                self._replan()
            return True
        remaining = self._remaining_blocks()
        result = self.lookahead.plan(self.tail, remaining, self.limits, self.override) if remaining and not self.holding else None
        self.plans += result is not None
        if result is None or not result.ok:
            self.blocks, self.queue_end = old
            return False
        self._install(result)
        return True

    def preview(self, from_s: float, blocks: Sequence[PathBlock], stop_at_from: bool) -> PlanResult | None:
        """Plan from the committed tail as if the revision were applied; changes nothing. None while holding."""
        if from_s < self.tail.s - SLACK or self.holding:
            return None
        saved = self.blocks
        self.blocks = self._revised(from_s, blocks, stop_at_from)
        try:
            remaining = self._remaining_blocks()
            return self.lookahead.plan(self.tail, remaining, self.limits, self.override) if remaining else None
        finally:
            self.blocks = saved

    def _revised(self, from_s: float, blocks: Sequence[PathBlock], stop_at_from: bool) -> list[PathBlock]:
        kept, s = [], 0.0
        for block in self.blocks:
            end = s + block.length
            if end <= from_s + SLACK:
                kept.append(block)
            elif s < from_s - SLACK:
                kept.append(replace(block, length=from_s - s, stop_after=False))
            s = end
        if kept:
            # Stops are only added: an existing stop at from_s (exact stop vertex) is never cleared.
            kept[-1] = replace(kept[-1], stop_after=stop_at_from or kept[-1].stop_after)
        return kept + list(blocks)

    def advance(self, dt: float) -> None:
        self.time += dt
        horizon = self.time + self.commit_latency
        if horizon <= self.tail_time:
            return
        prefix, state, rest = split_at(self.tail, self.suffix, horizon - self.tail_time)
        self.committed.extend(prefix)
        self.policies.extend([self.suffix_policy] * len(prefix))
        self.tail, self.suffix = state, rest
        self.tail_time += sum(element.duration for element in prefix)
        if not rest and self.tail_time < horizon:
            if abs(state.v) > REST_TOLERANCE or abs(state.a) > REST_TOLERANCE:
                raise InvariantError(f"plan exhausted while moving: {state}")
            idle = Phase(0.0, horizon - self.tail_time)
            self.committed.append(idle)
            self.policies.append(self.suffix_policy)
            self.tail, self.tail_time = trajectory(state, (idle,)).final, horizon
        self._check_witness()
        if self.replan_pending and not self.holding:
            self._replan()

    def executed(self) -> tuple[State, tuple[Element, ...], tuple[str, ...]]:
        return State(0.0, 0.0, 0.0), tuple(self.committed), tuple(self.policies)

    def _remaining_blocks(self) -> list[PathBlock]:
        remaining, s = [], 0.0
        for block in self.blocks:
            end = s + block.length
            if end > self.tail.s + SLACK:
                length = end - max(s, self.tail.s)
                remaining.append(block if length == block.length else replace(block, length=length))
            s = end
        return remaining

    def _replan(self) -> None:
        remaining = self._remaining_blocks()
        if remaining:
            result = self.lookahead.plan(self.tail, remaining, self.limits, self.override)
            self.plans += 1
            if not result.ok:
                # The previous suffix is still a valid stop; retry from later tails (always solvable at rest).
                self.failures.append(result)
                self.replan_pending = True
                self._check_witness()
                return
            self._install(result)
            return
        self.replan_pending = False
        self._log_override()
        self._check_witness()

    def _install(self, result: PlanResult) -> None:
        self.suffix, self.suffix_policy = result.elements, NORMAL
        if result.transient_end is not None:
            self.transients.append((self.tail.s, result.transient_end))
        self.replan_pending = False
        self._log_override()
        self._check_witness()

    def _log_override(self) -> None:
        if self.override_log[-1][1] != self.override:
            self.override_log.append((self.tail.s, self.override))

    def _check_witness(self) -> None:
        """Every committed prefix must keep a stop: the suffix ends at rest within the known queue."""
        end = trajectory(self.tail, self.suffix).final
        if abs(end.v) > REST_TOLERANCE or abs(end.a) > REST_TOLERANCE or end.s > self.queue_end + REST_TOLERANCE:
            raise InvariantError(f"no stop witness from tail {self.tail}: suffix ends at {end}, queue end {self.queue_end}")
        self.witness_checks += 1
        if self.on_witness is not None:
            self.on_witness(self.tail, self.suffix, self.suffix_policy)


Event = tuple[float, Callable[[StreamingRuntime], None]]


def run(runtime: StreamingRuntime, events: Sequence[Event], dt: float, max_time: float) -> StreamingRuntime:
    """Advance in steps of dt, firing events at their times, until all events fired and the plan is done."""
    pending = sorted(events, key=lambda event: event[0])
    index = 0
    while runtime.time < max_time:
        while index < len(pending) and pending[index][0] <= runtime.time + 1e-12:
            pending[index][1](runtime)
            index += 1
        if index == len(pending) and runtime.idle and runtime.tail_time >= runtime.time:
            break
        runtime.advance(dt)
    return runtime
