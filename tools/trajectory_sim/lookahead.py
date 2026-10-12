"""L5 lookahead: conservative feasible schedule over path blocks (each with constant limits) ending at rest.

Algorithm and assumptions: DESIGN.md section 3B. Depends on the L2 contract only.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Sequence

from .contracts import PlanResult, ScalarTimeLaw, SolveStatus
from .model import Element, Limits, PathBlock, State
from .phases import ramp_distance, trajectory

SEARCH_ITERATIONS = 100
SLACK = 1e-9


@dataclass(frozen=True, slots=True)
class Region:
    """Consecutive blocks with identical effective limits (`limits.speed` is the cap) and no stop between.
    `ceiling` is the hard speed cap (planner and axis limits, not override-scaled) that transients keep."""

    start: float
    length: float
    limits: Limits
    stop_at_end: bool
    ceiling: float = math.inf

    @property
    def cap(self) -> float:
        return self.limits.speed

    @property
    def end(self) -> float:
        return self.start + self.length


def combined(limits: Sequence[Limits]) -> Limits:
    return Limits(min(l.speed for l in limits), min(l.acceleration for l in limits), min(l.jerk for l in limits),
                  min(l.deceleration for l in limits))


def block_limits(block: PathBlock, limits: Limits, override: float) -> Limits:
    base = limits if block.limits is None else combined((limits, block.limits))
    return Limits(min(base.speed, block.speed_cap * override), base.acceleration, base.jerk, base.deceleration)


def regions_of(start_s: float, blocks: Sequence[PathBlock], limits: Limits, override: float) -> list[Region]:
    regions: list[Region] = []
    s = start_s
    for block in blocks:
        effective = block_limits(block, limits, override)
        ceiling = min(limits.speed, math.inf if block.limits is None else block.limits.speed)
        last = regions[-1] if regions else None
        if last and not last.stop_at_end and last.limits == effective and last.ceiling == ceiling:
            regions[-1] = Region(last.start, last.length + block.length, effective, block.stop_after, ceiling)
        else:
            regions.append(Region(s, block.length, effective, block.stop_after, ceiling))
        s += block.length
    return regions


def _largest(predicate: Callable[[float], bool], low: float, high: float) -> float | None:
    """Largest x in [low, high] with predicate(x), assuming predicate holds on a prefix of the interval."""
    if predicate(high):
        return high
    if not predicate(low):
        return None
    for _ in range(SEARCH_ITERATIONS):
        middle = 0.5 * (low + high)
        if not low < middle < high:
            break
        if predicate(middle):
            low = middle
        else:
            high = middle
    return low


def _merge_first(regions: list[Region]) -> list[Region] | None:
    if len(regions) < 2 or regions[0].stop_at_end:
        return None
    first, second = regions[0], regions[1]
    merged = Region(first.start, first.length + second.length, combined((first.limits, second.limits)), second.stop_at_end,
                    min(first.ceiling, second.ceiling))
    return [merged] + regions[2:]


def _trim(regions: list[Region], s: float) -> list[Region]:
    kept = [region for region in regions if region.end > s + SLACK]
    if kept and kept[0].start < s:
        first = kept[0]
        kept[0] = Region(s, first.end - s, first.limits, first.stop_at_end, first.ceiling)
    return kept


class ConservativeLookahead:
    """Reference implementation of contracts.Lookahead for straight paths."""

    def __init__(self, law: ScalarTimeLaw) -> None:
        self.law = law

    def plan(self, start: State, blocks: Sequence[PathBlock], limits: Limits, override: float = 1.0) -> PlanResult:
        if not blocks or not (override > 0.0 and math.isfinite(override)):
            return PlanResult(SolveStatus.INVALID_INPUT, start, detail="no blocks or invalid override")
        regions = regions_of(start.s, blocks, limits, override)
        if not self._exceeds_cap(start, regions[0].limits):
            return self._schedule(start, (), start, regions, None)
        # Override transient to the lowest cap met; if the queue is then too short to continue, the transient
        # is a stop instead (always admissible when the committed witness was) and planning resumes from rest.
        result = None
        for to_rest in (False, True):
            outcome = self._transient(start, regions, to_rest)
            if isinstance(outcome, PlanResult):
                result = outcome
                continue
            elements, state = outcome
            rest = _trim(regions, state.s)
            if not rest:
                result = PlanResult(SolveStatus.OK if to_rest else SolveStatus.INSUFFICIENT_DISTANCE, start, elements,
                                    (start.v, state.v), state.s, "" if to_rest else "override transient reaches queue end")
            else:
                result = self._schedule(start, elements, state, rest, state.s)
            if result.ok:
                return result
        return result

    def _schedule(self, start: State, elements: tuple[Element, ...], state: State, regions: list[Region],
                  transient_end: float | None) -> PlanResult:
        """Plan from `state` (reached from `start` by `elements`) over regions not exceeded by the state."""
        queue_end = regions[-1].end
        while True:
            n = len(regions)
            bound = [0.0] * (n + 1)
            for k in range(n - 1, 0, -1):
                entry_cap = 0.0 if regions[k - 1].stop_at_end else min(regions[k - 1].cap, regions[k].cap)
                region, after = regions[k], bound[k + 1]
                fits = _largest(lambda u: ramp_distance(u, after, region.limits) <= region.length, after, max(after, region.cap))
                bound[k] = min(entry_cap, after if fits is None else fits)
            exit_first = self._first_exit(state, regions[0], bound[1])
            if exit_first is not None:
                break
            # No zero-acceleration exit from region 0: cross its end with the profile's own acceleration by
            # planning region 0 and 1 as one region at the lower limits (never across a stop or above the state).
            merged = _merge_first(regions)
            if merged is None or self._exceeds_cap(state, merged[0].limits) or not self._within(state, merged[0].limits):
                return PlanResult(SolveStatus.INSUFFICIENT_DISTANCE, start, elements,
                                  detail="committed state cannot reach a feasible boundary speed in region 0")
            regions = merged
        speeds = [state.v, exit_first]
        for k in range(1, n):
            region, entry = regions[k], speeds[k]
            reach = _largest(lambda w: ramp_distance(entry, w, region.limits) <= region.length, entry, max(entry, region.cap))
            speeds.append(min(bound[k + 1], entry if reach is None else reach))

        for k, region in enumerate(regions):
            goal = State(region.end if k < n - 1 else queue_end, speeds[k + 1], 0.0)
            result = self.law.distance_transition(state, goal, region.limits)
            if not result.ok:
                return PlanResult(result.status, start, elements, tuple(speeds), transient_end,
                                  f"region {k} [{region.start:.6g}, {region.end:.6g}]: {result.detail}")
            elements += result.phases
            state = trajectory(state, result.phases).final
        return PlanResult(SolveStatus.OK, start, elements, tuple(speeds), transient_end)

    @staticmethod
    def _exceeds_cap(state: State, limits: Limits) -> bool:
        rise = 0.0 if math.isinf(limits.jerk) or state.a <= 0.0 else state.a * state.a / (2.0 * limits.jerk)
        return state.v + rise > limits.speed + SLACK

    @staticmethod
    def _within(state: State, limits: Limits) -> bool:
        return -limits.deceleration - SLACK <= state.a <= limits.acceleration + SLACK

    def _first_exit(self, state: State, region: Region, bound: float) -> float | None:
        """Largest exit speed <= bound reachable within region 0.

        need(w) has two local minima on [0, bound]: w = 0 and the natural speed v* = v0 + a0|a0|/(2J) where a0
        ramps straight to 0. It rises above v*, and between 0 and v* it is hump-shaped (DESIGN.md 3B), so each
        basin is a feasible prefix of [minimum, bound] once higher basins are infeasible.
        """
        limits = region.limits

        def need(w: float) -> float:
            distance = self.law.minimum_distance(state, w, 0.0, limits)
            return math.inf if distance is None else distance

        # Same distance tolerance as L2 accepts: replans start exactly on the old plan's curve, so exact fits
        # are the common case and differ from the remaining length only by rounding.
        if need(bound) <= region.length + SLACK:
            return bound
        rise = 0.0 if math.isinf(limits.jerk) else state.a * abs(state.a) / (2.0 * limits.jerk)
        natural = min(bound, max(0.0, state.v + rise))
        for low in (natural, 0.0):
            if need(low) <= region.length + SLACK:
                # Strict inside the search: converging onto length + SLACK would land on L2's acceptance edge.
                found = _largest(lambda w: need(w) <= region.length, low, bound)
                return low if found is None else found
        return None

    def _transient(self, state: State, regions: list[Region],
                   to_rest: bool) -> tuple[tuple[Element, ...], State] | PlanResult:
        """Decelerate to the lowest cap met on the way (or to rest) under the lowest limits met; caps are
        lifted only for this transient."""
        crossed = regions[:1]
        for _ in range(len(regions) + 1):
            limits = combined([region.limits for region in crossed])
            target = 0.0 if to_rest else limits.speed
            rise = state.a * state.a / (2.0 * limits.jerk) if limits.jerk_limited and state.a > 0.0 else 0.0
            ceiling = min(region.ceiling for region in crossed)
            lifted = Limits(min(max(state.v + rise, target) + 1.0, ceiling), limits.acceleration, limits.jerk, limits.deceleration)
            result = self.law.velocity_transition(state, target, 0.0, lifted)
            if not result.ok:
                return PlanResult(result.status, state, detail=f"override transient: {result.detail}")
            end = trajectory(state, result.phases).final
            if end.s > regions[-1].end + SLACK:
                return PlanResult(SolveStatus.INSUFFICIENT_DISTANCE, state, detail="override transient passes the queue end")
            now = [region for region in regions if region.start < end.s - SLACK]
            if any(region.stop_at_end and region.end < end.s - SLACK for region in now):
                return PlanResult(SolveStatus.INSUFFICIENT_DISTANCE, state, detail="override transient crosses an exact stop")
            if len(now) <= len(crossed):
                return result.phases, end
            crossed = now
        return PlanResult(SolveStatus.NO_CONVERGENCE, state, detail="override transient target did not settle")
