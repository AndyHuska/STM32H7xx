"""Closed-form constant-jerk propagation and complete scalar extrema (phases and acceleration steps)."""

from math import copysign, isinf, sqrt
from typing import Iterable

from .model import AccelerationStep, Element, Limits, Phase, Segment, State, Trajectory, require_finite


def ramp_distance(u: float, w: float, limits: Limits) -> float:
    """Distance of a minimum-time speed change u -> w with zero acceleration at both ends."""
    delta = abs(w - u)
    cap = limits.acceleration if w > u else limits.deceleration
    if isinf(limits.jerk):
        time = delta / cap
    elif delta <= cap * cap / limits.jerk:
        time = 2.0 * sqrt(delta / limits.jerk)
    else:
        time = delta / cap + cap / limits.jerk
    return 0.5 * (u + w) * time


def advance(start: State, phase: Element, time: float | None = None) -> State:
    elapsed = phase.duration if time is None else time
    require_finite(time=elapsed)
    if not 0.0 <= elapsed <= phase.duration:
        raise ValueError("sample time must lie inside the phase")
    if isinstance(phase, AccelerationStep):
        return State(start.s, start.v, phase.acceleration)
    return State(
        start.s + elapsed * (start.v + elapsed * (start.a / 2.0 + elapsed * phase.jerk / 6.0)),
        start.v + elapsed * (start.a + elapsed * phase.jerk / 2.0),
        start.a + elapsed * phase.jerk,
    )


def trajectory(initial: State, phases: Iterable[Element]) -> Trajectory:
    segments = []
    current = initial
    for phase in phases:
        end = advance(current, phase)
        segments.append(Segment(current, phase, end))
        current = end
    return Trajectory(initial, tuple(segments))


def propagate(start: State, elements: Iterable[Element]) -> tuple[float, float, float]:
    """Final (s, v, a) with arithmetic identical to `advance`, without building records (hot paths)."""
    s, v, a = start.s, start.v, start.a
    for element in elements:
        if isinstance(element, AccelerationStep):
            a = element.acceleration
            continue
        t, j = element.duration, element.jerk
        s, v, a = s + t * (v + t * (a / 2.0 + t * j / 6.0)), v + t * (a + t * j / 2.0), a + t * j
    return s, v, a


def split_at(start: State, elements: tuple[Element, ...], time: float) -> tuple[tuple[Element, ...], State, tuple[Element, ...]]:
    """Split at `time` into (prefix, state at time, suffix); steps at exactly `time` join the prefix."""
    prefix: list[Element] = []
    elapsed, state = 0.0, start
    for index, element in enumerate(elements):
        if elapsed + element.duration <= time:
            prefix.append(element)
            state = advance(state, element)
            elapsed += element.duration
            continue
        tau = time - elapsed
        if tau <= 0.0:
            return tuple(prefix), state, tuple(elements[index:])
        head, tail = Phase(element.jerk, tau), Phase(element.jerk, element.duration - tau)
        return tuple(prefix) + (head,), advance(state, head), (tail,) + tuple(elements[index + 1:])
    return tuple(prefix), state, ()


def critical_times(start: State, phase: Element) -> tuple[float, ...]:
    """Endpoints plus all interior position and velocity stationary points."""
    if isinstance(phase, AccelerationStep):
        return (0.0,)
    times = {0.0, phase.duration}
    if phase.jerk != 0.0:
        times.add(-start.a / phase.jerk)
        discriminant = start.a * start.a - 2.0 * phase.jerk * start.v
        require_finite(discriminant=discriminant)
        if discriminant >= 0.0:
            numerator = -start.a - copysign(sqrt(discriminant), start.a)
            if numerator != 0.0:
                times.add(numerator / phase.jerk)
                times.add(2.0 * start.v / numerator)
            else:
                times.add(0.0)
    elif start.a != 0.0:
        times.add(-start.v / start.a)
    return tuple(sorted(time for time in times if 0.0 <= time <= phase.duration))


def extrema(start: State, phase: Element) -> tuple[State, State]:
    """Return component-wise minima and maxima, not two simultaneous states.

    For a step both the pre-step and post-step acceleration count.
    """
    states = [advance(start, phase, time) for time in critical_times(start, phase)]
    if isinstance(phase, AccelerationStep):
        states.append(start)
    return (
        State(min(state.s for state in states), min(state.v for state in states), min(state.a for state in states)),
        State(max(state.s for state in states), max(state.v for state in states), max(state.a for state in states)),
    )