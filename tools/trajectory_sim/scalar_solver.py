"""L2 scalar time law: analytic jerk-limited transitions of path progress (s, v, a).

Implements contracts.ScalarTimeLaw. Units: mm, s. Profiles are built only from
constant-jerk phases with |j| = J or j = 0, so the jerk limit holds by construction;
with `limits.jerk = inf` ramps become `AccelerationStep`s (constant-acceleration motion).
Speed bounds are checked over full phases and reported, never clamped.
"""

from __future__ import annotations

import math
import sys

from .contracts import SolveResult, SolveStatus
from .model import AccelerationStep, Element, Limits, Phase, State
from .phases import extrema, propagate, trajectory

SPEED_SLACK = 1e-9      # mm/s, mm/s^2: admissible rounding beyond a limit
DISTANCE_SLACK = 1e-9   # mm: admissible rounding when matching a requested distance
MAX_BISECTIONS = 200
GOLDEN_ITERATIONS = 60


def _compact(elements: tuple[Element | None, ...]) -> tuple[Element, ...]:
    """Drop empty phases and merge adjacent steps (only the last value is ever observed)."""
    result: list[Element] = []
    for element in elements:
        if element is None or (isinstance(element, Phase) and element.duration <= 0.0):
            continue
        if isinstance(element, AccelerationStep) and result and isinstance(result[-1], AccelerationStep):
            result[-1] = element
        else:
            result.append(element)
    return tuple(result)


def _ramp(a_from: float, a_to: float, jerk: float) -> Element | None:
    if a_to == a_from:
        return None
    if math.isinf(jerk):
        return AccelerationStep(a_to)
    return Phase(math.copysign(jerk, a_to - a_from), abs(a_to - a_from) / jerk)


def acceleration_profile(a0: float, a1: float, dv: float, limits: Limits) -> tuple[Element, ...]:
    """Minimum-time ramp/plateau/ramp profile from acceleration a0 to a1 changing speed by dv.

    The peak lies above both endpoints when dv exceeds the direct a0->a1 ramp's speed change,
    otherwise below both; this one-switch structure is the time-optimal family. With unlimited
    jerk the formulas reduce to step/plateau/step (direct change 0, peak at the cap).
    """
    jerk = limits.jerk
    direct = (a0 + a1) * abs(a1 - a0) / (2.0 * jerk)
    # Ramp durations grow like sqrt(dv/J): pure float rounding in dv must not become a detour. Keep this
    # at rounding scale: a larger slack is an unmodelled speed step that optimisers will exploit.
    if abs(dv - direct) <= 64.0 * sys.float_info.epsilon * (limits.speed + abs(direct) + 1.0):
        return _compact((_ramp(a0, a1, jerk),))
    sign = 1.0 if dv > direct else -1.0
    cap = limits.acceleration if sign > 0 else limits.deceleration
    peak = sign * math.sqrt(max(0.0, sign * jerk * dv + (a0 * a0 + a1 * a1) / 2.0))
    plateau = 0.0
    if abs(peak) > cap:
        peak = sign * cap
        plateau = max(0.0, (dv - sign * (2.0 * cap * cap - a0 * a0 - a1 * a1) / (2.0 * jerk)) / peak)
    return _compact((_ramp(a0, peak, jerk), Phase(0.0, plateau), _ramp(peak, a1, jerk)))


def _speed_status(start: State, phases: tuple[Element, ...], limits: Limits) -> tuple[SolveStatus, str]:
    low, high = math.inf, -math.inf
    for segment in trajectory(start, phases).segments:
        minimum, maximum = extrema(segment.start, segment.phase)
        low, high = min(low, minimum.v), max(high, maximum.v)
    if low < -SPEED_SLACK:
        return SolveStatus.REVERSE_PROGRESS, f"minimum speed {low:.6g} mm/s"
    if high > limits.speed + SPEED_SLACK:
        return SolveStatus.SPEED_LIMIT, f"maximum speed {high:.6g} mm/s exceeds {limits.speed:.6g}"
    return SolveStatus.OK, ""


def _forced_failure(start_v: float, start_a: float, goal_v: float, goal_a: float, limits: Limits) -> SolveStatus | None:
    """Failure shared by every admissible profile: acceleration needs >= a^2/(2J) of speed to reach 0."""
    rise_start = start_a * start_a / (2.0 * limits.jerk)
    rise_goal = goal_a * goal_a / (2.0 * limits.jerk)
    if (start_a < 0.0 and start_v < rise_start - SPEED_SLACK) or (goal_a > 0.0 and goal_v < rise_goal - SPEED_SLACK):
        return SolveStatus.REVERSE_PROGRESS
    if (start_a > 0.0 and start_v + rise_start > limits.speed + SPEED_SLACK) or \
            (goal_a < 0.0 and goal_v + rise_goal > limits.speed + SPEED_SLACK):
        return SolveStatus.SPEED_LIMIT
    return None


def _invalid_boundary(state_v: float, state_a: float, limits: Limits) -> bool:
    return (not (math.isfinite(state_v) and math.isfinite(state_a))
            or state_v < -SPEED_SLACK or state_v > limits.speed + SPEED_SLACK
            or state_a > limits.acceleration + SPEED_SLACK or state_a < -limits.deceleration - SPEED_SLACK)


class AnalyticTimeLaw:
    """Reference float64 implementation of contracts.ScalarTimeLaw."""

    def velocity_transition(self, start: State, v1: float, a1: float, limits: Limits) -> SolveResult:
        if _invalid_boundary(start.v, start.a, limits) or _invalid_boundary(v1, a1, limits):
            return SolveResult(SolveStatus.INVALID_INPUT, start, detail="boundary state outside limits")
        phases = acceleration_profile(start.a, a1, v1 - start.v, limits)
        status, detail = _speed_status(start, phases, limits)
        proven = status is not SolveStatus.OK and _forced_failure(start.v, start.a, v1, a1, limits) is status
        return SolveResult(status, start, phases, detail, proven_infeasible=proven)

    def stop(self, start: State, limits: Limits) -> SolveResult:
        return self.velocity_transition(start, 0.0, 0.0, limits)

    def peak_profile(self, start: State, goal: State, peak: float, cruise: float, limits: Limits) -> tuple[Element, ...]:
        """Conservative composition: reach speed `peak` with a = 0, cruise, then reach goal (v, a).

        Not time-optimal (acceleration is forced through zero at the peak). Complete only for
        v0 == v1 with zero boundary accelerations; continuous in `peak`.
        """
        return _compact(acceleration_profile(start.a, 0.0, peak - start.v, limits)
                        + (Phase(0.0, cruise),)
                        + acceleration_profile(0.0, goal.a, goal.v - peak, limits))

    def _travelled(self, start: State, goal: State, peak: float, limits: Limits) -> float:
        return propagate(start, self.peak_profile(start, goal, peak, 0.0, limits))[0] - start.s

    def best_peak(self, start: State, goal: State, limits: Limits) -> float | None:
        """Cruise speed minimising this family's distance, or None if the goal forces speed above the cap.

        Travelled distance is not monotone in the cruise speed (below v0 + a0|a0|/(2J) the start half takes
        its no-a=0 branch), so this is a golden-section search plus physically meaningful candidates;
        unimodality is checked by tests, and `distance_transition` relies only on continuity above it.
        """
        low = max(0.0, goal.v - goal.a * abs(goal.a) / (2.0 * limits.jerk))
        if low > limits.speed + SPEED_SLACK:
            return None
        low, high = min(low, limits.speed), limits.speed

        def travelled(peak: float) -> float:
            return self._travelled(start, goal, peak, limits)

        ratio = (math.sqrt(5.0) - 1.0) / 2.0
        a, b = low, high
        x1, x2 = b - ratio * (b - a), a + ratio * (b - a)
        f1, f2 = travelled(x1), travelled(x2)
        for _ in range(GOLDEN_ITERATIONS):
            if f1 <= f2:
                b, x2, f2 = x2, x1, f1
                x1 = b - ratio * (b - a)
                f1 = travelled(x1)
            else:
                a, x1, f1 = x1, x2, f2
                x2 = a + ratio * (b - a)
                f2 = travelled(x2)
        natural = start.v + start.a * abs(start.a) / (2.0 * limits.jerk)
        golden = 0.5 * (a + b)
        preferred = [low] + [c for c in (natural, start.v, goal.v) if low <= c <= high] + [high, golden]
        distances = [travelled(c) for c in preferred]
        minimum = min(distances)
        # Exact physical candidates win ties: near them durations grow like sqrt(dc), so a rounding-level
        # gain at a nearby golden point would become a microsecond detour.
        return next(c for c, d in zip(preferred, distances) if d <= minimum + DISTANCE_SLACK)

    def minimum_distance(self, start: State, v1: float, a1: float, limits: Limits) -> float | None:
        if _invalid_boundary(start.v, start.a, limits) or _invalid_boundary(v1, a1, limits):
            return None
        goal = State(start.s, v1, a1)
        peak = self.best_peak(start, goal, limits)
        if peak is None:
            return None
        phases = self.peak_profile(start, goal, peak, 0.0, limits)
        if _speed_status(start, phases, limits)[0] is not SolveStatus.OK:
            return None
        return trajectory(start, phases).final.s - start.s

    def distance_transition(self, start: State, goal: State, limits: Limits) -> SolveResult:
        if _invalid_boundary(start.v, start.a, limits) or _invalid_boundary(goal.v, goal.a, limits):
            return SolveResult(SolveStatus.INVALID_INPUT, start, detail="boundary state outside limits")
        distance = goal.s - start.s
        if distance < -DISTANCE_SLACK:
            return SolveResult(SolveStatus.INVALID_INPUT, start, detail="goal behind start")

        def travelled(peak: float) -> float:
            return self._travelled(start, goal, peak, limits)

        forced = _forced_failure(start.v, start.a, goal.v, goal.a, limits)
        low = self.best_peak(start, goal, limits)
        if low is None:
            low = max(0.0, goal.v - goal.a * abs(goal.a) / (2.0 * limits.jerk))
            return SolveResult(SolveStatus.SPEED_LIMIT, start, self.peak_profile(start, goal, low, 0.0, limits),
                               f"goal acceleration forces speed {low:.6g} mm/s in this family",
                               proven_infeasible=forced is SolveStatus.SPEED_LIMIT)
        low_distance = travelled(low)
        if low_distance > distance + DISTANCE_SLACK:
            return SolveResult(SolveStatus.INSUFFICIENT_DISTANCE, start, self.peak_profile(start, goal, low, 0.0, limits),
                               f"needs {low_distance:.9g} mm, has {distance:.9g} mm (cruise-at-zero-acceleration family)")

        iterations = 0
        if low_distance >= distance - DISTANCE_SLACK:
            # Durations grow like sqrt(peak - low) here, so rounding-level surplus must not trigger a search.
            peak = low
        else:
            # Above v* = v0 + a0|a0|/(2J) distance rises with c and gives the time-optimal (peak) profile;
            # use the valley side [best, v*] only when the distance is too short for it.
            natural = min(limits.speed, max(low, start.v + start.a * abs(start.a) / (2.0 * limits.jerk)))
            natural_distance = travelled(natural)
            if abs(natural_distance - distance) <= DISTANCE_SLACK:
                low = high = natural
            elif natural_distance < distance:
                low, high = natural, limits.speed
            else:
                high = natural
            if low == high:
                peak = low
            elif travelled(high) <= distance:
                peak = high
            else:
                # Invariant: travelled(low) <= distance < travelled(high); continuity guarantees a root.
                while iterations < MAX_BISECTIONS:
                    middle = 0.5 * (low + high)
                    if not low < middle < high:
                        break
                    if travelled(middle) <= distance:
                        low = middle
                    else:
                        high = middle
                    iterations += 1
                peak = low
        residual = distance - travelled(peak)
        if residual > DISTANCE_SLACK and peak <= 0.0:
            return SolveResult(SolveStatus.NO_CONVERGENCE, start, detail="zero peak with remaining distance", iterations=iterations)
        # A cruise shorter than the distance slack is rounding, not motion; keep phases meaningful.
        cruise = residual / peak if peak > 0.0 and residual > DISTANCE_SLACK else 0.0
        phases = self.peak_profile(start, goal, peak, cruise, limits)
        status, detail = _speed_status(start, phases, limits)
        return SolveResult(status, start, phases, detail, iterations,
                           proven_infeasible=status is not SolveStatus.OK and forced is status)
