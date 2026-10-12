"""Independent certificate checks in double precision with explicit rounding bounds (firmware-portable);
no production propagation imports."""

from dataclasses import dataclass
import math
import sys

from .model import AccelerationStep, Limits, State, Trajectory, require_finite

EPS = sys.float_info.epsilon
GAMMA = 16.0 * EPS  # relative error bound for evaluating a sum of <= 4 products (<= 8 roundings) in double


@dataclass(frozen=True, slots=True)
class Tolerance:
    position: float = 1e-9
    speed: float = 1e-9
    acceleration: float = 1e-9
    jerk: float = 1e-9
    relative: float = 1e-12

    def __post_init__(self) -> None:
        values = (self.position, self.speed, self.acceleration, self.jerk, self.relative)
        require_finite(**dict(zip(("position", "speed", "acceleration", "jerk", "relative"), values)))
        if any(value < 0 for value in values):
            raise ValueError("tolerances must be nonnegative")


@dataclass(frozen=True, slots=True)
class Issue:
    code: str
    segment: int | None
    message: str
    time: float | None = None


class ValidationError(ValueError):
    pass


@dataclass(frozen=True, slots=True)
class Report:
    issues: tuple[Issue, ...]

    @property
    def valid(self) -> bool:
        return not self.issues

    def require_valid(self) -> None:
        if self.issues:
            raise ValidationError("; ".join(f"{issue.code}[{issue.segment}]: {issue.message}" for issue in self.issues))


def _matches(actual: float, expected: float, absolute: float, relative: float, rounding: float = 0.0) -> bool:
    """|actual - expected| within tolerance, widened by `rounding` (a bound of the error in `expected`)."""
    return abs(actual - expected) <= absolute + relative * abs(expected) + rounding + GAMMA * abs(expected)


def validate(
    path: Trajectory,
    limits: Limits,
    expected_final: State | None = None,
    tolerance: Tolerance = Tolerance(),
) -> Report:
    """Certify scalar phase bounds and declared states within explicit tolerances, in double.

    Speed is quadratic: endpoints and the interior zero-acceleration point exhaust its extrema.
    Acceleration is affine; jerk is constant. Limit checks are conservative: each value carries a
    forward rounding bound (GAMMA times the sum of term magnitudes) and fails unless the bound proves it
    within tolerance. Equality checks (continuity, dynamics, endpoint) accept differences up to the
    tolerance plus that rounding bound. Not a certificate for axis constraints or pulse timing.
    """
    issues: list[Issue] = []
    absolute_by_component = {"s": tolerance.position, "v": tolerance.speed, "a": tolerance.acceleration}

    def check_state(actual: State, expected: State, code: str, segment: int | None) -> None:
        for component, absolute in absolute_by_component.items():
            if not _matches(getattr(actual, component), getattr(expected, component), absolute, tolerance.relative):
                issues.append(Issue(code, segment, f"{component}: {getattr(actual, component)} != {getattr(expected, component)}"))

    speed_slack = tolerance.speed + tolerance.relative * limits.speed
    acceleration_slack = tolerance.acceleration + tolerance.relative * limits.acceleration
    deceleration_slack = tolerance.acceleration + tolerance.relative * limits.deceleration
    jerk_slack = tolerance.jerk + tolerance.relative * limits.jerk
    previous = path.initial
    for index, segment in enumerate(path.segments):
        check_state(segment.start, previous, "continuity", index)
        s0, v0, a0 = segment.start.s, segment.start.v, segment.start.a
        step = isinstance(segment.phase, AccelerationStep)
        if step:
            jerk, duration = 0.0, 0.0
            predicted = ((s0, 0.0), (v0, 0.0), (segment.phase.acceleration, 0.0))
        else:
            jerk, duration = segment.phase.jerk, segment.phase.duration
            t = duration
            s_terms = (v0 * t, a0 * t * t / 2.0, jerk * t * t * t / 6.0)
            v_terms = (a0 * t, jerk * t * t / 2.0)
            predicted = (
                (s0 + s_terms[0] + s_terms[1] + s_terms[2], GAMMA * (abs(s0) + sum(map(abs, s_terms)))),
                (v0 + v_terms[0] + v_terms[1], GAMMA * (abs(v0) + sum(map(abs, v_terms)))),
                (a0 + jerk * t, GAMMA * (abs(a0) + abs(jerk * t))),
            )
        for component, (expected, rounding) in zip(("s", "v", "a"), predicted):
            if not _matches(getattr(segment.end, component), expected, absolute_by_component[component], tolerance.relative, rounding):
                issues.append(Issue("dynamics", index, f"{component}: declared endpoint differs from integrated phase"))

        speeds = [(0.0, v0, 0.0), (duration, predicted[1][0], predicted[1][1])]
        if jerk != 0.0:
            vertex = -a0 / jerk
            if -GAMMA * duration < vertex < duration * (1.0 + GAMMA):
                # Closed form at the vertex; including a near-boundary vertex is conservative either way.
                rise = a0 * a0 / (2.0 * jerk)
                speeds.append((vertex, v0 - rise, GAMMA * (abs(v0) + abs(rise))))
        for elapsed, value, rounding in sorted(speeds):
            if value - rounding < -speed_slack:
                issues.append(Issue("reverse_progress", index, f"speed {value} is negative", elapsed))
            if value + rounding > limits.speed + speed_slack:
                issues.append(Issue("speed_limit", index, f"speed {value} exceeds {limits.speed}", elapsed))
        for value, rounding in ((a0, 0.0), predicted[2]):
            if value + rounding > limits.acceleration + acceleration_slack or value - rounding < -limits.deceleration - deceleration_slack:
                issues.append(Issue("acceleration_limit", index, f"acceleration {value} outside [-{limits.deceleration}, {limits.acceleration}]"))
        if step and limits.jerk != math.inf:
            issues.append(Issue("jerk_limit", index, f"acceleration step with jerk limit {limits.jerk}"))
        elif abs(jerk) > limits.jerk + jerk_slack:
            issues.append(Issue("jerk_limit", index, f"jerk {jerk} exceeds +/-{limits.jerk}"))
        previous = segment.end

    if not path.segments:
        if path.initial.v < -tolerance.speed:
            issues.append(Issue("reverse_progress", None, "initial speed is negative"))
        if path.initial.v > limits.speed + speed_slack:
            issues.append(Issue("speed_limit", None, "initial speed exceeds limit"))
        if not -limits.deceleration - deceleration_slack <= path.initial.a <= limits.acceleration + acceleration_slack:
            issues.append(Issue("acceleration_limit", None, "initial acceleration outside limits"))
    if expected_final is not None:
        check_state(path.final, expected_final, "endpoint", None)

    return Report(tuple(issues))


def validate_caps(path: Trajectory, caps: list[tuple[float, float, float]], tolerance: Tolerance = Tolerance()) -> Report:
    """Certify v <= cap wherever s may lie in each closed interval (s_from, s_to, cap); s must be nondecreasing.
    Same rigorous double bounds as validate_envelope."""
    issues: list[Issue] = []
    for overlap in _overlaps(path, [(low, high) for low, high, _ in caps], tolerance.position):
        low, high, cap = caps[overlap.interval]
        if _exceeds(overlap.speed, cap, tolerance.speed, tolerance):
            issues.append(Issue("speed_cap", overlap.segment, f"speed {overlap.speed:.9g} exceeds cap {cap} in [{low}, {high}]"))
    return Report(tuple(issues))


def _up(*terms: float) -> float:
    """Upper bound of the exact sum of nonnegative terms that were each computed with a few roundings."""
    return math.fsum(terms) * (1.0 + GAMMA)


@dataclass(frozen=True, slots=True)
class _Overlap:
    """Rigorous double bounds of one segment's motion while s may lie in one closed interval.

    The time window is widened so that it contains the true overlap; extrema over a superset are
    conservative. `speed` is an upper bound of the maximum speed, `acceleration_low/high` bound the
    acceleration range, `jerk` is exact (an input value)."""

    segment: int
    interval: int
    speed: float
    acceleration_low: float
    acceleration_high: float
    jerk: float
    step: bool


def _overlaps(path: Trajectory, intervals: list[tuple[float, float]], slack: float):
    """Yield every possible (segment, interval) overlap.

    Boundary attribution, at the declared position tolerance `slack` (L2 matches distances only to its own
    slack, so a phase may end or start a hair across a piece boundary): a phase overlapping an interval by
    <= `slack` is not attributed to it - speed and acceleration are continuous, so the phases inside the
    interval check the same boundary values. A step is a jerk discontinuity of an interval only strictly
    inside it; its pre-step acceleration belongs to the interval it leaves, the post-step one to the
    interval it enters. Intervals no longer than `slack` take everything that overlaps them."""
    order = sorted(range(len(intervals)), key=lambda k: intervals[k][0])
    for index, segment in enumerate(path.segments):
        s0, v0, a0 = segment.start.s, segment.start.v, segment.start.a
        step = isinstance(segment.phase, AccelerationStep)
        jerk, duration = (0.0, 0.0) if step else (segment.phase.jerk, segment.phase.duration)

        def position(t: float) -> tuple[float, float]:
            terms = (v0 * t, a0 * t * t / 2.0, jerk * t * t * t / 6.0)
            value = s0 + terms[0] + terms[1] + terms[2]
            return value, GAMMA * (abs(s0) + sum(abs(x) for x in terms))

        def speed(t: float) -> float:
            terms = (a0 * t, jerk * t * t / 2.0)
            return v0 + terms[0] + terms[1] + GAMMA * (abs(v0) + abs(terms[0]) + abs(terms[1]))

        def last_below(target: float) -> float:
            """Largest bisection time with s certainly < target (so the true crossing is later)."""
            low, high = 0.0, duration
            for _ in range(200):
                middle = 0.5 * (low + high)
                if not low < middle < high:
                    break
                value, error = position(middle)
                if value + error < target:
                    low = middle
                else:
                    high = middle
            return low

        def first_above(target: float) -> float:
            """Smallest bisection time with s certainly > target (so the true crossing is earlier)."""
            low, high = 0.0, duration
            for _ in range(200):
                middle = 0.5 * (low + high)
                if not low < middle < high:
                    break
                value, error = position(middle)
                if value - error > target:
                    high = middle
                else:
                    low = middle
            return high

        start_error = GAMMA * abs(s0)
        s_end, end_error = position(duration)
        for k in order:
            lower, upper = intervals[k]
            if lower > s_end + end_error:
                break
            if s_end + end_error < lower or s0 - start_error > upper:
                continue
            tiny = upper - lower <= slack
            if step:
                post = segment.phase.acceleration
                sides = [a for a, acts in ((a0, tiny or s0 > lower + slack), (post, tiny or s0 < upper - slack)) if acts]
                inside = tiny or lower + slack < s0 < upper - slack
                if sides:
                    yield _Overlap(index, k, v0, min(sides), max(sides), 0.0, inside)
                continue
            touch = not tiny and min(s_end, upper) - max(s0, lower) <= slack
            if touch:
                continue
            t_from = 0.0 if s0 + start_error >= lower else last_below(lower)
            t_to = duration if s_end - end_error <= upper else first_above(upper)
            peak = max(speed(t_from), speed(t_to))
            if jerk < 0.0:
                vertex = -a0 / jerk
                if t_from <= vertex * (1.0 + GAMMA) and vertex * (1.0 - GAMMA) <= t_to:
                    # Closed form avoids evaluating at a rounded vertex time.
                    rise = a0 * a0 / (2.0 * -jerk)
                    peak = max(peak, v0 + rise + GAMMA * (abs(v0) + rise))
            ends = [a0 + jerk * t for t in (t_from, t_to)]
            slack_a = GAMMA * (abs(a0) + abs(jerk) * t_to)
            yield _Overlap(index, k, peak, min(ends) - slack_a, max(ends) + slack_a, jerk, False)


def _exceeds(value: float, limit: float, absolute: float, tolerance: Tolerance) -> bool:
    return value > limit + absolute + tolerance.relative * limit


def validate_envelope(path: Trajectory, intervals: list[tuple[float, float, Limits]], tolerance: Tolerance = Tolerance(),
                      waived: frozenset[int] = frozenset()) -> Report:
    """Certify per-interval path limits in double with rigorous rounding bounds: v <= speed,
    -deceleration <= a <= acceleration, |j| <= jerk wherever s may lie in the closed interval. An
    acceleration step passes if an interval containing it has no jerk limit, or it is `waived` (hold)."""
    issues: list[Issue] = []
    unlimited_steps: set[int] = set()
    flagged: dict[int, str] = {}
    for overlap in _overlaps(path, [(low, high) for low, high, _ in intervals], tolerance.position):
        low, high, limits = intervals[overlap.interval]
        where = f"in [{low}, {high}]"
        if _exceeds(overlap.speed, limits.speed, tolerance.speed, tolerance):
            issues.append(Issue("speed_limit", overlap.segment, f"speed {overlap.speed:.9g} exceeds {limits.speed} {where}"))
        if _exceeds(overlap.acceleration_high, limits.acceleration, tolerance.acceleration, tolerance) or \
                _exceeds(-overlap.acceleration_low, limits.deceleration, tolerance.acceleration, tolerance):
            issues.append(Issue("acceleration_limit", overlap.segment,
                                f"acceleration [{overlap.acceleration_low:.9g}, {overlap.acceleration_high:.9g}] {where}"))
        if overlap.step:
            if not limits.jerk_limited:
                unlimited_steps.add(overlap.segment)
            else:
                flagged[overlap.segment] = f"acceleration step under jerk limit {limits.jerk} {where}"
        elif limits.jerk_limited and _exceeds(abs(overlap.jerk), limits.jerk, tolerance.jerk, tolerance):
            issues.append(Issue("jerk_limit", overlap.segment, f"jerk {overlap.jerk:.9g} exceeds {limits.jerk} {where}"))
    for segment, message in flagged.items():
        if segment not in unlimited_steps and segment not in waived:
            issues.append(Issue("jerk_limit", segment, message))
    return Report(tuple(issues))


def validate_axes(path: Trajectory, intervals: list[tuple[float, float, tuple[float, ...], tuple[float, ...], tuple[float, ...]]],
                  axes: list[Limits], tolerance: Tolerance = Tolerance(), waived: frozenset[int] = frozenset()) -> Report:
    """Certify axis limits along q(s) in double with rigorous rounding bounds, from per-interval bounds
    B1 >= |q_i'|, B2 >= |q_i''|, B3 >= |q_i'''|:
        B1 v <= V_i;  B1 |a| + B2 v^2 <= min(A_i, D_i) on curved intervals, or B1 a <= A_i and -B1 a <= D_i
        where B2 = B3 = 0 for every axis (straight: axis speed changes with the path speed's sign);
        B1 |j| + 3 B2 v |a| + B3 v^3 <= J_i for jerk-limited axes. Acceleration steps fail on jerk-limited
        moving axes unless waived."""
    issues: list[Issue] = []
    for overlap in _overlaps(path, [(low, high) for low, high, *_ in intervals], tolerance.position):
        low, high, first, second, third = intervals[overlap.interval]
        straight = not any(second) and not any(third)
        v = max(overlap.speed, 0.0)
        a_max = max(abs(overlap.acceleration_low), abs(overlap.acceleration_high))
        for axis, (b1, b2, b3, limits) in enumerate(zip(first, second, third, axes)):
            where = f"axis {axis} in [{low}, {high}]"
            if _exceeds(_up(b1 * v), limits.speed, tolerance.speed, tolerance):
                issues.append(Issue("axis_speed", overlap.segment, f"{b1 * v:.9g} > {limits.speed} {where}"))
            if straight:
                rising, falling = b1 * overlap.acceleration_high, -b1 * overlap.acceleration_low
                if _exceeds(rising + GAMMA * abs(rising), limits.acceleration, tolerance.acceleration, tolerance) or \
                        _exceeds(falling + GAMMA * abs(falling), limits.deceleration, tolerance.acceleration, tolerance):
                    issues.append(Issue("axis_acceleration", overlap.segment, f"[{-falling:.9g}, {rising:.9g}] {where}"))
            else:
                bound = _up(b1 * a_max, b2 * v * v)
                if _exceeds(bound, min(limits.acceleration, limits.deceleration), tolerance.acceleration, tolerance):
                    issues.append(Issue("axis_acceleration", overlap.segment, f"{bound:.9g} {where}"))
            if not limits.jerk_limited or (b1 == 0.0 and b2 == 0.0 and b3 == 0.0):
                continue
            if overlap.step:
                if overlap.segment not in waived:
                    issues.append(Issue("axis_jerk", overlap.segment, f"acceleration step {where}"))
                continue
            bound = _up(b1 * abs(overlap.jerk), 3.0 * b2 * v * a_max, b3 * v * v * v)
            if _exceeds(bound, limits.jerk, tolerance.jerk, tolerance):
                issues.append(Issue("axis_jerk", overlap.segment, f"{bound:.9g} > {limits.jerk} {where}"))
    return Report(tuple(issues))