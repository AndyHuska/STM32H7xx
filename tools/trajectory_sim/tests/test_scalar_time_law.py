"""Contract suite for contracts.ScalarTimeLaw (L2). Any implementation must pass it.

Oracles are independent of the implementation under test: hand derivations, the Decimal
validator, brute-force enumeration of the profile family, analytic feasibility boundaries,
and Ruckig (test-only) when installed.
"""

import math
import random
import unittest

from tools.trajectory_sim.contracts import ScalarTimeLaw, SolveStatus
from tools.trajectory_sim.model import AccelerationStep, Limits, Phase, State
from tools.trajectory_sim.phases import trajectory
from tools.trajectory_sim.scalar_solver import MAX_BISECTIONS, AnalyticTimeLaw
from tools.trajectory_sim.validation import Tolerance, validate

try:
    import ruckig
except ImportError:
    ruckig = None

TOL = Tolerance(position=1e-7, speed=1e-7, acceleration=1e-7, jerk=1e-9, relative=1e-9)
WITNESS_CODE = {SolveStatus.SPEED_LIMIT: "speed_limit", SolveStatus.REVERSE_PROGRESS: "reverse_progress"}
KNOWN_GAP_CASES = (  # seed 7117 cases 80, 353, 448: nonzero goal acceleration; Ruckig has valid profiles
    (State(0, 30.5043, -35.169), State(2.4699, 28.0122, 145.296), Limits(70.352, 1230.234, 5318.7)),
    (State(0, 121.5155, 0.0), State(3.5709, 129.5281, 526.581), Limits(133.024, 794.087, 26237.39)),
    (State(0, 104.5834, -582.179), State(5.4512, 13.2156, 183.512), Limits(146.664, 1747.424, 31680.97)),
)


def random_limits(rng: random.Random) -> Limits:
    acceleration = rng.uniform(10, 2000)
    deceleration = None if rng.random() < 0.5 else acceleration * rng.uniform(0.05, 3.0)
    jerk = math.inf if rng.random() < 0.2 else rng.uniform(50, 50000)
    return Limits(rng.uniform(5, 200), acceleration, jerk, deceleration)


def random_boundary(rng: random.Random, limits: Limits) -> tuple[float, float]:
    v = 0.0 if rng.random() < 0.15 else rng.uniform(0, limits.speed)
    a = 0.0 if rng.random() < 0.3 else rng.uniform(-limits.deceleration, limits.acceleration)
    return v, a


def ruckig_limits(inp, limits: Limits) -> None:
    inp.max_acceleration, inp.min_acceleration = [limits.acceleration], [-limits.deceleration]
    inp.max_jerk = [limits.jerk]


def ruckig_optimum(start: State, goal: State, limits: Limits) -> float | None:
    """Ruckig's time-optimal duration for a profile that stays inside 0 <= v <= V, else None."""
    otg, inp, out = ruckig.Ruckig(1), ruckig.InputParameter(1), ruckig.Trajectory(1)
    inp.current_position, inp.current_velocity, inp.current_acceleration = [start.s], [start.v], [start.a]
    inp.target_position, inp.target_velocity, inp.target_acceleration = [goal.s], [goal.v], [goal.a]
    inp.max_velocity, inp.min_velocity = [limits.speed], [0.0]
    ruckig_limits(inp, limits)
    try:
        if otg.calculate(inp, out) not in (ruckig.Result.Working, ruckig.Result.Finished):
            return None
    except ruckig.RuckigError:  # raised for targets that inevitably exceed a limit
        return None
    # Ruckig accepts unavoidable limit overshoot; keep only profiles inside our contract.
    low, high = ruckig_speed_range(out)
    return out.duration if low >= -1e-9 and high <= limits.speed + 1e-9 else None


def ruckig_speed_range(out, samples: int = 2000) -> tuple[float, float]:
    """Exact speed extrema: endpoints plus every acceleration zero crossing, located by bisection."""
    times = [k * out.duration / samples for k in range(samples + 1)]
    state = [out.at_time(t) for t in times]
    speeds = [state[0][1][0], state[-1][1][0]]
    for k in range(samples):
        if state[k][2][0] * state[k + 1][2][0] < 0.0:
            low, high = times[k], times[k + 1]
            for _ in range(60):
                middle = 0.5 * (low + high)
                if out.at_time(middle)[2][0] * state[k][2][0] > 0.0:
                    low = middle
                else:
                    high = middle
            speeds.append(out.at_time(0.5 * (low + high))[1][0])
        speeds.append(state[k][1][0])
    return min(speeds), max(speeds)


def ramp_area(x: float, p: float, jerk: float) -> float:
    return (x + p) / 2.0 * abs(p - x) / jerk


class TimeLawContract:
    """Mixin; subclasses set `law`."""

    law: ScalarTimeLaw

    def assert_witness(self, result, limits):
        report = validate(trajectory(result.start, result.phases), limits, tolerance=TOL)
        self.assertIn(WITNESS_CODE[result.status], {issue.code for issue in report.issues}, result.detail)

    def assert_steps_only_without_jerk_limit(self, result, limits):
        if limits.jerk_limited:
            self.assertFalse(any(isinstance(element, AccelerationStep) for element in result.phases))

    def test_second_order_hand_derived(self):
        limits = Limits(4, 2, math.inf)
        trapezoid = self.law.distance_transition(State(0, 0, 0), State(16, 0, 0), limits)
        self.assertEqual(trapezoid.status, SolveStatus.OK)
        self.assertEqual(trapezoid.phases, (AccelerationStep(2), Phase(0, 2), AccelerationStep(0), Phase(0, 2),
                                            AccelerationStep(-2), Phase(0, 2), AccelerationStep(0)))
        # Triangle: 4 mm at A = D = 2 peaks at 2*sqrt(2) mm/s; the two steps at the peak merge into one.
        triangle = self.law.distance_transition(State(0, 0, 0), State(4, 0, 0), Limits(10, 2, math.inf))
        self.assertEqual([type(e).__name__ for e in triangle.phases],
                         ["AccelerationStep", "Phase", "AccelerationStep", "Phase", "AccelerationStep"])
        self.assertAlmostEqual(triangle.duration, 2 * math.sqrt(2), places=9)
        validate(trajectory(State(0, 0, 0), triangle.phases), Limits(10, 2, math.inf), State(4, 0, 0), TOL).require_valid()

    def test_hand_derived_profiles(self):
        limits = Limits(10, 2, 1)
        triangle = self.law.velocity_transition(State(0, 0, 0), 4, 0, limits)
        self.assertEqual(triangle.status, SolveStatus.OK)
        self.assertEqual([(p.jerk, round(p.duration, 12)) for p in triangle.phases], [(1, 2), (-1, 2)])
        plateau = self.law.velocity_transition(State(0, 0, 0), 6, 0, limits)
        self.assertEqual([(p.jerk, round(p.duration, 12)) for p in plateau.phases], [(1, 2), (0, 1), (-1, 2)])
        seven = self.law.distance_transition(State(0, 0, 0), State(16, 0, 0), Limits(4, 2, 2))
        self.assertEqual(seven.status, SolveStatus.OK)
        self.assertEqual([(p.jerk, round(p.duration, 12)) for p in seven.phases],
                         [(2, 1), (0, 1), (-2, 1), (0, 1), (-2, 1), (0, 1), (2, 1)])
        # Asymmetric: braking from 6 mm/s at D = 1: ramp 1 s, plateau 5 s, ramp 1 s; distance v*T/2 = 21 mm.
        braking = self.law.stop(State(0, 6, 0), Limits(10, 2, 1, deceleration=1))
        self.assertEqual([(p.jerk, round(p.duration, 12)) for p in braking.phases], [(-1, 1), (0, 5), (1, 1)])
        self.assertAlmostEqual(trajectory(State(0, 6, 0), braking.phases).final.s, 21.0, places=12)

    def test_velocity_transitions_seeded_3000(self):
        rng = random.Random(2002)
        counts = {status: 0 for status in SolveStatus}
        for case in range(3000):
            limits = random_limits(rng)
            v0, a0 = random_boundary(rng, limits)
            v1, a1 = random_boundary(rng, limits)
            start = State(rng.uniform(-500, 500), v0, a0)
            result = self.law.velocity_transition(start, v1, a1, limits)
            counts[result.status] += 1
            with self.subTest(case=case, status=result.status):
                self.assert_steps_only_without_jerk_limit(result, limits)
                self.assertIn(result.status, (SolveStatus.OK, SolveStatus.SPEED_LIMIT, SolveStatus.REVERSE_PROGRESS))
                if result.ok:
                    path = trajectory(start, result.phases)
                    validate(path, limits, State(path.final.s, v1, a1), TOL).require_valid()
                else:
                    self.assert_witness(result, limits)
        self.assertGreater(counts[SolveStatus.OK], 1500, counts)

    def test_velocity_transition_not_slower_than_any_family_member(self):
        rng = random.Random(3003)
        checked = 0
        for _ in range(300):
            limits = random_limits(rng)
            v0, a0 = random_boundary(rng, limits)
            v1, a1 = random_boundary(rng, limits)
            result = self.law.velocity_transition(State(0, v0, a0), v1, a1, limits)
            if not result.ok:
                continue
            dv, jerk, best = v1 - v0, limits.jerk, math.inf
            for index in range(-400, 401):
                peak = (limits.acceleration if index > 0 else limits.deceleration) * index / 400.0
                if not (peak >= max(a0, a1) or peak <= min(a0, a1)) or peak == 0.0:
                    continue
                plateau = (dv - ramp_area(a0, peak, jerk) - ramp_area(peak, a1, jerk)) / peak
                if plateau >= 0.0:
                    best = min(best, abs(peak - a0) / jerk + plateau + abs(peak - a1) / jerk)
            if math.isfinite(best):
                checked += 1
                self.assertLessEqual(result.duration, best * (1 + 1e-9) + 1e-12)
        self.assertGreater(checked, 100)

    @unittest.skipIf(ruckig is None, "ruckig not installed")
    def test_velocity_transition_matches_ruckig_duration(self):
        rng = random.Random(4004)
        compared = 0
        for case in range(500):
            limits = random_limits(rng)
            v0, a0 = random_boundary(rng, limits)
            v1, a1 = random_boundary(rng, limits)
            result = self.law.velocity_transition(State(0, v0, a0), v1, a1, limits)
            if not result.ok:
                continue
            otg, inp, out = ruckig.Ruckig(1), ruckig.InputParameter(1), ruckig.Trajectory(1)
            inp.control_interface = ruckig.ControlInterface.Velocity
            inp.current_position, inp.current_velocity, inp.current_acceleration = [0.0], [v0], [a0]
            inp.target_velocity, inp.target_acceleration = [v1], [a1]
            inp.max_velocity = [limits.speed]
            ruckig_limits(inp, limits)
            if otg.calculate(inp, out) not in (ruckig.Result.Working, ruckig.Result.Finished):
                continue
            compared += 1
            with self.subTest(case=case):
                self.assertAlmostEqual(result.duration, out.duration, delta=1e-6 * max(1.0, out.duration))
        self.assertGreater(compared, 250)

    def test_stop_matches_analytic_feasibility_boundaries(self):
        rng = random.Random(5005)
        for case in range(3000):
            limits = random_limits(rng)
            v0, a0 = random_boundary(rng, limits)
            rise = a0 * a0 / (2 * limits.jerk)
            if a0 < 0 and v0 < rise * (1 - 1e-6):
                expected = SolveStatus.REVERSE_PROGRESS
            elif a0 > 0 and v0 + rise > limits.speed * (1 + 1e-6):
                expected = SolveStatus.SPEED_LIMIT
            elif (a0 < 0 and v0 <= rise * (1 + 1e-6)) or (a0 > 0 and v0 + rise >= limits.speed * (1 - 1e-6)):
                continue
            else:
                expected = SolveStatus.OK
            start = State(0, v0, a0)
            result = self.law.stop(start, limits)
            with self.subTest(case=case):
                self.assertEqual(result.status, expected, result.detail)
                self.assertEqual(result.proven_infeasible, expected is not SolveStatus.OK)
                if result.ok:
                    path = trajectory(start, result.phases)
                    validate(path, limits, State(path.final.s, 0, 0), TOL).require_valid()

    def test_stopping_distance_not_beaten_by_weaker_limits(self):
        rng = random.Random(6006)
        for case in range(500):
            limits = random_limits(rng)
            v0, a0 = random_boundary(rng, limits)
            start = State(0, v0, a0)
            result = self.law.stop(start, limits)
            if not result.ok:
                continue
            distance = trajectory(start, result.phases).final.s
            for scale_a, scale_j in ((0.5, 1.0), (1.0, 0.5), (0.8, 0.8)):
                weaker = Limits(limits.speed, limits.acceleration * scale_a, limits.jerk * scale_j, limits.deceleration * scale_a)
                if not -weaker.deceleration <= a0 <= weaker.acceleration:
                    continue
                alternative = self.law.stop(start, weaker)
                if alternative.ok:
                    with self.subTest(case=case, scale=(scale_a, scale_j)):
                        self.assertGreaterEqual(trajectory(start, alternative.phases).final.s, distance - 1e-9 * max(1, distance))

    def test_distance_transitions_seeded_2000(self):
        rng = random.Random(7007)
        counts = {status: 0 for status in SolveStatus}
        for case in range(2000):
            limits = random_limits(rng)
            v0, a0 = random_boundary(rng, limits)
            v1, a1 = random_boundary(rng, limits)
            start = State(rng.uniform(-500, 500), v0, a0)
            goal = State(start.s + 10 ** rng.uniform(-3, 2.7), v1, a1)
            result = self.law.distance_transition(start, goal, limits)
            counts[result.status] += 1
            with self.subTest(case=case, status=result.status):
                self.assertLessEqual(result.iterations, MAX_BISECTIONS)
                self.assert_steps_only_without_jerk_limit(result, limits)
                path = trajectory(start, result.phases)
                if result.ok:
                    validate(path, limits, goal, TOL).require_valid()
                elif result.status is SolveStatus.INSUFFICIENT_DISTANCE:
                    self.assertGreater(path.final.s, goal.s)
                else:
                    self.assertIn(result.status, WITNESS_CODE)
                    self.assert_witness(result, limits)
        self.assertEqual(counts[SolveStatus.INVALID_INPUT] + counts[SolveStatus.NO_CONVERGENCE], 0, counts)

    @unittest.skipIf(ruckig is None, "ruckig not installed")
    def test_distance_transition_against_ruckig(self):
        """Ruckig is time-optimal: no implementation may be faster or claim proven infeasibility where it solves."""
        rng = random.Random(7117)
        gaps, missed, both = [], 0, 0
        for case in range(600):
            limits = random_limits(rng)
            v0, a0 = random_boundary(rng, limits)
            v1, a1 = random_boundary(rng, limits)
            start = State(0.0, v0, a0)
            goal = State(10 ** rng.uniform(-3, 2.7), v1, a1)
            result = self.law.distance_transition(start, goal, limits)
            optimum = ruckig_optimum(start, goal, limits)
            if optimum is None:
                continue
            with self.subTest(case=case):
                if result.ok:
                    both += 1
                    self.assertGreaterEqual(result.duration, optimum * (1 - 1e-6) - 1e-9)
                    gaps.append(result.duration / optimum - 1 if optimum > 0 else 0.0)
                else:
                    self.assertFalse(result.proven_infeasible, result.detail)
                    missed += 1
        self.assertGreater(both, 100)
        self.assertLessEqual(missed, 0.02 * both)
        gaps.sort()
        print(f"\nvs Ruckig: {both} both solved, time gap median {gaps[len(gaps) // 2]:.2e} max {gaps[-1]:.2e}; "
              f"{missed} unsolved where Ruckig has a valid profile")

    def test_known_family_gap_cases_are_consistent(self):
        """Ruckig solves all KNOWN_GAP_CASES within 0 <= v <= V: a solution must be valid and not faster, and
        a failure must be an unproven INSUFFICIENT_DISTANCE."""
        for index, (start, goal, limits) in enumerate(KNOWN_GAP_CASES):
            result = self.law.distance_transition(start, goal, limits)
            with self.subTest(index=index):
                if result.ok:
                    validate(trajectory(start, result.phases), limits, goal, TOL).require_valid()
                    if ruckig is not None:
                        optimum = ruckig_optimum(start, goal, limits)
                        self.assertGreaterEqual(result.duration, optimum * (1 - 1e-6))
                else:
                    self.assertIs(result.status, SolveStatus.INSUFFICIENT_DISTANCE)
                    self.assertFalse(result.proven_infeasible)

    def test_rest_and_steady_boundaries_always_solvable(self):
        rng = random.Random(8008)
        for case in range(1000):
            limits = random_limits(rng)
            speed = 0.0 if case % 2 == 0 else rng.uniform(0, limits.speed)
            start = State(rng.uniform(-500, 500), speed, 0.0)
            goal = State(start.s + 10 ** rng.uniform(-4, 2.7), speed, 0.0)
            result = self.law.distance_transition(start, goal, limits)
            with self.subTest(case=case):
                self.assertEqual(result.status, SolveStatus.OK, result.detail)
                validate(trajectory(start, result.phases), limits, goal, TOL).require_valid()

    def test_split_moves_carry_state_without_reset(self):
        limits = Limits(50, 500, 5000)
        whole = self.law.distance_transition(State(0, 0, 0), State(30, 0, 0), limits)
        path = trajectory(State(0, 0, 0), whole.phases)
        boundary = path.segments[0].end
        first = self.law.distance_transition(State(0, 0, 0), boundary, limits)
        second = self.law.distance_transition(boundary, State(30, 0, 0), limits)
        self.assertTrue(first.ok and second.ok)
        joined = trajectory(State(0, 0, 0), first.phases + second.phases)
        validate(joined, limits, State(30, 0, 0), TOL).require_valid()
        self.assertEqual(boundary.a, limits.acceleration)
        self.assertAlmostEqual(joined.final.s, path.final.s, places=9)
        self.assertAlmostEqual(first.duration + second.duration, whole.duration, places=9)

    def test_reports_invalid_input(self):
        limits = Limits(10, 2, 1)
        cases = (
            self.law.velocity_transition(State(0, 11, 0), 0, 0, limits),
            self.law.velocity_transition(State(0, 0, 3), 0, 0, limits),
            self.law.velocity_transition(State(0, 0, 0), math.nan, 0, limits),
            self.law.velocity_transition(State(0, 0, 0), -1, 0, limits),
            self.law.distance_transition(State(5, 0, 0), State(4, 0, 0), limits),
        )
        for index, result in enumerate(cases):
            with self.subTest(index=index):
                self.assertEqual(result.status, SolveStatus.INVALID_INPUT)
                self.assertEqual(result.phases, ())

    def test_deterministic(self):
        limits = Limits(40, 300, 4000)
        start, goal = State(1, 12, -50), State(9, 3, 20)
        self.assertEqual(self.law.distance_transition(start, goal, limits), self.law.distance_transition(start, goal, limits))


class AnalyticTimeLawTests(TimeLawContract, unittest.TestCase):
    law = AnalyticTimeLaw()

    @unittest.skipIf(ruckig is None, "ruckig not installed")
    def test_peak_side_solutions_are_time_optimal(self):
        """When distance allows a cruise at or above v* (no valley), the result must equal Ruckig's optimum."""
        rng = random.Random(7117)
        peak_side = valley_side = 0
        for case in range(600):
            limits = random_limits(rng)
            v0, a0 = random_boundary(rng, limits)
            v1, a1 = random_boundary(rng, limits)
            start = State(0.0, v0, a0)
            goal = State(10 ** rng.uniform(-3, 2.7), v1, a1)
            result = self.law.distance_transition(start, goal, limits)
            optimum = ruckig_optimum(start, goal, limits)
            if not result.ok or optimum is None:
                continue
            floor = max(0.0, v1 - a1 * abs(a1) / (2 * limits.jerk))
            natural = min(limits.speed, max(floor, v0 + a0 * abs(a0) / (2 * limits.jerk)))
            if trajectory(start, self.law.peak_profile(start, goal, natural, 0.0, limits)).final.s > goal.s + 1e-9:
                valley_side += 1
                continue
            peak_side += 1
            with self.subTest(case=case):
                self.assertAlmostEqual(result.duration, optimum, delta=1e-9 * max(1.0, optimum))
        self.assertGreater(peak_side, 100)
        print(f"\npeak side optimal: {peak_side} cases; valley side (feasible, not optimal): {valley_side}")

    def test_best_peak_is_family_minimum(self):
        """best_peak must not be beaten by any sampled cruise speed (checks the unimodality assumption)."""
        rng = random.Random(9009)
        checked = 0
        for case in range(300):
            limits = random_limits(rng)
            v0, a0 = random_boundary(rng, limits)
            v1, a1 = random_boundary(rng, limits)
            start, goal = State(0, v0, a0), State(0, v1, a1)
            best = self.law.best_peak(start, goal, limits)
            if best is None:
                continue
            checked += 1
            minimum = trajectory(start, self.law.peak_profile(start, goal, best, 0.0, limits)).final.s
            low = max(0.0, v1 - a1 * abs(a1) / (2 * limits.jerk))
            for index in range(201):
                peak = low + (limits.speed - low) * index / 200
                distance = trajectory(start, self.law.peak_profile(start, goal, peak, 0.0, limits)).final.s
                with self.subTest(case=case, index=index):
                    self.assertGreaterEqual(distance, minimum - 1e-9 * max(1.0, abs(minimum)))
        self.assertGreater(checked, 200)

    def test_known_family_gaps(self):
        """Pins the exact current state of KNOWN_GAP_CASES so any change is noticed (update DESIGN.md 3.3):
        cases 0 and 2 are solved on the valley side (cruise below v*, slower than Ruckig); case 1 needs a
        profile with no interior a = 0 point, outside the family."""
        expected = (  # (status, family minimum distance, duration / Ruckig optimum)
            (SolveStatus.OK, None, 1.01502),
            (SolveStatus.INSUFFICIENT_DISTANCE, 5.03504, None),
            (SolveStatus.OK, None, 1.17935),
        )
        for index, ((start, goal, limits), (status, minimum, ratio)) in enumerate(zip(KNOWN_GAP_CASES, expected)):
            result = self.law.distance_transition(start, goal, limits)
            optimum = ruckig_optimum(start, goal, limits) if ruckig is not None else None
            with self.subTest(index=index):
                self.assertIs(result.status, status, result.detail)
                self.assertFalse(result.proven_infeasible)
                if ruckig is not None:
                    self.assertIsNotNone(optimum)
                if result.ok:
                    validate(trajectory(start, result.phases), limits, goal, TOL).require_valid()
                    floor = max(0.0, goal.v - goal.a * abs(goal.a) / (2 * limits.jerk))
                    natural = min(limits.speed, max(floor, start.v + start.a * abs(start.a) / (2 * limits.jerk)))
                    natural_end = trajectory(start, self.law.peak_profile(start, goal, natural, 0.0, limits)).final.s
                    self.assertGreater(natural_end, goal.s + 1e-9, "expected a valley-side solution")
                    if optimum is not None:
                        self.assertAlmostEqual(result.duration / optimum, ratio, delta=1e-5)
                else:
                    self.assertAlmostEqual(self.law.minimum_distance(start, goal.v, goal.a, limits), minimum, delta=1e-5)
                    self.assertGreater(minimum, goal.s - start.s)


if __name__ == "__main__":
    unittest.main()
