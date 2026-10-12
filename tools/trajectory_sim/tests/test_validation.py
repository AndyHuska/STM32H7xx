from dataclasses import replace
from decimal import Decimal, localcontext
import math
import random
import unittest

from tools.trajectory_sim.model import AccelerationStep, Limits, Phase, Segment, State, Trajectory
from tools.trajectory_sim.phases import advance, extrema, trajectory
from tools.trajectory_sim.validation import Tolerance, ValidationError, validate, validate_envelope


class ValidationTests(unittest.TestCase):
    def test_analytic_reduced_profiles(self):
        cases = (
            ("triangular_rest", State(0, 0, 0), (Phase(2, 1), Phase(-2, 1), Phase(-2, 1), Phase(2, 1)), State(4, 0, 0), Limits(2, 2, 2)),
            ("seven_phase", State(0, 0, 0), (Phase(2, 1), Phase(0, 1), Phase(-2, 1), Phase(0, 1), Phase(-2, 1), Phase(0, 1), Phase(2, 1)), State(16, 0, 0), Limits(4, 2, 2)),
            ("plateau_no_cruise", State(0, 0, 0), (Phase(2, 1), Phase(0, 1), Phase(-2, 1), Phase(-2, 1), Phase(0, 1), Phase(2, 1)), State(12, 0, 0), Limits(4, 2, 2)),
            ("acceleration_only", State(0, 5, 0), (Phase(2, 1), Phase(-2, 1)), State(12, 7, 0), Limits(7, 2, 2)),
            ("deceleration_only", State(0, 7, 0), (Phase(-2, 1), Phase(2, 1)), State(12, 5, 0), Limits(7, 2, 2)),
            ("nonzero_signed_acceleration", State(0, 10, -2), (Phase(2, 1),), State(28/3, 9, 0), Limits(10, 2, 2)),
            ("tiny_rest_move", State(0, 0, 0), (Phase(2, 1e-4), Phase(-2, 1e-4), Phase(-2, 1e-4), Phase(2, 1e-4)), State(4e-12, 0, 0), Limits(2e-8, 2e-4, 2)),
            ("cruise", State(0, 3, 0), (Phase(0, 2),), State(6, 3, 0), Limits(3, 1, 1)),
        )
        for name, initial, phases, expected, limits in cases:
            with self.subTest(name=name):
                path = trajectory(initial, phases)
                report = validate(path, limits, expected, Tolerance(position=1e-15, speed=1e-15, acceleration=1e-15))
                self.assertTrue(report.valid, report.issues)

    def test_interior_overspeed_despite_valid_endpoints(self):
        path = trajectory(State(0, 0, 2), (Phase(-2, 2),))
        report = validate(path, Limits(0.9, 2, 2))
        self.assertEqual([(issue.code, issue.time) for issue in report.issues], [("speed_limit", 1.0)])

    def test_interior_reverse_despite_positive_endpoints(self):
        path = trajectory(State(0, 0.25, -2), (Phase(2, 2),))
        report = validate(path, Limits(1, 2, 2))
        self.assertEqual([(issue.code, issue.time) for issue in report.issues], [("reverse_progress", 1.0)])

    def test_injected_boundary_snaps(self):
        first = Segment(State(0, 1, 0), Phase(0, 1), State(1, 1, 0))
        for component in ("s", "v", "a"):
            start = replace(first.end, **{component: getattr(first.end, component) + 0.1})
            second = Segment(start, Phase(0, 0.1), advance(start, Phase(0, 0.1)))
            with self.subTest(component=component):
                report = validate(Trajectory(first.start, (first, second)), Limits(10, 10, 10))
                self.assertIn("continuity", {issue.code for issue in report.issues})

    def test_injected_wrong_dynamics_and_endpoint(self):
        good = trajectory(State(0, 0, 0), (Phase(2, 1),))
        for component in ("s", "v", "a"):
            segment = good.segments[0]
            bad = replace(segment, end=replace(segment.end, **{component: getattr(segment.end, component)+0.01}))
            with self.subTest(component=component):
                report = validate(Trajectory(good.initial, (bad,)), Limits(10, 10, 10), good.final)
                self.assertEqual({issue.code for issue in report.issues}, {"dynamics", "endpoint"})
                with self.assertRaises(ValidationError):
                    report.require_valid()

    def test_limit_faults_and_initial_discontinuity(self):
        path = trajectory(State(0, 3, 0), (Phase(2, 1),))
        self.assertEqual({issue.code for issue in validate(path, Limits(3, 1, 1)).issues},
                         {"speed_limit", "acceleration_limit", "jerk_limit"})
        path = replace(path, initial=State(0, 0, 0))
        self.assertIn("continuity", {issue.code for issue in validate(path, Limits(10, 10, 10)).issues})

    def test_asymmetric_acceleration_bounds(self):
        limits = Limits(10, 2, 2, deceleration=1)
        braking = trajectory(State(0, 5, 0), (Phase(-2, 0.75),))
        speeding = trajectory(State(0, 0, 0), (Phase(2, 0.75),))
        self.assertEqual({issue.code for issue in validate(braking, limits).issues}, {"acceleration_limit"})
        self.assertTrue(validate(speeding, limits).valid)

    def test_acceleration_steps(self):
        path = trajectory(State(0, 0, 0), (AccelerationStep(2), Phase(0, 1), AccelerationStep(0)))
        validate(path, Limits(10, 2, math.inf), State(1, 2, 0)).require_valid()
        self.assertEqual({issue.code for issue in validate(path, Limits(10, 2, 1000)).issues}, {"jerk_limit"})
        self.assertEqual({issue.code for issue in validate(trajectory(State(0, 0, 0), (AccelerationStep(3),)), Limits(10, 2, math.inf)).issues},
                         {"acceleration_limit"})
        moved = Segment(State(0, 1, 0), AccelerationStep(1), State(0.1, 1, 1))
        self.assertEqual({issue.code for issue in validate(Trajectory(moved.start, (moved,)), Limits(10, 2, math.inf)).issues}, {"dynamics"})

    def test_rest_to_rest_wrong_final_acceleration_rejected(self):
        path = trajectory(State(0, 0, 0), (Phase(2, 1), Phase(-2, 2)))
        report = validate(path, Limits(2, 2, 2), State(path.final.s, 0, 0))
        self.assertIn("endpoint", {issue.code for issue in report.issues})

    def test_empty_path_and_zero_duration(self):
        state = State(0, 0, 0)
        validate(Trajectory(state, ()), Limits(0, 1, 1), state).require_valid()
        validate(trajectory(state, (Phase(0, 0),)), Limits(0, 1, 1), state).require_valid()
        report = validate(Trajectory(State(0, -1, 2), ()), Limits(0, 1, 1))
        self.assertEqual({issue.code for issue in report.issues}, {"reverse_progress", "acceleration_limit"})

    def test_repeated_2000_cycles_without_boundary_resets(self):
        cycle = (Phase(2, 1), Phase(-2, 1), Phase(-2, 1), Phase(2, 1))
        path = trajectory(State(0, 0, 0), cycle * 2000)
        validate(path, Limits(2, 2, 2), State(8000, 0, 0)).require_valid()
        for index in range(3, len(path.segments), 4):
            end = path.segments[index].end
            self.assertAlmostEqual(end.s, index + 1, places=8)
            self.assertEqual((end.v, end.a), (0, 0))

    def test_tolerances_are_explicit_not_clamping(self):
        state = State(0, -1e-10, 0)
        path = Trajectory(state, ())
        self.assertTrue(validate(path, Limits(1, 1, 1)).valid)
        self.assertFalse(validate(path, Limits(1, 1, 1), tolerance=Tolerance(speed=0, relative=0)).valid)
        self.assertEqual(path.initial.v, -1e-10)
        for value in (-1, math.nan, math.inf):
            with self.assertRaises(ValueError):
                Tolerance(position=value)

    def test_independent_decimal_endpoints_seeded_1000(self):
        generator = random.Random(384)
        with localcontext() as context:
            context.prec = 60
            for case in range(1000):
                position = generator.uniform(-100, 100)
                speed = generator.uniform(10, 20)
                acceleration = generator.uniform(-1, 1)
                jerk = generator.uniform(-2, 2)
                duration = generator.uniform(0.01, 1)
                decimals = tuple(Decimal.from_float(value) for value in (position, speed, acceleration, jerk, duration))
                position_d, speed_d, acceleration_d, jerk_d, duration_d = decimals
                expected = State(float(position_d + speed_d*duration_d + acceleration_d*duration_d**2/2 + jerk_d*duration_d**3/6),
                                 float(speed_d + acceleration_d*duration_d + jerk_d*duration_d**2/2), float(acceleration_d+jerk_d*duration_d))
                with self.subTest(case=case):
                    validate(trajectory(State(position, speed, acceleration), (Phase(jerk, duration),)), Limits(25, 4, 2), expected).require_valid()

    def test_envelope_boundary_attribution(self):
        """A phase ending on a piece boundary does not carry its jerk into the next piece; one that runs
        0.01 mm into it does. A step on the boundary into an unlimited-jerk piece passes; mid-piece fails."""
        path = trajectory(State(0, 1, 0), (Phase(6, 0.5),))
        end = path.final.s
        strict, loose = Limits(10, 10, 1), Limits(10, 10, 10)
        self.assertTrue(validate_envelope(path, [(0, end, loose), (end, end + 1, strict)]).valid)
        report = validate_envelope(path, [(0, end - 0.01, loose), (end - 0.01, end + 1, strict)])
        self.assertEqual({issue.code for issue in report.issues}, {"jerk_limit"})
        steps = trajectory(State(0, 1, 0), (Phase(0, 1), AccelerationStep(2), Phase(0, 0.5)))
        unlimited = Limits(10, 10, math.inf)
        self.assertTrue(validate_envelope(steps, [(0, 1, loose), (1, 5, unlimited)]).valid)
        report = validate_envelope(steps, [(0, 3, loose), (3, 5, unlimited)])
        self.assertEqual({issue.code for issue in report.issues}, {"jerk_limit"})
        self.assertTrue(validate_envelope(steps, [(0, 3, loose), (3, 5, unlimited)], waived=frozenset({1})).valid)

    def test_extrema_against_independent_dense_samples_seeded_200(self):
        generator = random.Random(822)
        for case in range(200):
            start = State(0, generator.uniform(-5, 5), generator.uniform(-5, 5))
            phase = Phase(generator.uniform(-5, 5), generator.uniform(0.01, 3))
            minimum, maximum = extrema(start, phase)
            with self.subTest(case=case):
                for index in range(101):
                    elapsed = phase.duration * index / 100
                    reference = (start.s + start.v*elapsed + start.a*elapsed**2/2 + phase.jerk*elapsed**3/6,
                                 start.v + start.a*elapsed + phase.jerk*elapsed**2/2, start.a+phase.jerk*elapsed)
                    for component, value in zip(("s", "v", "a"), reference):
                        self.assertLessEqual(getattr(minimum, component)-1e-10, value)
                        self.assertGreaterEqual(getattr(maximum, component)+1e-10, value)


if __name__ == "__main__":
    unittest.main()