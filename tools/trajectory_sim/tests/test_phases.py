from dataclasses import FrozenInstanceError, asdict
from fractions import Fraction
import math
import random
import unittest

from tools.trajectory_sim.model import AccelerationStep, Limits, Marker, Phase, State, Trajectory
from tools.trajectory_sim.phases import advance, critical_times, extrema, propagate, trajectory


class PhaseTests(unittest.TestCase):
    def assert_state_close(self, actual: State, expected: State) -> None:
        for component in ("s", "v", "a"):
            self.assertTrue(math.isclose(getattr(actual, component), getattr(expected, component), rel_tol=2e-12, abs_tol=2e-10), component)

    def test_hand_calculated_cases(self):
        self.assertEqual(advance(State(0, 0, 0), Phase(6, 2)), State(8, 12, 12))
        self.assertEqual(advance(State(10, 3, 2), Phase(0, 4)), State(38, 11, 2))
        self.assertEqual(advance(State(0, 8, -4), Phase(0, 2)), State(8, 0, -4))
        self.assertEqual(advance(State(0, 0, 0), Phase(2, 1), 0.5), State(1/24, 0.25, 1))

    def test_fraction_reference(self):
        for position, speed, acceleration, jerk, duration in ((2, 3, -4, 6, 2), (0, 0, 0, 5, 3), (-7, 10, -2, -3, 1)):
            expected = State(float(Fraction(position) + speed*duration + Fraction(acceleration*duration**2, 2) + Fraction(jerk*duration**3, 6)),
                             float(Fraction(speed) + acceleration*duration + Fraction(jerk*duration**2, 2)), float(acceleration + jerk*duration))
            self.assert_state_close(advance(State(position, speed, acceleration), Phase(jerk, duration)), expected)

    def test_propagate_matches_advance_exactly(self):
        generator = random.Random(77)
        for case in range(500):
            start = State(generator.uniform(-100, 100), generator.uniform(0, 50), generator.uniform(-20, 20))
            elements = tuple(AccelerationStep(generator.uniform(-5, 5)) if generator.random() < 0.2 else
                             Phase(generator.uniform(-50, 50), generator.uniform(0, 1)) for _ in range(6))
            final = trajectory(start, elements).final
            with self.subTest(case=case):
                self.assertEqual(propagate(start, elements), (final.s, final.v, final.a))

    def test_split_composition_seeded_3000(self):
        generator = random.Random(1062026)
        for case in range(3000):
            start = State(generator.uniform(-1000, 1000), generator.uniform(0, 100), generator.uniform(-50, 50))
            jerk = generator.uniform(-100, 100)
            duration = generator.uniform(0, 5)
            split = generator.uniform(0, duration)
            with self.subTest(case=case):
                combined = advance(start, Phase(jerk, duration))
                divided = advance(advance(start, Phase(jerk, split)), Phase(jerk, duration - split))
                self.assert_state_close(divided, combined)

    def test_derivative_consistency(self):
        start, phase = State(3, 4, -2), Phase(6, 2)
        elapsed, delta = 0.7, 1e-5
        before, after, middle = advance(start, phase, elapsed-delta), advance(start, phase, elapsed+delta), advance(start, phase, elapsed)
        self.assertAlmostEqual((after.s-before.s)/(2*delta), middle.v, places=8)
        self.assertAlmostEqual((after.v-before.v)/(2*delta), middle.a, places=8)
        self.assertAlmostEqual((after.a-before.a)/(2*delta), phase.jerk, places=8)

    def test_full_interval_extrema(self):
        minimum, maximum = extrema(State(0, 0, 2), Phase(-2, 2))
        self.assertEqual(minimum.v, 0)
        self.assertEqual(maximum.v, 1)
        minimum, maximum = extrema(State(0, 1, -2), Phase(0, 2))
        self.assertEqual(maximum.s, 0.25)
        self.assertEqual(minimum.s, -2)
        self.assertIn(0.5, critical_times(State(0, 1, -2), Phase(0, 2)))

    def test_zero_duration_and_immutable_records(self):
        start = State(3, 4, -5)
        self.assertEqual(advance(start, Phase(100, 0)), start)
        self.assertEqual(trajectory(start, []).final, start)
        with self.assertRaises(FrozenInstanceError):
            setattr(start, "v", 1)
        record = Trajectory(start, (), (Marker(7, 3),))
        self.assertEqual(asdict(record)["markers"][0]["source_id"], 7)

    def test_units(self):
        self.assertEqual(State.from_grbl(2, 120, -7200), State(2, 2, -2))
        self.assertEqual(Limits.from_grbl(600, 36000, 2160000), Limits(10, 10, 10))
        self.assertEqual(Limits.from_grbl(600, 36000, 2160000, 7200), Limits(10, 10, 10, deceleration=2))
        self.assertEqual(Limits(10, 10, 10).deceleration, 10)

    def test_acceleration_step(self):
        start = State(3, 4, -1)
        self.assertEqual(advance(start, AccelerationStep(2)), State(3, 4, 2))
        path = trajectory(start, (AccelerationStep(2), Phase(0, 1), AccelerationStep(0)))
        self.assertEqual(path.final, State(3 + 4 + 1, 6, 0))
        minimum, maximum = extrema(start, AccelerationStep(2))
        self.assertEqual((minimum.a, maximum.a), (-1, 2))
        self.assertEqual(critical_times(start, AccelerationStep(2)), (0.0,))
        for bad in ({"acceleration": math.nan}, {"acceleration": 1.0, "duration": 0.1}):
            with self.assertRaises(ValueError):
                AccelerationStep(**bad)

    def test_unlimited_jerk_limits(self):
        self.assertFalse(Limits(10, 2, math.inf).jerk_limited)
        self.assertTrue(Limits(10, 2, 5).jerk_limited)
        for jerk in (math.nan, -math.inf, 0.0):
            with self.assertRaises(ValueError):
                Limits(10, 2, jerk)
        with self.assertRaises(ValueError):
            Limits(math.inf, 2, 5)

    def test_invalid_input_and_overflow_rejected(self):
        for value in (math.nan, math.inf, -math.inf):
            with self.assertRaises(ValueError):
                State(value, 0, 0)
            with self.assertRaises(ValueError):
                Phase(0, value)
            with self.assertRaises(ValueError):
                Phase(value, 1)
        with self.assertRaises(ValueError):
            Phase(1, -1)
        with self.assertRaises(ValueError):
            Limits(1, 0, 1)
        for time in (-0.1, 1.1, math.nan):
            with self.assertRaises(ValueError):
                advance(State(0, 0, 0), Phase(1, 1), time)
        with self.assertRaises(ValueError):
            advance(State(0, 0, 0), Phase(1e308, 10))


if __name__ == "__main__":
    unittest.main()