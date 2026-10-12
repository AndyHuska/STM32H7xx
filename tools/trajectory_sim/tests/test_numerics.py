"""float32 target-precision study of solver output (numerics.propagate_f32 vs float64)."""

import random
import unittest

import numpy as np

from tools.trajectory_sim.model import AccelerationStep, Limits, Phase, State
from tools.trajectory_sim.numerics import propagate_f32
from tools.trajectory_sim.phases import trajectory
from tools.trajectory_sim.scalar_solver import AnalyticTimeLaw

EPS32 = float(np.finfo(np.float32).eps)


def magnitudes(start: State, phases: tuple[Phase, ...]) -> tuple[float, float]:
    """Sum of term magnitudes per phase: the scale float32 rounding error is proportional to."""
    position = speed = 0.0
    for segment in trajectory(start, phases).segments:
        t, s, v, a, j = segment.phase.duration, segment.start.s, segment.start.v, segment.start.a, segment.phase.jerk
        position += abs(s) + abs(v * t) + abs(a * t * t / 2) + abs(j * t ** 3 / 6)
        speed += abs(v) + abs(a * t) + abs(j * t * t / 2)
    return position, speed


class Float32Tests(unittest.TestCase):
    def test_propagation_identical_to_float64_for_exact_values(self):
        start, phases = State(0, 0, 0), (Phase(2, 1), Phase(-2, 1))
        self.assertEqual(propagate_f32(start, phases), trajectory(start, phases).final)
        steps = (AccelerationStep(2), Phase(0, 1), AccelerationStep(-1), Phase(0, 2), AccelerationStep(0))
        self.assertEqual(propagate_f32(start, steps), trajectory(start, steps).final)

    def test_solver_output_error_bounded_seeded_1000(self):
        rng, law = random.Random(1032), AnalyticTimeLaw()
        worst = {"relative": 0.0, "absolute": 0.0}
        for case in range(1000):
            limits = Limits(rng.uniform(5, 200), rng.uniform(10, 2000), rng.uniform(50, 50000))
            start = State(rng.choice((0.0, 1000.0)), 0.0, 0.0)
            goal = State(start.s + 10 ** rng.uniform(-3, 2.7), 0.0, 0.0)
            result = law.distance_transition(start, goal, limits)
            self.assertTrue(result.ok, result.detail)
            reference = trajectory(start, result.phases).final
            position_scale, speed_scale = magnitudes(State(0.0, 0.0, 0.0), result.phases)
            for mode in ("relative", "absolute"):
                f32 = propagate_f32(start, result.phases, relative=mode == "relative")
                scale = position_scale + (abs(start.s) * len(result.phases) if mode == "absolute" else 0.0)
                with self.subTest(case=case, mode=mode):
                    self.assertLessEqual(abs(f32.s - reference.s), 16 * EPS32 * scale)
                    self.assertLessEqual(abs(f32.v - reference.v), 16 * EPS32 * speed_scale)
                if start.s:
                    worst[mode] = max(worst[mode], abs(f32.s - reference.s))
        # Segment-relative storage must not be worse than carrying absolute coordinates.
        self.assertLessEqual(worst["relative"], worst["absolute"])
        print(f"\nfloat32 worst endpoint error at s0=1000 mm: relative {worst['relative']:.3g} mm, absolute {worst['absolute']:.3g} mm")


if __name__ == "__main__":
    unittest.main()
