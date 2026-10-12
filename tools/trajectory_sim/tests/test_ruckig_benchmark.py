"""Pins the speed benchmark against Ruckig (tests/bench_ruckig.py): equality where both are time-optimal."""

import unittest

try:
    import ruckig  # noqa: F401
except ImportError:
    ruckig = None

if ruckig is not None:
    from tools.trajectory_sim.tests.bench_ruckig import reference_line, straight_lines
    from tools.trajectory_sim.tests.machines import spectrum
    from tools.trajectory_sim.tests.test_pipeline import plan_time, zigzag


@unittest.skipIf(ruckig is None, "ruckig not installed")
class RuckigSpeedBenchmarkTests(unittest.TestCase):
    def test_straight_lines_are_time_optimal(self):
        """Whole stack on single straight moves equals Ruckig's time optimum (phase-synchronized multi-DoF,
        or 1-DoF along the line where Ruckig cannot keep the axes proportional)."""
        for label, axes in spectrum().items():
            ratios = straight_lines(axes, cases=20)["ratios"]
            with self.subTest(machine=label):
                self.assertLessEqual(max(abs(r - 1.0) for r in ratios), 1e-6)

    def test_g61_equals_and_g64_beats_the_ruckig_exact_stop_optimum(self):
        points = zigzag()
        for label, axes in spectrum().items():
            starts = [(0.0, 0.0, 0.0)] + points[:-1]
            baseline = sum(reference_line(axes, a, b, 100.0)[0] for a, b in zip(starts, points))
            with self.subTest(machine=label):
                self.assertAlmostEqual(plan_time(axes, points, None), baseline, delta=1e-6 * baseline)
                self.assertLess(plan_time(axes, points, 1.0), baseline)


if __name__ == "__main__":
    unittest.main()
