"""Validation of the time-optimal reference (optimal_reference.py) against Ruckig on straight rest-to-rest
lines, on every machine of the spectrum. Ruckig is exact there, so the reference must converge to it.

Error model (DESIGN.md 3D): the transcription is second order in the grid count N, so the relative excess at
N = 400 must be at most a quarter of that at N = 100 (second order would give 1/16), and within the declared
reference accuracy of 1e-4 relative used to interpret the benchmarks."""

import importlib.util
import math
import unittest

from tools.trajectory_sim.geometry import LinePiece
from tools.trajectory_sim.tests.bench_ruckig import reference_line
from tools.trajectory_sim.tests.machines import spectrum

HAS_CASADI = importlib.util.find_spec("casadi") is not None
ACCURACY = 1e-4
LINES = (((-6.0, 2.0, 3.0), 1e4), ((20.0, 0.0, 0.0), 100.0))


@unittest.skipUnless(HAS_CASADI, "casadi not installed (test-only dependency)")
class OptimalReferenceTests(unittest.TestCase):
    def test_converges_to_ruckig_on_straight_lines(self):
        from tools.trajectory_sim.tests.optimal_reference import optimal_time

        for label, axes in spectrum().items():
            for end, feed in LINES:
                length = math.dist((0.0, 0.0, 0.0), end)
                piece = LinePiece(0, feed, (0.0, 0.0, 0.0), tuple(c / length for c in end), length)
                ruckig_time = reference_line(axes, (0.0, 0.0, 0.0), end, feed)[0]
                coarse, fine = (optimal_time([(0.0, piece, True)], axes, feed, count=n)[0] / ruckig_time - 1.0
                                for n in (100, 400))
                with self.subTest(machine=label, end=end):
                    self.assertLessEqual(abs(fine), ACCURACY)
                    self.assertLessEqual(abs(fine), 0.25 * abs(coarse))


if __name__ == "__main__":
    unittest.main()
