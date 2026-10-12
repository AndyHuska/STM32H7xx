"""Hardware regression (2026-10-11): user test program on the real machine (H743, FW-1 + FW-2, jerk off, F57.15 mm/s).
Symptom on hardware: blended moves ran at ~8.7 mm/s instead of 57.15; some moves that start or end a blended
path ran at the right speed. P was not stated, so the firmware default ($780 = 0.01 mm) and larger values are run.

Each G1 section starts and ends at rest (G0 moves and G4 dwells in between synchronise the planner), so sections
are planned independently. Machine: tests/fixtures/machine_example_xyz.json (the firmware has one acceleration
per axis: $12x = maximumAcceleration), junction deviation $11 = 0.

Oracle (hand-derived, independent of the planner): a constant-acceleration block needs at most F^2 / (2a) to reach
the feed F from any entry speed and at most F^2 / (2a) to brake to any exit speed, so every straight block of
length >= F^2 / a must reach F whatever happens at its ends (blends trim a move's straight part; arcs have their
own caps, so the bound applies to the line blocks, not to whole moves). Checked on the executed profile: each
planned block through the host port of the C segment generator (test_firmware_stepper_law.ArcLawPrep), which
must also end every block exactly at its planned exit speed and length.
"""

import math
import unittest

from tools.trajectory_sim.model import Limits
from tools.trajectory_sim.tests.firmware_model import FirmwareMove, run
from tools.trajectory_sim.tests.machines import load_axes
from tools.trajectory_sim.tests.test_firmware_stepper_law import ArcLawPrep

FEED = 57.15
Z = -21.59
SECTIONS = {
    "pocket": [(13.51, 10.88), (13.78, 46.47), (23.95, 46.20), (24.02, 11.62), (29.55, 11.42), (29.28, 45.99), (33.12, 45.99),
               (34.13, 39.05), (38.65, 39.12), (38.65, 46.74), (52.60, 46.20), (52.46, 10.27), (75.24, 10.14)],
    "x_chain": [(74.71, 3.53), (54.08, 2.66), (51.18, 2.66), (46.80, 2.86), (42.76, 3.13), (39.05, 3.13), (35.55, 2.93),
                (32.18, 2.99), (28.74, 3.06), (26.04, 3.06), (11.75, 4.21)],
}
P_VALUES = (0.01, 0.1, 1.0)
MODES = (("current", "firmware"), ("FW-1+2", "adaptive"))
STEPS_PER_MM = (100.0, 100.0, 100.0)  # machine_example_xyz stepsPerMillimeterOrRevolution


def machine() -> list[Limits]:
    return [Limits(a.speed, a.acceleration, math.inf) for a in load_axes("machine_example_xyz")]


def programs() -> dict[str, tuple[list[tuple[float, float, float]], float]]:
    """name -> (points, feed) for the viewer (tests/view_paths.py); every section added above shows up there."""
    return {f"hw {name}": ([(x, y, Z) for x, y in xy], FEED) for name, xy in SECTIONS.items()}


class HardwareRegressionTests(unittest.TestCase):
    def test_long_blended_moves_reach_the_feed(self):
        axes = machine()
        for name, xy in SECTIONS.items():
            points = [(x, y, Z) for x, y in xy]
            for p in P_VALUES:
                for label, mode in MODES:
                    model = run(axes, [FirmwareMove(q, FEED, p) for q in points[1:]], points[0], junction_deviation=0.0,
                                arc_limits=mode, arc_k=0.9)
                    long_lines = 0
                    for block, (entry, exit, _) in zip(model.blocks, model.schedule()):
                        law_radius = block.radius if block.circle else None
                        segments = ArcLawPrep(block.circle or block.acceleration, law_radius, block.length, entry, exit,
                                              block.nominal).run()
                        with self.subTest(section=name, P=p, mode=label, block=block.start):
                            self.assertAlmostEqual(sum(s[1] for s in segments), block.length, delta=1e-9 * block.length)
                            self.assertAlmostEqual(segments[-1][2], exit, delta=1e-9 * FEED)
                            if block.kind == "line" and block.length >= FEED ** 2 / block.acceleration:
                                long_lines += 1
                                self.assertGreaterEqual(max(s[2] for s in segments), FEED * (1 - 1e-9))
                    with self.subTest(section=name, P=p, mode=label):
                        self.assertGreater(long_lines, 0)

    def test_blended_chain_never_stops_inside(self):
        """Hardware test 2 (2026-10-11): with $11 = 0 the x_chain stopped at interior points. Cause: line blocks take
        their direction from integer step targets, blend arcs from floats; on 1-2 mm trimmed lines at 100 steps/mm
        the rounding tilts the line by mrad, -cos exceeds the planner's 'straight' threshold (0.999999) and junction
        deviation with $11 = 0 gives a junction speed of 0. Joins into and out of a blend arc are tangent by
        construction (FW-4), so in a fully blended chain no interior block may be forced to rest. The model with
        step quantisation but without FW-4 must reproduce the hardware stops (pins the diagnosis)."""
        axes = machine()
        for name, xy in SECTIONS.items():
            points = [(x, y, Z) for x, y in xy]
            for p in P_VALUES:
                for fw4 in (False, True):
                    model = run(axes, [FirmwareMove(q, FEED, p) for q in points[1:]], points[0], junction_deviation=0.0,
                                arc_limits="adaptive", arc_k=0.9, steps_per_mm=STEPS_PER_MM, tangent_blend_junctions=fw4)
                    self.assertTrue(all(c.kind == "arc" for c in model.corners), (name, p))
                    stops = sum(1 for block in model.blocks[1:] if block.max_entry_sqr == 0.0)
                    with self.subTest(section=name, P=p, fw4=fw4):
                        if fw4:
                            self.assertEqual(stops, 0)
                        elif name == "x_chain":
                            self.assertGreater(stops, 0)


if __name__ == "__main__":
    unittest.main()
