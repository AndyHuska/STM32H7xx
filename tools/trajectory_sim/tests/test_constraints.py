"""Contract suite for L4 constraint projection (contracts.ConstraintProjection).

Oracle: axis velocity, acceleration and jerk computed from the piece's own derivatives at many arc
lengths, at the extreme admissible path states (v = V, a = +/-A, j = +/-J), compared with axis limits.
"""

import math
import random
import unittest

from tools.trajectory_sim.constraints import CURVED_SHARE, SHARES, ConservativeProjection, blocks
from tools.trajectory_sim.contracts import ConstraintProjection, DerivativeBounds
from tools.trajectory_sim.geometry import GeometryStream, LinePiece
from tools.trajectory_sim.model import ArcMove, Limits, LineMove
from tools.trajectory_sim.phases import ramp_distance
from tools.trajectory_sim.tests.test_geometry import random_unit, rotate_towards


def random_axes(rng, profile0=0.0):
    axes = []
    for _ in range(3):
        acceleration = rng.uniform(50, 3000)
        jerk = math.inf if rng.random() < profile0 else rng.uniform(100, 50000)
        axes.append(Limits(rng.uniform(20, 300), acceleration, jerk, acceleration * rng.uniform(0.1, 2.0)))
    return axes


def random_curved_pieces(rng):
    """Clothoid blends and arcs in random orientations."""
    pieces = []
    t_in = random_unit(rng)
    t_out = rotate_towards(t_in, random_unit(rng), rng.uniform(0.05, 3.0))
    b = (0.0, 0.0, 0.0)
    a = tuple(-10 * d for d in t_in)
    stream = GeometryStream(a)
    stream.append(LineMove(1, b, 50.0, rng.uniform(0.01, 1.0)), 0.0)
    revision = stream.append(LineMove(2, tuple(10 * d for d in t_out), 50.0), 0.0)
    pieces += [p for p in revision.pieces if not isinstance(p, LinePiece)]
    radius = rng.uniform(0.5, 50)
    normal = rng.randrange(3)
    start = tuple(radius if axis == (normal + 1) % 3 else 0.0 for axis in range(3))
    end = tuple(radius * math.cos(1.0) if axis == (normal + 1) % 3 else (radius * math.sin(1.0) if axis == (normal + 2) % 3 else 0.0)
                for axis in range(3))
    pieces += GeometryStream(start).propose(ArcMove(3, end, (0, 0, 0), normal, rng.random() < 0.5, 50.0), 0.0).pieces
    return pieces


class ProjectionContract:
    """Mixin; subclasses set `projection`."""

    projection: ConstraintProjection

    def test_protocol(self):
        self.assertIsInstance(self.projection, ConstraintProjection)

    def test_straight_line_hand_case(self):
        axes = [Limits(100, 1000, 1e4, 500), Limits(50, 2000, 5e3, 2000), Limits(10, 10, 10, 10)]
        limits = self.projection.path_limits(DerivativeBounds((0.6, 0.8, 0.0), (0, 0, 0), (0, 0, 0)), axes)
        self.assertAlmostEqual(limits.speed, 62.5, places=12)
        self.assertAlmostEqual(limits.acceleration, 1000 / 0.6, places=9)  # min(1000/0.6, 2000/0.8)
        self.assertAlmostEqual(limits.deceleration, 500 / 0.6, places=9)
        self.assertAlmostEqual(limits.jerk, 5e3 / 0.8, places=9)

    def test_constant_acceleration_axes_impose_no_jerk_limit(self):
        bounds = DerivativeBounds((0.6, 0.8, 0.0), (0, 0, 0), (0, 0, 0))
        mixed = [Limits(100, 1000, math.inf), Limits(100, 1000, 4000), Limits(1, 1, 1)]
        self.assertAlmostEqual(self.projection.path_limits(bounds, mixed).jerk, 5000.0, places=9)
        none = [Limits(100, 1000, math.inf), Limits(100, 1000, math.inf), Limits(1, 1, 1)]
        self.assertEqual(self.projection.path_limits(bounds, none).jerk, math.inf)

    def test_curved_pieces_keep_every_axis_within_limits(self):
        rng = random.Random(4501)
        checked = 0
        for case in range(100):
            axes = random_axes(rng, profile0=0.2)
            for piece in random_curved_pieces(rng):
                for share in (SHARES[0], CURVED_SHARE, SHARES[-1]):
                    limits = self.projection.path_limits(piece.bounds(), axes, share)
                    checked += 1
                    with self.subTest(case=case, piece=type(piece).__name__, share=share):
                        self.assertGreater(limits.speed, 0.0)
                        for k in range(21):
                            _, d1, d2, d3 = piece.evaluate(piece.length * k / 20)
                            v = limits.speed
                            for a in (limits.acceleration, -limits.deceleration):
                                for j in ((limits.jerk, -limits.jerk) if limits.jerk_limited else (0.0,)):
                                    for i, axis in enumerate(axes):
                                        self.assertLessEqual(abs(d1[i] * v), axis.speed * (1 + 1e-12) + 1e-12)
                                        self.assertLessEqual(abs(d1[i] * a + d2[i] * v * v),
                                                             min(axis.acceleration, axis.deceleration) * (1 + 1e-12) + 1e-9)
                                        if axis.jerk_limited:
                                            self.assertLessEqual(abs(d1[i] * j + 3 * d2[i] * v * a + d3[i] * v ** 3),
                                                                 axis.jerk * (1 + 1e-12) + 1e-9)
        self.assertGreater(checked, 1000)


class ConservativeProjectionTests(ProjectionContract, unittest.TestCase):
    projection = ConservativeProjection()

    def test_share_trades_acceleration_for_speed(self):
        """A and J are exactly `share` times the straight-line values; a smaller share never lowers the cap."""
        rng = random.Random(4502)
        for case in range(100):
            axes = random_axes(rng)
            for piece in random_curved_pieces(rng):
                bounds = piece.bounds()
                a_line = min(min(axis.acceleration, axis.deceleration) / b for b, axis in zip(bounds.first, axes) if b > 0)
                j_line = min(axis.jerk / b for b, axis in zip(bounds.first, axes) if b > 0)
                caps = []
                for share in SHARES:
                    limits = self.projection.path_limits(bounds, axes, share)
                    caps.append(limits.speed)
                    with self.subTest(case=case, share=share):
                        self.assertAlmostEqual(limits.acceleration, share * a_line, delta=1e-12 * a_line)
                        self.assertAlmostEqual(limits.jerk, share * j_line, delta=1e-12 * j_line)
                self.assertEqual(caps, sorted(caps, reverse=True))

    def test_allocation_picks_certified_shares_with_positive_caps(self):
        """Every curved block's limits are exactly one of the per-share limits (each certified above) and
        never the zero-cap full share; straight blocks keep their exact limits."""
        rng = random.Random(4503)
        for case in range(60):
            axes = random_axes(rng)
            t_in = random_unit(rng)
            t_out = rotate_towards(t_in, random_unit(rng), rng.uniform(0.1, 3.0))
            stream = GeometryStream((0, 0, 0))
            stream.append(LineMove(1, tuple(20 * d for d in t_in), 150.0, 10 ** rng.uniform(-3, 0)), 0.0)
            revision = stream.append(LineMove(2, tuple(20 * a + 20 * b for a, b in zip(t_in, t_out)), 150.0), 0.0)
            for block, piece in zip(blocks(revision.pieces, revision.stops, axes), revision.pieces):
                with self.subTest(case=case, piece=type(piece).__name__):
                    if isinstance(piece, LinePiece):
                        self.assertEqual(block.limits, self.projection.path_limits(piece.bounds(), axes))
                    else:
                        self.assertIn(block.limits, [self.projection.path_limits(piece.bounds(), axes, s) for s in SHARES])
                        self.assertGreater(block.limits.speed, 0.0)
        self.assertNotIn(1.0, SHARES)

    def test_blocks_follow_pieces(self):
        stream = GeometryStream((0, 0, 0))
        stream.append(LineMove(1, (10, 0, 0), 40.0, 0.1), 0.0)
        revision = stream.append(LineMove(2, (10, 10, 0), 30.0), 0.0)
        axes = [Limits(100, 1000, 1e4)] * 3
        result = blocks(revision.pieces, revision.stops, axes)
        self.assertEqual([b.length for b in result], [p.length for p in revision.pieces])
        self.assertEqual([b.stop_after for b in result], list(revision.stops))
        self.assertEqual([b.speed_cap for b in result], [30.0] * len(result))
        self.assertTrue(all(b.limits is not None for b in result))


if __name__ == "__main__":
    unittest.main()
