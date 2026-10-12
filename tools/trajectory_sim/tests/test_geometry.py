"""Contract suite for L3 path geometry (contracts.PathGeometry / PathPiece) plus reference-specific checks.

Oracles: hand-derived arcs, positions re-integrated from an independently written heading formula,
finite differences of positions, dense sampling with Lipschitz slack, and the programmed polyline.
"""

import math
import random
import unittest

from tools.trajectory_sim.contracts import PathGeometry, PathPiece, SolveStatus
from tools.trajectory_sim.geometry import ArcPiece, ClothoidPiece, GeometryStream, LinePiece, join_gap
from tools.trajectory_sim.model import ArcMove, LineMove


def sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def norm(a):
    return math.sqrt(sum(x * x for x in a))


def segment_distance(point, a, b):
    ab, ap = sub(b, a), sub(point, a)
    t = max(0.0, min(1.0, sum(x * y for x, y in zip(ap, ab)) / sum(x * x for x in ab)))
    return norm(sub(point, tuple(p + t * d for p, d in zip(a, ab))))


def random_unit(rng):
    while True:
        v = tuple(rng.uniform(-1, 1) for _ in range(3))
        if 0.1 < norm(v) <= 1:
            return tuple(x / norm(v) for x in v)


def rotate_towards(t, other, angle):
    """Unit vector at `angle` from unit t in the plane of t and other."""
    perpendicular = sub(other, tuple(sum(x * y for x, y in zip(other, t)) * x for x in t))
    n = tuple(x / norm(perpendicular) for x in perpendicular)
    return tuple(math.cos(angle) * a + math.sin(angle) * b for a, b in zip(t, n))


def higher_derivative_bounds(piece):
    """Independent bounds of |q^(3)|, |q^(4)|, |q^(5)| from the Frenet relations T' = kN, N' = -kT."""
    if isinstance(piece, LinePiece):
        return 0.0, 0.0, 0.0
    if isinstance(piece, ArcPiece):
        k = 1 / piece.radius
        return k ** 2, k ** 3, k ** 4
    c, k = piece.sharpness, piece.sharpness * max(piece.sigma_from, piece.sigma_to)  # q''' = cN - k^2 T, q'''' = -3ckT - k^3 N
    return c + k * k, 3 * c * k + k ** 3, 3 * c * c + 6 * c * k * k + k ** 4


def assert_derivatives(test, piece, samples=7):
    """Central differences of evaluate() positions agree with the declared q', q'', q''' within their own
    truncation (h^2 |q^(n+2)| / {6, 12, 4}) and rounding (eps |q| / h^n) error."""
    h = min(1e-3, piece.length / 50)
    m3, m4, m5 = higher_derivative_bounds(piece)
    for k in range(1, samples):
        u = piece.length * k / samples
        q = [piece.evaluate(u + m * h)[0] for m in (-2, -1, 0, 1, 2)]
        _, d1, d2, d3 = piece.evaluate(u)
        size = 1e-15 * max(1.0, max(abs(x) for x in q[2]))
        for i in range(3):
            fd1 = (q[3][i] - q[1][i]) / (2 * h)
            fd2 = (q[3][i] - 2 * q[2][i] + q[1][i]) / h ** 2
            fd3 = (q[4][i] - 2 * q[3][i] + 2 * q[1][i] - q[0][i]) / (2 * h ** 3)
            test.assertAlmostEqual(fd1, d1[i], delta=2 * (h * h * m3 / 6 + size / h) + 1e-12)
            test.assertAlmostEqual(fd2, d2[i], delta=2 * (h * h * m4 / 12 + 4 * size / h ** 2) + 1e-12)
            test.assertAlmostEqual(fd3, d3[i], delta=2 * (h * h * m5 / 4 + 6 * size / h ** 3) + 1e-12)
        test.assertAlmostEqual(norm(d1), 1.0, places=12)


def assert_bounds(test, piece, samples=400):
    bounds = piece.bounds()
    for k in range(samples + 1):
        _, d1, d2, d3 = piece.evaluate(piece.length * k / samples)
        for i in range(3):
            test.assertLessEqual(abs(d1[i]), bounds.first[i] + 1e-12)
            test.assertLessEqual(abs(d2[i]), bounds.second[i] * (1 + 1e-12) + 1e-12)
            test.assertLessEqual(abs(d3[i]), bounds.third[i] * (1 + 1e-12) + 1e-12)


def integrated_clothoid(piece, u, steps=4000):
    """Position by composite Simpson of the tangent from the heading psi = c sigma^2 / 2 (no series)."""
    sigma_end = piece.sigma_from - u if piece.reverse else piece.sigma_from + u
    h = sigma_end / steps
    total = [0.0, 0.0]
    for k in range(steps + 1):
        sigma = k * h
        weight = 1 if k in (0, steps) else (4 if k % 2 else 2)
        psi = 0.5 * piece.sharpness * sigma * sigma
        total[0] += weight * math.cos(psi)
        total[1] += weight * math.sin(psi)
    x, y = total[0] * h / 3, total[1] * h / 3
    return tuple(o + x * t + y * n for o, t, n in zip(piece.origin, piece.t0, piece.n0))


def corner(stream_start, a, b, c, tolerance, feed=50.0, committed=0.0, second_tolerance=None):
    stream = GeometryStream(stream_start if stream_start else a)
    stream.append(LineMove(1, b, feed, tolerance), 0.0)
    revision = stream.append(LineMove(2, c, feed, second_tolerance), committed)
    return stream, revision


def assert_path_continuous(test, stream):
    entries = stream.pieces()
    for (s0, before, stop), (s1, after, _) in zip(entries, entries[1:]):
        test.assertAlmostEqual(s0 + before.length, s1, delta=1e-9)
        position, tangent, curvature = join_gap(before, after)
        test.assertLess(position, 1e-9)
        if not stop:
            test.assertLess(tangent, 1e-9)
            test.assertLess(curvature, 1e-7 * max(1.0, norm(before.evaluate(before.length)[2])))


class GeometryContract:
    """Mixin; subclasses set `make(start)` returning a PathGeometry."""

    def test_protocols(self):
        stream = self.make((0, 0, 0))
        self.assertIsInstance(stream, PathGeometry)
        revision = stream.propose(LineMove(1, (1, 2, 3), 10.0, 0.01), 0.0)
        self.assertIsInstance(revision.pieces[0], PathPiece)

    def test_collinear_moves_continue_without_stop(self):
        stream = self.make((0, 0, 0))
        stream.append(LineMove(1, (3, 4, 0), 50.0, 0.01), 0.0)
        revision = stream.append(LineMove(2, (6, 8, 0), 50.0, 0.01), 0.0)
        self.assertEqual(revision.corner.kind, "continuous")
        self.assertFalse(revision.stop_at_from)
        self.assertAlmostEqual(revision.from_s, 5.0, places=12)

    def test_exact_stop_requested(self):
        stream = self.make((0, 0, 0))
        stream.append(LineMove(1, (10, 0, 0), 50.0, None), 0.0)
        revision = stream.append(LineMove(2, (10, 10, 0), 50.0, 0.1), 0.0)
        self.assertEqual(revision.corner.kind, "stop")
        self.assertTrue(revision.stop_at_from)

    def test_reversal_stops(self):
        stream = self.make((0, 0, 0))
        stream.append(LineMove(1, (10, 0, 0), 50.0, 0.5), 0.0)
        revision = stream.append(LineMove(2, (0, 0, 0), 50.0, 0.5), 0.0)
        self.assertEqual(revision.corner.kind, "stop")

    def test_random_corners_certified(self):
        """Deviation (Hausdorff, polyline <-> blend) <= P by independent dense sampling, joins G2."""
        rng = random.Random(4401)
        blends = 0
        for case in range(300):
            t_in = random_unit(rng)
            angle = rng.choice([rng.uniform(1e-6, 0.01), rng.uniform(0.01, 3.0), rng.uniform(3.0, math.pi - 1e-6)])
            t_out = rotate_towards(t_in, random_unit(rng), angle)
            a = tuple(rng.uniform(-200, 200) for _ in range(3))
            len_in, len_out = 10 ** rng.uniform(-2, 1.7), 10 ** rng.uniform(-2, 1.7)
            b = tuple(p + len_in * d for p, d in zip(a, t_in))
            c = tuple(p + len_out * d for p, d in zip(b, t_out))
            tolerance = 10 ** rng.uniform(-3, 0)
            stream = self.make(a)
            stream.append(LineMove(1, b, 50.0, tolerance), 0.0)
            revision = stream.append(LineMove(2, c, 50.0, None), 0.0)
            with self.subTest(case=case, angle=angle):
                self.assertTrue(revision.ok, revision.detail)
                assert_path_continuous(self, stream)
                if revision.corner.kind != "blend":
                    self.assertEqual(revision.corner.kind, "stop")
                    continue
                blends += 1
                self.assertLessEqual(revision.corner.deviation, tolerance * (1 + 1e-12))
                trim = len_in - revision.from_s
                self.assertLessEqual(trim, min(len_in, len_out / 2) * (1 + 1e-12) + 1e-12)
                curve = [p for p in revision.pieces if not isinstance(p, LinePiece)]
                points, spacing = [], 0.0
                for piece in curve:
                    n = 200
                    points += [piece.evaluate(piece.length * k / n)[0] for k in range(n + 1)]
                    spacing = max(spacing, piece.length / n)
                p_in = tuple(x - trim * d for x, d in zip(b, t_in))
                p_out = tuple(x + trim * d for x, d in zip(b, t_out))
                # Blend -> polyline: distance to the polyline at every sample (1-Lipschitz, unit speed).
                one_sided = max(min(segment_distance(p, a, b), segment_distance(p, b, c)) for p in points)
                self.assertLessEqual(one_sided, tolerance * (1 + 1e-9) + 1e-12)
                # Replaced polyline -> blend: polyline samples to the nearest blend sample, plus sample slack.
                worst = 0.0
                for k in range(101):
                    for x in (tuple(p + (q - p) * k / 100 for p, q in zip(p_in, b)), tuple(p + (q - p) * k / 100 for p, q in zip(b, p_out))):
                        worst = max(worst, min(norm(sub(x, p)) for p in points))
                self.assertLessEqual(worst, tolerance * (1 + 1e-9) + spacing / 2 + 1e-12)
                self.assertLess(norm(sub(curve[-1].evaluate(curve[-1].length)[1], t_out)), 1e-9)
        self.assertGreater(blends, 200)

    def test_tiny_segment_chain(self):
        """Approximated arc of 0.02-0.08 mm chords: every corner blends or stops; no negative pieces."""
        rng = random.Random(4402)
        stream = self.make((0, 0, 0))
        point, heading = (0.0, 0.0, 0.0), 0.0
        for k in range(150):
            heading += rng.uniform(-0.2, 0.2)
            length = rng.uniform(0.02, 0.08)
            point = (point[0] + length * math.cos(heading), point[1] + length * math.sin(heading), 0.0)
            revision = stream.append(LineMove(k, point, 30.0, 0.005), 0.0)
            self.assertTrue(revision.ok, revision.detail)
        assert_path_continuous(self, stream)
        self.assertTrue(all(piece.length > 0 for _, piece, _ in stream.pieces()))
        self.assertTrue(all(c.deviation <= 0.005 * (1 + 1e-12) for c in stream.corners))
        end = stream.pieces()[-1][1]
        self.assertLess(norm(sub(end.evaluate(end.length)[0], point)), 1e-9)

    def test_committed_position_limits_trim(self):
        stream = self.make((0, 0, 0))
        stream.append(LineMove(1, (10, 0, 0), 50.0, 0.5), 0.0)
        for committed in (0.0, 9.0, 9.99, 10.0):
            revision = stream.propose(LineMove(2, (10, 10, 0), 50.0, None), committed)
            with self.subTest(committed=committed):
                self.assertGreaterEqual(revision.from_s, committed - 1e-12)
                if committed == 10.0:
                    self.assertEqual(revision.corner.kind, "stop")
        no_blend = stream.propose(LineMove(2, (10, 10, 0), 50.0, None), 0.0, blend=False)
        self.assertEqual((no_blend.corner.kind, no_blend.from_s), ("stop", 10.0))

    def test_blend_consuming_a_line_keeps_the_exact_stop_before_it(self):
        """Regression (pipeline, 2026-10-10): a blend that consumes the whole line after a G61 vertex must
        not clear the stop at that vertex."""
        stream = self.make((0, 0, 0))
        stream.append(LineMove(1, (10, 0, 0), 50.0, None), 0.0)
        stream.append(LineMove(2, (10, 0.1, 0), 50.0, 0.5), 0.0)
        revision = stream.append(LineMove(3, (20, 0.1, 0), 50.0), 0.0)
        self.assertEqual(revision.corner.kind, "blend")
        self.assertAlmostEqual(revision.from_s, 10.0, places=12)
        stops = [(round(s0 + piece.length, 9), stop) for s0, piece, stop in stream.pieces()]
        self.assertIn((10.0, True), stops)

    def test_markers_once_per_source_in_order(self):
        rng = random.Random(4403)
        stream = self.make((0, 0, 0))
        point = (0.0, 0.0, 0.0)
        for k in range(40):
            if rng.random() < 0.1:
                target = point  # zero-length move
            else:
                target = tuple(p + rng.uniform(-5, 5) for p in point)
            stream.append(LineMove(k, target, 40.0, rng.choice([None, 0.01, 0.1])), 0.0)
            point = target
        markers = [stream.markers[k] for k in range(40)]
        self.assertEqual(markers, sorted(markers))
        self.assertAlmostEqual(markers[-1], stream.end_s, places=12)
        for corner_ in stream.corners:
            self.assertAlmostEqual(stream.markers[corner_.source_id], corner_.s, places=12)


class GeometryStreamTests(GeometryContract, unittest.TestCase):
    @staticmethod
    def make(start):
        return GeometryStream(start)

    def test_line_piece(self):
        piece = LinePiece(1, 10.0, (1, 2, 3), (0.6, 0.0, 0.8), 5.0)
        self.assertEqual(piece.evaluate(5.0)[0], (4.0, 2.0, 7.0))
        self.assertEqual(piece.bounds().first, (0.6, 0.0, 0.8))

    def test_hand_derived_arcs(self):
        cases = (  # (start, move, length, midpoint)
            ((10, 0, 0), ArcMove(1, (0, 10, 0), (0, 0, 0), 2, True, 20.0), 5 * math.pi, (10 / math.sqrt(2), 10 / math.sqrt(2), 0)),
            ((10, 0, 0), ArcMove(1, (0, 10, 0), (0, 0, 0), 2, False, 20.0), 15 * math.pi, (-10 / math.sqrt(2), -10 / math.sqrt(2), 0)),
            ((0, 5, 0), ArcMove(1, (0, 5, 0), (0, 0, 0), 2, True, 20.0), 10 * math.pi, (0, -5, 0)),
            ((0, 3, 0), ArcMove(1, (0, 0, 3), (0, 0, 0), 0, True, 20.0), 1.5 * math.pi, (0, 3 / math.sqrt(2), 3 / math.sqrt(2))),
            ((0, 0, 4), ArcMove(1, (4, 0, 0), (0, 0, 0), 1, True, 20.0), 2 * math.pi, (4 / math.sqrt(2), 0, 4 / math.sqrt(2))),
        )
        for index, (start, move, length, middle) in enumerate(cases):
            revision = GeometryStream(start).propose(move, 0.0)
            piece = revision.pieces[0]
            with self.subTest(index=index):
                self.assertIsInstance(piece, ArcPiece)
                self.assertAlmostEqual(piece.length, length, places=12)
                for got, want in zip(piece.evaluate(length / 2)[0], middle):
                    self.assertAlmostEqual(got, want, places=12)
                for got, want in zip(piece.evaluate(length)[0], move.end):
                    self.assertAlmostEqual(got, want, places=12)
                assert_derivatives(self, piece)
                assert_bounds(self, piece)
                bounds = piece.bounds()
                self.assertAlmostEqual(max(bounds.second), 1 / piece.radius, places=12)

    def test_invalid_arcs(self):
        stream = GeometryStream((10, 0, 0))
        cases = (
            ArcMove(1, (0, 10, 1), (0, 0, 0), 2, True, 20.0),        # helical
            ArcMove(1, (10, 0, 0), (10, 0, 0), 2, True, 20.0),       # zero radius full circle
            ArcMove(1, (0, 10.1, 0), (0, 0, 0), 2, True, 20.0),      # radius mismatch far beyond refit limit
        )
        for index, move in enumerate(cases):
            with self.subTest(index=index):
                self.assertIs(stream.propose(move, 0.0).status, SolveStatus.INVALID_INPUT)

    def test_arc_radius_refit_within_limit(self):
        revision = GeometryStream((10, 0, 0)).propose(ArcMove(1, (0, 10.002, 0), (0, 0, 0), 2, True, 20.0), 0.0)
        piece = revision.pieces[0]
        for got, want in zip(piece.evaluate(piece.length)[0], (0, 10.002, 0)):
            self.assertAlmostEqual(got, want, places=12)

    def test_arc_joins(self):
        stream = GeometryStream((0, 0, 0))
        stream.append(LineMove(1, (10, 0, 0), 30.0, 0.1), 0.0)
        tangent_arc = stream.append(ArcMove(2, (15, 5, 0), (10, 5, 0), 2, True, 30.0, 0.1), 0.0)
        self.assertEqual(tangent_arc.corner.kind, "stop")  # tangent but curvature jumps 0 -> 1/5
        same_circle = stream.append(ArcMove(3, (10, 10, 0), (10, 5, 0), 2, True, 30.0, 0.1), 0.0)
        self.assertEqual(same_circle.corner.kind, "continuous")
        exact = stream.append(ArcMove(4, (5, 5, 0), (10, 5, 0), 2, True, 30.0, None), 0.0)
        self.assertEqual(exact.corner.kind, "continuous")
        after_g61 = stream.append(ArcMove(5, (10, 0, 0), (10, 5, 0), 2, True, 30.0, 0.1), 0.0)
        self.assertEqual(after_g61.corner.kind, "stop")
        line_after = stream.append(LineMove(6, (20, 0, 0), 30.0, 0.1), 0.0)
        self.assertEqual(line_after.corner.kind, "stop")
        assert_path_continuous(self, stream)

    def test_clothoid_against_independent_integration(self):
        rng = random.Random(4404)
        for case in range(60):
            t_in = random_unit(rng)
            t_out = rotate_towards(t_in, random_unit(rng), rng.uniform(0.05, 3.0))
            b = tuple(rng.uniform(-50, 50) for _ in range(3))
            a = tuple(p - 20 * d for p, d in zip(b, t_in))
            c = tuple(p + 20 * d for p, d in zip(b, t_out))
            _, revision = corner(a, a, b, c, rng.uniform(0.01, 1.0))
            for piece in revision.pieces:
                if not isinstance(piece, ClothoidPiece):
                    continue
                with self.subTest(case=case, reverse=piece.reverse):
                    for k in range(5):
                        u = piece.length * k / 4
                        reference = integrated_clothoid(piece, u)
                        self.assertLess(norm(sub(piece.evaluate(u)[0], reference)), 1e-10 * max(1.0, piece.length))
                    assert_derivatives(self, piece)
                    assert_bounds(self, piece)


if __name__ == "__main__":
    unittest.main()
