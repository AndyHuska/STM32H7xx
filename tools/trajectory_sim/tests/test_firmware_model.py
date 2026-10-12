"""Checks of the test-only firmware model (tests/firmware_model.py) against hand derivations and an
independent phase-plane integration. Oracles do not use the model's planner code."""

import math
import unittest

from tools.trajectory_sim.model import Limits
from tools.trajectory_sim.tests.firmware_model import G61, G61_1, FirmwareMove, run

AXES = [Limits(200.0, 1000.0, math.inf), Limits(150.0, 800.0, math.inf), Limits(50.0, 400.0, math.inf)]


def trapezoid_rest_to_rest(length, speed, accel):
    if length >= speed * speed / accel:
        return speed / accel + length / speed
    return 2.0 * math.sqrt(length / accel)


def phase_plane_time(model, nodes_per_block=4000):
    """Independent oracle: fastest v(s) with v^2 <= cap^2 inside each block, v^2 <= max entry^2 at each block
    start, v = 0 at both ends, |d(v^2)/ds| <= 2 a per block (forward and backward sweeps on a fine grid),
    time = sum 2 ds / (v0 + v1) (exact for constant acceleration per interval). Unlimited planner window.
    Validates the planner and trapezoid timing; block limits are checked against hand values elsewhere."""
    s, cap2, acc = [0.0], [0.0], []
    for index, block in enumerate(model.blocks):
        if index:
            cap2[-1] = min(cap2[-1], block.max_entry_sqr)
        s0 = s[-1]
        for k in range(1, nodes_per_block + 1):
            s.append(s0 + block.length * k / nodes_per_block)
            cap2.append(block.nominal ** 2)
            acc.append(block.acceleration)
    cap2[-1] = 0.0
    v2 = cap2[:]
    for k in range(1, len(s)):
        v2[k] = min(v2[k], v2[k - 1] + 2.0 * acc[k - 1] * (s[k] - s[k - 1]))
    for k in range(len(s) - 2, -1, -1):
        v2[k] = min(v2[k], v2[k + 1] + 2.0 * acc[k] * (s[k + 1] - s[k]))
    v = [math.sqrt(max(0.0, x)) for x in v2]
    return sum(2.0 * (s[k + 1] - s[k]) / (v[k] + v[k + 1]) for k in range(len(s) - 1))


class FirmwareModelTests(unittest.TestCase):
    def test_single_line_is_a_trapezoid(self):
        for end, feed in (((20.0, 0.0, 0.0), 100.0), ((3.0, 4.0, 0.0), 1e4), ((0.0, 0.0, 0.5), 100.0)):
            model = run(AXES, [FirmwareMove(end, feed, 1.0)])
            length = math.dist((0, 0, 0), end)
            unit = [c / length for c in end]
            speed = min([feed] + [a.speed / abs(u) for u, a in zip(unit, AXES) if u])
            accel = min(a.acceleration / abs(u) for u, a in zip(unit, AXES) if u)
            self.assertAlmostEqual(model.duration(), trapezoid_rest_to_rest(length, speed, accel), delta=1e-12)

    def test_right_angle_arc_geometry_and_limits(self):
        """90 deg in XY at P: theta = pi/2, R = P/(sqrt2 - 1), trim = R, vertex-to-arc = P; arc blocks capped at
        sqrt(0.5 A_min R) (A_min = Z's 400) and accelerate at 0.5 A_min."""
        p = 0.5
        model = run(AXES, [FirmwareMove((20.0, 0.0, 0.0), 100.0, p), FirmwareMove((20.0, 20.0, 0.0), 100.0, p)])
        radius = p / (math.sqrt(2.0) - 1.0)
        self.assertEqual([b.kind for b in model.blocks], ["line", "arc", "arc", "line"])
        corner = model.corners[0]
        self.assertAlmostEqual(corner.radius, radius, delta=1e-12)
        self.assertAlmostEqual(corner.trim, radius, delta=1e-12)
        self.assertAlmostEqual(corner.deviation, p, delta=1e-12)
        line_in, arc1, arc2, line_out = model.blocks
        self.assertAlmostEqual(line_in.length, 20.0 - radius, delta=1e-12)
        self.assertAlmostEqual(arc1.length, radius * math.pi / 4, delta=1e-12)
        self.assertAlmostEqual(arc2.end[0], 20.0 - radius + radius * math.sin(math.pi / 2), delta=1e-12)
        for arc in (arc1, arc2):
            self.assertAlmostEqual(arc.nominal, math.sqrt(0.5 * 400.0 * radius), delta=1e-12)
            self.assertAlmostEqual(arc.acceleration, 200.0, delta=1e-12)
        # Tangent joins: entry limited only by the neighbouring nominal speeds.
        self.assertAlmostEqual(arc1.max_entry_sqr, arc1.nominal ** 2, delta=1e-9)
        self.assertAlmostEqual(line_out.max_entry_sqr, arc2.nominal ** 2, delta=1e-9)
        # Arc midpoint is the vertex moved P along the bisector.
        mid = arc1.end
        self.assertAlmostEqual(math.dist(mid, (20.0, 0.0, 0.0)), p, delta=1e-12)

    def test_planar_arc_limits_ignore_axes_outside_the_plane(self):
        """FW-1: an XY arc is limited by X and Y only: rate min(V_x, V_y, sqrt(0.5 min(A_x, A_y) R)), accel
        0.5 min(A_x, A_y); the current code also uses Z (V_z = 50, A_z = 400)."""
        p = 0.5
        moves = [FirmwareMove((20.0, 0.0, 0.0), 100.0, p), FirmwareMove((20.0, 20.0, 0.0), 100.0, p)]
        radius = p / (math.sqrt(2.0) - 1.0)
        planar = run(AXES, moves, arc_limits="planar")
        for arc in planar.blocks[1:3]:
            self.assertAlmostEqual(arc.nominal, min(100.0, math.sqrt(0.5 * 800.0 * radius)), delta=1e-12)
            self.assertAlmostEqual(arc.acceleration, 400.0, delta=1e-12)
        self.assertLess(planar.duration(), run(AXES, moves).duration())

    def test_circle_arc_limits_keep_every_axis_within_limits(self):
        """FW-2: v^2/R <= k A_p and a_t <= sqrt(1 - k^2) A_p, A_p = min A_i / amp_i, keep each axis' acceleration
        (two quadrature sinusoids, amplitude amp_i sqrt(a_t^2 + (v^2/R)^2) <= A_i) within its limit. Checked by
        second differences of sampled positions on tilted 3D corners; the bound is reached (ratio ~1) because
        the lines run at full axis acceleration. Error model: dt^2 terms of a piecewise-quadratic s(t) (<1e-6)."""
        moves = [FirmwareMove((10.0, 2.0, 1.0), 150.0, 0.8), FirmwareMove((14.0, 9.0, -1.5), 150.0, 0.8),
                 FirmwareMove((6.0, 12.0, 0.5), 150.0, 0.8)]
        dt = 2e-4
        for k in (0.5 ** 0.5, 0.9, 0.99):
            model = run(AXES, moves, arc_limits="circle", arc_k=k)
            arc = model.blocks[1]
            self.assertEqual(arc.kind, "arc")
            q = model.samples(dt)
            for i, axis in enumerate(AXES):
                worst = max(abs(q[j + 1][i] - 2 * q[j][i] + q[j - 1][i]) / dt ** 2 for j in range(1, len(q) - 1))
                with self.subTest(k=k, axis=i):
                    self.assertLessEqual(worst, axis.acceleration * (1 + 1e-6))

    def test_adaptive_arc_law_matches_direct_integration(self):
        """FW-2 adaptive: reach and arc block times from the closed form asin(w1) = asin(w0) + 2L/R vs a direct
        RK4 integration of d(v^2)/ds = 2 A sqrt(1 - (v^2 / (A R))^2) on a fine s grid (forward from the entry,
        backward from the exit, capped at the nominal speed), time = sum 2 ds / (v0 + v1). Error O(ds^2)."""
        moves = [FirmwareMove((20.0, 0.0, 0.0), 300.0, 1.0), FirmwareMove((20.0, 3.0, 0.0), 300.0, 1.0),
                 FirmwareMove((25.0, 3.0, 0.0), 300.0, 1.0)]
        model = run(AXES, moves, arc_limits="adaptive", arc_k=0.95)
        rows = model.schedule()
        arcs = [(b, r) for b, r in zip(model.blocks, rows) if b.circle]
        self.assertGreater(len(arcs), 0)
        for block, (u, w, duration) in arcs:
            c2 = block.circle * block.radius

            def slope(b2):
                return 2.0 * block.circle * math.sqrt(max(0.0, 1.0 - (b2 / c2) ** 2))

            def sweep(start_sqr, n):
                h, values = block.length / n, [start_sqr]
                for _ in range(n):
                    y = values[-1]
                    k1 = slope(y)
                    k2 = slope(y + 0.5 * h * k1)
                    k3 = slope(y + 0.5 * h * k2)
                    k4 = slope(y + h * k3)
                    values.append(min(block.nominal ** 2, y + h * (k1 + 2 * k2 + 2 * k3 + k4) / 6.0))
                return values

            n = 20000
            forward, backward = sweep(u * u, n), sweep(w * w, n)[::-1]
            v = [math.sqrt(min(f, b)) for f, b in zip(forward, backward)]
            h = block.length / n
            direct = sum(2.0 * h / (v[k] + v[k + 1]) for k in range(n))
            with self.subTest(arc=block.start):
                self.assertLessEqual(w * w, forward[-1] * (1 + 1e-9))
                self.assertAlmostEqual(duration, direct, delta=1e-6 * duration)

    def test_trim_clamped_to_half_the_shorter_leg(self):
        model = run(AXES, [FirmwareMove((2.0, 0.0, 0.0), 100.0, 3.0), FirmwareMove((2.0, 10.0, 0.0), 100.0, 3.0)])
        corner = model.corners[0]
        self.assertAlmostEqual(corner.trim, 1.0, delta=1e-12)
        self.assertAlmostEqual(corner.radius, 1.0, delta=1e-12)
        self.assertAlmostEqual(corner.deviation, math.sqrt(2.0) - 1.0, delta=1e-12)

    def test_g61_corner_uses_junction_deviation(self):
        """Exact path (G61): no arc, entry v^2 = A_j jd sin(theta/2) / (1 - sin(theta/2)); for 90 deg in XY
        A_j = min(A_x, A_y) / (1/sqrt2) along (-1, 1)/sqrt2 and sin(theta/2) = 1/sqrt2."""
        model = run(AXES, [FirmwareMove((20.0, 0.0, 0.0), 100.0, G61), FirmwareMove((20.0, 20.0, 0.0), 100.0, G61)],
                    junction_deviation=0.01)
        a_j = 800.0 * math.sqrt(2.0)
        sin_half = 1.0 / math.sqrt(2.0)
        self.assertAlmostEqual(model.blocks[1].max_entry_sqr, a_j * 0.01 * sin_half / (1.0 - sin_half), delta=1e-9)

    def test_exact_stop_and_reversal_stop(self):
        moves = [FirmwareMove((20.0, 0.0, 0.0), 100.0, G61_1), FirmwareMove((20.0, 20.0, 0.0), 100.0, G61_1)]
        expected = trapezoid_rest_to_rest(20.0, 100.0, 1000.0) + trapezoid_rest_to_rest(20.0, 100.0, 800.0)
        self.assertAlmostEqual(run(AXES, moves).duration(), expected, delta=1e-12)
        reversal = run(AXES, [FirmwareMove((20.0, 0.0, 0.0), 100.0, 1.0), FirmwareMove((5.0, 0.0, 0.0), 100.0, 1.0)])
        self.assertEqual([b.kind for b in reversal.blocks], ["line", "line"])
        self.assertEqual(reversal.blocks[1].max_entry_sqr, 0.0)

    def test_planner_matches_independent_phase_plane_integration(self):
        """Backward/forward planner + trapezoids vs a fine-grid phase-plane sweep (unlimited window): the
        grid error is O(ds^2) per accel/cruise switch, far below 1e-6 relative at 4000 nodes per block."""
        programs = {
            "corner": [((20.0, 0.0, 0.0), 0.5), ((20.0, 20.0, 0.0), 0.5)],
            "zigzag": [((10.0, 0.0, 0.0), 0.2), ((15.0, 3.0, 0.0), 0.2), ((15.5, 9.0, 1.0), 0.2), ((8.0, 9.0, 1.0), 0.2),
                       ((8.0, 9.2, 0.0), 0.2)],
            "mixed": [((10.0, 0.0, 0.0), G61), ((10.0, 5.0, 0.0), 1.0), ((4.0, 8.0, 0.0), G61_1), ((0.0, 0.0, 0.0), 0.05)],
        }
        for name, moves in programs.items():
            model = run(AXES, [FirmwareMove(end, 120.0, control) for end, control in moves])
            with self.subTest(program=name):
                self.assertAlmostEqual(model.duration(), phase_plane_time(model), delta=1e-6 * model.duration())

    def test_short_window_is_never_faster(self):
        """An earlier forced exit 0 only lowers the backward bounds, so speeds and time cannot improve."""
        moves = [FirmwareMove((2.0 * k, (k % 2) * 1.0, 0.0), 150.0, 0.05) for k in range(1, 30)]
        full = run(AXES, moves).duration()
        short = run(AXES, moves, buffer_blocks=4).duration()
        self.assertGreaterEqual(short, full * (1 - 1e-12))


if __name__ == "__main__":
    unittest.main()
