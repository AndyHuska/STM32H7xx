"""End-to-end L3 -> L4 -> L5 -> L6 scenarios, run against a spectrum of machine configurations.

Executed motion is certified by the independent double-precision validators (path continuity and
endpoint, per-piece path envelope, feed caps, per-axis limits from geometry derivative bounds) and,
independently of all derivative bounds, by finite differences of sampled axis positions q(s(t)).
"""

import bisect
import math
import random
import unittest

from tools.trajectory_sim.frontend import MotionFrontEnd
from tools.trajectory_sim.geometry import GeometryStream
from tools.trajectory_sim.lookahead import ConservativeLookahead
from tools.trajectory_sim.model import AccelerationStep, ArcMove, Limits, LineMove, State, Trajectory
from tools.trajectory_sim.phases import advance, trajectory
from tools.trajectory_sim.runtime import HOLD, StreamingRuntime, run
from tools.trajectory_sim.scalar_solver import DISTANCE_SLACK, AnalyticTimeLaw
from tools.trajectory_sim.tests.machines import spectrum
from tools.trajectory_sim.tests.test_lookahead import TOL, outside
from tools.trajectory_sim.tests.test_runtime import cap_intervals
from tools.trajectory_sim.validation import validate, validate_axes, validate_caps, validate_envelope

LAW = AnalyticTimeLaw()
PLANNER_LIMITS = Limits(1e6, 1e9, math.inf, 1e9)  # every real limit comes from the projected pieces
LATENCY, DT = 0.03, 0.005
MACHINES = spectrum()


class Pipeline:
    """Test harness around the production front end (L8); records what certification needs."""

    def __init__(self, axes, start=(0.0, 0.0, 0.0)):
        self.axes = axes
        self.stream = GeometryStream(start)
        self.runtime = StreamingRuntime(ConservativeLookahead(LAW), LAW, PLANNER_LIMITS, LATENCY)
        self.front = MotionFrontEnd(self.stream, self.runtime, ConservativeLookahead(LAW), axes)
        self.tolerances, self.last_point = {}, start

    def feed(self, move):
        self.tolerances[move.source_id] = move.tolerance
        self.last_point = move.end
        tail = self.runtime.tail.s
        revision = self.front.feed(move)
        assert revision.ok, revision.detail
        assert revision.from_s >= tail - 1e-9


def axis_samples(pipeline, dt):
    """Axis positions at times k*dt from the committed elements and the final geometry; flags hold samples."""
    start, elements, policies = pipeline.runtime.executed()
    rows, t, state = [], 0.0, start
    for element, policy in zip(elements, policies):
        if not isinstance(element, AccelerationStep) and element.duration > 0:
            rows.append((t, state, element, policy))
        state = advance(state, element)
        t += element.duration
    starts = [row[0] for row in rows]
    entries = pipeline.stream.pieces()
    piece_starts = [entry[0] for entry in entries]
    samples = []
    for k in range(int(t / dt)):
        now = k * dt
        row = rows[max(0, bisect.bisect_right(starts, now) - 1)]
        tau = now - row[0]
        s = row[1].s + row[1].v * tau + row[1].a * tau ** 2 / 2 + row[2].jerk * tau ** 3 / 6
        index = max(0, bisect.bisect_right(piece_starts, s) - 1)
        s0, piece, _ = entries[index]
        samples.append((piece.evaluate(min(max(s - s0, 0.0), piece.length))[0], row[3] == HOLD))
    return samples


class PipelineTests(unittest.TestCase):
    def certify(self, pipeline, label):
        runtime, stream, axes = pipeline.runtime, pipeline.stream, pipeline.axes
        start, elements, policies = runtime.executed()
        path = trajectory(start, elements)
        waived = frozenset(i for i, policy in enumerate(policies) if policy == HOLD)
        with self.subTest(machine=label, check="path"):
            self.assertAlmostEqual(runtime.queue_end, stream.end_s, delta=1e-9)
            validate(path, PLANNER_LIMITS, State(runtime.queue_end, 0.0, 0.0), TOL).require_valid()
            end = stream.pieces()[-1][1]
            self.assertLess(max(abs(x - y) for x, y in zip(end.evaluate(end.length)[0], pipeline.last_point)), 1e-9)
        with self.subTest(machine=label, check="envelope"):
            # Planned path limits bind normal motion; hold motion uses the per-axis hold budget and is
            # certified by the axis check below (jerk waived).
            intervals, s = [], 0.0
            for block in runtime.blocks:
                lim = block.limits
                intervals.append((s, s + block.length, Limits(lim.speed, lim.acceleration, lim.jerk, lim.deceleration)))
                s += block.length
            normal = Trajectory(path.initial, tuple(seg for i, seg in enumerate(path.segments) if i not in waived))
            validate_envelope(normal, intervals, TOL).require_valid()
            validate_caps(path, outside(cap_intervals(runtime), runtime.transients), TOL).require_valid()
        with self.subTest(machine=label, check="axes"):
            pieces = [(s0, s0 + piece.length, *(lambda b: (b.first, b.second, b.third))(piece.bounds()))
                      for s0, piece, _ in stream.pieces()]
            validate_axes(path, pieces, axes, TOL, waived).require_valid()
        with self.subTest(machine=label, check="deviation"):
            for corner in stream.corners:
                if corner.kind == "blend":
                    self.assertLessEqual(corner.deviation, pipeline.tolerances[corner.source_id] * (1 + 1e-12))
        with self.subTest(machine=label, check="finite differences"):
            self.assert_finite_differences(pipeline)

    def assert_finite_differences(self, pipeline, dt=1e-2):
        """Divided differences of q(s(t)) are weighted means of axis velocity, acceleration and jerk (axis
        positions are C^2 off hold), so they must respect the limits up to the position uncertainty: float
        rounding plus L2's declared DISTANCE_SLACK (a stop may land that far before a vertex and continue
        under the next piece's limits; |q'| = 1 and a vertex kink at most doubles it)."""
        samples = axis_samples(pipeline, dt)
        for k in range(len(samples) - 3):
            window = samples[k:k + 4]
            if any(hold for _, hold in window):
                continue
            q = [point for point, _ in window]
            for i, axis in enumerate(pipeline.axes):
                rounding = 1e-13 * max(1.0, max(abs(p[i]) for p in q)) + 2 * DISTANCE_SLACK
                self.assertLessEqual(abs(q[1][i] - q[0][i]) / dt, axis.speed * (1 + 1e-9) + 2 * rounding / dt)
                self.assertLessEqual(abs(q[2][i] - 2 * q[1][i] + q[0][i]) / dt ** 2,
                                     max(axis.acceleration, axis.deceleration) * (1 + 1e-9) + 4 * rounding / dt ** 2)
                if axis.jerk_limited:
                    third = abs(q[3][i] - 3 * q[2][i] + 3 * q[1][i] - q[0][i]) / dt ** 3
                    self.assertLessEqual(third, axis.jerk * (1 + 1e-9) + 8 * rounding / dt ** 3)

    def run_program(self, axes, moves_at, extra_events=(), max_time=600.0):
        pipeline = Pipeline(axes)
        events = [(t, lambda rt, m=move: pipeline.feed(m)) for t, move in moves_at] + list(extra_events)
        run(pipeline.runtime, events, DT, max_time)
        self.assertTrue(pipeline.runtime.idle, "program did not finish")
        return pipeline

    def test_square_with_g64(self):
        corners = [(40, 0, 0), (40, 30, 0), (0, 30, 5), (0, 0, 5), (0, 0, 0)]
        for label, axes in MACHINES.items():
            moves = [(0.0, LineMove(k, point, 120.0, 0.05)) for k, point in enumerate(corners)]
            pipeline = self.run_program(axes, moves)
            # Each corner is a certified blend or, where that is faster, an exact stop (DESIGN.md 1A).
            self.assertTrue(set(c.kind for c in pipeline.stream.corners) <= {"blend", "stop"})
            self.assertEqual(len(pipeline.stream.corners), 4)
            self.certify(pipeline, label)

    def test_random_streamed_programs(self):
        """Random 3D polylines arriving with gaps (starvation), mixed G61/G64 P, override and hold."""
        rng = random.Random(4601)
        kinds = {}
        for label, axes in MACHINES.items():
            for case in range(3):
                point, t, moves = (0.0, 0.0, 0.0), 0.0, []
                for k in range(10):
                    point = tuple(p + rng.uniform(-1, 1) * 10 ** rng.uniform(-1.5, 1.2) for p in point)
                    t += rng.choice([0.0, 0.0, rng.uniform(0.0, 0.6)])
                    moves.append((t, LineMove(k, point, rng.uniform(20, 150), rng.choice([None, 0.01, 0.1, 0.5]))))
                extra = []
                if rng.random() < 0.5:
                    extra.append((rng.uniform(0, t + 0.5), lambda rt, f=rng.uniform(0.3, 1.5): rt.set_override(f)))
                if rng.random() < 0.4:
                    hold_at = rng.uniform(0, t + 0.5)
                    extra += [(hold_at, lambda rt: rt.hold()), (hold_at + rng.uniform(0.1, 0.8), lambda rt: rt.resume())]
                pipeline = self.run_program(axes, moves, extra)
                for corner in pipeline.stream.corners:
                    kinds[corner.kind] = kinds.get(corner.kind, 0) + 1
                self.certify(pipeline, f"{label} case {case}")
        print(f"\nstreamed programs: corner kinds {kinds}")
        self.assertGreater(kinds.get("blend", 0), 20)

    def test_tiny_segment_arc_approximation(self):
        """0.05 mm chords of a 10 mm radius arc, the way grblHAL segments G2/G3 today."""
        for label, axes in MACHINES.items():
            moves, step = [], 0.005
            for k in range(1, 60):
                angle = k * step
                moves.append((0.0, LineMove(k, (10 * math.sin(angle), 10 - 10 * math.cos(angle), 0.0), 100.0, 0.002)))
            pipeline = self.run_program(axes, moves)
            self.assertTrue(all(c.kind == "blend" for c in pipeline.stream.corners))
            self.certify(pipeline, label)

    def test_arcs_and_joins(self):
        program = [
            LineMove(1, (10, 0, 0), 60.0, 0.1),
            ArcMove(2, (15, 5, 0), (10, 5, 0), 2, True, 60.0, 0.1),     # tangent, curvature jump: stop
            ArcMove(3, (10, 10, 0), (10, 5, 0), 2, True, 60.0, 0.1),    # same circle: continuous
            LineMove(4, (0, 10, 0), 60.0, 0.1),                         # arc -> line: stop
            ArcMove(5, (0, 10, 0), (0, 5, 0), 0, False, 30.0, None),    # full circle in YZ, not tangent: stop
            LineMove(6, (0, 0, 0), 60.0, None),
        ]
        for label, axes in MACHINES.items():
            pipeline = self.run_program(axes, [(0.0, move) for move in program])
            self.assertEqual([c.kind for c in pipeline.stream.corners], ["stop", "continuous", "stop", "stop", "stop"])
            self.certify(pipeline, label)

    def test_late_move_shrinks_or_stops_the_corner(self):
        """The second move arrives while the tail is near the vertex: the blend may only start ahead of
        the committed tail; otherwise the corner is an exact stop. Both are certified."""
        for label, axes in MACHINES.items():
            outcomes = set()
            for delay in (0.05, 0.2, 0.4, 0.8):
                moves = [(0.0, LineMove(1, (12, 0, 0), 100.0, 0.2)), (delay, LineMove(2, (12, 12, 0), 100.0))]
                pipeline = self.run_program(axes, moves)
                outcomes.add(pipeline.stream.corners[0].kind)
                self.certify(pipeline, f"{label} delay {delay}")
            print(f"\nlate move on {label}: {sorted(outcomes)}")

    def test_hold_at_rest_on_a_stop_vertex(self):
        """Regression (standard_high_accel, 2026-10-10): hold while idle at the end of an X move; a Y move with
        lower limits arrives during the hold. A hold at rest must not brake (a step to the X move's hold
        deceleration would act on the Y move), and braking from a vertex uses the block ahead."""
        for label, axes in MACHINES.items():
            moves = [(0.0, LineMove(1, (5, 0, 0), 100.0)), (3.0, LineMove(2, (5, 5, 0), 100.0))]
            extra = [(2.5, lambda rt: rt.hold()), (3.5, lambda rt: rt.resume())]
            pipeline = self.run_program(axes, moves, extra)
            hold_steps = [e for e, p in zip(*pipeline.runtime.executed()[1:]) if p == HOLD and isinstance(e, AccelerationStep)]
            with self.subTest(machine=label):
                self.assertTrue(all(step.acceleration == 0.0 for step in hold_steps))
            self.certify(pipeline, f"{label} hold at rest")

    def test_axis_aligned_right_angle_g64_beats_g61(self):
        """X decelerates while Y accelerates and the path is shorter. A G64 corner is never slower than an
        exact stop; with room to blend (P = 3 mm at F100: apex radius >> v^2/A) it is strictly faster on
        the moderate machines. Small P cannot gain much physically: a 90 deg corner within P has radius
        ~2.4 P, so the apex is jerk-limited to v ~ (J R^2)^(1/3) (~7 mm/s at P 0.05), and the front end then
        keeps the exact stop. Both executions certified."""
        rows = []
        for label, axes in MACHINES.items():
            stop_run = self.run_program(axes, [(0.0, LineMove(1, (20, 0, 0), 100.0)), (0.0, LineMove(2, (20, 20, 0), 100.0))])
            stop_time = motion_time(stop_run.runtime)
            row = [label, stop_time]
            for tolerance in RIGHT_ANGLE_P:
                moves = [(0.0, LineMove(1, (20, 0, 0), 100.0, tolerance)), (0.0, LineMove(2, (20, 20, 0), 100.0))]
                pipeline = self.run_program(axes, moves)
                blend_time = motion_time(pipeline.runtime)
                kind = pipeline.stream.corners[0].kind
                row.append((blend_time, apex_speed(pipeline) if kind == "blend" else 0.0))
                with self.subTest(machine=label, P=tolerance):
                    self.assertLessEqual(blend_time, stop_time * (1 + 1e-9))
                    if tolerance == 3.0:
                        self.assertEqual(kind, "blend")
                        self.assertLess(blend_time, stop_time * 0.95)
                    self.certify(pipeline, f"{label} P{tolerance}")
            rows.append(row)
        print("\naxis-aligned 90 deg, 20 mm legs, F100: G61 time | G64 time (apex mm/s, 0 = stop kept) per P " + str(RIGHT_ANGLE_P))
        for label, stop_time, *runs in rows:
            print(f"  {label:22s} {stop_time:7.3f} s | " + " | ".join(f"{t:7.3f} s ({v:6.2f})" for t, v in runs))

    def test_g64_never_slower_than_g61(self):
        """Requirement (DESIGN.md 1A): with the same limits a G64 run (any P) never takes longer than the same
        program with exact stops. Programs: zigzag with 30-150 deg corners, random 3D polylines, chords."""
        rng = random.Random(4602)
        programs = {"zigzag": zigzag()}
        for seed in range(3):
            point, path = (0.0, 0.0, 0.0), []
            for _ in range(8):
                point = tuple(p + rng.uniform(-1, 1) * 10 ** rng.uniform(-1, 1.3) for p in point)
                path.append(point)
            programs[f"random {seed}"] = path
        programs["chords"] = [(10 * math.sin(k * 0.02), 10 - 10 * math.cos(k * 0.02), 0.0) for k in range(1, 30)]
        worst = {}
        for label, axes in MACHINES.items():
            for name, points in programs.items():
                stop_time = plan_time(axes, points, None)
                for tolerance in P_VALUES:
                    blend_time = plan_time(axes, points, tolerance)
                    ratio = blend_time / stop_time
                    worst[label] = max(worst.get(label, 0.0), ratio)
                    with self.subTest(machine=label, program=name, P=tolerance):
                        self.assertLessEqual(blend_time, stop_time * (1 + 1e-9))
        print("\nG64 / G61 worst time ratio per machine: " + ", ".join(f"{k} {v:.3f}" for k, v in worst.items()))


P_VALUES = (0.001, 0.01, 0.05, 0.2, 1.0)
RIGHT_ANGLE_P = (0.01, 0.05, 0.2, 1.0, 3.0)


def zigzag(leg=20.0, angles=(30, 60, 90, 120, 150)):
    point, heading, points = (0.0, 0.0, 0.0), 0.0, []
    for k, angle in enumerate((0,) + angles):
        heading += math.radians(angle) * (1 if k % 2 else -1)
        point = (point[0] + leg * math.cos(heading), point[1] + leg * math.sin(heading), 0.0)
        points.append(point)
    return points


def plan_time(axes, points, tolerance, feed=100.0):
    """All moves queued before motion starts: the runtime's plan from rest is the whole executed motion."""
    pipeline = Pipeline(axes)
    for k, point in enumerate(points):
        pipeline.feed(LineMove(k, point, feed, tolerance))
    runtime = pipeline.runtime
    assert not runtime.failures and abs(trajectory(runtime.tail, runtime.suffix).final.s - runtime.queue_end) < 1e-6
    return sum(element.duration for element in runtime.suffix)


def motion_time(runtime):
    """Time at which the executed motion comes to rest for the last time."""
    start, elements, _ = runtime.executed()
    state, t, finish = start, 0.0, 0.0
    for element in elements:
        end = advance(state, element)
        if state.v > 0.0 or end.v > 0.0 or state.a != 0.0 or end.a != 0.0:
            finish = t + element.duration
        state, t = end, t + element.duration
    return finish


def apex_speed(pipeline):
    """Executed speed where s crosses the first corner's marker (blend midpoint)."""
    target = pipeline.stream.corners[0].s
    start, elements, _ = pipeline.runtime.executed()
    state = start
    for element in elements:
        end = advance(state, element)
        if state.s <= target <= end.s and element.duration > 0:
            low, high = 0.0, element.duration
            for _ in range(60):
                middle = 0.5 * (low + high)
                low, high = (middle, high) if advance(state, element, middle).s < target else (low, middle)
            return advance(state, element, high).v
        state = end
    return math.nan


if __name__ == "__main__":
    unittest.main()
