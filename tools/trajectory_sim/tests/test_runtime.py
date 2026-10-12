"""L6 streaming runtime scenarios; executed motion is certified with independent checks."""

import math
import random
import unittest

from tools.trajectory_sim.lookahead import ConservativeLookahead
from tools.trajectory_sim.model import AccelerationStep, Limits, PathBlock, Phase, State
from tools.trajectory_sim.phases import trajectory
from tools.trajectory_sim.runtime import HOLD, NORMAL, StreamingRuntime, run
from tools.trajectory_sim.scalar_solver import AnalyticTimeLaw
from tools.trajectory_sim.tests.test_lookahead import EXAMPLE_Z, TOL, outside
from tools.trajectory_sim.validation import validate, validate_caps

LAW = AnalyticTimeLaw()
LATENCY = 0.03
DT = 0.005


def make_runtime(limits):
    return StreamingRuntime(ConservativeLookahead(LAW), LAW, limits, LATENCY)


def cap_intervals(runtime):
    """Per-block caps with the override in force at each s (each override applies from the tail where the
    runtime installed a plan honouring it)."""
    intervals, s = [], 0.0
    changes = runtime.override_log
    for block in runtime.blocks:
        start, end = s, s + block.length
        cuts = sorted({start, end} | {position for position, _ in changes if start < position < end})
        for low, high in zip(cuts, cuts[1:]):
            factor = [f for position, f in changes if position <= low + 1e-12][-1]
            intervals.append((low, high, min(runtime.limits.speed, block.speed_cap * factor)))
        s = end
    return intervals


def low_speed_time(start, elements, epsilon):
    """Exact time spent with 0 < v < epsilon, and the number of separate low-speed intervals."""
    total, intervals, inside, state = 0.0, 0, False, start
    for element in elements:
        end = trajectory(state, (element,)).final
        if isinstance(element, Phase) and element.duration > 0:
            a, j, v0, t_end = state.a, element.jerk, state.v, element.duration
            cuts = {0.0, t_end}
            for level in (0.0, epsilon):
                if j != 0.0:
                    disc = a * a - 2 * j * (v0 - level)
                    if disc >= 0:
                        cuts |= {(-a + sign * math.sqrt(disc)) / j for sign in (1, -1)}
                elif a != 0.0:
                    cuts.add((level - v0) / a)
            points = sorted(t for t in cuts if 0.0 <= t <= t_end)
            for low, high in zip(points, points[1:]):
                middle = 0.5 * (low + high)
                now_inside = 1e-9 < v0 + a * middle + j * middle * middle / 2 < epsilon
                if now_inside:
                    total += high - low
                    intervals += not inside
                inside = now_inside
        state = end
    return total, intervals


class RuntimeTests(unittest.TestCase):
    def certify(self, runtime, jerk_limited=True):
        limits = runtime.limits
        start, elements, policies = runtime.executed()
        path = trajectory(start, elements)
        top = max([limits.speed] + [s.end.v for s in path.segments]) + 1e-6
        validate(path, Limits(top, limits.acceleration, math.inf, limits.deceleration), State(runtime.queue_end, 0, 0), TOL).require_valid()
        for element, policy in zip(elements, policies):
            if policy == NORMAL and jerk_limited:
                self.assertIsInstance(element, Phase)
                self.assertLessEqual(abs(element.jerk), limits.jerk * (1 + 1e-12))
        validate_caps(path, outside(cap_intervals(runtime), runtime.transients), TOL).require_valid()
        epsilon = 0.01
        crawl, intervals = low_speed_time(start, elements, epsilon)
        # Below epsilon a jerk-limited approach/departure/valley lasts at most 2*sqrt(2*eps/J) (eps/A + eps/D
        # without jerk); only a cruise at low speed is unbounded.
        per_interval = 2 * math.sqrt(2 * epsilon / limits.jerk) if jerk_limited else 0.0
        per_interval = max(per_interval, epsilon / limits.acceleration + epsilon / limits.deceleration)
        self.assertLessEqual(crawl, intervals * per_interval * 1.01 + 1e-9, "low-speed crawl")
        self.assertGreater(runtime.witness_checks, 0)
        return path

    def test_streaming_with_starvation(self):
        for jerk in (4000.0, math.inf):
            limits = Limits(120, 900, jerk, 500)
            runtime = make_runtime(limits)
            events = [(0.4 * k, lambda rt, k=k: rt.append(PathBlock(k, 15.0 if k % 3 else 2.0, 100.0))) for k in range(12)]
            run(runtime, events, DT, 60)
            with self.subTest(jerk=jerk):
                self.assertEqual(runtime.failures, [])
                self.certify(runtime, jerk_limited=math.isfinite(jerk))

    def test_hold_and_resume_with_latency(self):
        limits = Limits(150, 800, 6000, 400)
        runtime = make_runtime(limits)
        reference = make_runtime(limits)
        blocks = [PathBlock(k, 30.0, 140.0) for k in range(4)]
        events = [(0.0, lambda rt: [rt.append(b) for b in blocks]), (0.6, lambda rt: rt.hold()), (1.6, lambda rt: rt.resume())]
        run(runtime, events, DT, 60)
        run(reference, events[:1], DT, 60)
        self.assertEqual(runtime.failures, [])
        path = self.certify(runtime, jerk_limited=True)
        hold_steps = [e for e, p in zip(*runtime.executed()[1:]) if p == HOLD and isinstance(e, AccelerationStep)]
        self.assertTrue(hold_steps)
        # Motion committed before the hold request + latency is unchanged (queue latency accounted for).
        executed_before = trajectory(State(0, 0, 0), runtime.executed()[1])
        expected = trajectory(State(0, 0, 0), reference.executed()[1])
        sample = lambda p, t: next(s.end for s in p.segments if sum(x.phase.duration for x in p.segments[:p.segments.index(s) + 1]) >= t)
        for t in (0.2, 0.5, 0.6 + LATENCY - DT):
            self.assertAlmostEqual(sample(executed_before, t).s, sample(expected, t).s, delta=0.05)
        self.assertAlmostEqual(path.final.s, 120.0, places=6)

    def test_requests_take_effect_at_committed_tail(self):
        """Queue latency: hold and override requested at time T change nothing committed before T + latency;
        the new motion starts exactly at the committed tail."""
        limits = Limits(150, 800, 6000, 400)
        blocks = [PathBlock(k, 30.0, 140.0) for k in range(4)]
        load = (0.0, lambda rt: [rt.append(b) for b in blocks])
        reference = run(make_runtime(limits), [load], DT, 60)
        for request in ("hold", "override"):
            marks = []

            def act(rt, request=request):
                marks.append((len(rt.committed), rt.tail, rt.tail_time, rt.time))
                rt.hold() if request == "hold" else rt.set_override(0.4)

            runtime = run(make_runtime(limits), [load, (0.5, act)] + ([(1.5, lambda rt: rt.resume())] if request == "hold" else []), DT, 60)
            count, tail, tail_time, now = marks[0]
            with self.subTest(request=request):
                self.assertEqual(runtime.failures, [])
                self.assertGreaterEqual(tail_time, now + LATENCY - 1e-12)
                self.assertEqual(runtime.committed[:count], reference.committed[:count])
                self.assertNotEqual(runtime.committed[count:count + 1], reference.committed[count:count + 1])
                if request == "hold":
                    self.assertEqual(runtime.policies[count], HOLD)
                    self.assertEqual(set(runtime.policies[:count]), {NORMAL})
                else:
                    self.assertEqual(runtime.override_log[-1], (tail.s, 0.4))
                self.certify(runtime)

    def test_override_changes(self):
        limits = Limits(150, 800, 6000, 300)
        runtime = make_runtime(limits)
        events = [(0.0, lambda rt: [rt.append(PathBlock(k, 40.0, 150.0)) for k in range(4)]),
                  (0.5, lambda rt: rt.set_override(0.4)), (1.5, lambda rt: rt.set_override(1.2))]
        run(runtime, events, DT, 60)
        self.assertEqual(runtime.failures, [])
        self.assertTrue(runtime.transients)
        self.certify(runtime)

    def test_collinear_short_z_soak(self):
        """Regression: repeated pairs of collinear Z moves, second short, ending at rest (example machine Z)."""
        for jerk in (423.33, math.inf):
            limits = Limits(EXAMPLE_Z["speed"], EXAMPLE_Z["acceleration"], jerk, EXAMPLE_Z["deceleration"])
            runtime = make_runtime(limits)
            events = []
            for cycle in range(40):
                events.append((0.0, lambda rt, c=cycle: rt.append(PathBlock(2 * c, 10.0, EXAMPLE_Z["speed"]))))
                events.append((0.0, lambda rt, c=cycle: rt.append(PathBlock(2 * c + 1, 1.2, EXAMPLE_Z["speed"], stop_after=True))))
            run(runtime, events, DT, 600)
            with self.subTest(jerk=jerk):
                self.assertEqual(runtime.failures, [])
                path = self.certify(runtime, jerk_limited=math.isfinite(jerk))
                self.assertAlmostEqual(path.final.s, 40 * 11.2, places=6)

    def test_random_streaming_scenarios(self):
        rng = random.Random(6006)
        replans = failures = 0
        for case in range(25):
            jerk = math.inf if rng.random() < 0.2 else rng.uniform(500, 20000)
            limits = Limits(rng.uniform(40, 200), rng.uniform(100, 2000), jerk, rng.uniform(50, 2000))
            runtime = make_runtime(limits)
            events, t = [], 0.0
            for k in range(rng.randint(3, 15)):
                t += rng.uniform(0.0, 0.4)
                block = PathBlock(k, 10 ** rng.uniform(-1, 1.5), rng.uniform(5, limits.speed * 1.2), rng.random() < 0.15)
                events.append((t, lambda rt, b=block: rt.append(b)))
            if rng.random() < 0.5:
                events.append((rng.uniform(0, t + 0.5), lambda rt, f=rng.uniform(0.3, 1.5): rt.set_override(f)))
            if rng.random() < 0.3:
                hold_at = rng.uniform(0, t + 0.5)
                events += [(hold_at, lambda rt: rt.hold()), (hold_at + rng.uniform(0.1, 1.0), lambda rt: rt.resume())]
            run(runtime, events, DT, 300)
            replans += runtime.plans
            failures += len(runtime.failures)
            with self.subTest(case=case):
                self.certify(runtime, jerk_limited=math.isfinite(jerk))
        print(f"\nrandom streaming: {replans} replans, {failures} fell back to the previous stop witness")
        self.assertLessEqual(failures, 0.02 * replans)


if __name__ == "__main__":
    unittest.main()
