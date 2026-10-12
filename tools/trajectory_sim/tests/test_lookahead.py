"""Contract suite for contracts.Lookahead (L5) on straight paths, plus shared certification helpers."""

import math
import random
import unittest

from tools.trajectory_sim.contracts import Lookahead, SolveStatus
from tools.trajectory_sim.lookahead import ConservativeLookahead, Region, ramp_distance
from tools.trajectory_sim.model import Limits, PathBlock, State
from tools.trajectory_sim.phases import split_at, trajectory
from tools.trajectory_sim.scalar_solver import AnalyticTimeLaw
from tools.trajectory_sim.validation import Tolerance, validate, validate_caps

TOL = Tolerance(position=1e-6, speed=1e-6, acceleration=1e-6, jerk=1e-9, relative=1e-9)
EXAMPLE_Z = dict(speed=99.999833333333328, acceleration=999.99, deceleration=1000.0001666666667)  # fixtures/machine_example_xyz.json


def block_intervals(start_s, blocks, limits, override=1.0):
    intervals, s = [], start_s
    for block in blocks:
        intervals.append((s, s + block.length, min(limits.speed, block.speed_cap * override)))
        s += block.length
    return intervals


def outside(intervals, windows):
    """Remove override-transient windows (s_from, s_to) from cap intervals."""
    for low, high in windows:
        kept = []
        for s_from, s_to, cap in intervals:
            if s_to <= low or s_from >= high:
                kept.append((s_from, s_to, cap))
                continue
            if s_from < low:
                kept.append((s_from, low, cap))
            if s_to > high:
                kept.append((high, s_to, cap))
        intervals = kept
    return intervals


def certify_plan(test, start, blocks, limits, plan, override=1.0):
    path = trajectory(start, plan.elements)
    end = State(start.s + sum(block.length for block in blocks), 0.0, 0.0)
    speed = limits.speed if plan.transient_end is None else max(limits.speed, start.v + max(0.0, start.a) ** 2 / (2 * limits.jerk)) + 1e-6
    validate(path, Limits(speed, limits.acceleration, limits.jerk, limits.deceleration), end, TOL).require_valid()
    windows = [] if plan.transient_end is None else [(start.s, plan.transient_end)]
    validate_caps(path, outside(block_intervals(start.s, blocks, limits, override), windows), TOL).require_valid()


def random_limits(rng):
    acceleration = rng.uniform(10, 2000)
    deceleration = None if rng.random() < 0.5 else acceleration * rng.uniform(0.1, 3.0)
    jerk = math.inf if rng.random() < 0.2 else rng.uniform(100, 50000)
    return Limits(rng.uniform(20, 200), acceleration, jerk, deceleration)


def random_blocks(rng, limits, count=None):
    count = count or rng.randint(1, 8)
    return [PathBlock(index, 10 ** rng.uniform(-2, 1.7), rng.uniform(1, limits.speed * 1.2), rng.random() < 0.15)
            for index in range(count)]


def remaining_blocks(blocks, s_now):
    remaining, s = [], 0.0
    for block in blocks:
        if s + block.length > s_now + 1e-9:
            remaining.append(PathBlock(block.source_id, s + block.length - max(s, s_now), block.speed_cap, block.stop_after))
        s += block.length
    return remaining


class LookaheadContract:
    """Mixin; subclasses set `planner`."""

    planner: Lookahead

    def test_equal_cap_blocks_match_single_move(self):
        for jerk in (5000.0, math.inf):
            limits = Limits(100, 1000, jerk, 400)
            whole = self.planner.plan(State(0, 0, 0), [PathBlock(0, 16, 80)], limits)
            split = self.planner.plan(State(0, 0, 0), [PathBlock(i, length, 80) for i, length in enumerate((5, 3, 7, 1))], limits)
            with self.subTest(jerk=jerk):
                self.assertTrue(whole.ok and split.ok)
                self.assertEqual(whole.elements, split.elements)

    def test_random_plans_from_rest_certified(self):
        rng = random.Random(3003)
        for case in range(400):
            limits = random_limits(rng)
            blocks = random_blocks(rng, limits)
            start = State(rng.uniform(-100, 100), 0.0, 0.0)
            plan = self.planner.plan(start, blocks, limits)
            with self.subTest(case=case):
                self.assertEqual(plan.status, SolveStatus.OK, plan.detail)
                certify_plan(self, start, blocks, limits, plan)

    def test_replans_from_moving_states(self):
        """Replan from states inside earlier plans (as the runtime does); the old suffix is a valid witness."""
        rng = random.Random(4004)
        attempts = failures = 0
        for case in range(300):
            limits = random_limits(rng)
            blocks = random_blocks(rng, limits, rng.randint(2, 8))
            first = self.planner.plan(State(0, 0, 0), blocks, limits)
            self.assertTrue(first.ok, first.detail)
            _, state, _ = split_at(State(0, 0, 0), first.elements, rng.uniform(0, first.duration))
            remaining = remaining_blocks(blocks, state.s)
            if not remaining:
                continue
            attempts += 1
            plan = self.planner.plan(state, remaining, limits)
            with self.subTest(case=case):
                if plan.ok:
                    certify_plan(self, state, remaining, limits, plan)
                else:
                    failures += 1
                    self.assertTrue(plan.detail)
        print(f"\nreplan from moving states: {attempts - failures}/{attempts} OK")
        self.assertLessEqual(failures, 0.02 * attempts)

    def test_replans_after_queue_extension(self):
        """Blocks (and sometimes an override) arrive while moving; the old suffix still stops inside the longer
        queue, so a plan exists, though new cap boundaries may fall where the old profile has a != 0."""
        rng = random.Random(1212)
        attempts = failures = 0
        for case in range(300):
            limits = random_limits(rng)
            blocks = random_blocks(rng, limits, rng.randint(1, 6))
            first = self.planner.plan(State(0, 0, 0), blocks, limits)
            self.assertTrue(first.ok, first.detail)
            _, state, _ = split_at(State(0, 0, 0), first.elements, rng.uniform(0, first.duration))
            added = random_blocks(rng, limits, rng.randint(1, 4))
            extended = remaining_blocks(blocks, state.s) + [PathBlock(len(blocks) + i, b.length, b.speed_cap, b.stop_after)
                                                            for i, b in enumerate(added)]
            override = rng.uniform(0.3, 1.5) if rng.random() < 0.3 else 1.0
            attempts += 1
            plan = self.planner.plan(state, extended, limits, override)
            with self.subTest(case=case):
                if plan.ok:
                    certify_plan(self, state, extended, limits, plan, override)
                else:
                    failures += 1
                    self.assertTrue(plan.detail)
        print(f"\nreplan after queue extension: {attempts - failures}/{attempts} OK")
        self.assertLessEqual(failures, 0.02 * attempts)

    def test_override_reduction_uses_transient(self):
        limits = Limits(150, 800, 6000, 300)
        blocks = [PathBlock(0, 60, 150), PathBlock(1, 40, 150)]
        cruise = self.planner.plan(State(0, 0, 0), blocks, limits)
        _, state, _ = split_at(State(0, 0, 0), cruise.elements, 0.45)
        remaining = [PathBlock(0, 60 - state.s, 150), PathBlock(1, 40, 150)]
        plan = self.planner.plan(state, remaining, limits, override=0.3)
        self.assertTrue(plan.ok, plan.detail)
        self.assertIsNotNone(plan.transient_end)
        certify_plan(self, state, remaining, limits, plan, override=0.3)

    def test_collinear_short_second_z_move(self):
        """Regression input: two collinear Z moves, the second too short to reach full speed/acceleration."""
        for jerk in (423.33, math.inf):
            limits = Limits(EXAMPLE_Z["speed"], EXAMPLE_Z["acceleration"], jerk, EXAMPLE_Z["deceleration"])
            for second_cap in (EXAMPLE_Z["speed"], 50.0):
                blocks = [PathBlock(0, 10.0, EXAMPLE_Z["speed"]), PathBlock(1, 1.2, second_cap)]
                plan = self.planner.plan(State(0, 0, 0), blocks, limits)
                with self.subTest(jerk=jerk, second_cap=second_cap):
                    self.assertTrue(plan.ok, plan.detail)
                    certify_plan(self, State(0, 0, 0), blocks, limits, plan)

    def test_invalid_inputs_reported(self):
        limits = Limits(100, 1000, 5000)
        self.assertEqual(self.planner.plan(State(0, 0, 0), [], limits).status, SolveStatus.INVALID_INPUT)
        self.assertEqual(self.planner.plan(State(0, 0, 0), [PathBlock(0, 1, 10)], limits, override=0.0).status, SolveStatus.INVALID_INPUT)

    def test_streaming_regressions_seed_6006_case_3(self):
        """Committed states from test_runtime seed 6006 case 3, after blocks 5 and 6 arrived mid-deceleration:
        no a = 0 exit from block 4 exists, and an override of 0.469 lands where the cap cannot be reached
        before the queue end."""
        limits = Limits(80.62033250692795, 527.9051186196939, 12585.677684058646, 594.2447359502946)
        tail = [(0.8069387897629987, 58.60492159973155), (0.21034083292562045, 43.886324159659814)]
        block_4_end = 25.8320123088666
        cases = (
            (State(24.985431906974178, 24.858239093475035, -12.028651777948614), 1.0),
            (State(25.67572253847911, 20.534860620116703, -96.96479905517293), 1.0),
            (State(25.740251914138874, 24.300854640801386, -177.6086734979212), 0.4694894008573568),
        )
        for index, (state, override) in enumerate(cases):
            blocks = [PathBlock(4, block_4_end - state.s, 28.326116726411303)] + [PathBlock(5 + i, *b) for i, b in enumerate(tail)]
            plan = self.planner.plan(state, blocks, limits, override)
            with self.subTest(index=index):
                self.assertEqual(plan.status, SolveStatus.OK, plan.detail)
                certify_plan(self, state, blocks, limits, plan, override)


class ConservativeLookaheadTests(LookaheadContract, unittest.TestCase):
    planner = ConservativeLookahead(AnalyticTimeLaw())

    def test_protocol(self):
        self.assertIsInstance(self.planner, Lookahead)

    def test_ramp_distance_matches_time_law(self):
        rng, law = random.Random(5005), AnalyticTimeLaw()
        for case in range(1000):
            limits = random_limits(rng)
            u, w = rng.uniform(0, limits.speed), rng.uniform(0, limits.speed)
            result = law.velocity_transition(State(0, u, 0), w, 0.0, limits)
            with self.subTest(case=case):
                self.assertTrue(result.ok, result.detail)
                self.assertAlmostEqual(ramp_distance(u, w, limits), trajectory(State(0, u, 0), result.phases).final.s,
                                       delta=1e-9 * max(1.0, ramp_distance(u, w, limits)))

    def test_first_exit_is_largest_sampled_feasible_speed(self):
        """Brute-force oracle for the region-0 exit search: need(w) is not unimodal, so no sampled w above
        the result may fit, and None only when no sample fits."""
        rng, law = random.Random(1313), AnalyticTimeLaw()
        checked = 0
        for case in range(60):
            limits = random_limits(rng)
            plan = self.planner.plan(State(0, 0, 0), random_blocks(rng, limits), limits)
            _, state, _ = split_at(State(0, 0, 0), plan.elements, rng.uniform(0, plan.duration))
            region = Region(state.s, 10 ** rng.uniform(-2, 1), limits, False)
            bound = rng.uniform(0, limits.speed)
            found = self.planner._first_exit(state, region, bound)
            needs = [(w, law.minimum_distance(state, w, 0.0, limits)) for w in (bound * k / 80 for k in range(81))]
            feasible = [w for w, need in needs if need is not None and need <= region.length - 1e-9]
            with self.subTest(case=case):
                if found is None:
                    self.assertEqual(feasible, [])
                    continue
                checked += 1
                self.assertLessEqual(found, bound)
                self.assertLessEqual(law.minimum_distance(state, found, 0.0, limits), region.length + 1e-9)
                self.assertGreaterEqual(found, max(feasible, default=0.0) - 1e-9 * max(1.0, bound))
        self.assertGreater(checked, 30)


if __name__ == "__main__":
    unittest.main()
