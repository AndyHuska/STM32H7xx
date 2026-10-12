"""Host port of the circle-law arc ramps added to grbl/stepper.c st_prep_buffer (FW-2), normal operation:
profile from profile_distance / profile_speed_sqr, then the segment loop (Ramp_Accel / Ramp_Cruise /
Ramp_Decel) with 10 ms segments. Same formulas and branch order as the C code, in float64 and mm/s units
(the firmware uses mm/min; the logic is unit-independent).

Checks: the executed segment-end speeds stay on or under both law curves (from the entry and to the exit), the
block ends exactly at the exit speed and length (also when braking to rest: an earlier version stalled there,
ds -> 0 at v -> 0, which this test caught), and the time matches firmware_model.block_time.
Error model: segment-end speeds are taken from the law itself, so only the float rounding of asin/sin
(<1e-12 relative here) separates them from the curves."""

import math
import unittest

from tools.trajectory_sim.tests.firmware_model import Block, block_time

DT_SEGMENT = 0.01  # s (ACCELERATION_TICKS_PER_SECOND = 100)


class ArcLawPrep:
    """One planner block through the ported segment generator; `radius` None = constant-acceleration block."""

    def __init__(self, accel, radius, length, entry, exit, nominal):
        self.law = radius is not None
        self.a, self.r, self.c2 = accel, radius, accel * radius if self.law else 0.0
        self.length, self.entry, self.exit, self.nominal = length, entry, exit, nominal

    def distance(self, v2):
        if not self.law:
            return 0.5 * v2 / self.a
        return 0.5 * self.r * math.asin(min(1.0, v2 / self.c2))

    def speed_sqr(self, d):
        if not self.law:
            return 2.0 * self.a * max(0.0, d)
        return self.c2 * math.sin(min(0.5 * math.pi, 2.0 * max(0.0, d) / self.r))

    def law_step(self, speed, dt, accelerate):
        if not self.law:
            dv = self.a * dt
            return speed + dv, dt * (speed + 0.5 * dv)
        w = speed * speed / self.c2
        dv = self.a * math.sqrt(max(0.0, 1.0 - w * w)) * dt
        ds = dt * max(0.0, speed + 0.5 * dv if accelerate else speed - 0.5 * dv)
        return math.sqrt(self.speed_sqr(self.distance(speed * speed) + (ds if accelerate else -ds))), ds

    def run(self):
        """Segments as (duration, distance, end speed)."""
        L, entry2, exit2, n2 = self.length, self.entry ** 2, self.exit ** 2, self.nominal ** 2
        de, dx, dn = self.distance(entry2), self.distance(exit2), self.distance(n2)
        ramp, accelerate_until, decelerate_after = "accel", L, 0.0
        intersect = 0.5 * (L + de - dx)
        maximum = self.nominal
        if intersect > 0.0:
            if intersect < L:
                decelerate_after = dn - dx
                if decelerate_after < intersect:
                    if entry2 == n2:
                        ramp = "cruise"
                    else:
                        accelerate_until -= dn - de
                else:
                    accelerate_until = decelerate_after = intersect
                    maximum = math.sqrt(self.speed_sqr(intersect + dx))
            else:
                ramp = "decel"
        else:
            accelerate_until, maximum = 0.0, self.exit
        speed, mm_remaining, segments = self.entry, L, []
        while mm_remaining > 0.0:
            dt, time_var, start_mm = 0.0, DT_SEGMENT, mm_remaining
            while True:
                if ramp == "accel":
                    new_speed, ds = self.law_step(speed, time_var, True)
                    mm_remaining -= ds
                    if mm_remaining < accelerate_until:
                        mm_remaining = accelerate_until
                        time_var = 2.0 * (start_mm - mm_remaining) / (speed + maximum)
                        ramp = "decel" if mm_remaining == decelerate_after else "cruise"
                        speed = maximum
                    else:
                        speed = new_speed
                elif ramp == "cruise":
                    mm_var = mm_remaining - maximum * time_var
                    if mm_var < decelerate_after:
                        time_var = (mm_remaining - decelerate_after) / maximum
                        mm_remaining = decelerate_after
                        ramp = "decel"
                    else:
                        mm_remaining = mm_var
                else:
                    if self.law:
                        w = speed * speed / self.c2
                        speed_var = self.a * math.sqrt(max(0.0, 1.0 - w * w)) * time_var
                    else:
                        speed_var = self.a * time_var
                    mm_var = mm_remaining - time_var * (speed - 0.5 * speed_var)
                    if speed > speed_var and mm_var > 0.0:
                        mm_remaining = mm_var
                        speed = min(speed, math.sqrt(self.speed_sqr(dx + mm_var))) if self.law else speed - speed_var
                    else:
                        time_var = 2.0 * mm_remaining / (speed + self.exit)
                        mm_remaining, speed = 0.0, self.exit
                dt += time_var
                if dt < DT_SEGMENT and mm_remaining > 0.0:
                    time_var = DT_SEGMENT - dt
                else:
                    break
            segments.append((dt, start_mm - mm_remaining, speed))
            if len(segments) > 100000:
                raise RuntimeError("segment loop does not terminate")
        return segments


class StepperArcLawTests(unittest.TestCase):
    CASES = (  # (A mm/s^2, R mm, L mm, entry, exit, nominal mm/s): cruise, triangle, accel-only, decel-only, from rest
        (1000.0, 2.4, 1.9, 46.0, 46.0, 46.0),
        (1000.0, 10.0, 0.4, 30.0, 35.0, 90.0),
        (800.0, 25.0, 12.0, 10.0, 120.0, 134.0),
        # Decel-only from 120 to rest: feasible only if L = (R/2) asin(v^2 / (A R)) exactly (planner exit = reach).
        (800.0, 25.0, 12.5 * math.asin(120.0 ** 2 / (800.0 * 25.0)), 120.0, 0.0, 134.0),
        (1200.0, 5.0, 3.0, 0.0, 0.0, 70.0),
    )

    def test_segments_follow_the_law_and_land_on_the_exit(self):
        for accel, radius, length, entry, exit, nominal in self.CASES:
            prep = ArcLawPrep(accel, radius, length, entry, exit, nominal)
            segments = prep.run()
            with self.subTest(case=(accel, radius, length, entry, exit, nominal)):
                self.assertAlmostEqual(sum(s[1] for s in segments), length, delta=1e-9 * length)
                self.assertAlmostEqual(segments[-1][2], exit, delta=1e-9 * max(1.0, nominal))
                travelled = 0.0
                for _, distance, speed in segments:
                    travelled += distance
                    rise = prep.speed_sqr(prep.distance(entry * entry) + travelled)
                    fall = prep.speed_sqr(prep.distance(exit * exit) + length - travelled)
                    self.assertLessEqual(speed * speed, min(rise, fall, nominal * nominal) * (1 + 1e-9) + 1e-9)

    def test_profile_time_matches_the_model(self):
        """Executed time vs the model's exact block time: the segments differ only by the trapezoid rule at ramp
        junctions inside a segment and the per-segment distance estimate (O(dt^2) per segment)."""
        for accel, radius, length, entry, exit, nominal in self.CASES:
            block = Block(0, "arc", (0, 0, 0), (0, 0, 0), length, nominal, 0.0, 0.0, (1, 0, 0), (1, 0, 0),
                          (0, 0, 0), radius, length / radius, accel)
            executed = sum(s[0] for s in ArcLawPrep(accel, radius, length, entry, exit, nominal).run())
            with self.subTest(case=(accel, radius, length, entry, exit, nominal)):
                self.assertAlmostEqual(executed, block_time(block, entry, exit), delta=0.02 * block_time(block, entry, exit))


if __name__ == "__main__":
    unittest.main()
