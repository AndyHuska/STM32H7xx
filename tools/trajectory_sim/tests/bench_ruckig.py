"""Speed benchmark of the stack against Ruckig (time-optimal oracle), not just solvability.

Run: .\\.venv\\Scripts\\python.exe -m tools.trajectory_sim.tests.bench_ruckig

Where Ruckig is a valid reference:
  1. Scalar time law (L2): 1-DoF state-to-state moves with signed boundary accelerations.
  2. Single straight moves through the whole stack (front end -> geometry -> projection -> lookahead):
     Ruckig multi-DoF with phase synchronization moves all axes proportionally, i.e. along the same
     straight line, time-optimally under per-axis limits. Asymmetric deceleration is mapped per axis by
     travel direction (a monotone axis decelerates against its direction).
  3. Programs: Ruckig cannot follow a polyline with corners, but the sum of its per-segment rest-to-rest
     times is the time-optimal exact-stop (G61) baseline. Our G61 should equal it; G64 should beat it.
Path speed caps from the feed become per-axis velocity limits F |u_i| (phase sync keeps the path speed <= F).
"""

from __future__ import annotations

import math
import random
import statistics

import ruckig

from tools.trajectory_sim.model import Limits, State
from tools.trajectory_sim.scalar_solver import AnalyticTimeLaw
from tools.trajectory_sim.tests.machines import spectrum
from tools.trajectory_sim.tests.test_pipeline import plan_time, zigzag
from tools.trajectory_sim.tests.test_scalar_time_law import random_boundary, random_limits, ruckig_optimum

LAW = AnalyticTimeLaw()
PROGRAM_P = (0.05, 0.2, 1.0)


def ruckig_line(axes: list[Limits], start, end, feed: float) -> float | None:
    """Ruckig phase-synchronized rest-to-rest time along the straight line start -> end; None if Ruckig
    could not keep the axes proportional (it then falls back to time synchronization, not a line)."""
    delta = [b - a for a, b in zip(start, end)]
    length = math.sqrt(sum(d * d for d in delta))
    unit = [d / length for d in delta]
    otg, inp, out = ruckig.Ruckig(3), ruckig.InputParameter(3), ruckig.Trajectory(3)
    inp.current_position, inp.target_position = list(start), list(end)
    inp.current_velocity = inp.current_acceleration = [0.0] * 3
    inp.target_velocity = inp.target_acceleration = [0.0] * 3
    inp.max_velocity = [min(a.speed, feed * abs(u)) if u else a.speed for a, u in zip(axes, unit)]
    # Speeding up acts along the travel direction, slowing down against it.
    inp.max_acceleration = [a.deceleration if u < 0 else a.acceleration for a, u in zip(axes, unit)]
    inp.min_acceleration = [-(a.acceleration if u < 0 else a.deceleration) for a, u in zip(axes, unit)]
    inp.max_jerk = [a.jerk for a in axes]
    inp.synchronization = ruckig.Synchronization.Phase
    if otg.calculate(inp, out) not in (ruckig.Result.Working, ruckig.Result.Finished):
        return None
    for k in range(1, 50):
        position = out.at_time(out.duration * k / 50)[0]
        along = sum((p - s) * u for p, s, u in zip(position, start, unit))
        off = math.sqrt(sum((p - s - along * u) ** 2 for p, s, u in zip(position, start, unit)))
        if off > 1e-9 * max(1.0, length):
            return None
    return out.duration


def ruckig_path(axes: list[Limits], start, end, feed: float) -> float:
    """Ruckig 1-DoF rest-to-rest time along the line with the exact straight-line path limits
    (axis i moves u_i s, so V = min V_i/|u_i|, A = min A_i/|u_i|, D = min D_i/|u_i|, J likewise)."""
    length = math.dist(start, end)
    unit = [(b - a) / length for a, b in zip(start, end)]
    moving = [(abs(u), a) for u, a in zip(unit, axes) if u]
    otg, inp, out = ruckig.Ruckig(1), ruckig.InputParameter(1), ruckig.Trajectory(1)
    inp.current_position, inp.target_position = [0.0], [length]
    inp.current_velocity = inp.current_acceleration = inp.target_velocity = inp.target_acceleration = [0.0]
    inp.max_velocity = [min([feed] + [a.speed / u for u, a in moving])]
    inp.max_acceleration = [min(a.acceleration / u for u, a in moving)]
    inp.min_acceleration = [-min(a.deceleration / u for u, a in moving)]
    inp.max_jerk = [min([math.inf] + [a.jerk / u for u, a in moving if a.jerk_limited])]
    otg.calculate(inp, out)
    return out.duration


def reference_line(axes: list[Limits], start, end, feed: float) -> tuple[float, str]:
    """Phase-synchronized multi-DoF time when Ruckig can keep the axes proportional, else the 1-DoF path
    oracle; where both exist they must agree (cross-check of the limit mapping)."""
    path = ruckig_path(axes, start, end, feed)
    phase = ruckig_line(axes, start, end, feed)
    if phase is None:
        return path, "1-DoF"
    assert abs(phase - path) <= 1e-9 * max(1.0, path), (phase, path)
    return phase, "phase"


def scalar(cases: int = 600, seed: int = 7117) -> dict:
    """L2 distance transitions vs Ruckig's 1-DoF optimum (only cases where Ruckig stays in 0 <= v <= V)."""
    rng = random.Random(seed)
    ratios, unsolved = [], 0
    for _ in range(cases):
        limits = random_limits(rng)
        v0, a0 = random_boundary(rng, limits)
        v1, a1 = random_boundary(rng, limits)
        start, goal = State(0.0, v0, a0), State(10 ** rng.uniform(-3, 2.7), v1, a1)
        result = LAW.distance_transition(start, goal, limits)
        optimum = ruckig_optimum(start, goal, limits)
        if optimum is None or optimum <= 0.0:
            continue
        if result.ok:
            ratios.append(result.duration / optimum)
        else:
            unsolved += 1
    return {"compared": len(ratios), "unsolved": unsolved, "ratios": ratios}


def straight_lines(axes: list[Limits], cases: int = 60, seed: int = 4701) -> dict:
    rng = random.Random(seed)
    ratios, speeds, kinds = [], [], {"phase": 0, "1-DoF": 0}
    for _ in range(cases):
        direction = [rng.gauss(0, 1) for _ in range(3)]
        if rng.random() < 0.3:
            direction[rng.randrange(3)] = 0.0  # planar / axis-aligned moves are common in practice
        norm = math.sqrt(sum(d * d for d in direction)) or 1.0
        length = 10 ** rng.uniform(-1, 2)
        end = tuple(length * d / norm for d in direction)
        feed = rng.choice([30.0, 100.0, 1e4])
        reference, kind = reference_line(axes, (0.0, 0.0, 0.0), end, feed)
        kinds[kind] += 1
        ours = plan_time(axes, [end], None, feed)
        ratios.append(ours / reference)
        speeds.append((length / ours, length / reference))
    return {"ratios": ratios, "speeds": speeds, "kinds": kinds}


def programs(axes: list[Limits], seed: int = 4702) -> list[dict]:
    rng = random.Random(seed)
    sets = {"zigzag": zigzag()}
    for k in range(2):
        point, path = (0.0, 0.0, 0.0), []
        for _ in range(8):
            point = tuple(p + rng.uniform(-1, 1) * 10 ** rng.uniform(-0.5, 1.3) for p in point)
            path.append(point)
        sets[f"random {k}"] = path
    rows = []
    for name, points in sets.items():
        segments = [reference_line(axes, a, b, 100.0)[0] for a, b in zip([(0.0, 0.0, 0.0)] + points[:-1], points)]
        baseline = sum(segments)
        g61 = plan_time(axes, points, None)
        length = sum(math.dist(a, b) for a, b in zip([(0.0, 0.0, 0.0)] + points[:-1], points))
        rows.append({"program": name, "length": length, "ruckig_g61": baseline, "g61": g61,
                     "g64": {p: plan_time(axes, points, p) for p in PROGRAM_P}})
    return rows


def _stats(values: list[float]) -> str:
    if not values:
        return "n/a"
    ordered = sorted(values)
    p95 = ordered[min(len(ordered) - 1, int(0.95 * len(ordered)))]
    return f"median {statistics.median(ordered):.6f}  p95 {p95:.6f}  max {ordered[-1]:.6f}  min {ordered[0]:.6f}"


def main() -> None:
    result = scalar()
    print(f"L2 scalar vs Ruckig 1-DoF ({result['compared']} compared, {result['unsolved']} unsolved by us):")
    print(f"   time ratio ours/Ruckig  {_stats(result['ratios'])}")
    for label, axes in spectrum().items():
        lines = straight_lines(axes)
        mean_ours = statistics.mean(o for o, _ in lines["speeds"])
        mean_ref = statistics.mean(r for _, r in lines["speeds"])
        print(f"\n{label}: straight lines through the stack ({len(lines['ratios'])} compared; Ruckig reference "
              f"{lines['kinds']['phase']} phase-synchronized multi-DoF, {lines['kinds']['1-DoF']} 1-DoF path)")
        print(f"   time ratio ours/Ruckig  {_stats(lines['ratios'])}")
        print(f"   mean path speed         ours {mean_ours:.2f} mm/s  Ruckig {mean_ref:.2f} mm/s")
        for row in programs(axes):
            base = row["ruckig_g61"]
            g64 = "  ".join(f"P{p}: {t:.3f} s ({100 * (t / base - 1):+.1f}%)" for p, t in row["g64"].items())
            print(f"   {row['program']:9s} {row['length']:6.1f} mm  Ruckig G61 {base:.3f} s  ours G61 {row['g61']:.3f} s"
                  f" ({100 * (row['g61'] / base - 1):+.2f}%)  G64 {g64}")


if __name__ == "__main__":
    main()
