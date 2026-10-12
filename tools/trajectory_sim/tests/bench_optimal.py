"""G64 corner benchmark against the time-optimal reference (tests/optimal_reference.py, CasADi/IPOPT).

Run: .\\.venv\\Scripts\\python.exe -m tools.trajectory_sim.tests.bench_optimal [count]

Simple corners (two 20 mm legs from rest to rest, F100): the axis-aligned 90 deg corner and each corner of
tests/test_pipeline.zigzag (30-150 deg). Times per machine and P, as % relative to the G61 optimum (sum of
Ruckig rest-to-rest legs, exact):
  plan    our pipeline (front end decides blend or stop)
  blend   our planner on the forced blend geometry (L4 projection + L5 schedule)
  env     optimum under the planner's own L4 path limits on that geometry (no L5 rules)
  opt     optimum of that geometry under the per-axis limits (physics of our clothoid blend)
  best    min of opt over P' <= P and the G61 optimum (our blend family's achievable time, monotone in P)
plan - blend = front-end decision, blend - env = L5 scheduling, env - opt = L4 projection: conservatism.
opt / best vs G61 = physics of the geometry. Reference error at count 200: ~1e-4 relative (DESIGN.md 3D).
"""

from __future__ import annotations

import math
import sys
from concurrent.futures import ProcessPoolExecutor

from tools.trajectory_sim.constraints import blocks
from tools.trajectory_sim.geometry import ClothoidPiece, GeometryStream
from tools.trajectory_sim.lookahead import ConservativeLookahead
from tools.trajectory_sim.model import LineMove, State
from tools.trajectory_sim.scalar_solver import AnalyticTimeLaw
from tools.trajectory_sim.tests.bench_ruckig import reference_line
from tools.trajectory_sim.tests.machines import spectrum
from tools.trajectory_sim.tests.optimal_reference import optimal_time
from tools.trajectory_sim.tests.test_pipeline import PLANNER_LIMITS, Pipeline, zigzag

LAW = AnalyticTimeLaw()
FEED = 100.0
P_LADDER = (0.01, 0.05, 0.2, 1.0, 3.0)


def corners() -> dict[str, list[tuple[float, float, float]]]:
    """name -> [start, vertex, end]."""
    result = {"90 axis": [(0.0, 0.0, 0.0), (20.0, 0.0, 0.0), (20.0, 20.0, 0.0)]}
    points = [(0.0, 0.0, 0.0)] + zigzag()
    for k, angle in enumerate((30, 60, 90, 120, 150), start=1):
        result[f"{angle} zig"] = points[k - 1:k + 2]
    return result


def planned(axes, points, tolerance) -> tuple[float, str]:
    pipeline = Pipeline(axes, points[0])
    for k, point in enumerate(points[1:]):
        pipeline.feed(LineMove(k, point, FEED, tolerance))
    runtime = pipeline.runtime
    assert not runtime.failures
    return sum(e.duration for e in runtime.suffix), pipeline.stream.corners[0].kind


def forced_blend(points, tolerance):
    stream = GeometryStream(points[0])
    for k, point in enumerate(points[1:]):
        revision = stream.propose(LineMove(k, point, FEED, tolerance), 0.0, True)
        assert revision.ok, revision.detail
        stream.accept(revision)
    assert stream.corners[0].kind == "blend"
    return stream.pieces()


def corner_row(axes, points, tolerance, count) -> dict:
    plan, kind = planned(axes, points, tolerance)
    entries = forced_blend(points, tolerance)
    pieces, stops = [e[1] for e in entries], [e[2] for e in entries]
    path_blocks = blocks(pieces, stops, axes)
    schedule = ConservativeLookahead(LAW).plan(State(0.0, 0.0, 0.0), path_blocks, PLANNER_LIMITS)
    assert schedule.ok, schedule.detail
    env = optimal_time(entries, axes, count=count, envelope=[b.limits for b in path_blocks])[0]
    opt, s, v = optimal_time(entries, axes, count=count)
    blend_s = [e[0] for e in entries if isinstance(e[1], ClothoidPiece)]
    apex = min(vk for sk, vk in zip(s, v) if blend_s[0] <= sk <= blend_s[-1])
    return {"plan": plan, "kind": kind, "blend": schedule.duration, "env": env, "opt": opt, "apex_opt": apex}


def machine_rows(args) -> tuple[str, list]:
    label, count = args
    axes = spectrum()[label]
    rows = []
    for name, points in corners().items():
        g61 = sum(reference_line(axes, a, b, FEED)[0] for a, b in zip(points, points[1:]))
        cells = {p: corner_row(axes, points, p, count) for p in P_LADDER}
        best = g61
        for p in P_LADDER:
            best = min(best, cells[p]["opt"])
            cells[p]["best"] = best
        rows.append((name, g61, cells))
    return label, rows


def main() -> None:
    count = int(sys.argv[1]) if len(sys.argv) > 1 else 200
    jobs = [(label, count) for label in spectrum()]
    with ProcessPoolExecutor(max_workers=len(jobs)) as pool:
        results = list(pool.map(machine_rows, jobs))
    keys = ("plan", "blend", "env", "opt", "best")
    for label, rows in results:
        print(f"\n{label}: % vs G61 optimum; columns per P: " + " / ".join(keys) + " (kind of plan, optimum apex mm/s)")
        print(f"  {'corner':8s} {'G61 s':>7s} | " + " | ".join(f"P{p:<5g}{'':37s}" for p in P_LADDER))
        for name, g61, cells in rows:
            parts = []
            for p in P_LADDER:
                c = cells[p]
                rel = "/".join(f"{100 * (c[k] / g61 - 1):+5.1f}" for k in keys)
                parts.append(f"{rel} {c['kind'][0]} {c['apex_opt']:5.1f}")
            print(f"  {name:8s} {g61:7.4f} | " + " | ".join(parts))


if __name__ == "__main__":
    main()
