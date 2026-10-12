"""Current firmware (tests/firmware_model.py: H743, ENABLE_PATH_BLENDING=1, ENABLE_JERK_ACCELERATION=0) vs our
pipeline on the same constant-acceleration machine.

Run: .\\.venv\\Scripts\\python.exe -m tools.trajectory_sim.tests.bench_firmware [fw2]
  fw2: firmware-only sweep of the arc-limit variants (Firmware_Planner_Modifications.md FW-1 / FW-2 tables):
       time vs exact stop per program and P on the symmetric standard machine.

Machines: the standard axes with jerk = inf and A = D = min(A, D) per axis (the firmware has one acceleration
per axis, so a user would configure the smaller one), and a variant with Z acceleration raised to Y's (the
firmware caps every blend arc by the smallest XYZ acceleration). Both sides use the same P semantics:
maximum deviation from the programmed corner (firmware arc: vertex-to-arc distance; ours: certified
Hausdorff bound). Programs are fully queued (no starvation, so the firmware's pending-move timeouts never
fire). Times in seconds; "ours/fw" < 1 means our pipeline is faster.
Baseline: G61.1 = exact stop (ours and the firmware's coincide with the Ruckig rest-to-rest sum). The firmware
runs with junction deviation $11 = 0 (user decision 2026-10-11): any finite value lets G61 and unblended G64
corners pass the vertex at speed, i.e. an instantaneous change of axis velocity, which no machine can follow.
With $11 = 0 firmware G61 equals G61.1 (asserted).
Also reported: the firmware's largest axis acceleration / limit from second differences of sampled positions
(G64 only; dt 1 ms), to confirm its blends stay within axis limits.
"""

from __future__ import annotations

import math
import random
import sys
from concurrent.futures import ProcessPoolExecutor

from tools.trajectory_sim.model import LineMove, Limits
from tools.trajectory_sim.tests.bench_optimal import corners
from tools.trajectory_sim.tests.firmware_model import G61, G61_1, FirmwareMove, run
from tools.trajectory_sim.tests.machines import load_axes
from tools.trajectory_sim.tests.test_pipeline import Pipeline, zigzag

FEED = 100.0
P_VALUES = (0.01, 0.05, 0.2, 1.0)
SAMPLE_DT = 1e-3
JUNCTION_DEVIATION = 0.0


def machines() -> dict[str, list[Limits]]:
    standard = load_axes("machine_standard_xyz")
    symmetric = [Limits(a.speed, min(a.acceleration, a.deceleration), math.inf) for a in standard]
    fast_z = symmetric[:2] + [Limits(symmetric[2].speed, symmetric[1].acceleration, math.inf)]
    return {"trapezoidal_symmetric": symmetric, "trapezoidal_symmetric_fast_z": fast_z}


def programs() -> dict[str, list[tuple[float, float, float]]]:
    result = {name: points for name, points in corners().items()}
    result["zigzag"] = [(0.0, 0.0, 0.0)] + zigzag()
    rng = random.Random(4602)  # same draws as test_pipeline.test_g64_never_slower_than_g61
    for seed in range(3):
        point, path = (0.0, 0.0, 0.0), [(0.0, 0.0, 0.0)]
        for _ in range(8):
            point = tuple(p + rng.uniform(-1, 1) * 10 ** rng.uniform(-1, 1.3) for p in point)
            path.append(point)
        result[f"random {seed}"] = path
    result["chords"] = [(0.0, 0.0, 0.0)] + [(10 * math.sin(k * 0.02), 10 - 10 * math.cos(k * 0.02), 0.0) for k in range(1, 30)]
    result["square"] = [(0.0, 0.0, 0.0), (40, 0, 0), (40, 30, 0), (0, 30, 5), (0, 0, 5), (0, 0, 0)]
    return result


def ours(axes, points, tolerance) -> float:
    pipeline = Pipeline(axes, points[0])
    for k, point in enumerate(points[1:]):
        pipeline.feed(LineMove(k, point, FEED, tolerance))
    runtime = pipeline.runtime
    assert not runtime.failures
    return sum(e.duration for e in runtime.suffix)


def firmware(axes, points, control, arc_limits="firmware", arc_k=0.5 ** 0.5):
    return run(axes, [FirmwareMove(p, FEED, control) for p in points[1:]], points[0],
               junction_deviation=JUNCTION_DEVIATION, arc_limits=arc_limits, arc_k=arc_k)


def max_axis_accel_ratio(model, axes) -> float:
    q = model.samples(SAMPLE_DT)
    worst = 0.0
    for k in range(1, len(q) - 1):
        for i, axis in enumerate(axes):
            worst = max(worst, abs(q[k + 1][i] - 2 * q[k][i] + q[k - 1][i]) / SAMPLE_DT ** 2 / axis.acceleration)
    return worst


def row(args):
    label, name = args
    axes, points = machines()[label], programs()[name]
    result = {"g61_1": ours(axes, points, None), "fw_g61_1": firmware(axes, points, G61_1).duration(),
              "fw_g61": firmware(axes, points, G61).duration(), "p": {}}
    for p in P_VALUES:
        model = firmware(axes, points, p)
        planar = firmware(axes, points, p, "planar")
        result["p"][p] = (model.duration(), planar.duration(), ours(axes, points, p),
                          max(max_axis_accel_ratio(model, axes), max_axis_accel_ratio(planar, axes)))
    return label, name, result


def main() -> None:
    jobs = [(label, name) for label in machines() for name in programs()]
    with ProcessPoolExecutor() as pool:
        results = list(pool.map(row, jobs))
    current = None
    for label, name, r in results:
        if label != current:
            current = label
            print(f"\n{label}: per P: fw G64 / fw+FW-1 / ours (s); ours/fw; max axis accel / limit of both fw runs")
            print(f"  {'program':10s} {'G61.1':>7s} | " + " | ".join(f"P{p:<5g}{'':37s}" for p in P_VALUES))
        assert abs(r["g61_1"] - r["fw_g61_1"]) <= 1e-6 * r["g61_1"], (name, r["g61_1"], r["fw_g61_1"])
        assert abs(r["fw_g61"] - r["fw_g61_1"]) <= 1e-12 * r["g61_1"], (name, r["fw_g61"], r["fw_g61_1"])
        cells = []
        for p in P_VALUES:
            fw, planar, us, accel = r["p"][p]
            cells.append(f"{fw:7.4f} {planar:7.4f} {us:7.4f} {us / fw:6.3f} ({accel:4.2f})")
        print(f"  {name:10s} {r['g61_1']:7.4f} | " + " | ".join(cells))


FW2_VARIANTS = (("current", "firmware", 0.7071), ("FW-1", "planar", 0.7071), ("k0.707", "circle", 0.7071),
                ("k0.9", "circle", 0.9), ("k0.99", "circle", 0.99), ("ad0.9", "adaptive", 0.9),
                ("ad0.99", "adaptive", 0.99), ("ad1", "adaptive", 1.0))


def fw2_sweep() -> None:
    """Firmware model only: arc-limit variants (ad = adaptive circle law with k_max), % vs exact stop."""
    axes = machines()["trapezoidal_symmetric"]
    for p in (0.05, 0.2, 1.0):
        print(f"\nP{p}: time vs G61.1 (%)")
        print(f"  {'program':10s} " + " ".join(f"{label:>7s}" for label, _, _ in FW2_VARIANTS))
        for name, points in programs().items():
            stop = firmware(axes, points, G61_1).duration()
            cells = [100 * (firmware(axes, points, p, mode, k).duration() / stop - 1) for _, mode, k in FW2_VARIANTS]
            print(f"  {name:10s} " + " ".join(f"{c:+7.1f}" for c in cells))


if __name__ == "__main__":
    fw2_sweep() if sys.argv[1:] == ["fw2"] else main()
