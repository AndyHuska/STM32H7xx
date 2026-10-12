"""Interactive 3D viewer: exact-stop path vs the G64 path of each architecture, per program and P.

Run: .\\.venv\\Scripts\\python.exe -m tools.trajectory_sim.tests.view_paths

Left: 3D path (exact stop = grey dashed polyline; each G64 run in its own colour). Right: path speed vs time,
so total times compare directly (also listed in the legend). Buttons flip program, P, machine and zoom on
each corner (blends at small P are invisible at full scale); check boxes show/hide runs. Mouse: drag rotates.
Runs (machines from bench_firmware: constant acceleration, A = D, firmware junction deviation $11 = 0; plus the
real machine of the hardware tests):
  exact stop          G61.1 (firmware model; identical to our G61)
  fw current          firmware model, current code
  fw FW-1             per-axis (planar) arc limits
  fw FW-1+2 adapt     plus the acceleration-circle arc law (k_max = K_MAX)
  ours (clothoid)     our pipeline (front end, L3 clothoid blends, L4, L5)
Programs: bench_firmware.programs() (F100) plus every section of test_firmware_hw_regressions.SECTIONS (own feed);
add a hardware routine there and it appears here.
Results are cached per (machine, program, P); our pipeline takes up to a minute on the chord program.
"""

from __future__ import annotations

import bisect
import math
from dataclasses import dataclass

import matplotlib.pyplot as plt
from matplotlib.widgets import Button, CheckButtons

from tools.trajectory_sim.model import AccelerationStep, LineMove
from tools.trajectory_sim.phases import advance
from tools.trajectory_sim.tests import test_firmware_hw_regressions as hardware
from tools.trajectory_sim.tests.bench_firmware import FEED, JUNCTION_DEVIATION, machines, programs
from tools.trajectory_sim.tests.firmware_model import G61_1, FirmwareMove, point, run, speed_at
from tools.trajectory_sim.tests.test_pipeline import Pipeline

P_VALUES = (0.01, 0.05, 0.2, 1.0, 3.0)
K_MAX = 0.9
STEP = 0.02     # mm between path samples
DT = 0.002      # s between samples of our time-domain plan

RUNS = (("exact stop", "0.45"), ("fw current", "tab:red"), ("fw FW-1", "tab:orange"),
        ("fw FW-1+2 adapt", "tab:green"), ("ours (clothoid)", "tab:blue"))


@dataclass
class Trace:
    xyz: list[tuple[float, float, float]]
    t: list[float]
    v: list[float]
    duration: float   # exact planned time (the sampled t is for plotting only)


def firmware_trace(axes, points, control, feed, **options) -> Trace:
    model = run(axes, [FirmwareMove(p, feed, control) for p in points[1:]], points[0],
                junction_deviation=JUNCTION_DEVIATION, **options)
    xyz, t, v = [], [0.0], []
    for block, (entry, exit, _) in zip(model.blocks, model.schedule()):
        n = max(4, min(400, math.ceil(block.length / STEP)))
        for k in range(0 if not xyz else 1, n + 1):
            s = block.length * k / n
            speed = speed_at(block, entry, exit, s)
            if v:
                ds = block.length / n
                t.append(t[-1] + 2.0 * ds / max(v[-1] + speed, 1e-12))
            xyz.append(point(block, s))
            v.append(speed)
    return Trace(xyz, t, v, model.duration())


def ours_trace(axes, points, tolerance, feed) -> Trace:
    pipeline = Pipeline(axes, points[0])
    for k, p in enumerate(points[1:]):
        pipeline.feed(LineMove(k, p, feed, tolerance))
    entries = pipeline.stream.pieces()
    starts = [e[0] for e in entries]
    state, now = pipeline.runtime.tail, 0.0
    samples = [(0.0, state.s, state.v)]
    for element in pipeline.runtime.suffix:
        if isinstance(element, AccelerationStep) or element.duration <= 0.0:
            state = advance(state, element)
            continue
        n = max(1, math.ceil(element.duration / DT))
        for k in range(1, n + 1):
            tau = element.duration * k / n
            moved = advance(state, element, tau) if k < n else advance(state, element)
            samples.append((now + tau, moved.s, moved.v))
        state, now = advance(state, element), now + element.duration
    xyz = []
    for _, s, _ in samples:
        index = max(0, bisect.bisect_right(starts, s) - 1)
        s0, piece, _ = entries[index]
        xyz.append(piece.evaluate(min(max(s - s0, 0.0), piece.length))[0])
    return Trace(xyz, [x[0] for x in samples], [x[2] for x in samples], now)


def compute(axes, points, tolerance, feed) -> dict[str, Trace]:
    return {
        "exact stop": firmware_trace(axes, points, G61_1, feed),
        "fw current": firmware_trace(axes, points, tolerance, feed),
        "fw FW-1": firmware_trace(axes, points, tolerance, feed, arc_limits="planar"),
        "fw FW-1+2 adapt": firmware_trace(axes, points, tolerance, feed, arc_limits="adaptive", arc_k=K_MAX),
        "ours (clothoid)": ours_trace(axes, points, tolerance, feed),
    }


class Viewer:
    def __init__(self) -> None:
        self.machines = {**machines(), "example (hw)": hardware.machine()}
        self.programs = {**{name: (points, FEED) for name, points in programs().items()}, **hardware.programs()}
        self.machine_names, self.program_names = list(self.machines), list(self.programs)
        self.m = self.prog = 0
        self.p = P_VALUES.index(1.0)
        self.corner = -1   # -1 = whole program, else index of the vertex
        self.visible = {name: True for name, _ in RUNS}
        self.cache: dict[tuple[int, int, int], dict[str, Trace]] = {}

        self.fig = plt.figure(figsize=(15, 8))
        self.ax3d = self.fig.add_axes((0.02, 0.17, 0.52, 0.78), projection="3d")
        self.axv = self.fig.add_axes((0.60, 0.40, 0.38, 0.52))
        buttons = [("< program", lambda: self.step("prog", -1)), ("program >", lambda: self.step("prog", 1)),
                   ("< P", lambda: self.step("p", -1)), ("P >", lambda: self.step("p", 1)),
                   ("< corner", lambda: self.step("corner", -1)), ("corner >", lambda: self.step("corner", 1)),
                   ("machine", lambda: self.step("m", 1))]
        self.buttons = []
        for k, (label, action) in enumerate(buttons):
            button = Button(self.fig.add_axes((0.03 + 0.075 * k, 0.04, 0.07, 0.06)), label)
            button.on_clicked(lambda _event, act=action: act())
            self.buttons.append(button)
        self.checks = CheckButtons(self.fig.add_axes((0.58, 0.04, 0.15, 0.26)), [n for n, _ in RUNS],
                                   [True] * len(RUNS))
        for label, (_, colour) in zip(self.checks.labels, RUNS):
            label.set_color(colour)
        self.checks.on_clicked(self.toggle)
        self.info = self.fig.text(0.75, 0.06, "", family="monospace", fontsize=9, va="bottom")
        self.draw()

    def step(self, what: str, delta: int) -> None:
        if what == "prog":
            self.prog = (self.prog + delta) % len(self.program_names)
            self.corner = -1
        elif what == "p":
            self.p = (self.p + delta) % len(P_VALUES)
        elif what == "m":
            self.m = (self.m + delta) % len(self.machine_names)
        else:
            vertices = len(self.programs[self.program_names[self.prog]][0]) - 2
            self.corner = (self.corner + 1 + delta) % (vertices + 1) - 1
        self.draw()

    def toggle(self, label: str) -> None:
        self.visible[label] = not self.visible[label]
        self.draw()

    def traces(self) -> dict[str, Trace]:
        key = (self.m, self.prog, self.p)
        if key not in self.cache:
            self.fig.suptitle("computing ...")
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            axes = self.machines[self.machine_names[self.m]]
            points, feed = self.programs[self.program_names[self.prog]]
            self.cache[key] = compute(axes, points, P_VALUES[self.p], feed)
        return self.cache[key]

    def draw(self) -> None:
        traces = self.traces()
        points, feed = self.programs[self.program_names[self.prog]]
        tolerance = P_VALUES[self.p]
        self.ax3d.clear()
        self.axv.clear()
        for name, colour in RUNS:
            if not self.visible[name]:
                continue
            trace = traces[name]
            xs, ys, zs = zip(*trace.xyz)
            style = dict(color=colour, linestyle="--", linewidth=1.0) if name == "exact stop" else dict(color=colour, linewidth=1.6)
            self.ax3d.plot(xs, ys, zs, axlim_clip=True, **style)
            self.axv.plot(trace.t, trace.v, label=f"{name}: {trace.duration:.4f} s", **style)
        self.ax3d.scatter(*zip(*points), color="k", s=10, axlim_clip=True)
        self.frame(points, tolerance)
        self.ax3d.set_xlabel("X mm")
        self.ax3d.set_ylabel("Y mm")
        self.ax3d.set_zlabel("Z mm")
        self.axv.set_xlabel("time s")
        self.axv.set_ylabel("path speed mm/s")
        self.axv.grid(True, alpha=0.3)
        self.axv.legend(fontsize=8, loc="upper right")
        where = "whole program" if self.corner < 0 else f"corner {self.corner + 1} (box +/- {self.half_box(tolerance):.3g} mm)"
        self.fig.suptitle(f"{self.program_names[self.prog]}   G64 P{tolerance:g}   F{feed:g}   "
                          f"machine {self.machine_names[self.m]}   [{where}]")
        stop = traces["exact stop"].duration
        lines = [f"{'run':19s} {'time s':>7s} {'vs stop':>7s}"]
        for name, _ in RUNS:
            lines.append(f"{name:19s} {traces[name].duration:7.4f} {100 * (traces[name].duration / stop - 1):+6.1f}%")
        self.info.set_text("\n".join(lines))
        self.fig.canvas.draw_idle()

    @staticmethod
    def half_box(tolerance: float) -> float:
        return max(6.0 * tolerance, 0.3)

    def frame(self, points, tolerance) -> None:
        if self.corner < 0:
            low = [min(p[i] for p in points) for i in range(3)]
            high = [max(p[i] for p in points) for i in range(3)]
            centre = [(a + b) / 2 for a, b in zip(low, high)]
            half = max(max(b - a for a, b in zip(low, high)) / 2, 1e-3)
        else:
            centre, half = points[self.corner + 1], self.half_box(tolerance)
        self.ax3d.set_xlim(centre[0] - half, centre[0] + half)
        self.ax3d.set_ylim(centre[1] - half, centre[1] + half)
        self.ax3d.set_zlim(centre[2] - half, centre[2] + half)
        self.ax3d.set_box_aspect((1, 1, 1))


def main() -> None:
    Viewer()
    plt.show()


if __name__ == "__main__":
    main()
