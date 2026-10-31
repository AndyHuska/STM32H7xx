"""Interactive 3D viewer: exact-stop path vs the G64 path of each architecture, per program and P.

Run: .\\.venv\\Scripts\\python.exe -m tools.trajectory_sim.tests.view_paths

Left: 3D path (exact stop = grey dashed polyline; each G64 run in its own colour). Right: path speed vs time,
so total times compare directly (also listed in the legend). Buttons flip program, P, G64 K, machine and zoom on
each corner (blends at small P are invisible at full scale); check boxes show/hide runs. Mouse: drag rotates.
G64 K (FW-5) applies to the constant-accel firmware run: 0 = largest arc within P, else arcs sized for the feed at
K * A_plane. The table lists time, % vs exact stop and the largest corner deviation (mm) of each run.
Runs (machines from bench_firmware: constant acceleration, A = D, firmware junction deviation $11 = 0; plus the
real machine of the hardware tests):
  exact stop          G61.1 (firmware model; identical to our G61)
    Blended Constant Accel   firmware model with adaptive acceleration-circle blending
    Blended S-Curve          our pipeline (front end, L3 clothoid blends, L4, L5)
Programs: bench_firmware.programs() (F100) plus every section of test_firmware_hw_regressions.SECTIONS (own feed);
add a hardware routine there and it appears here.
Results are cached per (machine, program, P, K); our pipeline takes up to a minute on the chord program.
Open CSV loads reference run A; Compare CSV overlays recording B using a completion anchor on the first shared
path name. The time shift is fixed for the entire recording; positions and subsequent timing differences are retained.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from tkinter import filedialog, messagebox
from typing import Any, Sequence, cast

import matplotlib.pyplot as plt
from matplotlib.colors import Normalize, to_rgba
from matplotlib.widgets import Button, CheckButtons
from mpl_toolkits.mplot3d import Axes3D

from tools.trajectory_sim.model import AccelerationStep, LineMove, Vector
from tools.trajectory_sim.phases import advance
from tools.trajectory_sim.tests import test_firmware_hw_regressions as hardware
from tools.trajectory_sim.tests.bench_firmware import FEED, JUNCTION_DEVIATION, machines, programs
from tools.trajectory_sim.tests.firmware_model import G61_1, FirmwareMove, point, run, speed_at
from tools.trajectory_sim.tests.run_log import RunLog, RunMove, load_run_log
from tools.trajectory_sim.tests.test_pipeline import Pipeline

P_VALUES = (0.01, 0.05, 0.2, 1.0, 3.0)
K_VALUES = (0.0, 0.3, 0.6, 0.9)
K_MAX = 0.9
STEP = 0.02     # mm between path samples
DT = 0.002      # s between samples of our time-domain plan

RUNS = (("exact stop", "0.45"), ("Blended Constant Accel", "tab:green"),
    ("Blended S-Curve", "tab:blue"))


@dataclass
class Trace:
    xyz: list[tuple[float, float, float]]
    t: list[float]
    v: list[float]
    duration: float   # exact planned time (the sampled t is for plotting only)
    deviation: float = 0.0   # largest corner deviation from the programmed path (mm)
    move_end_times: dict[str, float] = field(default_factory=dict)


@dataclass
class HoverSource:
    label: str
    times: list[float]
    xyz: list[Vector]
    speeds: Sequence[float | None]
    artist: Any
    colors: Sequence[str | tuple[float, float, float, float]] | None = None


def upper_bound(values: Sequence[float], target: float) -> int:
    low, high = 0, len(values)
    while low < high:
        middle = (low + high) // 2
        if values[middle] <= target:
            low = middle + 1
        else:
            high = middle
    return low


def status_point_colors(samples, mode: str):
    values = [sample.xyz[2] if mode == "z" else sample.path_speed for sample in samples]
    valid = [value for value in values if value is not None and math.isfinite(value)]
    if not valid:
        return [to_rgba("#888888")] * len(samples), None
    low, high = min(valid), max(valid)
    norm = Normalize(low, high if high > low else low + 1.0)
    color_map = plt.get_cmap("RdYlGn")
    colors = [color_map(norm(value)) if value is not None and math.isfinite(value) else to_rgba("#888888")
              for value in values]
    return colors, (low, high)


def position_at_time(times: list[float], xyz: list[Vector], time: float) -> Vector | None:
    if not times or len(times) != len(xyz) or time < times[0] or time > times[-1]:
        return None
    right = upper_bound(times, time)
    left = right - 1
    if times[left] == time or right == len(times):
        return xyz[left]
    span = times[right] - times[left]
    if span <= 0.0:
        return xyz[right]
    ratio = (time - times[left]) / span
    left_x, left_y, left_z = xyz[left]
    right_x, right_y, right_z = xyz[right]
    return (left_x + ratio * (right_x - left_x), left_y + ratio * (right_y - left_y),
            left_z + ratio * (right_z - left_z))


def value_at_time(times: list[float], values: Sequence[float | None], time: float) -> float | None:
    if not times or len(times) != len(values) or time < times[0] or time > times[-1]:
        return None
    right = upper_bound(times, time)
    left = right - 1
    if times[left] == time or right == len(times):
        return values[left]
    span = times[right] - times[left]
    if span <= 0.0:
        return values[right]
    value_left, value_right = values[left], values[right]
    if value_left is None or value_right is None:
        return None
    ratio = (time - times[left]) / span
    return value_left + ratio * (value_right - value_left)


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
    return Trace(xyz, t, v, model.duration(), max((c.deviation for c in model.corners), default=0.0))


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
        index = max(0, upper_bound(starts, s) - 1)
        s0, piece, _ = entries[index]
        xyz.append(piece.evaluate(min(max(s - s0, 0.0), piece.length))[0])
    return Trace(xyz, [x[0] for x in samples], [x[2] for x in samples], now,
                 max((c.deviation for c in pipeline.stream.corners if c.kind == "blend"), default=0.0))


def compute(axes, points, tolerance, feed, k=0.0) -> dict[str, Trace]:
    return {
        "exact stop": firmware_trace(axes, points, G61_1, feed),
        "Blended Constant Accel": firmware_trace(axes, points, tolerance, feed,
                                                   arc_limits="adaptive", arc_k=K_MAX, blend_k=k),
        "Blended S-Curve": ours_trace(axes, points, tolerance, feed),
    }


def rapid_feed(axes, move: RunMove) -> float:
    delta = tuple(end - start for start, end in zip(move.start, move.end))
    length = math.sqrt(sum(component * component for component in delta))
    if length <= 0.0:
        return min(axis.speed for axis in axes)
    return min(axis.speed / abs(component / length) for axis, component in zip(axes, delta) if component != 0.0)


def logged_firmware_trace(axes, moves, junction_deviation, start=None, *, force_exact=False, **options) -> Trace:
    initial_position = moves[0].start if start is None else start
    firmware_moves = []
    for move in moves:
        feed = move.feed if move.feed is not None else rapid_feed(axes, move)
        control = G61_1 if force_exact or move.blend_mode == "exact" else move.tolerance
        firmware_moves.append(FirmwareMove(move.end, feed, control, move.stop_after, move.blend_k))
    model = run(axes, firmware_moves, initial_position, junction_deviation=junction_deviation, **options)
    xyz, times, speeds = [], [0.0], []
    schedule = model.schedule()
    source_end_times = {}
    model_time = 0.0
    for block, (_, _, duration) in zip(model.blocks, schedule):
        model_time += duration
        source_end_times[block.source] = model_time
    previous_end_time = 0.0
    for index in range(len(moves)):
        if index in source_end_times:
            previous_end_time = source_end_times[index]
        else:
            source_end_times[index] = previous_end_time
    for block, (entry, exit, _) in zip(model.blocks, schedule):
        count = max(4, min(400, math.ceil(block.length / STEP)))
        for index in range(0 if not xyz else 1, count + 1):
            distance = block.length * index / count
            speed = speed_at(block, entry, exit, distance)
            if speeds:
                ds = block.length / count
                times.append(times[-1] + 2.0 * ds / max(speeds[-1] + speed, 1e-12))
            xyz.append(point(block, distance))
            speeds.append(speed)
    move_end_times = {move.move_id: source_end_times[index] for index, move in enumerate(moves)
                      if index in source_end_times}
    return Trace(xyz, times, speeds, model_time,
                 deviation=max((corner.deviation for corner in model.corners), default=0.0),
                 move_end_times=move_end_times)


def logged_ours_trace(axes, moves, start=None) -> Trace:
    initial_position = moves[0].start if start is None else start
    pipeline = Pipeline(axes, initial_position)
    for source_id, move in enumerate(moves):
        feed = move.feed if move.feed is not None else rapid_feed(axes, move)
        tolerance = move.tolerance if move.blend_mode == "g64" and not move.stop_after else None
        pipeline.feed(LineMove(source_id, move.end, feed, tolerance))
    entries = pipeline.stream.pieces()
    if not entries:
        return Trace([initial_position], [0.0], [0.0], 0.0,
                     move_end_times={move.move_id: 0.0 for move in moves})
    starts = [entry[0] for entry in entries]
    state, now = pipeline.runtime.tail, 0.0
    samples = [(0.0, state.s, state.v)]
    for element in pipeline.runtime.suffix:
        if isinstance(element, AccelerationStep) or element.duration <= 0.0:
            state = advance(state, element)
            continue
        count = max(1, math.ceil(element.duration / DT))
        for index in range(1, count + 1):
            tau = element.duration * index / count
            moved = advance(state, element, tau) if index < count else advance(state, element)
            samples.append((now + tau, moved.s, moved.v))
        state, now = advance(state, element), now + element.duration
    xyz = []
    for _, distance, _ in samples:
        index = max(0, upper_bound(starts, distance) - 1)
        start_distance, piece, _ = entries[index]
        xyz.append(piece.evaluate(min(max(distance - start_distance, 0.0), piece.length))[0])
    move_end_times = {}
    for source_id, move in enumerate(moves):
        marker = pipeline.stream.markers.get(source_id)
        if marker is None:
            continue
        marker_position = float(marker)
        sample_index = min(range(len(samples)), key=lambda index: abs(samples[index][1] - marker_position))
        move_end_times[move.move_id] = samples[sample_index][0]
    return Trace(xyz, [sample[0] for sample in samples], [sample[2] for sample in samples], now,
                 move_end_times=move_end_times)


def add_logged_dwells(trace: Trace, moves, events) -> Trace:
    dwell_indices: dict[int, int] = {}
    dwell_after_moves: dict[int, int] = {}
    for event in events:
        if event.event != "dwell_start" or event.dwell_ms is None:
            continue
        preceding = [index for index, move in enumerate(moves)
                     if (not event.path_id or move.path_id == event.path_id) and move.sequence < event.sequence]
        move_index = preceding[-1] if preceding else 0
        after_index = preceding[-1] if preceding else -1
        dwell_after_moves[after_index] = dwell_after_moves.get(after_index, 0) + event.dwell_ms
        target = moves[move_index].end if preceding else moves[0].start
        sample_index = min(range(len(trace.xyz)),
                           key=lambda index: sum((a - b) ** 2 for a, b in zip(trace.xyz[index], target)))
        dwell_indices[sample_index] = dwell_indices.get(sample_index, 0) + event.dwell_ms

    if not dwell_indices:
        return trace
    xyz, times, speeds = [], [], []
    shift = 0.0
    for index, (position, time, speed) in enumerate(zip(trace.xyz, trace.t, trace.v)):
        shifted_time = time + shift
        xyz.append(position)
        times.append(shifted_time)
        speeds.append(speed)
        dwell = dwell_indices.get(index, 0) / 1000.0
        if dwell:
            shift += dwell
            xyz.append(position)
            times.append(shifted_time + dwell)
            speeds.append(0.0)
    move_end_times = {}
    delay = 0.0
    for index, move in enumerate(moves):
        delay += dwell_after_moves.get(index - 1, 0) / 1000.0
        if move.move_id in trace.move_end_times:
            move_end_times[move.move_id] = trace.move_end_times[move.move_id] + delay
    return Trace(xyz, times, speeds, trace.duration + shift, deviation=trace.deviation,
                 move_end_times=move_end_times)


def status_motion_start(samples, settings) -> float | None:
    if not samples:
        return None
    first = samples[0]
    thresholds = []
    for axis in "xyz":
        steps = float(settings.get(f"steps_per_mm_{axis}", 0.0))
        thresholds.append(max(1e-3, 0.5 / steps) if steps > 0.0 else 1e-3)

    def is_moving(sample) -> bool:
        if sample.path_speed is not None and sample.path_speed > 0.01:
            return True
        return any(abs(position - origin) >= threshold
                   for position, origin, threshold in zip(sample.xyz, first.xyz, thresholds))

    candidate = first if is_moving(first) else None
    for sample in samples[1:]:
        if is_moving(sample):
            if candidate is not None:
                return candidate.elapsed_ms / 1000.0
            candidate = sample
        else:
            candidate = None
    return candidate.elapsed_ms / 1000.0 if candidate is not None else None


def delay_trace(trace: Trace, start: Vector, delay: float) -> Trace:
    if delay <= 0.0:
        return trace
    move_end_times = {move_id: time + delay for move_id, time in trace.move_end_times.items()}
    return Trace([start, start, *trace.xyz], [0.0, delay, *(delay + time for time in trace.t)],
                 [0.0, 0.0, *trace.v], trace.duration, deviation=trace.deviation,
                 move_end_times=move_end_times)


def moves_with_event_stops(moves, events) -> tuple[RunMove, ...]:
    stop_after_ids = set()
    for event in events:
        if event.event not in ("dwell_start", "stop"):
            continue
        preceding = [move for move in moves
                     if (not event.path_id or move.path_id == event.path_id) and move.sequence < event.sequence]
        if preceding:
            stop_after_ids.add(preceding[-1].move_id)
    return tuple(replace(move, stop_after=move.stop_after or move.move_id in stop_after_ids)
                 for move in moves)


def compute_logged(axes, moves, settings, events=(), start=None, motion_start=0.0) -> dict[str, Trace]:
    junction_deviation = float(settings.get("junction_deviation_mm", JUNCTION_DEVIATION))
    planned_moves = moves_with_event_stops(moves, events)
    traces = {
        "exact stop": logged_firmware_trace(axes, planned_moves, junction_deviation, start, force_exact=True),
        "Blended Constant Accel": logged_firmware_trace(axes, planned_moves, junction_deviation, start,
                                                          arc_limits="adaptive", arc_k=K_MAX),
        "Blended S-Curve": logged_ours_trace(axes, planned_moves, start),
    }
    initial_position = moves[0].start if start is None else start
    return {name: delay_trace(add_logged_dwells(trace, moves, events), initial_position, motion_start)
            for name, trace in traces.items()}


def align_recorded_runs(reference: RunLog, comparison: RunLog) -> tuple[float, str]:
    if reference.frame != comparison.frame:
        raise ValueError("Runs must use the same coordinate frame")
    if not reference.samples or not comparison.samples:
        raise ValueError("Both runs must contain status samples")

    def named_paths(log):
        paths = {}
        for move in log.moves:
            if move.path_name:
                paths.setdefault(move.path_name, {}).setdefault(move.path_id, []).append(move)
        return paths

    reference_paths, comparison_paths = named_paths(reference), named_paths(comparison)
    common = next((name for name in reference_paths if name in comparison_paths), None)
    if common is None:
        raise ValueError("Runs have no matching path names")
    if len(reference_paths[common]) != 1 or len(comparison_paths[common]) != 1:
        raise ValueError(f"Path name {common!r} is ambiguous within a run")
    first_moves = next(iter(reference_paths[common].values()))
    second_moves = next(iter(comparison_paths[common].values()))
    first_feed_moves = [move for move in first_moves if move.kind == "linear"]
    second_feed_moves = [move for move in second_moves if move.kind == "linear"]
    if first_feed_moves and second_feed_moves:
        first_moves, second_moves = first_feed_moves, second_feed_moves
    first_completions = {event.move_id: event.elapsed_ms / 1000.0 for event in reference.events
                         if event.event == "move_complete"}
    second_completions = {event.move_id: event.elapsed_ms / 1000.0 for event in comparison.events
                          if event.event == "move_complete"}
    for first, second in zip(first_moves, second_moves):
        first_delta = tuple(end - start for start, end in zip(first.start, first.end))
        second_delta = tuple(end - start for start, end in zip(second.start, second.end))
        if first.kind != second.kind or math.dist(first_delta, second_delta) > 0.02:
            raise ValueError(f"Path {common!r} has different move geometry/order; cannot align automatically")
        if math.dist(first.start, first.end) <= 1e-6:
            continue
        if first.move_id in first_completions and second.move_id in second_completions:
            return (first_completions[first.move_id] - second_completions[second.move_id],
                    f"{common}: {first.move_id} / {second.move_id}")
    raise ValueError(f"Path {common!r} has no corresponding motion-completion callbacks")


class Viewer:
    def __init__(self) -> None:
        self.machines = {**machines(), "example (hw)": hardware.machine()}
        self.programs = {**{name: (points, FEED) for name, points in programs().items()}, **hardware.programs()}
        self.machine_names, self.program_names = list(self.machines), list(self.programs)
        self.m = self.prog = 0
        self.p = P_VALUES.index(1.0)
        self.k = 0
        self.corner = -1   # -1 = whole program, else index of the vertex
        self.visible = {name: True for name, _ in RUNS}
        self.cache: dict[tuple[int, int, int, int], dict[str, Trace]] = {}
        self.run_log: RunLog | None = None
        self.comparison_log: RunLog | None = None
        self.comparison_offset = 0.0
        self.comparison_anchor = ""
        self.comparison_artists = None
        self.log_cache: dict[tuple[str, tuple], dict[str, Trace]] = {}
        self.path_index = 0
        self.log_corner = -1
        self.real_color_mode = "z"
        self.hover_sources: list[HoverSource] = []
        self.hover_cursor = None
        self.status_samples = []
        self.status_points_artist = None
        self.status_velocity_artist = None
        self.status_hover_source = None
        self.status_timed_samples: list[tuple[int, float, float]] = []

        self.fig = plt.figure(figsize=(15, 8))
        self.ax3d = cast(Axes3D, self.fig.add_axes((0.02, 0.17, 0.52, 0.78), projection="3d"))
        self.axv = self.fig.add_axes((0.60, 0.43, 0.38, 0.49))
        buttons = [("< program", lambda: self.step("prog", -1)), ("program >", lambda: self.step("prog", 1)),
                   ("< P", lambda: self.step("p", -1)), ("P >", lambda: self.step("p", 1)),
                   ("< K", lambda: self.step("k", -1)), ("K >", lambda: self.step("k", 1)),
                   ("< corner", lambda: self.step("corner", -1)), ("corner >", lambda: self.step("corner", 1)),
                   ("machine", lambda: self.step("m", 1))]
        self.buttons = []
        for k, (label, action) in enumerate(buttons):
            button = Button(self.fig.add_axes((0.02 + 0.0595 * k, 0.04, 0.056, 0.06)), label)
            button.on_clicked(lambda _event, act=action: act())
            self.buttons.append(button)
        self.update_p_controls()
        self.checks = CheckButtons(self.fig.add_axes((0.58, 0.03, 0.16, 0.19)), [n for n, _ in RUNS],
                                   [True] * len(RUNS))
        for label, (_, colour) in zip(self.checks.labels, RUNS):
            label.set_color(colour)
        self.checks.on_clicked(self.toggle)
        self.path_buttons = []
        for k, delta in enumerate((-1, 1)):
            button = Button(self.fig.add_axes((0.58 + 0.07 * k, 0.30, 0.065, 0.05)),
                            "Path <" if delta < 0 else "Path >")
            button.on_clicked(lambda _event, step=delta: self.step("logpath", step))
            self.path_buttons.append(button)
        self.full_program_button = Button(self.fig.add_axes((0.72, 0.30, 0.085, 0.05)), "Full program")
        self.full_program_button.on_clicked(self.show_full_program)
        self.open_button = Button(self.fig.add_axes((0.81, 0.30, 0.075, 0.05)), "Open CSV")
        self.open_button.on_clicked(self.open_run_log)
        self.builtin_button = Button(self.fig.add_axes((0.89, 0.30, 0.09, 0.05)), "Built-in runs")
        self.builtin_button.on_clicked(self.show_builtin_runs)
        self.color_z_button = Button(self.fig.add_axes((0.58, 0.235, 0.09, 0.05)), "Color by Z")
        self.color_z_button.on_clicked(lambda _event: self.set_real_color_mode("z"))
        self.color_speed_button = Button(self.fig.add_axes((0.68, 0.235, 0.10, 0.05)), "Color by speed")
        self.color_speed_button.on_clicked(lambda _event: self.set_real_color_mode("speed"))
        self.compare_button = Button(self.fig.add_axes((0.79, 0.235, 0.09, 0.05)), "Compare CSV")
        self.compare_button.on_clicked(self.open_comparison_log)
        self.clear_compare_button = Button(self.fig.add_axes((0.89, 0.235, 0.09, 0.05)), "Clear comparison")
        self.clear_compare_button.label.set_fontsize(8)
        self.clear_compare_button.on_clicked(self.clear_comparison)
        self.update_color_buttons()
        self.info = self.fig.text(0.75, 0.03, "", family="monospace", fontsize=9, va="bottom")
        self.fig.canvas.mpl_connect("motion_notify_event", self.on_plot_hover)
        self.draw()

    def reset_hover_sources(self) -> None:
        self.hover_sources.clear()
        self.comparison_artists = None
        self.status_points_artist = None
        self.status_velocity_artist = None
        self.status_hover_source = None
        self.status_timed_samples = []
        self.hover_cursor = self.axv.axvline(0.0, color="#555555", linewidth=0.9, alpha=0.7,
                                             visible=False, label="_nolegend_")
        self.hover_annotation = self.axv.annotate(
            "", xy=(0.0, 0.0), xytext=(12, 12), textcoords="offset points",
            bbox={"boxstyle": "round,pad=0.35", "fc": "white", "ec": "#666666", "alpha": 0.95},
            arrowprops={"arrowstyle": "->", "color": "#555555", "lw": 0.7},
            fontsize=8, family="monospace", zorder=20)
        self.hover_annotation.set_visible(False)

    def add_hover_source(self, label: str, times: list[float], xyz: list[Vector], speeds: Sequence[float | None],
                         color: str, marker: str = "o",
                         colors: Sequence[str | tuple[float, float, float, float]] | None = None
                         ) -> HoverSource | None:
        if not times or len(times) != len(xyz):
            return None
        artist = cast(Any, self.ax3d).scatter([xyz[0][0]], [xyz[0][1]], [xyz[0][2]], color=color,
                                               marker=marker, s=58, edgecolors="black", linewidths=0.7,
                                               depthshade=False, label="_nolegend_")
        source = HoverSource(label, times, xyz, speeds, artist, colors)
        self.hover_sources.append(source)
        return source

    def on_plot_hover(self, event) -> None:
        if self.hover_cursor is None:
            return
        if event.inaxes is not self.axv or event.xdata is None:
            self.hover_cursor.set_visible(False)
            self.hover_annotation.set_visible(False)
            for source in self.hover_sources:
                source.artist.set_visible(False)
            self.fig.canvas.draw_idle()
            return

        time = float(event.xdata)
        if event.ydata is None:
            return
        self.hover_cursor.set_xdata([time, time])
        self.hover_cursor.set_visible(True)
        self.hover_annotation.xy = (time, float(event.ydata))
        self.hover_annotation.set_position((-150, 12) if time > sum(self.axv.get_xlim()) / 2.0 else (12, 12))
        lines = [f"Time: {time:.3f} s"]
        for source in self.hover_sources:
            position = position_at_time(source.times, source.xyz, time)
            speed = value_at_time(source.times, source.speeds, time)
            source.artist.set_visible(position is not None)
            if position is None:
                lines.append(f"{source.label}: n/a")
                continue
            source.artist._offsets3d = ([position[0]], [position[1]], [position[2]])
            if source.colors:
                nearest = min(upper_bound(source.times, time), len(source.colors) - 1)
                source.artist.set_facecolor(source.colors[nearest])
            velocity = "n/a" if speed is None else f"{speed:.2f} mm/s"
            lines.append(f"{source.label}: X{position[0]:.2f} Y{position[1]:.2f} Z{position[2]:.2f}  V{velocity}")
        self.hover_annotation.set_text("\n".join(lines))
        self.hover_annotation.set_visible(True)
        self.fig.canvas.draw_idle()

    def open_run_log(self, _event=None) -> None:
        path = filedialog.askopenfilename(filetypes=[("Run log CSV", "*.csv"), ("All files", "*.*")])
        if not path:
            return
        try:
            self.run_log = load_run_log(path)
        except (OSError, ValueError) as error:
            messagebox.showerror("Could not open run log", str(error))
            return
        self.comparison_log = None
        self.comparison_offset = 0.0
        self.comparison_anchor = ""
        self.update_p_controls()
        self.path_index = 0
        self.log_corner = -1
        self.log_cache.clear()
        self.draw()

    def show_builtin_runs(self, _event=None) -> None:
        self.run_log = None
        self.comparison_log = None
        self.update_p_controls()
        self.log_cache.clear()
        self.draw()

    def open_comparison_log(self, _event=None) -> None:
        if self.run_log is None:
            messagebox.showinfo("Compare runs", "Open the reference CSV first.")
            return
        path = filedialog.askopenfilename(filetypes=[("Run log CSV", "*.csv"), ("All files", "*.*")])
        if not path:
            return
        try:
            comparison = load_run_log(path)
            offset, anchor = align_recorded_runs(self.run_log, comparison)
        except (OSError, ValueError) as error:
            messagebox.showerror("Could not compare runs", str(error))
            return
        self.comparison_log = comparison
        self.comparison_offset, self.comparison_anchor = offset, anchor
        self.log_corner = -2
        self.draw()

    def clear_comparison(self, _event=None) -> None:
        self.comparison_log = None
        self.comparison_offset = 0.0
        self.comparison_anchor = ""
        if self.run_log is not None:
            self.draw()

    def recorded_colors(self, mode):
        comparison = getattr(self, "comparison_log", None)
        secondary = list(comparison.samples) if comparison is not None else []
        colors, bounds = status_point_colors([*self.status_samples, *secondary], mode)
        count = len(self.status_samples)
        return colors[:count], colors[count:], bounds

    def show_full_program(self, _event=None) -> None:
        if self.run_log is None:
            return
        self.log_corner = -2
        self.draw()

    def update_p_controls(self) -> None:
        enabled = self.run_log is None
        for button in self.buttons[2:6]:  # P and K apply to built-in runs only
            button.ax.set_visible(enabled)

    def set_real_color_mode(self, mode: str) -> None:
        self.real_color_mode = mode
        self.update_color_buttons()
        if self.run_log is not None:
            colors, comparison_colors, color_range = self.recorded_colors(mode)
            if self.status_points_artist is not None:
                self.status_points_artist.set_facecolor(colors)
            if self.status_velocity_artist is not None:
                self.status_velocity_artist.set_facecolor(
                    [colors[index] for index, _, _ in self.status_timed_samples])
            if self.status_hover_source is not None:
                self.status_hover_source.colors = colors
            artists = getattr(self, "comparison_artists", None)
            if artists is not None:
                points, velocity, hover, indices = artists
                points.set_facecolor(comparison_colors)
                if velocity is not None:
                    velocity.set_facecolor([comparison_colors[index] for index in indices])
                if hover is not None:
                    hover.colors = comparison_colors
            lines = [line for line in self.info.get_text().splitlines() if not line.startswith("point color:")]
            color_label = "Z (mm)" if mode == "z" else "path speed (mm/s)"
            color_line = (f"point color: {color_label} unavailable" if color_range is None else
                          f"point color: {color_label} {color_range[0]:g} to {color_range[1]:g}")
            lines.append(color_line)
            self.info.set_text("\n".join(lines))
            self.fig.canvas.draw_idle()

    def update_color_buttons(self) -> None:
        for button, mode in ((self.color_z_button, "z"), (self.color_speed_button, "speed")):
            button.ax.set_facecolor("#f2c94c" if self.real_color_mode == mode else "#eeeeee")

    def log_paths(self) -> list[str]:
        if self.run_log is None:
            return []
        return list(dict.fromkeys(move.path_id for move in self.run_log.moves))

    def selected_log_moves(self) -> tuple[RunMove, ...]:
        run_log = self.run_log
        if run_log is None:
            return ()
        paths = self.log_paths()
        if not paths:
            return ()
        path_id = paths[self.path_index]
        return tuple(move for move in run_log.moves if move.path_id == path_id)

    def step(self, what: str, delta: int) -> None:
        if what == "logpath":
            paths = self.log_paths()
            if paths:
                self.path_index = (self.path_index + delta) % len(paths)
                self.log_corner = -1
        elif what == "corner" and self.run_log is not None:
            vertices = max(0, len(self.selected_log_moves()) - 1)
            self.log_corner = (self.log_corner + 1 + delta) % (vertices + 1) - 1
        elif what == "prog":
            self.prog = (self.prog + delta) % len(self.program_names)
            self.corner = -1
        elif what == "p":
            if self.run_log is not None:
                return
            self.p = (self.p + delta) % len(P_VALUES)
        elif what == "k":
            if self.run_log is not None:
                return
            self.k = (self.k + delta) % len(K_VALUES)
        elif what == "m":
            self.m = (self.m + delta) % len(self.machine_names)
        else:
            vertices = len(self.programs[self.program_names[self.prog]][0]) - 2
            self.corner = (self.corner + 1 + delta) % (vertices + 1) - 1
        self.draw()

    def toggle(self, label: str | None) -> None:
        if label is None:
            return
        self.visible[label] = not self.visible[label]
        self.draw()

    def traces(self) -> dict[str, Trace]:
        key = (self.m, self.prog, self.p, self.k)
        if key not in self.cache:
            self.fig.suptitle("computing ...")
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            axes = self.machines[self.machine_names[self.m]]
            points, feed = self.programs[self.program_names[self.prog]]
            self.cache[key] = compute(axes, points, P_VALUES[self.p], feed, K_VALUES[self.k])
        return self.cache[key]

    def draw(self) -> None:
        if self.run_log is not None:
            self.draw_run_log()
            return
        traces = self.traces()
        points, feed = self.programs[self.program_names[self.prog]]
        tolerance = P_VALUES[self.p]
        self.ax3d.clear()
        self.axv.clear()
        self.reset_hover_sources()
        for name, colour in RUNS:
            if not self.visible[name]:
                continue
            trace = traces[name]
            xs, ys, zs = zip(*trace.xyz)
            self.ax3d.plot(xs, ys, zs, color=colour, linestyle="--" if name == "exact stop" else "-",
                           linewidth=1.0 if name == "exact stop" else 1.6, axlim_clip=True)
            self.axv.plot(trace.t, trace.v, color=colour, linestyle="--" if name == "exact stop" else "-",
                          linewidth=1.0 if name == "exact stop" else 1.6,
                          label=f"{name}: {trace.duration:.4f} s")
            self.add_hover_source(name, trace.t, trace.xyz, trace.v, colour)
        cast(Any, self.ax3d).scatter(*zip(*points), color="k", s=10, axlim_clip=True)
        self.frame(points, tolerance)
        self.ax3d.set_xlabel("X mm")
        self.ax3d.set_ylabel("Y mm")
        self.ax3d.set_zlabel("Z mm")
        self.axv.set_xlabel("time s")
        self.axv.set_ylabel("path speed mm/s")
        self.axv.grid(True, alpha=0.3)
        self.axv.legend(fontsize=8, loc="upper right")
        where = "whole program" if self.corner < 0 else f"corner {self.corner + 1} (box +/- {self.half_box(tolerance):.3g} mm)"
        self.fig.suptitle(f"{self.program_names[self.prog]}   G64 P{tolerance:g} K{K_VALUES[self.k]:g} (constant accel)   F{feed:g}   "
                          f"machine {self.machine_names[self.m]}   [{where}]")
        stop = traces["exact stop"].duration
        lines = [f"{'run':22s} {'time s':>7s} {'vs stop':>7s} {'dev mm':>7s}"]
        for name, _ in RUNS:
            trace = traces[name]
            lines.append(f"{name:22s} {trace.duration:7.4f} {100 * (trace.duration / stop - 1):+6.1f}% {trace.deviation:7.4f}")
        self.info.set_text("\n".join(lines))
        self.fig.canvas.draw_idle()

    def draw_run_log(self) -> None:
        run_log = self.run_log
        if run_log is None:
            return
        paths = self.log_paths()
        if not paths:
            return
        path_id = paths[self.path_index]
        selected_moves = self.selected_log_moves()
        axes = list(run_log.axes) if len(run_log.axes) == 3 else self.machines[self.machine_names[self.m]]
        cache_key = (run_log.run_id, tuple(axes))
        if cache_key not in self.log_cache:
            self.fig.suptitle("computing complete logged run ...")
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            initial_position = run_log.samples[0].xyz if run_log.samples else run_log.moves[0].start
            motion_start = status_motion_start(run_log.samples, run_log.settings) or 0.0
            self.log_cache[cache_key] = compute_logged(axes, run_log.moves, run_log.settings,
                                                       run_log.events, initial_position, motion_start)
        else:
            motion_start = status_motion_start(run_log.samples, run_log.settings) or 0.0
        traces = self.log_cache[cache_key]

        observed = list(run_log.samples)
        self.status_samples = observed
        color_label = "Z (mm)" if self.real_color_mode == "z" else "path speed (mm/s)"
        status_colors, comparison_colors, color_range = self.recorded_colors(self.real_color_mode)
        comparison = getattr(self, "comparison_log", None)
        self.ax3d.clear()
        self.axv.clear()
        self.reset_hover_sources()
        for name, colour in RUNS:
            if not self.visible[name]:
                continue
            trace = traces[name]
            xs, ys, zs = zip(*trace.xyz)
            self.ax3d.plot(xs, ys, zs, color=colour, linestyle="--" if name == "exact stop" else "-",
                           linewidth=1.0 if name == "exact stop" else 1.6, axlim_clip=True)
            self.axv.plot(trace.t, trace.v, color=colour, linestyle="--" if name == "exact stop" else "-",
                          linewidth=1.0 if name == "exact stop" else 1.6,
                          label=f"{name}: {trace.duration:.4f} s")
            self.add_hover_source(name, trace.t, trace.xyz, trace.v, colour)
        if observed:
            obs_x, obs_y, obs_z = zip(*(sample.xyz for sample in observed))
            self.ax3d.plot(obs_x, obs_y, obs_z, color="#777777", linestyle=":", linewidth=0.8,
                           label="observed path", axlim_clip=True)
            self.status_points_artist = cast(Any, self.ax3d).scatter(
                obs_x, obs_y, obs_z, c=status_colors, s=15, label="Run A", axlim_clip=True)
            status_times = [sample.elapsed_ms / 1000.0 for sample in observed]
            self.status_hover_source = self.add_hover_source(
                "Run A" if comparison is not None else "Observed", status_times, list(sample.xyz for sample in observed),
                [sample.path_speed for sample in observed], "black", "D", status_colors)
            self.status_timed_samples = []
            for index, sample in enumerate(observed):
                speed = sample.path_speed
                if speed is not None:
                    self.status_timed_samples.append((index, status_times[index], speed))
            if self.status_timed_samples:
                self.status_velocity_artist = self.axv.scatter(
                    [time for _, time, _ in self.status_timed_samples],
                    [speed for _, _, speed in self.status_timed_samples],
                    c=[status_colors[index] for index, _, _ in self.status_timed_samples], s=14,
                    label=f"Run A: {run_log.run_id}" if comparison is not None else "observed status")
        if comparison is not None:
            self.draw_comparison(comparison, comparison_colors)
        run_points = [run_log.moves[0].start, *(move.end for move in run_log.moves)]
        selected_points = [selected_moves[0].start, *(move.end for move in selected_moves)]
        tolerance = max((move.tolerance or 0.0 for move in selected_moves), default=0.0)
        cast(Any, self.ax3d).scatter(*zip(*run_points), color="black", s=10, axlim_clip=True)
        self.ax3d.plot(*zip(*selected_points), color="tab:cyan", linewidth=2.4, label="selected path",
                   axlim_clip=True)
        frame_points = [point for trace in traces.values() for point in trace.xyz]
        frame_points.extend(sample.xyz for sample in observed)
        if comparison is not None:
            frame_points.extend(sample.xyz for sample in comparison.samples)
        focus_points = list(selected_points)
        if comparison is not None and self.log_corner == -1:
            names = {move.path_name for move in selected_moves}
            for move in comparison.moves:
                if move.path_name in names:
                    focus_points.extend((move.start, move.end))
        self.frame_logged(frame_points, focus_points, tolerance)
        self.ax3d.set_xlabel("X mm")
        self.ax3d.set_ylabel("Y mm")
        self.ax3d.set_zlabel("Z mm")
        self.axv.set_xlabel("elapsed time s")
        self.axv.set_ylabel("path speed mm/s")
        self.axv.grid(True, alpha=0.3)
        self.frame_logged_time(run_log, traces, selected_moves)
        self.axv.legend(fontsize=8, loc="upper right")
        path_name = selected_moves[0].path_name or path_id
        machine_source = "file axis limits" if len(run_log.axes) == 3 else f"selected machine {self.machine_names[self.m]}"
        completion = "" if run_log.ended_normally else "  INCOMPLETE"
        if self.log_corner == -2:
            corner_label = "full program"
        elif self.log_corner == -1:
            corner_label = f"path view {path_name}"
        else:
            corner_label = f"corner {self.log_corner + 1} of {path_name}"
        self.fig.suptitle(f"{run_log.run_id}  {run_log.firmware} {run_log.build_id}  {path_name} "
                          f"({self.path_index + 1}/{len(paths)})  [{corner_label}; {machine_source}]{completion}")
        timing_lines = [f"observed full-run samples: {len(observed)}", f"frame: {run_log.frame}"]
        if comparison is not None:
            self.fig.suptitle(f"A: {run_log.run_id}\nB: {comparison.run_id}   [{corner_label}]{completion}",
                              fontsize=10)
            timing_lines = [f"B shift: {self.comparison_offset:+.3f} s",
                            f"anchor: {self.comparison_anchor}",
                            f"samples A/B: {len(observed)}/{len(comparison.samples)}"]
            if not comparison.ended_normally:
                timing_lines.append("Run B: incomplete capture")
        completed_ids = {event.move_id for event in run_log.events if event.event == "move_complete"}
        without_callback = [move.move_id for move in run_log.moves if move.move_id not in completed_ids]
        if without_callback:
            timing_lines.append(f"run moves without completion callback: {len(without_callback)}")
        model_trace = traces["Blended Constant Accel"]
        timing_lines.append(f"simulated motion delayed: {motion_start:.3f} s")
        if color_range is None:
            timing_lines.append(f"point color: {color_label} unavailable")
        else:
            timing_lines.append(f"point color: {color_label} {color_range[0]:g} to {color_range[1]:g}")
        completion_errors = []
        for event in run_log.events:
            if event.event == "move_complete" and event.move_id in model_trace.move_end_times:
                simulated = model_trace.move_end_times[event.move_id]
                observed_time = event.elapsed_ms / 1000.0
                completion_errors.append(observed_time - simulated)
        if len(completion_errors) > 1:
            subsequent = completion_errors[1:]
            mean_error = sum(subsequent) / len(subsequent)
            peak_error = max(abs(error) for error in subsequent)
            timing_lines.append(f"completion error mean/peak: {mean_error:+.4f}/{peak_error:.4f} s")
        self.info.set_text("\n".join(timing_lines))
        self.fig.canvas.draw_idle()

    def draw_comparison(self, comparison, colors) -> None:
        samples = list(comparison.samples)
        times = [sample.elapsed_ms / 1000.0 + self.comparison_offset for sample in samples]
        positions = [sample.xyz for sample in samples]
        xs, ys, zs = zip(*positions)
        self.ax3d.plot(xs, ys, zs, color="tab:purple", linestyle="--", linewidth=0.8,
                       label="Run B path", axlim_clip=True)
        points = cast(Any, self.ax3d).scatter(xs, ys, zs, c=colors, marker="^", s=22,
                                             edgecolors="tab:purple", linewidths=0.4, axlim_clip=True)
        hover = self.add_hover_source("Run B", times, positions, [sample.path_speed for sample in samples],
                                       "tab:purple", "^", colors)
        timed = [(index, sample.path_speed) for index, sample in enumerate(samples)
                 if sample.path_speed is not None]
        indices = [index for index, _ in timed]
        velocity = None
        if timed:
            velocity = self.axv.scatter([times[index] for index in indices],
                                        [speed for _, speed in timed], c=[colors[index] for index in indices],
                                        marker="^", s=22, edgecolors="tab:purple", linewidths=0.4,
                                        label=f"Run B: {comparison.run_id}")
        self.comparison_artists = (points, velocity, hover, indices)

    def frame_logged(self, run_points, selected_points, tolerance) -> None:
        if self.log_corner == -2:
            low = [min(point[axis] for point in run_points) for axis in range(3)]
            high = [max(point[axis] for point in run_points) for axis in range(3)]
            centre = [(a + b) / 2 for a, b in zip(low, high)]
            half = max(max(b - a for a, b in zip(low, high)) / 2, 1e-3)
            half += max(half, 1.0) * 1e-9
        elif self.log_corner == -1:
            low = [min(point[axis] for point in selected_points) for axis in range(3)]
            high = [max(point[axis] for point in selected_points) for axis in range(3)]
            centre = [(a + b) / 2 for a, b in zip(low, high)]
            half = max(max(b - a for a, b in zip(low, high)) / 2, self.half_box(tolerance))
            half += max(half, 1.0) * 1e-9
        else:
            corner = selected_points[self.log_corner + 1]
            centre = corner
            half = self.half_box(tolerance)
        self.ax3d.set_xlim(centre[0] - half, centre[0] + half)
        self.ax3d.set_ylim(centre[1] - half, centre[1] + half)
        self.ax3d.set_zlim(centre[2] - half, centre[2] + half)
        self.ax3d.set_box_aspect((1, 1, 1))

    def frame_logged_time(self, run_log, traces, selected_moves) -> None:
        comparison = getattr(self, "comparison_log", None)
        comparison_times = ([sample.elapsed_ms / 1000.0 + self.comparison_offset for sample in comparison.samples]
                            if comparison is not None else [])
        full_end = max(
            [sample.elapsed_ms / 1000.0 for sample in run_log.samples]
            + [event.elapsed_ms / 1000.0 for event in run_log.events]
            + [time for trace in traces.values() for time in trace.t]
            + comparison_times
            + [1e-3]
        )
        if self.log_corner == -2:
            self.axv.set_xlim(min([0.0, *comparison_times]), full_end)
            return

        trace = traces["Blended Constant Accel"]
        first_index = next(index for index, move in enumerate(run_log.moves)
                           if move.move_id == selected_moves[0].move_id)
        last_index = next(index for index, move in enumerate(run_log.moves)
                          if move.move_id == selected_moves[-1].move_id)
        start_time = (trace.move_end_times.get(run_log.moves[first_index - 1].move_id, 0.0)
                      if first_index else 0.0)
        end_time = trace.move_end_times.get(selected_moves[-1].move_id, start_time)
        if self.log_corner == -1:
            matching_names = {move.path_name for move in selected_moves}
            for log, offset in ((run_log, 0.0), (comparison, getattr(self, "comparison_offset", 0.0))):
                if log is None:
                    continue
                indices = [index for index, move in enumerate(log.moves) if move.path_name in matching_names]
                if not indices:
                    continue
                completions = {event.move_id: event.elapsed_ms / 1000.0 + offset for event in log.events
                               if event.event == "move_complete"}
                observed_ends = [completions[log.moves[index].move_id] for index in indices
                                 if log.moves[index].move_id in completions]
                if observed_ends:
                    start_time = min(start_time, *observed_ends)
                    end_time = max(end_time, *observed_ends)
                if indices[0] and log.moves[indices[0] - 1].move_id in completions:
                    start_time = min(start_time, completions[log.moves[indices[0] - 1].move_id])
        if end_time < start_time:
            start_time, end_time = end_time, start_time
        if self.log_corner >= 0:
            corner_index = min(first_index + self.log_corner, last_index)
            centre_time = trace.move_end_times.get(run_log.moves[corner_index].move_id,
                                                   (start_time + end_time) / 2)
            span = max(0.05, (end_time - start_time) * 0.08)
            start_time = max(start_time, centre_time - span)
            end_time = min(end_time, centre_time + span)
        padding = max(0.05, (end_time - start_time) * 0.05)
        self.axv.set_xlim(max(0.0, start_time - padding), min(full_end, end_time + padding))

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
    viewer = Viewer()
    setattr(viewer.fig, "_trajectory_viewer", viewer)
    plt.show()


if __name__ == "__main__":
    main()
