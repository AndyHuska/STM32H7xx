"""Tests for the normalized typed-record run-log reader."""

import csv
import gc
import io
import unittest
import weakref
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

from tools.trajectory_sim.tests.run_log import FIELDS, parse_run_log
from tools.trajectory_sim.tests import view_paths
from tools.trajectory_sim.tests.view_paths import (RUNS, HoverSource, Viewer, compute, compute_logged,
                                                   position_at_time, status_motion_start, status_point_colors,
                                                   value_at_time)

RUN_ID = "test-run"


def rows_for_two_paths():
    rows = [
        {"record_type": "run", "sequence": 0, "elapsed_ms": 0, "run_id": RUN_ID, "frame": "work",
         "schema_version": 1, "firmware": "grblHAL", "build_id": "test-build"},
        {"record_type": "axis_limit", "sequence": 1, "elapsed_ms": 0, "run_id": RUN_ID, "axis": "X",
         "max_speed_mm_s": 200, "max_accel_mm_s2": 1000, "max_decel_mm_s2": 900, "max_jerk_mm_s3": "inf"},
        {"record_type": "axis_limit", "sequence": 2, "elapsed_ms": 0, "run_id": RUN_ID, "axis": "Y",
         "max_speed_mm_s": 200, "max_accel_mm_s2": 1000, "max_decel_mm_s2": 900, "max_jerk_mm_s3": "inf"},
        {"record_type": "axis_limit", "sequence": 3, "elapsed_ms": 0, "run_id": RUN_ID, "axis": "Z",
         "max_speed_mm_s": 80, "max_accel_mm_s2": 500, "max_decel_mm_s2": 450, "max_jerk_mm_s3": "inf"},
        {"record_type": "event", "sequence": 4, "elapsed_ms": 0, "run_id": RUN_ID, "event": "run_start"},
        {"record_type": "event", "sequence": 5, "elapsed_ms": 2, "run_id": RUN_ID, "path_id": "L01",
         "path_name": "Test L01", "event": "path_start"},
        {"record_type": "move", "sequence": 6, "elapsed_ms": 3, "run_id": RUN_ID, "path_id": "L01",
         "path_name": "Test L01", "move_id": "M001", "move_kind": "linear", "frame": "work",
         "start_x_mm": 0, "start_y_mm": 0, "start_z_mm": 0, "end_x_mm": 10, "end_y_mm": 0,
         "end_z_mm": 0, "feed_mm_s": 100, "blend_mode": "g64", "blend_tolerance_mm": 0.2,
         "stop_after": "false"},
        {"record_type": "move", "sequence": 7, "elapsed_ms": 4, "run_id": RUN_ID, "path_id": "L01",
         "path_name": "Test L01", "move_id": "M001B", "move_kind": "linear", "frame": "work",
         "start_x_mm": 10, "start_y_mm": 0, "start_z_mm": 0, "end_x_mm": 11, "end_y_mm": 0,
         "end_z_mm": 0, "feed_mm_s": 100, "blend_mode": "g64", "blend_tolerance_mm": 0.2,
         "stop_after": "false"},
        {"record_type": "status", "sequence": 8, "elapsed_ms": 20, "run_id": RUN_ID, "x_mm": 0,
         "y_mm": 0, "z_mm": 0, "path_speed_mm_s": 0},
        {"record_type": "event", "sequence": 9, "elapsed_ms": 50, "run_id": RUN_ID, "path_id": "L01",
         "path_name": "Test L01", "move_id": "M001", "event": "move_complete"},
        {"record_type": "event", "sequence": 10, "elapsed_ms": 51, "run_id": RUN_ID, "path_id": "L01",
         "path_name": "Test L01", "event": "path_end"},
        {"record_type": "event", "sequence": 11, "elapsed_ms": 52, "run_id": RUN_ID, "path_id": "L02",
         "path_name": "Test L02", "event": "path_start"},
        {"record_type": "move", "sequence": 12, "elapsed_ms": 53, "run_id": RUN_ID, "path_id": "L02",
         "path_name": "Test L02", "move_id": "M002", "move_kind": "rapid", "frame": "work",
         "start_x_mm": 11, "start_y_mm": 0, "start_z_mm": 0, "end_x_mm": 11, "end_y_mm": 5,
         "end_z_mm": 0, "blend_mode": "exact", "stop_after": "true"},
        {"record_type": "status", "sequence": 13, "elapsed_ms": 70, "run_id": RUN_ID, "x_mm": 11,
         "y_mm": 2, "z_mm": 0},
        {"record_type": "event", "sequence": 14, "elapsed_ms": 80, "run_id": RUN_ID, "path_id": "L02",
         "path_name": "Test L02", "move_id": "M002", "event": "move_complete"},
        {"record_type": "event", "sequence": 15, "elapsed_ms": 81, "run_id": RUN_ID, "path_id": "L02",
         "path_name": "Test L02", "event": "path_end"},
        {"record_type": "event", "sequence": 16, "elapsed_ms": 82, "run_id": RUN_ID, "event": "run_end"},
    ]
    return rows


def csv_for_rows(rows):
    stream = io.StringIO()
    writer = csv.DictWriter(stream, fieldnames=FIELDS, lineterminator="\n")
    writer.writeheader()
    writer.writerows(rows)
    return stream.getvalue()


class RunLogTests(unittest.TestCase):
    def test_comparison_aligns_first_shared_path_without_matching_ids(self):
        reference = parse_run_log(io.StringIO(csv_for_rows(rows_for_two_paths())))
        comparison = replace(
            reference, run_id="second-run",
            moves=tuple(replace(move, move_id="B-" + move.move_id, path_id="B-" + move.path_id)
                        for move in reference.moves),
            events=tuple(replace(event, elapsed_ms=event.elapsed_ms + 2500,
                                 move_id="B-" + event.move_id if event.move_id else "",
                                 path_id="B-" + event.path_id if event.path_id else "")
                         for event in reference.events),
            samples=tuple(replace(sample, elapsed_ms=sample.elapsed_ms + 2500)
                          for sample in reference.samples))

        offset, anchor = view_paths.align_recorded_runs(reference, comparison)

        self.assertEqual(offset, -2.5)
        self.assertIn("Test L01", anchor)
        self.assertIn("B-M001", anchor)
        self.assertEqual(comparison.samples[0].elapsed_ms, 2520)
        delayed_tail = replace(comparison, samples=tuple(
            replace(sample, elapsed_ms=sample.elapsed_ms + (500 if index else 0))
            for index, sample in enumerate(comparison.samples)))
        self.assertEqual(view_paths.align_recorded_runs(reference, delayed_tail)[0], offset)
        self.assertAlmostEqual(delayed_tail.samples[-1].elapsed_ms / 1000.0 + offset
                               - reference.samples[-1].elapsed_ms / 1000.0, 0.5)

    def test_comparison_rejects_missing_or_ambiguous_alignment(self):
        reference = parse_run_log(io.StringIO(csv_for_rows(rows_for_two_paths())))
        cases = [
            (replace(reference, events=()), "no corresponding"),
            (replace(reference, frame="different"), "same coordinate frame"),
            (replace(reference, moves=tuple(replace(move, path_name="other") for move in reference.moves)),
             "no matching path names"),
            (replace(reference, moves=tuple(replace(move, path_name="Test L01") for move in reference.moves)),
             "ambiguous"),
        ]
        for comparison, message in cases:
            with self.subTest(message=message), self.assertRaisesRegex(ValueError, message):
                view_paths.align_recorded_runs(reference, comparison)

    def test_comparison_overlay_hover_colors_and_clear(self):
        from matplotlib.backends.backend_agg import FigureCanvasAgg
        from matplotlib.figure import Figure

        figure = Figure(figsize=(15, 8))
        FigureCanvasAgg(figure)
        with patch.object(view_paths.plt, "figure", return_value=figure), patch.object(Viewer, "draw"):
            viewer = Viewer()
        reference = parse_run_log(io.StringIO(csv_for_rows(rows_for_two_paths())))
        viewer.run_log = reference
        viewer.comparison_log = replace(
            reference, run_id="second-run",
            events=tuple(replace(event, elapsed_ms=event.elapsed_ms + 1000) for event in reference.events),
            samples=tuple(replace(sample, elapsed_ms=sample.elapsed_ms + 1000,
                                  xyz=(sample.xyz[0], sample.xyz[1], 500.0))
                          for sample in reference.samples))
        viewer.comparison_offset, viewer.comparison_anchor = view_paths.align_recorded_runs(
            reference, viewer.comparison_log)
        viewer.log_corner = -2
        viewer.draw_run_log()

        self.assertEqual(len(viewer.log_cache), 1)
        self.assertEqual([source.label for source in viewer.hover_sources][-2:], ["Run A", "Run B"])
        self.assertAlmostEqual(viewer.hover_sources[-1].times[0], reference.samples[0].elapsed_ms / 1000.0)
        self.assertGreaterEqual(viewer.ax3d.get_zlim()[1], 500.0)
        viewer.on_plot_hover(SimpleNamespace(inaxes=viewer.axv, xdata=0.03, ydata=20.0))
        self.assertIn("Run A:", viewer.hover_annotation.get_text())
        self.assertIn("Run B:", viewer.hover_annotation.get_text())
        points = viewer.comparison_artists[0]
        cursor = viewer.hover_cursor
        viewer.set_real_color_mode("speed")
        self.assertIs(points, viewer.comparison_artists[0])
        self.assertIs(cursor, viewer.hover_cursor)
        viewer.clear_comparison()
        self.assertIsNone(viewer.comparison_log)
        self.assertIsNone(viewer.comparison_artists)
        self.assertEqual(viewer.hover_sources[-1].label, "Observed")

    def test_logged_k_changes_circular_blend_geometry(self):
        run_log = parse_run_log(io.StringIO(csv_for_rows(rows_for_two_paths())))
        first = replace(run_log.moves[0], start=(0.0, 0.0, 0.0), end=(20.0, 0.0, 0.0),
                        feed=30.0, tolerance=3.0, blend_k=0.0)
        second = replace(first, sequence=first.sequence + 1, move_id="corner-exit",
                         start=first.end, end=(20.0, 20.0, 0.0), blend_k=0.8)
        sized = view_paths.logged_firmware_trace(run_log.axes, (first, second), 0.0,
                                                 arc_limits="adaptive", arc_k=0.9)
        largest = view_paths.logged_firmware_trace(run_log.axes, (first, replace(second, blend_k=0.0)),
                                                   0.0, arc_limits="adaptive", arc_k=0.9)

        radius = 30.0 ** 2 / (0.8 * 1000.0)
        self.assertAlmostEqual(sized.deviation, radius * (2.0 ** 0.5 - 1.0))
        self.assertAlmostEqual(largest.deviation, 3.0)
        self.assertLess(sized.deviation, largest.deviation)
        self.assertIn("corner-exit", sized.move_end_times)

    def test_optional_g64_k_is_preserved_and_validated(self):
        for value in (0.0, 0.8, 1.0):
            with self.subTest(value=value):
                rows = rows_for_two_paths()
                rows[6]["blend_k"] = value
                run_log = parse_run_log(io.StringIO(csv_for_rows(rows)))
                self.assertEqual(run_log.moves[0].blend_k, value)
                self.assertIsNone(run_log.moves[1].blend_k)

        for value in (-0.1, 1.1, "nan", "inf"):
            with self.subTest(invalid=value):
                rows = rows_for_two_paths()
                rows[6]["blend_k"] = value
                with self.assertRaisesRegex(ValueError, "blend_k"):
                    parse_run_log(io.StringIO(csv_for_rows(rows)))

        rows = rows_for_two_paths()
        exact_move = next(row for row in rows if row.get("blend_mode") == "exact")
        exact_move["blend_k"] = 0.8
        with self.assertRaisesRegex(ValueError, "only applies to g64"):
            parse_run_log(io.StringIO(csv_for_rows(rows)))

    def test_original_header_without_g64_k_remains_supported(self):
        stream = io.StringIO()
        writer = csv.DictWriter(stream, fieldnames=FIELDS[:-1])
        writer.writeheader()
        writer.writerows(rows_for_two_paths())

        run_log = parse_run_log(io.StringIO(stream.getvalue()))

        self.assertTrue(all(move.blend_k is None for move in run_log.moves))

    def test_window_retains_viewer_through_garbage_collection(self):
        figure = SimpleNamespace()
        references = []

        class StubViewer:
            def __init__(self):
                self.fig = figure
                references.append(weakref.ref(self))

        def collect_during_show():
            gc.collect()
            self.assertIsNotNone(references[0]())

        with patch.object(view_paths, "Viewer", StubViewer), patch.object(
                view_paths.plt, "show", side_effect=collect_during_show):
            view_paths.main()

        gc.collect()
        self.assertIsNotNone(references[0]())
        self.assertIs(figure._trajectory_viewer, references[0]())

    def test_status_color_scale_runs_from_red_to_green(self):
        samples = [SimpleNamespace(xyz=(0.0, 0.0, 0.0), path_speed=0.0),
                   SimpleNamespace(xyz=(0.0, 0.0, 1.0), path_speed=100.0)]

        for mode, upper in (("z", 1.0), ("speed", 100.0)):
            with self.subTest(mode=mode):
                colors, bounds = status_point_colors(samples, mode)
                self.assertEqual(bounds, (0.0, upper))
                self.assertGreater(colors[0][0], colors[0][1])
                self.assertGreater(colors[1][1], colors[1][0])

    def test_hover_position_interpolates_and_rejects_out_of_range_times(self):
        times = [0.0, 2.0]
        positions = [(0.0, 2.0, 4.0), (10.0, 6.0, 8.0)]
        speeds = [0.0, 20.0]

        self.assertEqual(position_at_time(times, positions, 0.0), positions[0])
        self.assertEqual(position_at_time(times, positions, 1.0), (5.0, 4.0, 6.0))
        self.assertEqual(position_at_time(times, positions, 2.0), positions[1])
        self.assertEqual(value_at_time(times, speeds, 1.0), 10.0)
        self.assertIsNone(position_at_time(times, positions, -0.1))
        self.assertIsNone(position_at_time(times, positions, 2.1))
        self.assertIsNone(value_at_time(times, [0.0, None], 1.0))

    def test_hover_tooltip_reports_interpolated_curve_values(self):
        viewer = Viewer.__new__(Viewer)
        viewer.axv = object()
        viewer.fig = Mock()
        viewer.fig.canvas.draw_idle = Mock()
        viewer.hover_cursor = Mock()
        viewer.hover_annotation = Mock()
        viewer.hover_sources = [HoverSource("trace", [0.0, 1.0], [(0.0, 0.0, 0.0), (10.0, 20.0, 30.0)],
                                            [0.0, 100.0], Mock())]
        viewer.axv = Mock()
        viewer.axv.get_xlim.return_value = (0.0, 1.0)

        viewer.on_plot_hover(SimpleNamespace(inaxes=viewer.axv, xdata=0.5, ydata=50.0))

        viewer.hover_annotation.set_text.assert_called_once_with(
            "Time: 0.500 s\ntrace: X5.00 Y10.00 Z15.00  V50.00 mm/s")
        self.assertEqual(viewer.hover_sources[0].artist._offsets3d, ([5.0], [10.0], [15.0]))

    def test_builtin_and_logged_runs_only_include_selected_variants(self):
        run_log = parse_run_log(io.StringIO(csv_for_rows(rows_for_two_paths())))
        points = [run_log.moves[0].start, *(move.end for move in run_log.moves)]

        self.assertEqual(set(compute(run_log.axes, points, 0.2, 100)), {name for name, _ in RUNS})
        self.assertEqual(set(compute_logged(run_log.axes, run_log.moves, run_log.settings)),
                         {name for name, _ in RUNS})

    def test_spec_header_matches_parser(self):
        spec = Path(__file__).resolve().parents[1] / "RUN_LOG_FORMAT.md"
        header = next(line for line in spec.read_text(encoding="utf-8").splitlines()
                      if line.startswith("record_type,sequence"))

        self.assertEqual(tuple(next(csv.reader([header]))), FIELDS)

    def test_parses_interleaved_two_path_run(self):
        run_log = parse_run_log(io.StringIO(csv_for_rows(rows_for_two_paths())))

        self.assertEqual(run_log.run_id, RUN_ID)
        self.assertEqual(run_log.frame, "work")
        self.assertEqual(len(run_log.axes), 3)
        self.assertEqual([move.path_id for move in run_log.moves], ["L01", "L01", "L02"])
        self.assertEqual(run_log.moves[0].feed, 100)
        self.assertIsNone(run_log.moves[2].feed)
        self.assertEqual(run_log.samples[0].xyz, (0.0, 0.0, 0.0))
        self.assertIsNone(run_log.samples[1].path_speed)
        self.assertEqual(run_log.events[2].event, "move_complete")
        self.assertTrue(run_log.ended_normally)

    def test_rejects_noncontiguous_sequence(self):
        rows = rows_for_two_paths()
        rows[2]["sequence"] = 9

        with self.assertRaisesRegex(ValueError, "sequence must increase"):
            parse_run_log(io.StringIO(csv_for_rows(rows)))

    def test_rejects_move_start_discontinuity(self):
        rows = rows_for_two_paths()
        next_move = next(row for row in rows if row.get("move_id") == "M002")
        next_move["start_y_mm"] = 0.5

        with self.assertRaisesRegex(ValueError, "start does not match"):
            parse_run_log(io.StringIO(csv_for_rows(rows)))

    def test_rejects_completion_for_unknown_move(self):
        rows = rows_for_two_paths()
        completion = next(row for row in rows if row.get("event") == "move_complete")
        completion["move_id"] = "missing"

        with self.assertRaisesRegex(ValueError, "unknown or completed"):
            parse_run_log(io.StringIO(csv_for_rows(rows)))

    def test_rejects_header_mismatch(self):
        text = csv_for_rows(rows_for_two_paths()).replace("path_speed_mm_s", "wrong_field", 1)

        with self.assertRaisesRegex(ValueError, "header does not match"):
            parse_run_log(io.StringIO(text))

    def test_simulates_per_move_controls_and_rapid_feed(self):
        rows = rows_for_two_paths()
        for row in rows[9:]:
            row["sequence"] += 1
        rows.insert(9, {"record_type": "status", "sequence": 9, "elapsed_ms": 30, "run_id": RUN_ID,
                        "x_mm": 1, "y_mm": 0, "z_mm": 0, "path_speed_mm_s": 40})
        run_log = parse_run_log(io.StringIO(csv_for_rows(rows)))
        motion_start = status_motion_start(run_log.samples, run_log.settings)
        self.assertAlmostEqual(motion_start, 0.03)

        traces = compute_logged(run_log.axes, run_log.moves, run_log.settings, start=run_log.samples[0].xyz,
                                motion_start=motion_start)

        self.assertEqual(set(traces), {"exact stop", "Blended Constant Accel", "Blended S-Curve"})
        self.assertTrue(all(trace.duration > 0.0 and trace.xyz for trace in traces.values()))
        for trace in traces.values():
            self.assertEqual(trace.xyz[0], run_log.samples[0].xyz)
            self.assertEqual(trace.t[:2], [0.0, motion_start])
            self.assertEqual(trace.v[:2], [0.0, 0.0])
            self.assertIn("M002", trace.move_end_times)
            self.assertGreater(trace.move_end_times["M002"], trace.move_end_times["M001"])

    def test_path_and_corner_navigation_are_scoped(self):
        viewer = Viewer.__new__(Viewer)
        viewer.run_log = parse_run_log(io.StringIO(csv_for_rows(rows_for_two_paths())))
        viewer.path_index = 0
        viewer.log_corner = -1
        viewer.draw = Mock()

        viewer.step("corner", 1)
        self.assertEqual(viewer.log_corner, 0)
        viewer.step("logpath", 1)
        self.assertEqual(viewer.selected_log_moves()[0].path_id, "L02")
        self.assertEqual(viewer.log_corner, -1)
        viewer.show_full_program()
        self.assertEqual(viewer.log_corner, -2)
        viewer.step("logpath", -1)
        self.assertEqual(viewer.log_corner, -1)

    def test_global_p_step_does_not_redraw_a_loaded_run(self):
        viewer = Viewer.__new__(Viewer)
        viewer.run_log = parse_run_log(io.StringIO(csv_for_rows(rows_for_two_paths())))
        viewer.p = 2
        viewer.draw = Mock()

        viewer.step("p", 1)

        self.assertEqual(viewer.p, 2)
        viewer.draw.assert_not_called()

    def test_color_switch_updates_status_artists_without_redraw(self):
        viewer = Viewer.__new__(Viewer)
        viewer.run_log = parse_run_log(io.StringIO(csv_for_rows(rows_for_two_paths())))
        viewer.real_color_mode = "z"
        viewer.status_samples = list(viewer.run_log.samples)
        viewer.status_points_artist = Mock()
        viewer.status_velocity_artist = Mock()
        viewer.status_timed_samples = [(0, 0.02, 0.0)]
        viewer.status_hover_source = SimpleNamespace(colors=["old"])
        viewer.color_z_button = Mock()
        viewer.color_speed_button = Mock()
        viewer.info = Mock()
        viewer.info.get_text.return_value = "observed samples: 1\npoint color: Z (mm) 0 to 0"
        viewer.fig = Mock()
        viewer.draw = Mock()

        viewer.set_real_color_mode("speed")

        self.assertEqual(viewer.real_color_mode, "speed")
        self.assertNotEqual(viewer.status_hover_source.colors, ["old"])
        viewer.status_points_artist.set_facecolor.assert_called_once()
        viewer.status_velocity_artist.set_facecolor.assert_called_once()
        viewer.fig.canvas.draw_idle.assert_called_once()
        viewer.draw.assert_not_called()

    def test_dwell_forces_stop_and_extends_simulated_time(self):
        stopped_rows = rows_for_two_paths()
        stopped_rows[6]["stop_after"] = "true"
        stopped_run = parse_run_log(io.StringIO(csv_for_rows(stopped_rows)))
        stopped_traces = compute_logged(stopped_run.axes, stopped_run.moves, stopped_run.settings)

        rows = rows_for_two_paths()
        for row in rows[7:]:
            row["sequence"] += 1
        rows.insert(7, {"record_type": "event", "sequence": 7, "elapsed_ms": 4, "run_id": RUN_ID,
                        "event": "dwell_start", "dwell_ms": 100})
        run_log = parse_run_log(io.StringIO(csv_for_rows(rows)))
        traces = compute_logged(run_log.axes, run_log.moves, run_log.settings, run_log.events)

        self.assertFalse(run_log.moves[0].stop_after)
        self.assertTrue(all(abs(trace.duration - 0.1 - stopped_traces[name].duration) < 1e-9
                            for name, trace in traces.items()))


if __name__ == "__main__":
    unittest.main()
