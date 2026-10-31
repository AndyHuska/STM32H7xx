"""Reader for the normalized, typed-record trajectory comparison CSV."""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import TextIO

from tools.trajectory_sim.model import Limits, Vector

FIELDS = (
    "record_type", "sequence", "elapsed_ms", "run_id", "path_id", "path_name", "move_id", "event",
    "move_kind", "frame", "schema_version", "firmware", "build_id", "machine_id", "setting_name",
    "setting_value", "axis", "max_speed_mm_s", "max_accel_mm_s2", "max_decel_mm_s2", "max_jerk_mm_s3",
    "start_x_mm", "start_y_mm", "start_z_mm", "end_x_mm", "end_y_mm", "end_z_mm", "feed_mm_s",
    "blend_mode", "blend_tolerance_mm", "stop_after", "dwell_ms", "x_mm", "y_mm", "z_mm",
    "path_speed_mm_s", "blend_k",
)


@dataclass(frozen=True)
class RunMove:
    sequence: int
    elapsed_ms: int
    path_id: str
    path_name: str
    move_id: str
    kind: str
    start: Vector
    end: Vector
    feed: float | None
    blend_mode: str
    tolerance: float | None
    stop_after: bool
    blend_k: float | None = None


@dataclass(frozen=True)
class StatusSample:
    sequence: int
    elapsed_ms: int
    xyz: Vector
    path_speed: float | None


@dataclass(frozen=True)
class RunEvent:
    sequence: int
    elapsed_ms: int
    path_id: str
    move_id: str
    event: str
    dwell_ms: int | None


@dataclass(frozen=True)
class RunLog:
    run_id: str
    frame: str
    firmware: str
    build_id: str
    machine_id: str
    axes: tuple[Limits, ...]
    settings: dict[str, str]
    moves: tuple[RunMove, ...]
    events: tuple[RunEvent, ...]
    samples: tuple[StatusSample, ...]
    ended_normally: bool


def _required(row: dict[str, str], field: str, line: int) -> str:
    value = row.get(field, "").strip()
    if not value:
        raise ValueError(f"line {line}: {field} is required for {row.get('record_type', 'record')}")
    return value


def _integer(row: dict[str, str], field: str, line: int) -> int:
    value = _required(row, field, line)
    try:
        number = int(value)
    except ValueError as error:
        raise ValueError(f"line {line}: {field} must be an integer") from error
    if number < 0:
        raise ValueError(f"line {line}: {field} must be nonnegative")
    return number


def _number(row: dict[str, str], field: str, line: int, *, allow_inf: bool = False) -> float:
    value = _required(row, field, line)
    if allow_inf and value == "inf":
        return math.inf
    try:
        number = float(value)
    except ValueError as error:
        raise ValueError(f"line {line}: {field} must be numeric") from error
    if not math.isfinite(number):
        raise ValueError(f"line {line}: {field} must be finite")
    return number


def _vector(row: dict[str, str], prefix: str, line: int) -> Vector:
    return tuple(_number(row, f"{prefix}{axis}_mm", line) for axis in "xyz")  # type: ignore[return-value]


def parse_run_log(stream: TextIO) -> RunLog:
    """Parse and validate one run log. Raises ValueError with the offending CSV line on invalid input."""
    reader = csv.DictReader(stream)
    if tuple(reader.fieldnames or ()) not in (FIELDS, FIELDS[:-1]):
        raise ValueError("CSV header does not match run-log schema version 1")

    metadata = None
    axes: dict[str, Limits] = {}
    settings: dict[str, str] = {}
    moves: list[RunMove] = []
    events: list[RunEvent] = []
    samples: list[StatusSample] = []
    move_ids: set[str] = set()
    move_paths: dict[str, str] = {}
    completed_moves: set[str] = set()
    path_ids: set[str] = set()
    active_path = ""
    seen_runtime = False
    ended_normally = False
    previous_sequence = -1
    previous_elapsed = -1
    previous_endpoint: Vector | None = None

    for line, row in enumerate(reader, start=2):
        if None in row:
            raise ValueError(f"line {line}: record has more fields than the header")
        if any(value is None for value in row.values()):
            raise ValueError(f"line {line}: record has fewer fields than the header")
        record_type = _required(row, "record_type", line)
        sequence = _integer(row, "sequence", line)
        elapsed_ms = _integer(row, "elapsed_ms", line)
        run_id = _required(row, "run_id", line)
        if sequence != previous_sequence + 1:
            raise ValueError(f"line {line}: sequence must increase by one from zero")
        if elapsed_ms < previous_elapsed:
            raise ValueError(f"line {line}: elapsed_ms must be monotonic")
        previous_sequence, previous_elapsed = sequence, elapsed_ms

        if record_type == "run":
            if metadata is not None or sequence != 0 or elapsed_ms != 0:
                raise ValueError(f"line {line}: run record must be the first data row at sequence/time zero")
            if _integer(row, "schema_version", line) != 1:
                raise ValueError(f"line {line}: unsupported schema_version")
            metadata = (run_id, _required(row, "frame", line), _required(row, "firmware", line),
                        _required(row, "build_id", line), row["machine_id"].strip())
            continue

        if metadata is None:
            raise ValueError(f"line {line}: run record must precede all other records")
        if run_id != metadata[0]:
            raise ValueError(f"line {line}: run_id does not match the run record")

        if record_type in ("setting", "axis_limit"):
            if seen_runtime:
                raise ValueError(f"line {line}: {record_type} records must precede runtime records")
            if record_type == "setting":
                name = _required(row, "setting_name", line)
                if name in settings:
                    raise ValueError(f"line {line}: duplicate setting_name {name!r}")
                settings[name] = _required(row, "setting_value", line)
            else:
                axis = _required(row, "axis", line)
                if axis in axes:
                    raise ValueError(f"line {line}: duplicate axis_limit for {axis!r}")
                speed = _number(row, "max_speed_mm_s", line)
                acceleration = _number(row, "max_accel_mm_s2", line)
                deceleration = _number(row, "max_decel_mm_s2", line)
                jerk = _number(row, "max_jerk_mm_s3", line, allow_inf=True)
                axes[axis] = Limits(speed, acceleration, jerk, deceleration)
            continue

        seen_runtime = True
        if record_type == "move":
            path_id = _required(row, "path_id", line)
            move_id = _required(row, "move_id", line)
            kind = _required(row, "move_kind", line)
            blend_mode = _required(row, "blend_mode", line)
            if not active_path or path_id != active_path:
                raise ValueError(f"line {line}: move {move_id!r} does not belong to the active path")
            if move_id in move_ids:
                raise ValueError(f"line {line}: duplicate move_id {move_id!r}")
            if kind not in ("linear", "rapid"):
                raise ValueError(f"line {line}: unsupported move_kind {kind!r}")
            if blend_mode not in ("exact", "g64"):
                raise ValueError(f"line {line}: blend_mode must be exact or g64")
            start, end = _vector(row, "start_", line), _vector(row, "end_", line)
            if previous_endpoint is not None and any(abs(a - b) > 1e-6 for a, b in zip(start, previous_endpoint)):
                raise ValueError(f"line {line}: move {move_id!r} start does not match preceding move endpoint")
            feed = _number(row, "feed_mm_s", line) if kind == "linear" else None
            tolerance = _number(row, "blend_tolerance_mm", line) if blend_mode == "g64" else None
            blend_k = _number(row, "blend_k", line) if row.get("blend_k", "").strip() else None
            if blend_k is not None and (blend_mode != "g64" or not 0.0 <= blend_k <= 1.0):
                raise ValueError(f"line {line}: blend_k must be 0..1 and only applies to g64 moves")
            if feed is not None and feed <= 0.0:
                raise ValueError(f"line {line}: feed_mm_s must be positive")
            if tolerance is not None and tolerance <= 0.0:
                raise ValueError(f"line {line}: blend_tolerance_mm must be positive")
            stop_value = _required(row, "stop_after", line).lower()
            if stop_value not in ("true", "false"):
                raise ValueError(f"line {line}: stop_after must be true or false")
            moves.append(RunMove(sequence, elapsed_ms, path_id, row["path_name"].strip(), move_id, kind,
                                 start, end, feed, blend_mode, tolerance, stop_value == "true", blend_k))
            move_ids.add(move_id)
            move_paths[move_id] = path_id
            previous_endpoint = end
        elif record_type == "status":
            speed = _number(row, "path_speed_mm_s", line) if row["path_speed_mm_s"].strip() else None
            if speed is not None and speed < 0.0:
                raise ValueError(f"line {line}: path_speed_mm_s must be nonnegative")
            samples.append(StatusSample(sequence, elapsed_ms, _vector(row, "", line), speed))
        elif record_type == "event":
            name = _required(row, "event", line)
            path_id = row["path_id"].strip()
            move_id = row["move_id"].strip()
            if name == "path_start":
                if active_path or not path_id:
                    raise ValueError(f"line {line}: path_start requires no active path and a path_id")
                _required(row, "path_name", line)
                if path_id in path_ids:
                    raise ValueError(f"line {line}: duplicate path_id {path_id!r}")
                path_ids.add(path_id)
                active_path = path_id
            elif name == "path_end":
                _required(row, "path_name", line)
                if not active_path or path_id != active_path:
                    raise ValueError(f"line {line}: path_end does not match the active path")
                active_path = ""
            elif name == "move_complete":
                if move_id not in move_ids or move_id in completed_moves:
                    raise ValueError(f"line {line}: move_complete references an unknown or completed move_id")
                if path_id and path_id != move_paths[move_id]:
                    raise ValueError(f"line {line}: move_complete path_id does not match move {move_id!r}")
                completed_moves.add(move_id)
            elif name in ("dwell_start", "dwell_end", "stop", "run_start", "run_end"):
                pass
            else:
                raise ValueError(f"line {line}: unsupported event {name!r}")
            dwell_ms = _integer(row, "dwell_ms", line) if name == "dwell_start" else None
            events.append(RunEvent(sequence, elapsed_ms, path_id, move_id, name, dwell_ms))
            ended_normally = name == "run_end" or (ended_normally and name != "run_start")
        else:
            raise ValueError(f"line {line}: unsupported record_type {record_type!r}")

    if metadata is None:
        raise ValueError("run log is empty")
    if not moves:
        raise ValueError("run log contains no move records")
    ordered_axes = tuple(axes[name] for name in "XYZ" if name in axes)
    return RunLog(*metadata, ordered_axes, settings, tuple(moves), tuple(events), tuple(samples), ended_normally)


def load_run_log(path: str | Path) -> RunLog:
    """Read a UTF-8 run log from disk."""
    with Path(path).open("r", encoding="utf-8-sig", newline="") as stream:
        return parse_run_log(stream)
