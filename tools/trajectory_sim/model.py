"""Immutable scalar records; kinematic feasibility is checked by validation."""

from dataclasses import dataclass
from math import inf, isfinite, isnan


def require_finite(**values: float) -> None:
    for name, value in values.items():
        if not isfinite(value):
            raise ValueError(f"{name} must be finite")


@dataclass(frozen=True, slots=True)
class State:
    """Position (mm), path speed (mm/s), signed acceleration (mm/s^2)."""

    s: float
    v: float
    a: float

    def __post_init__(self) -> None:
        require_finite(s=self.s, v=self.v, a=self.a)

    @classmethod
    def from_grbl(cls, s: float, v: float, a: float) -> "State":
        """Convert grblHAL mm/min and mm/min^2 to simulator units."""
        return cls(s, v / 60.0, a / 3600.0)


@dataclass(frozen=True, slots=True)
class Limits:
    """Path speed cap, acceleration bounds -deceleration <= a <= acceleration, symmetric jerk cap.

    `deceleration=None` tracks `acceleration` (same rule as grblHAL $81x = 0).
    `jerk=inf` means no jerk limit (constant-acceleration / trapezoidal-velocity motion); profiles
    may then contain `AccelerationStep` elements.
    """

    speed: float
    acceleration: float
    jerk: float
    deceleration: float | None = None

    def __post_init__(self) -> None:
        if self.deceleration is None:
            object.__setattr__(self, "deceleration", self.acceleration)
        require_finite(speed=self.speed, acceleration=self.acceleration, deceleration=self.deceleration)
        if isnan(self.jerk):
            raise ValueError("jerk must not be NaN")
        if self.speed < 0 or self.acceleration <= 0 or self.jerk <= 0 or self.deceleration <= 0:
            raise ValueError("speed must be nonnegative; acceleration, deceleration and jerk must be positive")

    @property
    def jerk_limited(self) -> bool:
        return self.jerk != inf

    @classmethod
    def from_grbl(cls, speed: float, acceleration: float, jerk: float, deceleration: float = 0.0) -> "Limits":
        """Convert grblHAL internal minute-based limits; deceleration 0 tracks acceleration."""
        return cls(speed / 60.0, acceleration / 3600.0, jerk / 216000.0, deceleration / 3600.0 if deceleration else None)


@dataclass(frozen=True, slots=True)
class Phase:
    """Constant signed jerk (mm/s^3) for a duration (seconds)."""

    jerk: float
    duration: float

    def __post_init__(self) -> None:
        require_finite(jerk=self.jerk, duration=self.duration)
        if self.duration < 0:
            raise ValueError("phase duration must be nonnegative")


@dataclass(frozen=True, slots=True)
class AccelerationStep:
    """Instantaneous change of acceleration to `acceleration` (mm/s^2); s and v are unchanged.

    Admissible only where jerk is unlimited or an explicit policy waives jerk continuity.
    """

    acceleration: float
    duration: float = 0.0

    def __post_init__(self) -> None:
        require_finite(acceleration=self.acceleration)
        if self.duration != 0.0:
            raise ValueError("an acceleration step has zero duration")


Element = Phase | AccelerationStep


@dataclass(frozen=True, slots=True)
class PathBlock:
    """One piece of path: length (mm), programmed speed cap (mm/s, scaled by feed override).

    `stop_after` forces v = a = 0 at its end (exact stop, reversal, dwell). `limits`, when given, are
    the piece's own path limits (from axis projection, not scaled by override); None uses the planner's.
    `hold_deceleration` is the path deceleration the piece allows when jerk is waived (hold policy) at
    any speed up to its cap; None means `limits.deceleration`.
    """

    source_id: int
    length: float
    speed_cap: float
    stop_after: bool = False
    limits: Limits | None = None
    hold_deceleration: float | None = None

    def __post_init__(self) -> None:
        require_finite(length=self.length, speed_cap=self.speed_cap)
        if self.length <= 0 or self.speed_cap <= 0:
            raise ValueError("block length and speed cap must be positive")


Vector = tuple[float, float, float]


@dataclass(frozen=True, slots=True)
class LineMove:
    """Straight source move to `end` (mm, XYZ). `tolerance` is G64 P (mm) for the corner at its end;
    None means exact stop there (G61)."""

    source_id: int
    end: Vector
    feed: float
    tolerance: float | None = None

    def __post_init__(self) -> None:
        require_finite(x=self.end[0], y=self.end[1], z=self.end[2], feed=self.feed)
        if self.feed <= 0 or (self.tolerance is not None and not (isfinite(self.tolerance) and self.tolerance > 0)):
            raise ValueError("feed must be positive; tolerance positive or None")


@dataclass(frozen=True, slots=True)
class ArcMove:
    """Planar circular source move about `center`, in the plane normal to axis `normal` (0=X, 1=Y, 2=Z),
    counterclockwise about +normal when `ccw` (mapping G2/G3 per plane is the parser's job). end == start
    means a full circle. Joins with a curvature jump are exact stops, so `tolerance` only matters where an
    arc continues the same circle."""

    source_id: int
    end: Vector
    center: Vector
    normal: int
    ccw: bool
    feed: float
    tolerance: float | None = None

    def __post_init__(self) -> None:
        require_finite(x=self.end[0], y=self.end[1], z=self.end[2], feed=self.feed,
                       cx=self.center[0], cy=self.center[1], cz=self.center[2])
        if self.normal not in (0, 1, 2) or self.feed <= 0:
            raise ValueError("normal must be 0, 1 or 2; feed positive")


Move = LineMove | ArcMove


@dataclass(frozen=True, slots=True)
class Segment:
    """Explicit boundary states permit validation of imported/generated records."""

    start: State
    phase: Element
    end: State


@dataclass(frozen=True, slots=True)
class Marker:
    """Source identity at a path position; execution lifecycle belongs to runtime."""

    source_id: int
    s: float

    def __post_init__(self) -> None:
        require_finite(s=self.s)


@dataclass(frozen=True, slots=True)
class Trajectory:
    initial: State
    segments: tuple[Segment, ...]
    markers: tuple[Marker, ...] = ()

    def __post_init__(self) -> None:
        object.__setattr__(self, "segments", tuple(self.segments))
        object.__setattr__(self, "markers", tuple(self.markers))

    @property
    def final(self) -> State:
        return self.segments[-1].end if self.segments else self.initial