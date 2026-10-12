"""Layer registry and inter-layer contracts for the trajectory simulator.

Every package module belongs to exactly one layer and may import only the sibling
modules and third-party packages its layer lists; tests/test_architecture.py enforces
this so each layer can be replaced, evaluated, and later ported in isolation.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Protocol, Sequence, runtime_checkable

from .model import Element, Limits, Move, PathBlock, State, Vector


@dataclass(frozen=True, slots=True)
class Layer:
    name: str
    level: int
    modules: tuple[str, ...]
    may_import: tuple[str, ...]
    third_party: tuple[str, ...]
    contract: str
    implemented: bool


LAYERS: tuple[Layer, ...] = (
    Layer("L0 data model", 0, ("model",), (), (),
          "Immutable units-explicit records (state, limits, phase, segment, marker). Input finiteness only; no kinematics.", True),
    Layer("C contracts", 0, ("contracts",), ("model",), (),
          "Layer registry, result/status types, and Protocols implemented by interchangeable layer implementations.", True),
    Layer("V validation", 0, ("validation",), ("model",), (),
          "Independent certificate checks; must not import any production computation layer.", True),
    Layer("L1 kinematics", 1, ("phases", "numerics"), ("model",), ("numpy",),
          "Exact constant-jerk propagation and extrema (float64); float32 mirror for target-precision studies.", True),
    Layer("L2 scalar time law", 2, ("scalar_solver",), ("model", "contracts", "phases"), (),
          "ScalarTimeLaw: 1-DoF jerk-limited transitions, stopping, distance-constrained moves with explicit failure status.", True),
    Layer("L3 path geometry", 3, ("geometry",), ("model", "contracts"), (),
          "q(s) and derivatives in arc length, G64 blends with deviation certificates, source markers.", True),
    Layer("L4 constraint projection", 4, ("constraints",), ("model", "contracts", "phases", "geometry"), (),
          "Per-axis limits along q(s) mapped to path-speed/acceleration/jerk envelopes (M4: constant per piece).", True),
    Layer("L5 lookahead", 5, ("lookahead",), ("model", "contracts", "phases", "scalar_solver", "constraints"), (),
          "Lookahead: finite-horizon feasible scheduling that always ends at rest at the queue end (M3: straight path).", True),
    Layer("L6 runtime", 6, ("runtime",), ("model", "contracts", "phases", "lookahead"), (),
          "Streaming, commit horizon, override, hold/cancel, resume over committed elements (markers in M5).", True),
    Layer("L7 executor", 7, ("executor",), ("model", "contracts", "phases", "geometry"), ("numpy",),
          "Committed phases to axis steps: ideal step crossings and firmware-like segment/timer model.", False),
    Layer("L8 front end", 8, ("frontend",),
          ("model", "contracts", "phases", "geometry", "constraints", "lookahead", "runtime"), (),
          "Feeds source moves through geometry, projection and runtime; decides each corner (blend only if faster "
          "than an exact stop and acceptable to the runtime).", True),
)


def layer_of(module: str) -> Layer:
    matches = [layer for layer in LAYERS if module in layer.modules]
    if len(matches) != 1:
        raise KeyError(f"module {module!r} must be registered in exactly one layer")
    return matches[0]


class SolveStatus(Enum):
    OK = "ok"
    INVALID_INPUT = "invalid_input"
    SPEED_LIMIT = "speed_limit"
    REVERSE_PROGRESS = "reverse_progress"
    INSUFFICIENT_DISTANCE = "insufficient_distance"
    NO_CONVERGENCE = "no_convergence"


@dataclass(frozen=True, slots=True)
class SolveResult:
    """Phases start at `start`. On SPEED_LIMIT, REVERSE_PROGRESS and INSUFFICIENT_DISTANCE
    the phases are the witness profile demonstrating the failure, so callers can verify it.

    `proven_infeasible` is True only when no admissible profile exists at all; False means only
    this implementation's profile family failed, so another method may still succeed.
    """

    status: SolveStatus
    start: State
    phases: tuple[Element, ...] = ()
    detail: str = ""
    iterations: int = 0
    proven_infeasible: bool = False

    @property
    def ok(self) -> bool:
        return self.status is SolveStatus.OK

    @property
    def duration(self) -> float:
        return sum(phase.duration for phase in self.phases)


@runtime_checkable
class ScalarTimeLaw(Protocol):
    """L2 contract. On OK, propagating `phases` from `start` reaches the requested target
    within the implementation's declared tolerance, with 0 <= v <= limits.speed,
    -limits.deceleration <= a <= limits.acceleration and |j| <= limits.jerk over every full phase.
    `AccelerationStep`s appear only when `limits.jerk` is infinite."""

    def velocity_transition(self, start: State, v1: float, a1: float, limits: Limits) -> SolveResult:
        """Reach speed v1 with acceleration a1; distance is unconstrained."""
        ...

    def stop(self, start: State, limits: Limits) -> SolveResult:
        """Reach v = 0, a = 0 in minimum time; its distance is the braking reserve."""
        ...

    def distance_transition(self, start: State, goal: State, limits: Limits) -> SolveResult:
        """Reach goal (s, v, a) exactly."""
        ...

    def minimum_distance(self, start: State, v1: float, a1: float, limits: Limits) -> float | None:
        """Distance this implementation needs to reach (v1, a1); None if its profiles cannot.
        `distance_transition` must succeed for any distance >= this (up to the speed cap)."""
        ...


@dataclass(frozen=True, slots=True)
class PlanResult:
    """L5 output. On OK `elements` start at `start` and end at rest at the end of the last block."""

    status: SolveStatus
    start: State
    elements: tuple[Element, ...] = ()
    boundary_speeds: tuple[float, ...] = ()
    transient_end: float | None = None
    detail: str = ""

    @property
    def ok(self) -> bool:
        return self.status is SolveStatus.OK

    @property
    def duration(self) -> float:
        return sum(element.duration for element in self.elements)


@runtime_checkable
class Lookahead(Protocol):
    """L5 contract: plan from `start` over `blocks` (beginning at start.s) to rest at their end, obeying
    limits and each block's cap min(speed_cap * override, limits.speed), except inside an override
    transient that ends at `transient_end`. Equal-cap source boundaries must not alter the profile."""

    def plan(self, start: State, blocks: Sequence[PathBlock], limits: Limits, override: float = 1.0) -> PlanResult:
        ...


@dataclass(frozen=True, slots=True)
class DerivativeBounds:
    """Certified per-axis upper bounds of |q'|, |q''|, |q'''| over a whole piece (arc-length parameter)."""

    first: Vector
    second: Vector
    third: Vector


@runtime_checkable
class PathPiece(Protocol):
    """L3 primitive parameterised by arc length u in [0, length]: |q'(u)| = 1."""

    source_id: int
    feed: float
    length: float

    def evaluate(self, u: float) -> tuple[Vector, Vector, Vector, Vector]:
        """(q, q', q'', q''') at u."""
        ...

    def bounds(self) -> DerivativeBounds:
        ...


@dataclass(frozen=True, slots=True)
class Corner:
    """Decision and certificate for the junction at the end of source move `source_id`.

    kind: "blend" (curvature-continuous blend replaces the vertex), "continuous" (q, q', q'' already
    continuous), "stop" (exact stop at the vertex). `deviation` bounds the Hausdorff distance between
    the blend and the polyline it replaces; `s` is where the source boundary marker lies on the path.
    """

    source_id: int
    vertex: Vector
    kind: str
    s: float
    deviation: float = 0.0
    detail: str = ""


@dataclass(frozen=True, slots=True)
class Revision:
    """L3 output for one appended move: replace the path from `from_s` on with `pieces` (each followed by a
    stop when `stops[k]`); `stop_at_from` is the stop flag at from_s itself. The new path end is a stop."""

    status: SolveStatus
    from_s: float
    stop_at_from: bool = False
    pieces: tuple[PathPiece, ...] = ()
    stops: tuple[bool, ...] = ()
    corner: Corner | None = None
    move: Move | None = None
    detail: str = ""

    @property
    def ok(self) -> bool:
        return self.status is SolveStatus.OK


@runtime_checkable
class PathGeometry(Protocol):
    """L3 contract: streaming geometry. `propose` is pure; nothing before `committed_s` may change. With
    blend=False every non-continuous junction is an exact stop (always admissible for the runtime)."""

    def propose(self, move: Move, committed_s: float, blend: bool = True) -> Revision:
        ...

    def accept(self, revision: Revision) -> None:
        ...

    def pieces(self) -> list[tuple[float, PathPiece, bool]]:
        """Current path: (start s, piece, stop after) in order."""
        ...


@runtime_checkable
class ConstraintProjection(Protocol):
    """L4 contract: constant path limits for a piece such that every state with 0 <= v <= speed,
    -deceleration <= a <= acceleration, |j| <= jerk keeps each axis within its limits. `share` in (0, 1]
    trades acceleration/jerk on curved pieces for speed; it has no effect on straight pieces."""

    def path_limits(self, bounds: DerivativeBounds, axes: Sequence[Limits], share: float = 0.5) -> Limits:
        ...


@runtime_checkable
class FrontEnd(Protocol):
    """L8 contract: accept one source move; returns the accepted revision. Never changes committed motion;
    with the same limits, a G64 corner is used only where it is faster than an exact stop."""

    def feed(self, move: Move) -> Revision:
        ...
