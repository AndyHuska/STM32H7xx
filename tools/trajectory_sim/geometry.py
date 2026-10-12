"""L3 path geometry: arc-length pieces (line, arc, clothoid), G64 corner blends with certificates, streaming.

Design, derivations and the deviation proof: DESIGN.md section 3C. Units: mm.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .contracts import Corner, DerivativeBounds, Revision, SolveStatus
from .model import ArcMove, LineMove, Move, Vector

JOIN_TOLERANCE = 1e-9         # mm, rad and 1/mm: rounding-level mismatch still counted as continuous
REVERSAL_MARGIN = 1e-9        # rad: deflections within this of pi are reversals (exact stop)
MIN_BLEND_LENGTH = 1e-9       # mm: shorter blends are not certifiable at float resolution
ARC_REFIT_LIMIT = 0.005       # mm: largest centre correction accepted for inconsistent arc radii
SERIES_TOLERANCE = 1e-18      # relative truncation of the clothoid power series
BLEND_SEGMENTS = 4            # sub-pieces per blend half


def _add(a: Vector, b: Vector) -> Vector:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _sub(a: Vector, b: Vector) -> Vector:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _scale(k: float, a: Vector) -> Vector:
    return (k * a[0], k * a[1], k * a[2])


def _dot(a: Vector, b: Vector) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a: Vector, b: Vector) -> Vector:
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _norm(a: Vector) -> float:
    return math.sqrt(_dot(a, a))


def _unit(a: Vector) -> Vector:
    return _scale(1.0 / _norm(a), a)


def _combine(c1: float, a: Vector, c2: float, b: Vector) -> Vector:
    return (c1 * a[0] + c2 * b[0], c1 * a[1] + c2 * b[1], c1 * a[2] + c2 * b[2])


def sinusoid_max(a: float, b: float, low: float, high: float) -> float:
    """max |a cos(t) + b sin(t)| over t in [low, high]: endpoints plus interior stationary points."""
    values = [abs(a * math.cos(t) + b * math.sin(t)) for t in (low, high)]
    amplitude = math.hypot(a, b)
    if amplitude > 0.0:
        phase = math.atan2(b, a)
        k = math.ceil((low - phase) / math.pi)
        if phase + k * math.pi <= high:
            values.append(amplitude)
    return max(values)


def fresnel(c: float, sigma: float) -> tuple[float, float]:
    """(X, Y) = integral_0^sigma (cos, sin)(c u^2 / 2) du by its power series (converges for any x)."""
    x = 0.5 * c * sigma * sigma
    real = imag = 0.0
    power = 1.0  # x^n / n!
    n = 0
    while True:
        term = power / (2 * n + 1)
        sign = -1.0 if (n // 2) % 2 else 1.0
        if n % 2 == 0:
            real += sign * term
        else:
            imag += sign * term
        n += 1
        power *= x / n
        if n > x and power <= SERIES_TOLERANCE:
            return sigma * real, sigma * imag


@dataclass(frozen=True, slots=True)
class LinePiece:
    source_id: int
    feed: float
    start: Vector
    direction: Vector
    length: float

    def evaluate(self, u: float) -> tuple[Vector, Vector, Vector, Vector]:
        zero = (0.0, 0.0, 0.0)
        return _add(self.start, _scale(u, self.direction)), self.direction, zero, zero

    def bounds(self) -> DerivativeBounds:
        return DerivativeBounds(tuple(abs(c) for c in self.direction), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))


@dataclass(frozen=True, slots=True)
class ArcPiece:
    """Circle of `radius` about `center` in the plane spanned by orthonormal e1, e2; angle phi0 + sign*u/r."""

    source_id: int
    feed: float
    center: Vector
    radius: float
    e1: Vector
    e2: Vector
    phi0: float
    sign: float
    length: float

    def evaluate(self, u: float) -> tuple[Vector, Vector, Vector, Vector]:
        theta = self.phi0 + self.sign * u / self.radius
        cos, sin = math.cos(theta), math.sin(theta)
        radial = _combine(cos, self.e1, sin, self.e2)
        tangent = _scale(self.sign, _combine(-sin, self.e1, cos, self.e2))
        return (_add(self.center, _scale(self.radius, radial)), tangent,
                _scale(-1.0 / self.radius, radial), _scale(-1.0 / self.radius ** 2, tangent))

    def bounds(self) -> DerivativeBounds:
        end = self.phi0 + self.sign * self.length / self.radius
        low, high = min(self.phi0, end), max(self.phi0, end)
        radial = tuple(sinusoid_max(self.e1[i], self.e2[i], low, high) for i in range(3))
        tangent = tuple(sinusoid_max(self.e2[i], -self.e1[i], low, high) for i in range(3))
        return DerivativeBounds(tangent, tuple(r / self.radius for r in radial), tuple(t / self.radius ** 2 for t in tangent))


@dataclass(frozen=True, slots=True)
class ClothoidPiece:
    """Part sigma in [sigma_from, sigma_to] (traversed in that order, so reversed when sigma_to < sigma_from) of
    the clothoid with curvature c*sigma from `origin` along t0, turning toward n0."""

    source_id: int
    feed: float
    origin: Vector
    t0: Vector
    n0: Vector
    sharpness: float
    sigma_from: float
    sigma_to: float

    @property
    def length(self) -> float:
        return abs(self.sigma_to - self.sigma_from)

    @property
    def reverse(self) -> bool:
        return self.sigma_to < self.sigma_from

    def _canonical(self, sigma: float) -> tuple[Vector, Vector, Vector, Vector]:
        c = self.sharpness
        psi = 0.5 * c * sigma * sigma
        cos, sin = math.cos(psi), math.sin(psi)
        x, y = fresnel(c, sigma)
        tangent = _combine(cos, self.t0, sin, self.n0)
        normal = _combine(-sin, self.t0, cos, self.n0)
        kappa = c * sigma
        return (_add(self.origin, _combine(x, self.t0, y, self.n0)), tangent, _scale(kappa, normal),
                _combine(c, normal, -kappa * kappa, tangent))

    def evaluate(self, u: float) -> tuple[Vector, Vector, Vector, Vector]:
        if not self.reverse:
            return self._canonical(self.sigma_from + u)
        q, d1, d2, d3 = self._canonical(self.sigma_from - u)
        return q, _scale(-1.0, d1), d2, _scale(-1.0, d3)

    def bounds(self) -> DerivativeBounds:
        low, high = sorted((self.sigma_from, self.sigma_to))
        psi_low, psi_high = 0.5 * self.sharpness * low * low, 0.5 * self.sharpness * high * high
        tangent = tuple(sinusoid_max(self.t0[i], self.n0[i], psi_low, psi_high) for i in range(3))
        normal = tuple(sinusoid_max(self.n0[i], -self.t0[i], psi_low, psi_high) for i in range(3))
        kappa = self.sharpness * high
        return DerivativeBounds(tangent, tuple(kappa * n for n in normal),
                                tuple(self.sharpness * n + kappa * kappa * t for n, t in zip(normal, tangent)))


Piece = LinePiece | ArcPiece | ClothoidPiece


@dataclass(frozen=True, slots=True)
class Blend:
    pieces: tuple[ClothoidPiece, ...]
    trim: float
    half_length: float
    deviation: float


def blend_ratios(deflection: float) -> tuple[float, float, float]:
    """For a symmetric clothoid pair of half length 1 turning `deflection`: (trim, deviation, series error)."""
    cs, sn = fresnel(deflection, 1.0)
    error = 1e-13  # series truncation (1e-18) plus rounding of ~30 terms, generously
    half = 0.5 * deflection
    return cs + sn * math.tan(half), (sn + error) / math.cos(half), error


def corner_blend(vertex: Vector, t_in: Vector, t_out: Vector, tolerance: float, max_trim: float,
                 source_in: int, source_out: int, feed: float) -> Blend | str:
    """Symmetric clothoid pair replacing the vertex, trim <= max_trim, certified Hausdorff deviation <= tolerance."""
    deflection = math.atan2(_norm(_cross(t_in, t_out)), _dot(t_in, t_out))
    if deflection >= math.pi - REVERSAL_MARGIN:
        return "reversal"
    trim_ratio, deviation_ratio, _ = blend_ratios(deflection)
    half_length = min(tolerance / deviation_ratio, max_trim / trim_ratio)
    if not half_length >= MIN_BLEND_LENGTH:
        return f"blend half length {half_length:.3g} mm below {MIN_BLEND_LENGTH} mm"
    trim = half_length * trim_ratio
    n_in = _unit(_combine(1.0, t_out, -_dot(t_out, t_in), t_in))
    bisector = _unit(_sub(t_out, t_in))
    mirrored = _combine(2.0 * _dot(n_in, bisector), bisector, -1.0, n_in)
    sharpness = deflection / half_length ** 2
    p_in, p_out = _sub(vertex, _scale(trim, t_in)), _add(vertex, _scale(trim, t_out))
    # Sub-pieces let per-piece limits follow curvature c*sigma instead of its maximum at the apex.
    cuts = [half_length * k / BLEND_SEGMENTS for k in range(BLEND_SEGMENTS + 1)]
    first = tuple(ClothoidPiece(source_in, feed, p_in, t_in, n_in, sharpness, a, b) for a, b in zip(cuts, cuts[1:]))
    second = tuple(ClothoidPiece(source_out, feed, p_out, _scale(-1.0, t_out), mirrored, sharpness, b, a)
                   for a, b in reversed(list(zip(cuts, cuts[1:]))))
    return Blend(first + second, trim, half_length, half_length * deviation_ratio)


def join_gap(before: Piece, after: Piece) -> tuple[float, float, float]:
    """Position, tangent and curvature-vector mismatch between the end of `before` and the start of `after`."""
    q0, d0, k0, _ = before.evaluate(before.length)
    q1, d1, k1, _ = after.evaluate(0.0)
    return _norm(_sub(q0, q1)), _norm(_sub(d0, d1)), _norm(_sub(k0, k1))


def arc_piece(start: Vector, move: ArcMove) -> ArcPiece | str:
    k = move.normal
    i, j = (k + 1) % 3, (k + 2) % 3
    if abs(move.end[k] - start[k]) > JOIN_TOLERANCE:
        return "helical arcs are not supported"
    e1 = tuple(1.0 if axis == i else 0.0 for axis in range(3))
    e2 = tuple(1.0 if axis == j else 0.0 for axis in range(3))
    center = tuple(start[k] if axis == k else move.center[axis] for axis in range(3))
    chord = _sub(move.end, start)
    full = _norm(chord) <= JOIN_TOLERANCE
    if not full:
        # Refit the centre onto the chord bisector so both endpoints lie on one circle.
        middle = _add(start, _scale(0.5, chord))
        across = _unit(_cross(tuple(1.0 if axis == k else 0.0 for axis in range(3)), chord))
        refit = _add(middle, _scale(_dot(_sub(center, middle), across), across))
        if _norm(_sub(refit, center)) > ARC_REFIT_LIMIT:
            return f"arc radius mismatch moves the centre {_norm(_sub(refit, center)):.3g} mm"
        center = refit
    radius = _norm(_sub(start, center))
    if not radius > JOIN_TOLERANCE:
        return "zero arc radius"
    a0 = math.atan2(start[j] - center[j], start[i] - center[i])
    a1 = math.atan2(move.end[j] - center[j], move.end[i] - center[i])
    sweep = 2.0 * math.pi if full else ((a1 - a0) if move.ccw else (a0 - a1)) % (2.0 * math.pi)
    return ArcPiece(move.source_id, move.feed, center, radius, e1, e2, a0, 1.0 if move.ccw else -1.0, radius * sweep)


@dataclass(frozen=True, slots=True)
class _Entry:
    start: float
    piece: Piece
    stop_after: bool


class GeometryStream:
    """Reference implementation of contracts.PathGeometry for XYZ lines and planar arcs."""

    def __init__(self, start: Vector) -> None:
        self.origin = start
        self.entries: list[_Entry] = []
        self.corners: list[Corner] = []
        self.markers: dict[int, float] = {}
        self.pending: list[int] = []  # sources whose boundary marker is the current path end
        self.last_move: Move | None = None

    @property
    def end_s(self) -> float:
        return self.entries[-1].start + self.entries[-1].piece.length if self.entries else 0.0

    @property
    def end_point(self) -> Vector:
        if not self.entries:
            return self.origin
        last = self.entries[-1].piece
        return last.evaluate(last.length)[0]

    def pieces(self) -> list[tuple[float, Piece, bool]]:
        return [(entry.start, entry.piece, entry.stop_after) for entry in self.entries]

    def propose(self, move: Move, committed_s: float, blend: bool = True) -> Revision:
        start, end_s = self.end_point, self.end_s
        previous = self.last_move
        if isinstance(move, LineMove):
            chord = _sub(move.end, start)
            length = _norm(chord)
            if length <= JOIN_TOLERANCE:
                # No motion; a stop request (here or pending) stays in force at the path end.
                stop = move.tolerance is None or (previous is not None and previous.tolerance is None)
                return Revision(SolveStatus.OK, end_s, stop, move=move, detail="zero-length move")
            new: Piece = LinePiece(move.source_id, move.feed, start, _scale(1.0 / length, chord), length)
        else:
            new = arc_piece(start, move)
            if isinstance(new, str):
                return Revision(SolveStatus.INVALID_INPUT, end_s, detail=f"source {move.source_id}: {new}")
        last = self.entries[-1].piece if self.entries else None
        if previous is None or last is None:
            return Revision(SolveStatus.OK, end_s, False, (new,), (False,), move=move)
        vertex = start
        if previous.tolerance is None:
            return self._stop(previous, move, vertex, end_s, new, "exact stop requested")
        _, tangent, curvature = join_gap(last, new)
        if tangent <= JOIN_TOLERANCE and curvature <= JOIN_TOLERANCE:
            corner = Corner(previous.source_id, vertex, "continuous", end_s)
            return Revision(SolveStatus.OK, end_s, False, (new,), (False,), corner, move)
        if not (isinstance(last, LinePiece) and isinstance(new, LinePiece)):
            return self._stop(previous, move, vertex, end_s, new, "curvature jump at an arc join")
        if not blend:
            return self._stop(previous, move, vertex, end_s, new, "blend not admissible for the runtime")
        max_trim = min(last.length, 0.5 * new.length, end_s - committed_s)
        result = corner_blend(vertex, last.direction, new.direction, previous.tolerance, max_trim,
                              previous.source_id, move.source_id, min(previous.feed, move.feed))
        if isinstance(result, str):
            return self._stop(previous, move, vertex, end_s, new, result)
        rest = new.length - result.trim
        pieces: tuple[Piece, ...] = result.pieces
        if rest > 0.0:
            pieces += (LinePiece(move.source_id, move.feed, _add(new.start, _scale(result.trim, new.direction)), new.direction, rest),)
        # Never before the incoming line's start (that piece may be a blend); a rounding-level remainder of
        # the line is consumed rather than left as a micro piece.
        from_s = max(self.entries[-1].start, end_s - result.trim)
        if from_s - self.entries[-1].start <= JOIN_TOLERANCE:
            from_s = self.entries[-1].start
        corner = Corner(previous.source_id, vertex, "blend", from_s + result.half_length, result.deviation)
        return Revision(SolveStatus.OK, from_s, False, pieces, (False,) * len(pieces), corner, move)

    @staticmethod
    def _stop(previous: Move, move: Move, vertex: Vector, end_s: float, new: Piece, detail: str) -> Revision:
        corner = Corner(previous.source_id, vertex, "stop", end_s, detail=detail)
        return Revision(SolveStatus.OK, end_s, True, (new,), (False,), corner, move)

    def accept(self, revision: Revision) -> None:
        if not revision.ok or revision.move is None:
            raise ValueError(f"cannot accept a failed revision: {revision.detail}")
        kept: list[_Entry] = []
        for entry in self.entries:
            end = entry.start + entry.piece.length
            if end <= revision.from_s:
                kept.append(entry)
            elif entry.start < revision.from_s:
                piece = entry.piece
                if not isinstance(piece, LinePiece):
                    raise ValueError("only line pieces can be trimmed")
                kept.append(_Entry(entry.start, LinePiece(piece.source_id, piece.feed, piece.start, piece.direction,
                                                          revision.from_s - entry.start), False))
        if kept:
            # A stop already at from_s (e.g. G61 vertex where a blend consumes the next line) is kept;
            # trimmed entries were created without a stop.
            kept[-1] = _Entry(kept[-1].start, kept[-1].piece, revision.stop_at_from or kept[-1].stop_after)
        s = revision.from_s
        for piece, stop in zip(revision.pieces, revision.stops):
            kept.append(_Entry(s, piece, stop))
            s += piece.length
        self.entries = kept
        move = revision.move
        if revision.corner is not None:
            self.corners.append(revision.corner)
            for source in self.pending:
                self.markers[source] = revision.corner.s
        if revision.pieces:
            self.pending = [move.source_id]
            self.last_move = move
        else:
            self.pending.append(move.source_id)
            if move.tolerance is None:
                self.last_move = move
        self.markers[move.source_id] = s

    def append(self, move: Move, committed_s: float, blend: bool = True) -> Revision:
        revision = self.propose(move, committed_s, blend)
        if revision.ok:
            self.accept(revision)
        return revision
