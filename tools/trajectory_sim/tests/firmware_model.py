"""Test-only model of the current grblHAL firmware path control, for characterisation (no firmware changes).

Build modelled: H743 with ENABLE_PATH_BLENDING=1, ENABLE_JERK_ACCELERATION=0 (constant-acceleration blocks).
Sources: grbl/motion_control.c (mc_line, blend_resolve_corner), grbl/planner.c (plan_buffer_line,
planner_recalculate), grbl/gcode.c (G61 / G61.1 / G64 P), grbl/config.h defaults.

Modelled exactly (in float64; the firmware runs float32):
- Control modes: G61 = exact PATH (corners at the junction-deviation speed, no stop, no blend); G61.1 = exact
  stop (junction speed 0 when this or the previous block is G61.1); G64 P = tangent-arc blend.
- G64: a G1 move is held back (pending) and blended with the next G1 move if both are G64 with P > 0 and
  XYZ-only; the corner uses the OUTGOING move's P. Interior angle theta = acos(-in.out) must lie in
  (0.001, pi - 0.001). R = P / (1/sin(theta/2) - 1), trim = R / tan(theta/2); if trim > min(len_in, len_out)/2
  it is clamped there and R = trim tan(theta/2). len_in is the pending move's remaining length (already
  trimmed by the previous corner). Emitted as: line to p1 (incoming feed), half arc (incoming feed), half arc
  (outgoing feed); the rest of the outgoing move stays pending. A failed corner flushes the pending move
  unblended (junction deviation applies). The last pending move is flushed at program end.
- Block limits: line: rate = min V_i/|u_i|, accel = min A_i/|u_i|. Arc (unit vector = tangent at the block
  start): rate = min(that, min_i V_i over XYZ, sqrt(0.5 A_min R)), accel = min(that, 0.5 A_min), A_min over
  X, Y and Z even for a planar arc. Nominal = min(feed, rate) >= MINIMUM_FEED_RATE.
- Junctions: first block from rest 0; collinear / tangent: unlimited; reversal-like (cos > 0.999999): 0;
  else v^2 = A_j jd sin(theta/2) / (1 - sin(theta/2)), A_j = accel limit along unit(u - u_prev).
  max_entry^2 = min(min(nominal, previous nominal)^2, junction^2).
- Planner: grbl backward pass (exit 0 at the buffer end) + forward pass, with a sliding window of
  `buffer_blocks` blocks (default 100) while executing a fully queued program; the executing block's entry is
  fixed. Each block runs a trapezoid (accelerate, cruise, decelerate) at its own acceleration.
Not modelled: step quantisation of targets / unit vectors, segment-time discretisation in the stepper
(ACCELERATION_TICKS_PER_SECOND = 100), streaming starvation and the pending-move timeouts (10/30/250 ms),
feed override, separate deceleration (the firmware has a single acceleration per axis).

`arc_limits="planar"` models proposed modification FW-1 (Firmware_Planner_Modifications.md): only axes in
the arc's plane limit it. With amp_i = |projection of the plane onto axis i| = hypot(radial_i, tangent_i),
|axis velocity| <= amp_i v and |axis acceleration| <= amp_i (|a_t| + v^2 / R), so rate = min V_i / amp_i,
accel = 0.5 min A_i / amp_i and v^2 / R <= 0.5 min A_i / amp_i keep every axis within its limits (same 50/50
split as the firmware; axes with amp_i = 0, e.g. Z for an XY arc, impose nothing).

`arc_limits="circle"` models FW-2: on the arc axis i's acceleration is a_t t_i(phi) - (v^2/R) r_i(phi), two
quadrature sinusoids of amplitude amp_i, so its exact maximum over the arc is amp_i sqrt(a_t^2 + (v^2/R)^2).
With A_p = min A_i / amp_i the admissible set is the circle a_t^2 + (v^2/R)^2 <= A_p^2. A constant block
acceleration needs a fixed point on it: v^2/R <= k A_p and a_t <= sqrt(1 - k^2) A_p (`arc_k` = k in (0, 1)).
(The current 50/50 sum split is the point (0.5, 0.5), inside the circle: k = 0.707 dominates it.)

`arc_limits="adaptive"` models FW-2 with an adaptive k: the arc uses the whole circle, a_t = A_p sqrt(1 - w^2)
with w = v^2 / (A_p R), so k = w at every point. Then dw/ds = (2/R) sqrt(1 - w^2), i.e. asin(w) grows linearly:
asin(w1) = asin(w0) + 2 L / R. The planner passes use this closed form instead of v1^2 = v0^2 + 2 a L for arc
blocks (both passes stay exact and monotone); `arc_k` is then only the cruise cap k_max (v^2 <= k_max A_p R,
the lateral-acceleration step allowed at the line -> arc tangent point). Ramp time on an arc:
t = R / (2 sqrt(A_p R)) * integral dphi / sqrt(sin phi) between the asin(w) values (Gauss-Legendre).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Sequence

import numpy

from tools.trajectory_sim.model import Limits, Vector

MINIMUM_FEED_RATE = 1.0 / 60.0   # mm/s (config.h: 1 mm/min)
MINIMUM_JUNCTION_SPEED = 0.0     # mm/s
DEFAULT_JUNCTION_DEVIATION = 0.01  # mm ($11 default)
DEFAULT_PLANNER_BUFFER_BLOCKS = 100

G61, G61_1 = "G61", "G61.1"


@dataclass(frozen=True)
class FirmwareMove:
    """G1 move to `end`; `control` is G61, G61.1 or the G64 P value (mm, > 0)."""

    end: Vector
    feed: float
    control: str | float


@dataclass
class Block:
    source: int
    kind: str                     # "line" | "arc"
    start: Vector
    end: Vector
    length: float
    nominal: float
    acceleration: float
    max_entry_sqr: float
    start_dir: Vector
    end_dir: Vector
    center: Vector | None = None  # arc only
    radius: float = 0.0
    sweep: float = 0.0            # arc only (rad, > 0)
    circle: float = 0.0           # adaptive arcs: A_p of the acceleration-circle law (0 = constant acceleration)


@dataclass
class Corner:
    source: int                   # index of the incoming move
    kind: str                     # "arc" | "junction" | "stop" | "end"
    radius: float = 0.0
    trim: float = 0.0
    deviation: float = 0.0        # vertex to arc distance (= P unless the trim was clamped)
    detail: str = ""


def _sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def _add(a, b):
    return tuple(x + y for x, y in zip(a, b))


def _scale(k, a):
    return tuple(k * x for x in a)


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def _norm(a):
    return math.sqrt(_dot(a, a))


def _unit(a):
    return _scale(1.0 / _norm(a), a)


@dataclass
class FirmwareModel:
    axes: Sequence[Limits]
    start: Vector = (0.0, 0.0, 0.0)
    junction_deviation: float = DEFAULT_JUNCTION_DEVIATION
    buffer_blocks: int = DEFAULT_PLANNER_BUFFER_BLOCKS
    arc_limits: str = "firmware"   # "firmware" | "planar" (FW-1) | "circle" (FW-2 fixed k) | "adaptive" (FW-2 law)
    arc_k: float = 0.5 ** 0.5     # FW-2: fixed centripetal share (circle) or cruise cap k_max (adaptive)
    # Firmware line blocks take direction and length from integer step targets (pl.position); arcs from floats.
    steps_per_mm: Sequence[float] | None = None
    tangent_blend_junctions: bool = False   # FW-4: joins into/out of a blend arc are tangent by construction
    blocks: list[Block] = field(default_factory=list)
    corners: list[Corner] = field(default_factory=list)

    def __post_init__(self) -> None:
        self._position = tuple(self.start)
        self._pending: tuple[Vector, Vector, FirmwareMove, int] | None = None
        self._previous_unit: Vector | None = None
        self._previous_nominal = 0.0
        self._previous_exact_stop = False
        self._previous_arc = False
        self._steps = self._to_steps(self.start)
        self._source = -1

    def _to_steps(self, point):
        return None if self.steps_per_mm is None else tuple(round(c * k) for c, k in zip(point, self.steps_per_mm))

    # ---- motion_control.c ----
    def feed(self, move: FirmwareMove) -> None:
        self._source += 1
        candidate = not isinstance(move.control, str) and move.control > 0.0
        if candidate:
            if self._pending is not None:
                if self._resolve_corner(move):
                    return
                self._flush("no_corner")
            self._pending = (self._position, move.end, move, self._source)
            return
        self._flush("forced")
        self._plan_line(self._position, move.end, move, self._source)

    def finish(self) -> None:
        self._flush("end")

    def _flush(self, reason: str) -> None:
        if self._pending is None:
            return
        start, target, move, source = self._pending
        self._pending = None
        self._plan_line(start, target, move, source)

    def _resolve_corner(self, move: FirmwareMove) -> bool:
        a, b, pending, source = self._pending
        c = move.end
        tolerance = move.control
        d_in, d_out = _sub(b, a), _sub(c, b)
        len_in, len_out = _norm(d_in), _norm(d_out)
        if len_in <= 0.0 or len_out <= 0.0:
            return False
        t_in, t_out = _scale(1.0 / len_in, d_in), _scale(1.0 / len_out, d_out)
        theta = math.acos(max(-1.0, min(1.0, -_dot(t_in, t_out))))
        sin_half = math.sin(0.5 * theta)
        if theta < 0.001 or theta > math.pi - 0.001 or sin_half <= 0.0:
            self.corners.append(Corner(source, "junction", detail="angle outside blend range"))
            return False
        bisector = _sub(t_out, t_in)
        if _norm(bisector) <= 1e-6:
            return False
        bisector = _unit(bisector)
        radius = tolerance / (1.0 / sin_half - 1.0)
        trim = radius / math.tan(0.5 * theta)
        if trim > 0.5 * min(len_in, len_out):
            trim = 0.5 * min(len_in, len_out)
            radius = trim * math.tan(0.5 * theta)
        if radius <= 0.0 or trim <= 0.0:
            return False
        p1, p2 = _sub(b, _scale(trim, t_in)), _add(b, _scale(trim, t_out))
        center = _add(b, _scale(radius / sin_half, bisector))
        sweep = math.pi - theta
        t_mid = _unit(_add(t_in, t_out))
        mid = _add(center, _scale(radius, _unit(_sub(_add(p1, p2), _scale(2.0, center)))))
        self.corners.append(Corner(source, "arc", radius, trim, radius / sin_half - radius))
        self._plan_line(a, p1, pending, source)
        self._plan_arc(p1, mid, center, radius, 0.5 * sweep, t_in, t_mid, pending, source)
        self._plan_arc(mid, p2, center, radius, 0.5 * sweep, t_mid, t_out, move, self._source)
        self._pending = (p2, c, move, self._source)
        return True

    # ---- planner.c ----
    def _rate(self, unit) -> float:
        return min(axis.speed / abs(u) for u, axis in zip(unit, self.axes) if u != 0.0)

    def _accel(self, unit) -> float:
        return min(axis.acceleration / abs(u) for u, axis in zip(unit, self.axes) if u != 0.0)

    def _plan_line(self, start, end, move: FirmwareMove, source: int) -> None:
        delta = _sub(end, start)
        if self.steps_per_mm is not None:
            target = self._to_steps(end)
            delta = tuple((t - p) / k for t, p, k in zip(target, self._steps, self.steps_per_mm))
            if not any(t != p for t, p in zip(target, self._steps)):
                return
            self._steps = target
        length = _norm(delta)
        if length <= 0.0:
            return
        unit = _scale(1.0 / length, delta)
        self._append(Block(source, "line", start, end, length, 0.0, self._accel(unit), 0.0, unit, unit),
                     move.feed, self._rate(unit), move.control == G61_1)

    def _plan_arc(self, start, end, center, radius, sweep, t_start, t_end, move: FirmwareMove, source: int) -> None:
        if self.arc_limits in ("planar", "circle", "adaptive"):
            radial = _scale(1.0 / radius, _sub(start, center))
            amp = [math.hypot(r, t) for r, t in zip(radial, t_start)]
            moving = [(m, axis) for m, axis in zip(amp, self.axes) if m > 1e-12]
            a_plane = min(axis.acceleration / m for m, axis in moving)
            if self.arc_limits == "planar":
                centripetal, tangential = 0.5, 0.5
            elif self.arc_limits == "circle":
                centripetal, tangential = self.arc_k, math.sqrt(1.0 - self.arc_k ** 2)
            else:
                centripetal, tangential = self.arc_k, 1.0  # tangential follows the circle law
            rate = min(min(axis.speed / m for m, axis in moving), math.sqrt(centripetal * a_plane * radius))
            accel = tangential * a_plane
        else:
            a_min = min(axis.acceleration for axis in self.axes[:3])
            v_min = min(axis.speed for axis in self.axes[:3])
            rate = min(self._rate(t_start), v_min, math.sqrt(0.5 * a_min * radius))
            accel = min(self._accel(t_start), 0.5 * a_min)
        block = Block(source, "arc", start, end, radius * sweep, 0.0, accel, 0.0, t_start, t_end, center, radius, sweep,
                      accel if self.arc_limits == "adaptive" else 0.0)
        if self.steps_per_mm is not None:
            self._steps = self._to_steps(end)
        self._append(block, min(move.feed, rate), rate, False)

    def _append(self, block: Block, programmed: float, rate: float, exact_stop: bool) -> None:
        block.nominal = max(MINIMUM_FEED_RATE, min(programmed, rate))
        if self._previous_unit is None:
            junction_sqr = 0.0
        else:
            cos_theta = -_dot(self._previous_unit, block.start_dir)
            if cos_theta > 0.999999:
                junction_sqr = MINIMUM_JUNCTION_SPEED ** 2
            elif cos_theta < -0.999999:
                junction_sqr = math.inf
            else:
                junction_accel = self._accel(_unit(_sub(block.start_dir, self._previous_unit)))
                sin_d2 = math.sqrt(0.5 * (1.0 - cos_theta))
                junction_sqr = max(MINIMUM_JUNCTION_SPEED ** 2,
                                   junction_accel * self.junction_deviation * sin_d2 / (1.0 - sin_d2))
            if self.tangent_blend_junctions and (block.kind == "arc" or self._previous_arc):
                junction_sqr = math.inf
            if exact_stop or self._previous_exact_stop:
                junction_sqr = 0.0
        block.max_entry_sqr = min(min(block.nominal, self._previous_nominal) ** 2, junction_sqr)
        self._previous_nominal = block.nominal
        self._previous_unit = block.end_dir
        self._previous_exact_stop = exact_stop
        self._previous_arc = block.kind == "arc"
        self._position = block.end
        self.blocks.append(block)

    # ---- execution ----
    def schedule(self) -> list[tuple[float, float, float]]:
        """(entry speed, exit speed, duration) per block, executing the fully queued program with a sliding
        planner window of `buffer_blocks` blocks (exit 0 at the window end)."""
        self.finish()
        rows, entry = [], 0.0
        n = len(self.blocks)
        for k, block in enumerate(self.blocks):
            window = self.blocks[k:min(n, k + self.buffer_blocks)]
            exit_sqr = 0.0
            for later in reversed(window[1:]):   # backward pass down to block k's exit
                exit_sqr = min(later.max_entry_sqr, reach_sqr(later, exit_sqr))
            exit_sqr = min(exit_sqr, reach_sqr(block, entry * entry))   # forward pass
            exit = math.sqrt(exit_sqr)
            rows.append((entry, exit, block_time(block, entry, exit)))
            entry = exit
        return rows

    def duration(self) -> float:
        return sum(row[2] for row in self.schedule())

    def samples(self, dt: float) -> list[Vector]:
        """Positions at times k*dt of the scheduled motion (for finite-difference characterisation)."""
        rows = self.schedule()
        points, t_block, k = [], 0.0, 0
        total = sum(r[2] for r in rows)
        t = 0.0
        while t <= total:
            while k < len(rows) - 1 and t > t_block + rows[k][2]:
                t_block += rows[k][2]
                k += 1
            entry, exit, _ = rows[k]
            block = self.blocks[k]
            if block.circle:
                raise NotImplementedError("sampling of adaptive arcs")
            s = _distance(entry, exit, block.nominal, block.acceleration, block.length, t - t_block)
            points.append(point(block, s))
            t += dt
        return points


def reach_sqr(block: Block, from_sqr: float) -> float:
    """Largest v^2 at one end of the block from v^2 = from_sqr at the other (acceleration and braking are
    symmetric): v^2 + 2 a L, or on adaptive arcs the circle law asin(w1) = asin(w0) + 2 L / R."""
    if not block.circle:
        return from_sqr + 2.0 * block.acceleration * block.length
    c2 = block.circle * block.radius
    phi = math.asin(min(1.0, from_sqr / c2)) + 2.0 * block.length / block.radius
    return c2 if phi >= 0.5 * math.pi else c2 * math.sin(phi)


_NODES, _WEIGHTS = numpy.polynomial.legendre.leggauss(48)


def _arc_ramp_time(block: Block, phi_a: float, phi_b: float) -> float:
    """Time to move along the circle law between asin(w) = phi_a and phi_b (phi_b >= phi_a); substitution
    phi = phi_a + (phi_b - phi_a) x^2 removes the 1/sqrt singularity at rest."""
    if phi_b <= phi_a:
        return 0.0
    x = 0.5 * (_NODES + 1.0)
    span = phi_b - phi_a
    integrand = 2.0 * span * x / numpy.sqrt(numpy.sin(phi_a + span * x * x))
    return 0.5 * block.radius / math.sqrt(block.circle * block.radius) * 0.5 * float(numpy.dot(_WEIGHTS, integrand))


def block_time(block: Block, u: float, w: float) -> float:
    if not block.circle:
        return _trapezoid(u, w, block.nominal, block.acceleration, block.length)
    c2 = block.circle * block.radius
    phi_u, phi_w = (math.asin(min(1.0, x * x / c2)) for x in (u, w))
    phi_n = math.asin(min(1.0, block.nominal ** 2 / c2))
    phi_peak = 0.5 * (phi_u + phi_w) + block.length / block.radius
    if phi_peak <= phi_n:
        return _arc_ramp_time(block, phi_u, phi_peak) + _arc_ramp_time(block, phi_w, phi_peak)
    cruise = block.length - 0.5 * block.radius * (2.0 * phi_n - phi_u - phi_w)
    return _arc_ramp_time(block, phi_u, phi_n) + _arc_ramp_time(block, phi_w, phi_n) + max(0.0, cruise) / block.nominal


def _profile(u: float, w: float, nominal: float, accel: float, length: float) -> tuple[float, float, float]:
    """(peak speed, acceleration time, deceleration time) of the block's trapezoid."""
    peak = min(nominal, math.sqrt(max(0.0, accel * length + 0.5 * (u * u + w * w))))
    peak = max(peak, u, w)
    return peak, (peak - u) / accel, (peak - w) / accel


def _trapezoid(u: float, w: float, nominal: float, accel: float, length: float) -> float:
    peak, t_up, t_down = _profile(u, w, nominal, accel, length)
    cruise = length - (peak * peak - u * u) / (2.0 * accel) - (peak * peak - w * w) / (2.0 * accel)
    return t_up + t_down + max(0.0, cruise) / peak


def _distance(u: float, w: float, nominal: float, accel: float, length: float, t: float) -> float:
    peak, t_up, t_down = _profile(u, w, nominal, accel, length)
    d_up = (peak * peak - u * u) / (2.0 * accel)
    d_down = (peak * peak - w * w) / (2.0 * accel)
    t_cruise = max(0.0, length - d_up - d_down) / peak
    if t <= t_up:
        return u * t + 0.5 * accel * t * t
    if t <= t_up + t_cruise:
        return d_up + peak * (t - t_up)
    tau = min(t - t_up - t_cruise, t_down)
    return min(length, d_up + peak * t_cruise + peak * tau - 0.5 * accel * tau * tau)


def speed_at(block: Block, u: float, w: float, s: float) -> float:
    """Planned speed at distance s into the block (entry u, exit w): min of cap, acceleration from the entry
    and braking to the exit, under the block's law."""
    if not block.circle:
        v2 = min(block.nominal ** 2, u * u + 2.0 * block.acceleration * s, w * w + 2.0 * block.acceleration * (block.length - s))
    else:
        c2 = block.circle * block.radius
        phi_u, phi_w = (math.asin(min(1.0, x * x / c2)) for x in (u, w))
        v2 = min(block.nominal ** 2, c2 * math.sin(min(0.5 * math.pi, phi_u + 2.0 * s / block.radius)),
                 c2 * math.sin(min(0.5 * math.pi, phi_w + 2.0 * (block.length - s) / block.radius)))
    return math.sqrt(max(0.0, v2))


def point(block: Block, s: float) -> Vector:
    if block.kind == "line":
        return _add(block.start, _scale(s, block.start_dir))
    angle = s / block.radius
    radial = _scale(1.0 / block.radius, _sub(block.start, block.center))
    return _add(block.center, _add(_scale(block.radius * math.cos(angle), radial),
                                   _scale(block.radius * math.sin(angle), block.start_dir)))


def run(axes: Sequence[Limits], moves: Sequence[FirmwareMove], start: Vector = (0.0, 0.0, 0.0), **options) -> FirmwareModel:
    model = FirmwareModel(axes, start, **options)
    for move in moves:
        model.feed(move)
    model.finish()
    return model
