"""L4 constraint projection (M4): constant path limits per piece from certified derivative bounds.

For axis i on a piece with bounds B1 >= |q_i'|, B2 >= |q_i''|, B3 >= |q_i'''|:
    |axis velocity| <= B1 v,  |axis acceleration| <= B1 |a| + B2 v^2,
    |axis jerk| <= B1 |j| + 3 B2 v |a| + B3 v^3.
Derivation and share allocation: DESIGN.md section 3C.
"""

from __future__ import annotations

import math
from typing import Sequence

from .contracts import DerivativeBounds
from .geometry import Piece
from .model import Limits, PathBlock
from .phases import ramp_distance

CURVED_SHARE = 0.5   # default share of the straight-line A and J kept on a curved piece
# Allocation grid. A full share is excluded: it leaves no acceleration budget for the curve (cap 0).
SHARES = tuple(2.0 ** -k for k in range(8, 0, -1))
SEARCH_ITERATIONS = 100


def _straight(bounds: DerivativeBounds, axes: Sequence[Limits]) -> Limits:
    """Exact on a line: axis i moves u_i v, so its speed grows exactly when the path speed does."""
    moving = [(b, axis) for b, axis in zip(bounds.first, axes) if b > 0.0]
    jerk = min((axis.jerk / b for b, axis in moving if axis.jerk_limited), default=math.inf)
    return Limits(min(axis.speed / b for b, axis in moving), min(axis.acceleration / b for b, axis in moving), jerk,
                  min(axis.deceleration / b for b, axis in moving))


def curved(bounds: DerivativeBounds) -> bool:
    return any(bounds.second) or any(bounds.third)


class ConservativeProjection:
    """Reference implementation of contracts.ConstraintProjection.

    Curved pieces: path |a| <= share * A_line and |j| <= share * J_line, where A_line, J_line are the
    straight-line values for the same tangent bounds (axis acceleration bounded by min(A_i, D_i) both ways,
    since the sign of the axis speed change is not tracked); the cap is the largest v keeping every axis
    within limits at that |a| and |j| (monotone in v). A smaller share trades the ability to change speed
    on the piece for a higher cap.
    """

    def path_limits(self, bounds: DerivativeBounds, axes: Sequence[Limits], share: float = CURVED_SHARE) -> Limits:
        if not curved(bounds):
            return _straight(bounds, axes)
        rows = [(b1, b2, b3, min(axis.acceleration, axis.deceleration), axis)
                for b1, b2, b3, axis in zip(bounds.first, bounds.second, bounds.third, axes) if b1 or b2 or b3]
        a_path = share * min(cap / b1 for b1, _, _, cap, _ in rows if b1 > 0.0)
        j_path = share * min((axis.jerk / b1 for b1, _, _, _, axis in rows if b1 > 0.0 and axis.jerk_limited), default=math.inf)
        top = min(axis.speed / b1 for b1, _, _, _, axis in rows if b1 > 0.0)

        def admissible(v: float) -> bool:
            for b1, b2, b3, cap, axis in rows:
                if b1 * a_path + b2 * v * v > cap:
                    return False
                if axis.jerk_limited and b1 * j_path + 3.0 * b2 * v * a_path + b3 * v ** 3 > axis.jerk:
                    return False
            return True

        speed = top
        if not admissible(top):
            low, high = 0.0, top
            for _ in range(SEARCH_ITERATIONS):
                middle = 0.5 * (low + high)
                if not low < middle < high:
                    break
                if admissible(middle):
                    low = middle
                else:
                    high = middle
            speed = low
        return Limits(speed, a_path, j_path, a_path)


def _reach(cap: float, other: float, limits: Limits, length: float, braking: bool) -> float:
    """Largest speed <= cap from which the piece can brake to `other` (braking) or which it can reach
    from `other` (accelerating) within `length`, with a = 0 at both ends (conservative, like L5)."""
    if other >= cap:
        return cap

    def fits(u: float) -> bool:
        return (ramp_distance(u, other, limits) if braking else ramp_distance(other, u, limits)) <= length

    if fits(cap):
        return cap
    low, high = other, cap
    for _ in range(SEARCH_ITERATIONS):
        middle = 0.5 * (low + high)
        if not low < middle < high:
            break
        if fits(middle):
            low = middle
        else:
            high = middle
    return low


def blocks(pieces: Sequence[Piece], stops: Sequence[bool], axes: Sequence[Limits],
           projection: ConservativeProjection | None = None) -> list[PathBlock]:
    """PathBlocks for L5: feed as the override-scaled cap, projected limits per piece.

    Share selection: a backward pass gives the highest speed each piece can carry and still brake to what
    follows, a forward pass the highest speed it can reach from what precedes (each piece taking its best
    share). Each curved piece then gets the share maximising the smaller of the two, so caps follow what
    the neighbours allow instead of the piece's own curvature alone. Ends of the sequence are unconstrained.
    """
    projection = projection or ConservativeProjection()
    n = len(pieces)
    options = []
    for piece in pieces:
        bounds = piece.bounds()
        options.append([projection.path_limits(bounds, axes, share) for share in SHARES] if curved(bounds)
                       else [projection.path_limits(bounds, axes)])

    def cap(k: int, limits: Limits) -> float:
        return min(limits.speed, pieces[k].feed)

    carry = [math.inf] * (n + 1)   # backward: speed piece k may carry and still brake to the next
    for k in reversed(range(n)):
        after = 0.0 if stops[k] else carry[k + 1]
        carry[k] = max(_reach(cap(k, lim), after, lim, pieces[k].length, True) for lim in options[k])
    reach = [math.inf] * (n + 1)   # forward: speed piece k can reach from the previous
    for k in range(n):
        before = 0.0 if k > 0 and stops[k - 1] else reach[k]
        reach[k + 1] = max(_reach(cap(k, lim), before, lim, pieces[k].length, False) for lim in options[k])
    chosen = []
    for k in range(n):
        after = 0.0 if stops[k] else carry[k + 1]
        before = 0.0 if k > 0 and stops[k - 1] else reach[k]
        chosen.append(max(options[k], key=lambda lim: min(_reach(cap(k, lim), after, lim, pieces[k].length, True),
                                                         _reach(cap(k, lim), before, lim, pieces[k].length, False))))
    return [PathBlock(piece.source_id, piece.length, piece.feed, stop, lim, hold_deceleration(piece.bounds(), axes, lim))
            for piece, stop, lim in zip(pieces, stops, chosen)]


def hold_deceleration(bounds: DerivativeBounds, axes: Sequence[Limits], limits: Limits) -> float:
    """Path deceleration admissible with jerk waived at every speed up to the piece's cap: B1 |a| + B2 v^2 <=
    min(A_i, D_i) (curved) or B1 |a| <= D_i (straight). Never below the planned deceleration."""
    if not curved(bounds):
        return limits.deceleration
    v = limits.speed
    room = min(((min(axis.acceleration, axis.deceleration) - b2 * v * v) / b1)
               for b1, b2, axis in zip(bounds.first, bounds.second, axes) if b1 > 0.0)
    return max(limits.deceleration, room)
