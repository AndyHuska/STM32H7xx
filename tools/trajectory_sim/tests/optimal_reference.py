"""Time-optimal traversal of a fixed path under per-axis limits: the reference for G64 (test-only, CasADi/IPOPT).

The path is split at stop flags into rest-to-rest sections (independent problems; a kink inside a section is
an error). Each section is transcribed on an arc-length grid s_0..s_N with node speeds v_k and path
accelerations a_k as variables:
    v_k+1^2 - v_k^2 = ds (a_k + a_k+1)          (d(v^2)/ds = 2a, trapezoid rule)
    T = sum 2 ds / (v_k + v_k+1)                (exact for constant path acceleration per interval)
    |q_i'| v <= V_i,  v <= feed                 (nodes)
    axis acceleration q_i' a + q_i'' v^2 in [-D_i, A_i] when the axis moves forward (q_i' > 0), in [-A_i, D_i]
        when backward, and |.| <= A_i when q_i' = 0: D applies only while the axis speed magnitude falls
        (the physical semantics; our planner is stricter on curves)                        (nodes)
    axis jerk q_i' j + 3 q_i'' v a + q_i''' v^3 within +/-J_i, with j = da/dt = v_mean (a_k+1 - a_k) / ds, the
        interval's own time ds / v_mean (exact for a constant-jerk start from rest up to O(1/k^2)) (midpoints)
    v = 0 at both ends; a = 0 there too unless no moving axis is jerk-limited (then a may step at rest).
Constraints hold at grid points only, so the result converges to the continuous optimum as the grid refines.
Near rest a jerk-limited start has s ~ t^3: with a uniform s grid the first intervals overstate the jerk and the
error decays only like N^(-1/3). The grid is graded toward both section ends with I_x(5, 5) (spacing ~ x^5),
uniform in x. Validated against Ruckig on straight rest-to-rest lines on every machine of the spectrum:
relative excess ~6e-4 / 1.5e-4 / 3.5e-5 / 1.1e-5 at count 50 / 100 / 200 / 400 (second order).

`envelope` replaces the axis model by constant path limits per piece (speed, acceleration, deceleration,
jerk on v, a, j), e.g. the planner's own L4 output: the optimum under the planner's constraint set, which
separates L4 projection conservatism from L5 scheduling conservatism.
"""

from __future__ import annotations

import math
from typing import Sequence

import casadi

from tools.trajectory_sim.model import Limits

KINK = 1e-9  # tangent mismatch above which a join inside a section is a kink
GRADING = 5  # node spacing ~ x^GRADING near rest
# Variables and constraints are scaled by hand. IPOPT's gradient scaling must stay off: the objective's
# gradient at the rest nodes is huge, so it would shrink the objective ~1e-8 and stop with ~1e-3 slack.
IPOPT = {"print_level": 0, "max_iter": 5000, "tol": 1e-12, "acceptable_tol": 1e-9, "sb": "yes",
         "nlp_scaling_method": "none"}


def sections(entries) -> list[list[tuple[float, object, int]]]:
    """Split (start s, piece, stop after) entries into rest-to-rest sections of (start s, piece, entry index)."""
    result, current = [], []
    for index, (s0, piece, stop) in enumerate(entries):
        if current:
            before = current[-1][1]
            d0, d1 = before.evaluate(before.length)[1], piece.evaluate(0.0)[1]
            if math.dist(d0, d1) > KINK:
                raise ValueError(f"kink at s = {s0} inside a section: an exact stop is required there")
        current.append((s0, piece, index))
        if stop:
            result.append(current)
            current = []
    if current:
        result.append(current)
    return result


def graded(x: float, order: int | None = None) -> float:
    """Regularised incomplete beta I_x(p, p): symmetric map of [0, 1], ~ x^p near both ends."""
    order = order or GRADING
    n = 2 * order - 1
    return sum(math.comb(n, j) * x ** j * (1.0 - x) ** (n - j) for j in range(order, n + 1))


def grid(section, count: int, minimum: int = 12) -> list[tuple[float, int, float, tuple[int, ...]]]:
    """Nodes (s, position in section, u, positions of the pieces containing the node): each piece gets
    max(minimum, its length share of `count`) intervals graded toward both of its ends, so rest ends and piece
    boundaries (where the envelope's limits jump) are resolved alike."""
    begin = section[0][0]
    length = section[-1][0] + section[-1][1].length - begin
    nodes = []
    last = len(section) - 1
    for p, (s0, piece, _) in enumerate(section):
        intervals = max(minimum, math.ceil(count * piece.length / length))
        if not nodes:
            nodes.append((s0, p, 0.0, (p,)))
        nodes += [(s0 + piece.length * graded(k / intervals), p, piece.length * graded(k / intervals), (p,))
                  for k in range(1, intervals)]
        nodes.append((s0 + piece.length, p, piece.length, (p, p + 1) if p < last else (p,)))
    return nodes


def _section_time(section, axes: Sequence[Limits], feed: float | None, count: int, minimum: int,
                  envelope: Sequence[Limits] | None) -> tuple[float, list, list]:
    nodes = grid(section, count, minimum)
    n = len(nodes)
    s = [node[0] for node in nodes]
    pieces = [entry[1] for entry in section]
    derivatives = [pieces[node[1]].evaluate(node[2])[1:] for node in nodes]
    mids = []  # (position in section, q', q'', q''') at interval midpoints
    for k in range(n - 1):
        p = nodes[k + 1][1]
        u_mid = 0.5 * (nodes[k][2] + nodes[k + 1][2]) if nodes[k][1] == p else 0.5 * nodes[k + 1][2]
        mids.append((p, *pieces[p].evaluate(u_mid)[1:]))

    def feed_of(p: int) -> float:
        return pieces[p].feed if feed is None else feed

    def path_limits(p: int) -> Limits:
        return envelope[section[p][2]]

    if envelope is None:
        caps = [min([feed_of(node[1])] + [axis.speed / abs(q) for q, axis in zip(d[0], axes) if abs(q) > 1e-12])
                for node, d in zip(nodes, derivatives)]
        a_scale = max(axis.acceleration for axis in axes)
    else:
        caps = [min(min(feed_of(p), path_limits(p).speed) for p in node[3]) for node in nodes]
        a_scale = max(path_limits(p).acceleration for p in range(len(section)))
    # Variables scaled to O(1): v = v_scale * w, a = a_scale * x; constraints divided by their limits.
    v_scale = max(caps)
    length = s[-1] - s[0]
    t_scale = length / v_scale

    opti = casadi.Opti()
    w = opti.variable(n)
    x = opti.variable(n)
    vel, acc = v_scale * w, a_scale * x
    opti.subject_to([w[0] == 0, w[n - 1] == 0])
    for k in (0, n - 1):
        if envelope is None:
            jerk_limited = any(axis.jerk_limited and abs(q) > 1e-12 for q, axis in zip(derivatives[k][0], axes))
        else:
            jerk_limited = path_limits(nodes[k][1]).jerk_limited
        if jerk_limited:
            opti.subject_to(x[k] == 0)
    for k in range(1, n - 1):
        opti.subject_to(opti.bounded(0.0, w[k], caps[k] / v_scale))
    for k in range(n):
        if envelope is not None:
            for p in nodes[k][3]:
                lim = path_limits(p)
                opti.subject_to(opti.bounded(-lim.deceleration / lim.acceleration, acc[k] / lim.acceleration, 1.0))
            continue
        d1, d2, _ = derivatives[k]
        for i, axis in enumerate(axes):
            axis_acc = d1[i] * acc[k] + d2[i] * vel[k] ** 2
            if d1[i] > 1e-12:
                low, high = axis.deceleration, axis.acceleration
            elif d1[i] < -1e-12:
                low, high = axis.acceleration, axis.deceleration
            elif d2[i] != 0.0:
                low = high = axis.acceleration
            else:
                continue
            opti.subject_to(opti.bounded(-low / high, axis_acc / high, 1.0))
    total = 0
    for k in range(n - 1):
        ds = s[k + 1] - s[k]
        opti.subject_to((vel[k + 1] ** 2 - vel[k] ** 2) / (v_scale * v_scale)
                        == ds * (acc[k] + acc[k + 1]) / (v_scale * v_scale))
        total += 2 * ds / (vel[k] + vel[k + 1])
        p, d1, d2, d3 = mids[k]
        v = 0.5 * (vel[k] + vel[k + 1])
        jerk = v * (acc[k + 1] - acc[k]) / ds
        if envelope is not None:
            lim = path_limits(p)
            if lim.jerk_limited:
                opti.subject_to(opti.bounded(-1.0, jerk / lim.jerk, 1.0))
            continue
        a_mid = 0.5 * (acc[k] + acc[k + 1])
        for i, axis in enumerate(axes):
            if axis.jerk_limited and (d1[i] or d2[i] or d3[i]):
                axis_jerk = d1[i] * jerk + 3 * d2[i] * v * a_mid + d3[i] * v ** 3
                opti.subject_to(opti.bounded(-1.0, axis_jerk / axis.jerk, 1.0))
    opti.minimize(total / t_scale)
    opti.set_initial(w, [0.5 * caps[k] / v_scale * math.sin(math.pi * (s[k] - s[0]) / length) for k in range(n)])
    opti.set_initial(x, 0.0)
    opti.solver("ipopt", {"print_time": False, "detect_simple_bounds": True}, dict(IPOPT))
    solution = opti.solve()
    velocities = [v_scale * max(float(solution.value(w[k])), 0.0) for k in range(n)]
    return float(solution.value(total)), s, velocities


def optimal_time(entries, axes: Sequence[Limits], feed: float | None = None, count: int = 200,
                 minimum: int = 12, envelope: Sequence[Limits] | None = None) -> tuple[float, list[float], list[float]]:
    """(time, s nodes, v at nodes) of the time-optimal traversal of `entries` ((start s, piece, stop after)
    as from GeometryStream.pieces()), at rest at every stop flag and at the end. `feed` None = per-piece feed;
    `count` = graded intervals per rest-to-rest section; `envelope` = optional path limits per entry."""
    total, s_all, v_all = 0.0, [], []
    for section in sections(entries):
        time, s, v = _section_time(section, axes, feed, count, minimum, envelope)
        total += time
        s_all += s
        v_all += v
    return total, s_all, v_all
