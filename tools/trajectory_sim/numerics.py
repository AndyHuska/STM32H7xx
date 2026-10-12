"""L1 float32 propagation mirror for target-precision studies (not used by reference solvers)."""

from __future__ import annotations

from typing import Iterable

import numpy as np

from .model import AccelerationStep, Element, State

_F = np.float32


def propagate_f32(start: State, phases: Iterable[Element], relative: bool = True) -> State:
    """Propagate float32-rounded phases in float32 using phases.advance's operation order.

    With relative=True position accumulates from 0 and start.s is added back in float64,
    modelling segment-relative storage; otherwise absolute s is carried in float32.
    """
    s = _F(0.0) if relative else _F(start.s)
    v, a = _F(start.v), _F(start.a)
    two, six = _F(2.0), _F(6.0)
    for phase in phases:
        if isinstance(phase, AccelerationStep):
            a = _F(phase.acceleration)
            continue
        j, t = _F(phase.jerk), _F(phase.duration)
        s = s + t * (v + t * (a / two + t * j / six))
        v = v + t * (a + t * j / two)
        a = a + t * j
    return State(float(s) + (start.s if relative else 0.0), float(v), float(a))
