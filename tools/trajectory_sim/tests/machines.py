"""Machine configurations for spectrum testing: fixtures plus derived variants.

Mapping (DESIGN.md section 7, Option A): accelerationProfile 0 -> axis jerk = inf, 2 -> accelerationJerk,
1 (sine) -> unsupported.
"""

import json
import math
from pathlib import Path

from tools.trajectory_sim.model import Limits

FIXTURES = Path(__file__).resolve().parents[1] / "fixtures"


def load_axes(name: str) -> list[Limits]:
    data = json.loads((FIXTURES / f"{name}.json").read_text(encoding="utf-8"))
    axes = []
    for key in ("axisX", "axisY", "axisZ"):
        axis = data[key]
        profile = axis["accelerationProfile"]
        if profile == 1:
            raise ValueError(f"{name} {key}: sine profile unsupported")
        jerk = math.inf if profile == 0 else axis["accelerationJerk"]
        axes.append(Limits(axis["maximumVelocity"], axis["maximumAcceleration"], jerk, axis["maximumDeceleration"]))
    return axes


def _with(axes: list[Limits], **changes) -> list[Limits]:
    return [Limits(changes.get("speed", a.speed), changes.get("acceleration", a.acceleration),
                   changes.get("jerk", a.jerk), changes.get("deceleration", a.deceleration)) for a in axes]


def _full_rate(axes: list[Limits]) -> list[Limits]:
    """Acceleration and deceleration raised to sqrt(V J) per axis: the highest rate a 0 -> V change can use
    under the jerk limit (a triangular acceleration profile; any higher A is never reached), so time to V
    is the jerk-limited minimum 2 sqrt(V/J). Matching the trapezoidal time V/A exactly would need
    (V/A)^2 >= 4 V/J, which the standard jerk values do not allow on any axis."""
    return [Limits(a.speed, math.sqrt(a.speed * a.jerk), a.jerk, math.sqrt(a.speed * a.jerk)) for a in axes]


def spectrum() -> dict[str, list[Limits]]:
    """Named axis sets for basic-behaviour iterations; every pipeline scenario runs against all of them."""
    standard = load_axes("machine_standard_xyz")
    return {
        "standard": standard,                                         # full speed in ~0.1-0.2 s, D within 25% of A
        "standard_trapezoidal": _with(standard, jerk=math.inf),       # all axes profile 0
        "standard_mixed": standard[:2] + _with(standard[2:], jerk=math.inf),  # Z profile 0, X/Y S-curve
        "standard_high_accel": _full_rate(standard),                  # same jerk, A = D = sqrt(V J): fastest ramp it allows
        "high_jerk": _with(standard, jerk=500000.0),                  # nearly trapezoidal, still jerk-limited
    }


def extreme() -> dict[str, list[Limits]]:
    """Machines kept out of the default spectrum (slow to simulate); opt in when needed."""
    return {"example": load_axes("machine_example_xyz")}          # real machine: D = A/10, J = 21 on X/Y
