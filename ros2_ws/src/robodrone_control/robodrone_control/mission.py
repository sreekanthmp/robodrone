"""Pure-Python helpers for the offboard demo.

Nothing in here imports ROS, so it can be unit-tested anywhere
(``python3 -m pytest`` from the package directory).

Frame convention
----------------
MAVROS exposes local positions in ROS ENU: x = east, y = north, z = up.
It converts to/from the autopilot's NED frame internally. Everything in this
module is ENU, in metres.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Sequence, Tuple

Vec3 = Tuple[float, float, float]

# Hard ceilings. The ROS parameters of the demo node can be set lower than
# these but never higher; raising them means editing code on purpose.
HARD_MAX_ALTITUDE_M = 10.0
HARD_MAX_SQUARE_SIDE_M = 5.0


@dataclass(frozen=True)
class DemoLimits:
    """Validated, clamped demo parameters."""

    takeoff_altitude: float
    square_side: float
    tolerance: float


class InvalidParameter(ValueError):
    """Raised when a demo parameter is unusable (NaN, <= 0, ...)."""


def _require_positive(name: str, value: float) -> float:
    if not isinstance(value, (int, float)) or math.isnan(value) or value <= 0.0:
        raise InvalidParameter(f"{name} must be a positive number, got {value!r}")
    return float(value)


def validate_limits(
    takeoff_altitude: float, square_side: float, tolerance: float
) -> DemoLimits:
    """Reject nonsense and clamp to the hard ceilings above."""
    alt = min(_require_positive("takeoff_altitude", takeoff_altitude), HARD_MAX_ALTITUDE_M)
    side = min(_require_positive("square_side", square_side), HARD_MAX_SQUARE_SIDE_M)
    tol = _require_positive("position_tolerance", tolerance)
    return DemoLimits(takeoff_altitude=alt, square_side=side, tolerance=tol)


def square_waypoints(origin: Vec3, side: float, altitude_above_origin: float) -> List[Vec3]:
    """Four corners of a square, starting one ``side`` east of ``origin``.

    Path: east -> north -> west -> back to the start corner, all at
    ``origin.z + altitude_above_origin``. The final waypoint is directly above
    the origin so the drone ends where it began.
    """
    ox, oy, oz = origin
    z = oz + altitude_above_origin
    return [
        (ox + side, oy, z),
        (ox + side, oy + side, z),
        (ox, oy + side, z),
        (ox, oy, z),
    ]


def distance(a: Sequence[float], b: Sequence[float]) -> float:
    """Euclidean distance between two 3-vectors."""
    return math.sqrt(sum((p - q) ** 2 for p, q in zip(a, b)))


def reached(position: Sequence[float], target: Sequence[float], tolerance: float) -> bool:
    """True when ``position`` is within ``tolerance`` metres of ``target``."""
    return distance(position, target) <= tolerance


def altitude_reached(z: float, target_z: float, tolerance: float) -> bool:
    """Vertical-only check, used while waiting for takeoff to finish."""
    return abs(z - target_z) <= tolerance
