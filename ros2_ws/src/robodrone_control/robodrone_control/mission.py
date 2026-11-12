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


# ---------------------------------------------------------------------------
# Delivery mission helpers
# ---------------------------------------------------------------------------

# Hard ceilings for the delivery mission. As with the demo limits, a parameter
# can ask for less but never more; raising these means editing code on purpose.
HARD_MAX_CRUISE_ALTITUDE_M = 30.0
HARD_MAX_RANGE_M = 500.0
MIN_TARGET_DISTANCE_M = 5.0   # a "delivery" closer than this is almost certainly a typo
MIN_DROP_ALTITUDE_M = 1.5     # never descend lower than this above the launch point
MIN_SERVO_CHANNEL = 5         # outputs 1-4 are motors on a quad: never command them
MAX_SERVO_CHANNEL = 16
MIN_PWM_US, MAX_PWM_US = 800, 2200
MIN_PWM_SEPARATION_US = 200   # hold and release must be clearly different
MAX_RELEASE_TIME_S = 30.0

# WGS84
_A = 6378137.0
_E2 = 6.69437999014e-3


def enu_offset(lat0: float, lon0: float, lat1: float, lon1: float) -> Tuple[float, float]:
    """(east_m, north_m) from point 0 to point 1 on the WGS84 ellipsoid.

    Local-tangent-plane approximation evaluated at the mid latitude: centimetre
    accurate over a few kilometres, far more than a GPS needs. Not for
    intercontinental distances. Longitude difference is wrapped to +-180 so a
    route across the antimeridian still works.
    """
    phi = math.radians((lat0 + lat1) / 2.0)
    w = math.sqrt(1.0 - _E2 * math.sin(phi) ** 2)
    radius_prime_vertical = _A / w
    radius_meridian = _A * (1.0 - _E2) / (w ** 3)
    dlon = (lon1 - lon0 + 180.0) % 360.0 - 180.0
    east = math.radians(dlon) * radius_prime_vertical * math.cos(phi)
    north = math.radians(lat1 - lat0) * radius_meridian
    return east, north


def distance_xy(a: Sequence[float], b: Sequence[float]) -> float:
    """Horizontal distance between two points (ignores z)."""
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _valid_lat(v: float) -> bool:
    return isinstance(v, (int, float)) and not math.isnan(v) and -90.0 <= v <= 90.0


def _valid_lon(v: float) -> bool:
    return isinstance(v, (int, float)) and not math.isnan(v) and -180.0 <= v <= 180.0


def _isnan(v: float) -> bool:
    return isinstance(v, float) and math.isnan(v)


def resolve_target(
    origin_lat: float,
    origin_lon: float,
    target_lat: float,
    target_lon: float,
    north_m: float,
    east_m: float,
) -> Tuple[float, float]:
    """Where to deliver, as (east_m, north_m) from the launch point.

    Exactly one way of saying it is accepted: a GPS position
    (``target_lat`` + ``target_lon``) or an offset from launch
    (``north_m`` + ``east_m``). Unset values are NaN. Giving both, neither, or
    half of one is an error, because guessing which the operator meant is how a
    delivery ends up in the wrong place.
    """
    ll = [not _isnan(target_lat), not _isnan(target_lon)]
    off = [not _isnan(north_m), not _isnan(east_m)]
    if any(ll) and not all(ll):
        raise InvalidParameter("target_lat and target_lon must be given together")
    if any(off) and not all(off):
        raise InvalidParameter("target_north_m and target_east_m must be given together")
    if all(ll) == all(off):
        raise InvalidParameter(
            "give the target either as target_lat+target_lon or as "
            "target_north_m+target_east_m (not both, not neither)"
        )
    if all(ll):
        if not (_valid_lat(target_lat) and _valid_lon(target_lon)):
            raise InvalidParameter(f"target {target_lat!r}, {target_lon!r} is not a valid position")
        if not (_valid_lat(origin_lat) and _valid_lon(origin_lon)):
            raise InvalidParameter("vehicle position is not valid")
        return enu_offset(origin_lat, origin_lon, target_lat, target_lon)
    if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in (north_m, east_m)):
        raise InvalidParameter("target_north_m / target_east_m must be finite numbers")
    return float(east_m), float(north_m)


@dataclass(frozen=True)
class DeliveryLimits:
    """Validated, clamped delivery-mission parameters."""

    cruise_altitude: float
    drop_altitude: float
    max_range: float
    horizontal_tolerance: float   # GPS wanders by metres: do not make this too tight
    vertical_tolerance: float
    servo_channel: int
    hold_pwm: int
    release_pwm: int
    release_time: float


def validate_delivery(
    cruise_altitude: float,
    drop_altitude: float,
    max_range: float,
    horizontal_tolerance: float,
    vertical_tolerance: float,
    servo_channel: int,
    hold_pwm: int,
    release_pwm: int,
    release_time: float,
) -> DeliveryLimits:
    """Reject nonsense and clamp to the hard ceilings above."""
    cruise = min(_require_positive("cruise_altitude", cruise_altitude), HARD_MAX_CRUISE_ALTITUDE_M)
    drop = _require_positive("drop_altitude", drop_altitude)
    if drop < MIN_DROP_ALTITUDE_M:
        raise InvalidParameter(f"drop_altitude must be at least {MIN_DROP_ALTITUDE_M} m")
    if drop > cruise:
        raise InvalidParameter("drop_altitude must not be higher than cruise_altitude")
    rng = min(_require_positive("max_range", max_range), HARD_MAX_RANGE_M)
    htol = _require_positive("horizontal_tolerance", horizontal_tolerance)
    vtol = _require_positive("vertical_tolerance", vertical_tolerance)
    if vtol > drop - 0.5 * MIN_DROP_ALTITUDE_M:
        raise InvalidParameter("vertical_tolerance is too large for the drop altitude")
    if (
        isinstance(servo_channel, bool)
        or not isinstance(servo_channel, int)
        or not MIN_SERVO_CHANNEL <= servo_channel <= MAX_SERVO_CHANNEL
    ):
        raise InvalidParameter(
            f"servo_channel must be an integer {MIN_SERVO_CHANNEL}-{MAX_SERVO_CHANNEL} "
            "(outputs 1-4 drive the motors)"
        )
    for name, pwm in (("hold_pwm", hold_pwm), ("release_pwm", release_pwm)):
        if isinstance(pwm, bool) or not isinstance(pwm, int) or not MIN_PWM_US <= pwm <= MAX_PWM_US:
            raise InvalidParameter(f"{name} must be an integer {MIN_PWM_US}-{MAX_PWM_US} microseconds")
    if abs(hold_pwm - release_pwm) < MIN_PWM_SEPARATION_US:
        raise InvalidParameter(
            f"hold_pwm and release_pwm must differ by at least {MIN_PWM_SEPARATION_US} us"
        )
    rel = _require_positive("release_time", release_time)
    if rel > MAX_RELEASE_TIME_S:
        raise InvalidParameter(f"release_time must be at most {MAX_RELEASE_TIME_S} s")
    return DeliveryLimits(
        cruise_altitude=cruise,
        drop_altitude=drop,
        max_range=rng,
        horizontal_tolerance=htol,
        vertical_tolerance=vtol,
        servo_channel=servo_channel,
        hold_pwm=hold_pwm,
        release_pwm=release_pwm,
        release_time=rel,
    )


def check_target_range(east_m: float, north_m: float, limits: DeliveryLimits) -> float:
    """Return the target distance, or raise if it is too near or beyond ``max_range``."""
    d = math.hypot(east_m, north_m)
    if d < MIN_TARGET_DISTANCE_M:
        raise InvalidParameter(
            f"target is only {d:.1f} m from launch (minimum {MIN_TARGET_DISTANCE_M} m)"
        )
    if d > limits.max_range:
        raise InvalidParameter(
            f"target is {d:.0f} m from launch, beyond max_range {limits.max_range:.0f} m"
        )
    return d
