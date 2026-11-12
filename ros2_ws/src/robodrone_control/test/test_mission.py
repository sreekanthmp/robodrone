import math

import pytest

from robodrone_control import mission


def test_square_is_closed_and_level():
    origin = (1.0, 2.0, 0.5)
    pts = mission.square_waypoints(origin, side=3.0, altitude_above_origin=2.0)

    assert len(pts) == 4
    assert {p[2] for p in pts} == {2.5}  # level at origin.z + altitude
    # ends directly above where it started
    assert pts[-1][:2] == origin[:2]
    # every leg is exactly one side long
    corners = [(origin[0], origin[1], 2.5)] + pts
    for a, b in zip(corners, corners[1:]):
        assert mission.distance(a, b) == pytest.approx(3.0)


def test_square_first_leg_goes_east():
    pts = mission.square_waypoints((0.0, 0.0, 0.0), 2.0, 1.0)
    assert pts[0] == (2.0, 0.0, 1.0)  # +x is east in ENU


def test_limits_clamp_to_hard_ceilings():
    lim = mission.validate_limits(takeoff_altitude=100.0, square_side=100.0, tolerance=0.3)
    assert lim.takeoff_altitude == mission.HARD_MAX_ALTITUDE_M
    assert lim.square_side == mission.HARD_MAX_SQUARE_SIDE_M
    assert lim.tolerance == 0.3


def test_limits_pass_through_when_below_ceiling():
    lim = mission.validate_limits(2.0, 1.5, 0.25)
    assert (lim.takeoff_altitude, lim.square_side, lim.tolerance) == (2.0, 1.5, 0.25)


@pytest.mark.parametrize("bad", [0.0, -1.0, float("nan")])
@pytest.mark.parametrize("field", ["takeoff_altitude", "square_side", "tolerance"])
def test_limits_reject_unusable_values(field, bad):
    kwargs = dict(takeoff_altitude=2.0, square_side=2.0, tolerance=0.3)
    kwargs[field] = bad
    with pytest.raises(mission.InvalidParameter):
        mission.validate_limits(**kwargs)


def test_limits_reject_non_numbers():
    with pytest.raises(mission.InvalidParameter):
        mission.validate_limits("2", 2.0, 0.3)  # type: ignore[arg-type]


def test_reached_uses_3d_distance():
    assert mission.reached((0, 0, 0), (0.1, 0.1, 0.1), 0.3)
    assert not mission.reached((0, 0, 0), (0.3, 0.3, 0.3), 0.3)  # ~0.52 m away


def test_altitude_reached_ignores_horizontal_error():
    assert mission.altitude_reached(1.9, 2.0, 0.2)
    assert not mission.altitude_reached(1.5, 2.0, 0.2)


def test_distance_is_symmetric():
    a, b = (1.0, 2.0, 3.0), (4.0, 6.0, 3.0)
    assert mission.distance(a, b) == mission.distance(b, a) == pytest.approx(5.0)
    assert not math.isnan(mission.distance(a, a))


# ---------------------------------------------------------------------------
# Delivery helpers
# ---------------------------------------------------------------------------

NAN = float("nan")
GOOD = dict(
    cruise_altitude=15.0, drop_altitude=3.0, max_range=100.0,
    horizontal_tolerance=2.0, vertical_tolerance=0.5,
    servo_channel=9, hold_pwm=1100, release_pwm=1900, release_time=2.0,
)


def _haversine_m(lat0, lon0, lat1, lon1, r=6371008.8):
    p0, p1 = math.radians(lat0), math.radians(lat1)
    dp, dl = p1 - p0, math.radians(lon1 - lon0)
    h = math.sin(dp / 2) ** 2 + math.cos(p0) * math.cos(p1) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(math.sqrt(h))


def test_enu_equator_degree_lengths():
    east, north = mission.enu_offset(0.0, 0.0, 1.0, 1.0)
    assert north == pytest.approx(110_574, abs=700)   # ~1 deg latitude at the equator
    assert east == pytest.approx(111_319, abs=700)    # ~1 deg longitude at the equator


def test_enu_at_45_degrees_longitude_shrinks_by_cos_latitude():
    east, north = mission.enu_offset(45.0, 10.0, 45.0, 11.0)
    assert north == pytest.approx(0.0, abs=1e-6)
    assert east == pytest.approx(78_846, abs=100)     # ~1 deg longitude at 45 N
    _, north = mission.enu_offset(45.0, 10.0, 46.0, 10.0)
    assert north == pytest.approx(111_132, abs=150)   # ~1 deg latitude at 45 N


def test_enu_signs_and_zero():
    assert mission.enu_offset(10.0, 20.0, 10.0, 20.0) == (0.0, 0.0)
    east, north = mission.enu_offset(10.0, 20.0, 9.999, 19.999)  # south-west
    assert east < 0 and north < 0
    east, north = mission.enu_offset(10.0, 20.0, 10.001, 20.001)  # north-east
    assert east > 0 and north > 0


def test_enu_agrees_with_an_independent_spherical_calculation():
    # SITL's default start (Canberra, southern hemisphere, east longitude).
    lat0, lon0 = -35.363261, 149.165230
    for dlat, dlon in [(0.009, 0.0), (0.0, 0.011), (0.006, -0.008), (-0.004, 0.005)]:
        east, north = mission.enu_offset(lat0, lon0, lat0 + dlat, lon0 + dlon)
        expected = _haversine_m(lat0, lon0, lat0 + dlat, lon0 + dlon)
        assert math.hypot(east, north) == pytest.approx(expected, rel=0.005)


def test_enu_is_antisymmetric():
    a = mission.enu_offset(-35.36, 149.16, -35.35, 149.18)
    b = mission.enu_offset(-35.35, 149.18, -35.36, 149.16)
    assert a[0] == pytest.approx(-b[0], abs=0.01) and a[1] == pytest.approx(-b[1], abs=0.01)


def test_enu_wraps_across_the_antimeridian():
    east, _ = mission.enu_offset(0.0, 179.9999, 0.0, -179.9999)
    assert east == pytest.approx(22.3, abs=0.5)  # 0.0002 deg of longitude, not ~40,000 km


def test_distance_xy_ignores_height():
    assert mission.distance_xy((0, 0, 0), (3, 4, 100)) == pytest.approx(5.0)


def test_resolve_target_from_offsets_is_east_then_north():
    assert mission.resolve_target(0.0, 0.0, NAN, NAN, north_m=60.0, east_m=40.0) == (40.0, 60.0)


def test_resolve_target_from_gps():
    e, n = mission.resolve_target(-35.363261, 149.165230, -35.362361, 149.165230, NAN, NAN)
    assert e == pytest.approx(0.0, abs=0.01) and n == pytest.approx(99.9, abs=0.6)


@pytest.mark.parametrize(
    "lat, lon, north, east",
    [
        (-35.36, 149.16, 10.0, 10.0),   # both ways given
        (NAN, NAN, NAN, NAN),           # neither
        (-35.36, NAN, NAN, NAN),        # half a GPS position
        (NAN, 149.16, NAN, NAN),
        (NAN, NAN, 10.0, NAN),          # half an offset
        (NAN, NAN, NAN, 10.0),
        (95.0, 10.0, NAN, NAN),         # impossible latitude
        (10.0, 190.0, NAN, NAN),        # impossible longitude
        (NAN, NAN, float("inf"), 5.0),  # not finite
    ],
)
def test_resolve_target_rejects_ambiguous_or_invalid(lat, lon, north, east):
    with pytest.raises(mission.InvalidParameter):
        mission.resolve_target(-35.36, 149.16, lat, lon, north, east)


def test_delivery_limits_pass_through():
    lim = mission.validate_delivery(**GOOD)
    assert (lim.cruise_altitude, lim.drop_altitude, lim.max_range) == (15.0, 3.0, 100.0)
    assert (lim.servo_channel, lim.hold_pwm, lim.release_pwm) == (9, 1100, 1900)


def test_delivery_limits_clamp_to_hard_ceilings():
    lim = mission.validate_delivery(**{**GOOD, "cruise_altitude": 500.0, "max_range": 99999.0})
    assert lim.cruise_altitude == mission.HARD_MAX_CRUISE_ALTITUDE_M
    assert lim.max_range == mission.HARD_MAX_RANGE_M


@pytest.mark.parametrize(
    "override",
    [
        {"drop_altitude": 1.0},          # lower than the minimum drop height
        {"drop_altitude": 20.0},         # higher than cruise
        {"cruise_altitude": 0.0},
        {"cruise_altitude": NAN},
        {"max_range": -5.0},
        {"horizontal_tolerance": 0.0},
        {"vertical_tolerance": 0.0},
        {"vertical_tolerance": 2.9},     # would let it settle below the safe drop height
        {"servo_channel": 1},            # motor outputs are off limits
        {"servo_channel": 4},
        {"servo_channel": 17},
        {"servo_channel": True},
        {"servo_channel": 9.0},
        {"hold_pwm": 500},
        {"release_pwm": 2500},
        {"release_pwm": 1200},           # too close to hold_pwm (1100)
        {"hold_pwm": "1100"},
        {"release_time": 0.0},
        {"release_time": 31.0},
    ],
)
def test_delivery_limits_reject_unsafe_or_unusable_values(override):
    with pytest.raises(mission.InvalidParameter):
        mission.validate_delivery(**{**GOOD, **override})


def test_target_range_checks():
    lim = mission.validate_delivery(**GOOD)
    assert mission.check_target_range(30.0, 40.0, lim) == pytest.approx(50.0)
    with pytest.raises(mission.InvalidParameter):
        mission.check_target_range(1.0, 1.0, lim)       # too close: likely a typo
    with pytest.raises(mission.InvalidParameter):
        mission.check_target_range(90.0, 90.0, lim)     # ~127 m > max_range 100
