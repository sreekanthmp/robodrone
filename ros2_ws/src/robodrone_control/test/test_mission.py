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
