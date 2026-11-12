"""Uses REAL rclpy. Skipped automatically when ROS 2 is not installed.

These reproduce the bug that the fake-based tests cannot see: ROS 2 rejecting
`-p target_north_m:=60` (an integer) for a parameter declared with a decimal
default, which crashed the node at start-up.
"""

import pytest

rclpy = pytest.importorskip("rclpy")
from rclpy.exceptions import InvalidParameterTypeException  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402


def _args(*pairs):
    out = ["--ros-args"]
    for p in pairs:
        out += ["-p", p]
    return out


def test_plain_declare_rejects_an_integer_for_a_decimal_parameter():
    # The behaviour params.py works around. If a future ROS release stops doing
    # this, params.py is no longer needed and this test will say so.
    rclpy.init()
    try:
        node = Node("t_plain", parameter_overrides=[Parameter("x", value=60)])
        with pytest.raises(InvalidParameterTypeException):
            node.declare_parameter("x", 1.0)
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_helper_accepts_integer_and_decimal_overrides():
    from robodrone_control import params

    rclpy.init()
    try:
        node = Node("t_helper", parameter_overrides=[Parameter("a", value=60),
                                                     Parameter("b", value=2.5)])
        for name in ("a", "b", "c"):
            params.declare_number(node, name, 1.0)
        assert params.number(node, "a") == 60.0 and isinstance(params.number(node, "a"), float)
        assert params.number(node, "b") == 2.5
        assert params.number(node, "c") == 1.0               # default when not overridden
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_delivery_mission_starts_with_integer_command_line_parameters():
    """The exact form that crashed: `-p target_north_m:=60`."""
    rclpy.init(args=_args("target_north_m:=60", "target_east_m:=40", "cruise_altitude:=20",
                          "servo_channel:=7", "step_timeout:=30"))
    try:
        from robodrone_control import delivery_mission as dm

        node = dm.DeliveryMission()
        try:
            assert node.phase == dm.Phase.WAIT_FCU            # not ABORTED, not a crash
            assert node._target_params[2:] == (60.0, 40.0)
            assert node._limits.cruise_altitude == 20.0
            assert node._limits.servo_channel == 7
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


def test_delivery_mission_accepts_a_whole_decimal_for_the_servo_channel():
    rclpy.init(args=_args("target_north_m:=60", "target_east_m:=40", "servo_channel:=9.0"))
    try:
        from robodrone_control import delivery_mission as dm

        node = dm.DeliveryMission()
        try:
            assert node.phase == dm.Phase.WAIT_FCU and node._limits.servo_channel == 9
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


@pytest.mark.parametrize("arg", ["cruise_altitude:=abc", "servo_channel:=9.5", "servo_channel:=3"])
def test_delivery_mission_aborts_cleanly_on_a_bad_value(arg):
    rclpy.init(args=_args("target_north_m:=60", "target_east_m:=40", arg))
    try:
        from robodrone_control import delivery_mission as dm

        node = dm.DeliveryMission()
        try:
            assert node.phase == dm.Phase.ABORTED and node.failed   # message, not a traceback
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


def test_offboard_demo_and_stream_requester_accept_integers_too():
    rclpy.init(args=_args("takeoff_altitude:=2", "square_side:=3", "step_timeout:=20",
                          "rate:=10", "refresh_period:=30"))
    try:
        from robodrone_control import offboard_demo as od
        from robodrone_control import request_stream_rates as rs

        demo = od.OffboardDemo()
        req = rs.StreamRateRequester()
        try:
            assert demo.phase == od.Phase.WAIT_FCU
            assert demo._limits.takeoff_altitude == 2.0 and demo._limits.square_side == 3.0
            assert req._rate == 10 and req._refresh == 30.0
        finally:
            demo.destroy_node()
            req.destroy_node()
    finally:
        rclpy.shutdown()
