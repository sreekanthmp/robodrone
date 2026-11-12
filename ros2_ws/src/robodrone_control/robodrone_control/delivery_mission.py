"""Fly a delivery in ArduPilot GUIDED mode through MAVROS.

Sequence: check GPS and the target -> latch payload -> GUIDED -> arm -> takeoff
-> transit to the target at cruise altitude -> descend -> release the payload
(servo) -> climb -> return to the launch point -> LAND -> disarm.

Safety behaviour (covered by test/test_delivery_mission.py):

* **Dry run by default.** Without ``enable_flight:=true`` the node only checks
  the link, GPS and the target, prints where it would go, and exits.
* **Refuses unclear or out-of-bounds targets.** Exactly one of a GPS target or
  a north/east offset, between 5 m and ``max_range`` (hard cap 500 m) from the
  launch point. Cruise altitude is capped at 30 m.
* **The pilot always wins.** If the flight mode ever leaves GUIDED while we are
  flying, the node assumes the pilot took over, sends nothing further and exits.
* **Failsafe goes home.** Any fault while armed and flying (timeout, rejected or
  unconfirmed command, flight-time limit, Ctrl-C) commands ``failsafe_mode``
  (default RTL, so a failure 300 m out does not land you in a field), falling
  back to LAND if that is rejected, and shouting if even LAND cannot be sent.
* **A release only counts if the servo output actually changed.** The node
  sends DO_SET_SERVO and then watches ``/mavros/rc/out``. If that does not show
  the new PWM, the delivery is NOT recorded as made and the drone comes home.
  This confirms the autopilot's *output*, not that the payload left the
  aircraft: fit your own sensor if that matters.
* **Never touches motor outputs.** servo_channel must be 5-16.
* Exit code 0 only if the payload was released, the drone landed, and nothing
  went wrong along the way.

Frames: MAVROS local positions are ENU (x east, y north, z up) relative to the
EKF origin. Altitudes here are **relative to the launch height**; there is no
terrain following, so a target on a hill or in a hollow is not compensated.
"""

from __future__ import annotations

import enum
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import RCOut, State
from mavros_msgs.srv import CommandBool, CommandLong, CommandTOL, SetMode
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import NavSatFix

from robodrone_control import mission

TICK_HZ = 10.0
DISARM_TIMEOUT_S = 300.0          # an RTL can be long
MIN_TRANSIT_SPEED_MS = 1.0        # only used to scale the transit timeout
FIX_MAX_AGE_S = 3.0
PWM_CONFIRM_TOL_US = 50
MAV_CMD_DO_SET_SERVO = 183
MAX_LAND_ATTEMPTS = 3
LAND_MODE = "LAND"


class Phase(enum.Enum):
    WAIT_FCU = enum.auto()
    WAIT_FIX = enum.auto()
    HOLD_PAYLOAD = enum.auto()
    SET_GUIDED = enum.auto()
    ARM = enum.auto()
    TAKEOFF_CMD = enum.auto()
    CLIMB = enum.auto()
    TRANSIT = enum.auto()
    DESCEND = enum.auto()
    RELEASE_CMD = enum.auto()
    RELEASE_WAIT = enum.auto()
    ASCEND = enum.auto()
    RETURN = enum.auto()
    LAND_CMD = enum.auto()        # normal landing, or the failsafe (RTL/LAND) command
    WAIT_DISARM = enum.auto()
    DONE = enum.auto()
    ABORTED = enum.auto()


TERMINAL = frozenset({Phase.DONE, Phase.ABORTED})
# Armed (or about to be) in GUIDED, mission in progress: the pilot may override.
FLYING = frozenset(
    {
        Phase.ARM,
        Phase.TAKEOFF_CMD,
        Phase.CLIMB,
        Phase.TRANSIT,
        Phase.DESCEND,
        Phase.RELEASE_CMD,
        Phase.RELEASE_WAIT,
        Phase.ASCEND,
        Phase.RETURN,
    }
)

_PENDING = object()


class DeliveryMission(Node):
    def __init__(self) -> None:
        super().__init__("delivery_mission")

        nan = float("nan")
        self.declare_parameter("enable_flight", False)
        self.declare_parameter("target_lat", nan)
        self.declare_parameter("target_lon", nan)
        self.declare_parameter("target_north_m", nan)
        self.declare_parameter("target_east_m", nan)
        self.declare_parameter("cruise_altitude", 15.0)
        self.declare_parameter("drop_altitude", 3.0)
        self.declare_parameter("max_range", 100.0)
        self.declare_parameter("horizontal_tolerance", 2.0)
        self.declare_parameter("vertical_tolerance", 0.5)
        self.declare_parameter("servo_channel", 9)
        self.declare_parameter("hold_pwm", 1100)
        self.declare_parameter("release_pwm", 1900)
        self.declare_parameter("release_time", 2.0)
        self.declare_parameter("confirm_timeout", 3.0)
        self.declare_parameter("step_timeout", 60.0)
        self.declare_parameter("max_flight_time", 600.0)
        self.declare_parameter("failsafe_mode", "RTL")
        self.declare_parameter("guided_mode", "GUIDED")
        self.declare_parameter("mavros_ns", "/mavros")

        def p(name):
            return self.get_parameter(name).value

        self._enable_flight = bool(p("enable_flight"))
        self._guided = str(p("guided_mode"))
        self._timeout = float(p("step_timeout"))
        self._confirm_timeout = float(p("confirm_timeout"))
        self._max_flight_time = float(p("max_flight_time"))
        self._failsafe_mode = str(p("failsafe_mode")).upper()
        self._target_params = (p("target_lat"), p("target_lon"), p("target_north_m"), p("target_east_m"))
        ns = str(p("mavros_ns")).rstrip("/")

        self._state: State | None = None
        self._pos: tuple[float, float, float] | None = None
        self._orientation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
        self._fix: NavSatFix | None = None
        self._fix_time = 0.0
        self._rc_out: list[int] | None = None
        self._rc_out_time = -1.0
        self._origin: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._target_xy: tuple[float, float] = (0.0, 0.0)
        self._distance = 0.0
        self._t_armed: float | None = None
        self._ack_time = 0.0
        self._land_plan: list[str] = []
        self._land_idx = 0
        self._future = None
        self._issued = False
        self._acked = False
        self._t0 = 0.0
        self._timer = None
        self.phase = Phase.WAIT_FCU
        # Anything went wrong (even if we then got home safely) -> non-zero exit.
        self.failed = False
        # True only once a release was commanded AND the servo output confirmed it.
        self.delivered = False

        self._limits: mission.DeliveryLimits | None = None
        try:
            if self._failsafe_mode not in ("RTL", LAND_MODE):
                raise mission.InvalidParameter("failsafe_mode must be RTL or LAND")
            if not (self._confirm_timeout > 0 and self._timeout > 0 and self._max_flight_time > 0):
                raise mission.InvalidParameter(
                    "confirm_timeout, step_timeout and max_flight_time must be positive"
                )
            self._limits = mission.validate_delivery(
                p("cruise_altitude"),
                p("drop_altitude"),
                p("max_range"),
                p("horizontal_tolerance"),
                p("vertical_tolerance"),
                p("servo_channel"),
                p("hold_pwm"),
                p("release_pwm"),
                p("release_time"),
            )
        except mission.InvalidParameter as exc:
            self.get_logger().error(f"bad parameter: {exc}")
            self.failed = True
            self.phase = Phase.ABORTED
            return

        self.create_subscription(State, f"{ns}/state", self._on_state, 10)
        # Best-effort subscribers connect to best-effort AND reliable publishers.
        self.create_subscription(
            PoseStamped, f"{ns}/local_position/pose", self._on_pose, qos_profile_sensor_data
        )
        self.create_subscription(
            NavSatFix, f"{ns}/global_position/global", self._on_fix, qos_profile_sensor_data
        )
        self.create_subscription(RCOut, f"{ns}/rc/out", self._on_rc_out, qos_profile_sensor_data)
        # MAVROS' setpoint subscriber is BEST_EFFORT, so a default RELIABLE publisher is fine.
        self._setpoint_pub = self.create_publisher(
            PoseStamped, f"{ns}/setpoint_position/local", 10
        )
        self._cli_mode = self.create_client(SetMode, f"{ns}/set_mode")
        self._cli_arm = self.create_client(CommandBool, f"{ns}/cmd/arming")
        self._cli_takeoff = self.create_client(CommandTOL, f"{ns}/cmd/takeoff")
        self._cli_command = self.create_client(CommandLong, f"{ns}/cmd/command")

        self._t0 = self._now()
        self._timer = self.create_timer(1.0 / TICK_HZ, self._tick)
        self.get_logger().info(
            f"delivery_mission ready ({'FLIGHT ENABLED' if self._enable_flight else 'dry run'}): "
            f"cruise={self._limits.cruise_altitude} m, drop={self._limits.drop_altitude} m, "
            f"max_range={self._limits.max_range} m, servo={self._limits.servo_channel}, "
            f"failsafe={self._failsafe_mode}"
        )

    # -- plumbing ---------------------------------------------------------

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_state(self, msg: State) -> None:
        self._state = msg

    def _on_pose(self, msg: PoseStamped) -> None:
        p, q = msg.pose.position, msg.pose.orientation
        self._pos = (p.x, p.y, p.z)
        self._orientation = (q.x, q.y, q.z, q.w)

    def _on_fix(self, msg: NavSatFix) -> None:
        self._fix = msg
        self._fix_time = self._now()

    def _on_rc_out(self, msg: RCOut) -> None:
        self._rc_out = list(msg.channels)
        self._rc_out_time = self._now()

    def _enter(self, phase: Phase) -> None:
        self.get_logger().info(f"phase: {self.phase.name} -> {phase.name}")
        self.phase = phase
        self._future = None
        self._issued = False
        self._acked = False
        self._t0 = self._now()

    def _finish(self, phase: Phase, message: str, error: bool = False) -> None:
        (self.get_logger().error if error else self.get_logger().info)(message)
        self.failed = self.failed or error
        self.phase = phase
        if self._timer is not None:
            self._timer.cancel()

    def _armed(self) -> bool:
        return self._state is not None and bool(self._state.armed)

    def _start(self, client, request) -> bool:
        """Send ``request`` once the service is up. False means: try next tick."""
        if not client.service_is_ready():
            return False
        self._future = client.call_async(request)
        self._issued = True
        return True

    def _poll(self):
        """``_PENDING`` while waiting, else the response. Raises on call failure."""
        if self._future is None or not self._future.done():
            return _PENDING
        future, self._future = self._future, None
        exc = future.exception()
        if exc is not None:
            raise RuntimeError(f"service call failed: {exc}")
        return future.result()

    def _servo_command(self, pwm: int):
        request = CommandLong.Request()
        request.broadcast = False
        request.command = MAV_CMD_DO_SET_SERVO
        request.confirmation = 0
        request.param1 = float(self._limits.servo_channel)
        request.param2 = float(pwm)
        return request

    def _servo_at(self, pwm: int) -> bool:
        """True if a /mavros/rc/out sample taken AFTER the command shows ``pwm``."""
        idx = self._limits.servo_channel - 1
        return (
            self._rc_out is not None
            and idx < len(self._rc_out)
            and self._rc_out_time >= self._ack_time
            and abs(self._rc_out[idx] - pwm) <= PWM_CONFIRM_TOL_US
        )

    def _at(self, target: tuple[float, float, float]) -> bool:
        if self._pos is None:
            return False
        return (
            mission.distance_xy(self._pos, target) <= self._limits.horizontal_tolerance
            and mission.altitude_reached(self._pos[2], target[2], self._limits.vertical_tolerance)
        )

    # -- targets ------------------------------------------------------------

    def _above_target(self, altitude: float) -> tuple[float, float, float]:
        return (self._target_xy[0], self._target_xy[1], self._origin[2] + altitude)

    def _above_origin(self, altitude: float) -> tuple[float, float, float]:
        return (self._origin[0], self._origin[1], self._origin[2] + altitude)

    # -- failure handling ---------------------------------------------------

    def _begin_land(self, failsafe: bool) -> None:
        if failsafe and self._failsafe_mode != LAND_MODE:
            self._land_plan = [self._failsafe_mode] + [LAND_MODE] * MAX_LAND_ATTEMPTS
        else:
            self._land_plan = [LAND_MODE] * MAX_LAND_ATTEMPTS
        self._land_idx = 0
        self._enter(Phase.LAND_CMD)

    def _advance_land(self, reason: str) -> None:
        self.get_logger().warn(f"{self._land_plan[self._land_idx]} not commanded: {reason}")
        self._land_idx += 1
        if self._land_idx >= len(self._land_plan):
            self._finish(
                Phase.ABORTED, "COULD NOT COMMAND RTL/LAND - TAKE OVER MANUALLY NOW", error=True
            )
        else:
            self._enter(Phase.LAND_CMD)

    def _fail(self, reason: str) -> None:
        self.get_logger().error(reason)
        self.failed = True
        if self.phase in FLYING and self._armed():
            self.get_logger().warn(
                f"armed and flying: commanding {self._failsafe_mode} "
                f"(payload {'DELIVERED' if self.delivered else 'NOT delivered'})"
            )
            self._begin_land(failsafe=True)
        elif self.phase == Phase.LAND_CMD and self._armed():
            self._advance_land(reason)
        else:
            self._finish(Phase.ABORTED, f"aborted in {self.phase.name}", error=True)

    def emergency_failsafe(self) -> None:
        """Best-effort RTL/LAND on Ctrl-C. Does nothing if we are not the ones flying."""
        if self.phase not in FLYING or not self._armed():
            return
        self.get_logger().warn(f"interrupted while flying: commanding {self._failsafe_mode}")
        modes = [self._failsafe_mode] if self._failsafe_mode == LAND_MODE else [self._failsafe_mode, LAND_MODE]
        for mode in modes:
            request = SetMode.Request()
            request.base_mode = 0
            request.custom_mode = mode
            future = self._cli_mode.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            # Only fall back to LAND on an explicit rejection. No answer in time
            # usually just means the RTL was accepted slowly; do not override it.
            if not (future.done() and future.exception() is None and not future.result().mode_sent):
                return

    # -- state machine ------------------------------------------------------

    def _limit(self) -> float:
        if self.phase == Phase.WAIT_DISARM:
            return DISARM_TIMEOUT_S
        if self.phase in (Phase.TRANSIT, Phase.RETURN):
            return self._timeout + self._distance / MIN_TRANSIT_SPEED_MS
        return self._timeout

    def _tick(self) -> None:
        if self.phase in TERMINAL:
            return

        # 1. The pilot overrides everything.
        if (
            self.phase in FLYING
            and self._state is not None
            and self._state.mode != self._guided
        ):
            self._finish(
                Phase.ABORTED,
                f"mode changed to {self._state.mode}: assuming the pilot took control, "
                "sending nothing further",
                error=True,
            )
            return

        now = self._now()

        # 2. Total flight time limit (battery margin; ArduPilot's own failsafes still apply).
        if (
            self.phase in FLYING
            and self._t_armed is not None
            and now - self._t_armed > self._max_flight_time
        ):
            self._fail(f"flight time limit of {self._max_flight_time:.0f} s exceeded")
            return

        # 3. Per-step timeout.
        limit = self._limit()
        if now - self._t0 > limit:
            self._fail(f"timeout in {self.phase.name} after {limit:.0f} s")
            return

        # 4. This phase's work.
        try:
            getattr(self, f"_do_{self.phase.name.lower()}")()
        except RuntimeError as exc:
            self._fail(str(exc))

    def _do_wait_fcu(self) -> None:
        if self._state is not None and self._state.connected:
            self._enter(Phase.WAIT_FIX)

    def _do_wait_fix(self) -> None:
        if self._pos is None or self._fix is None:
            return
        if self._now() - self._fix_time > FIX_MAX_AGE_S or self._fix.status.status < 0:
            return  # stale or no GPS fix: keep waiting; the step timeout ends this
        try:
            east, north = mission.resolve_target(
                self._fix.latitude, self._fix.longitude, *self._target_params
            )
            self._distance = mission.check_target_range(east, north, self._limits)
        except mission.InvalidParameter as exc:
            self._fail(f"bad target: {exc}")
            return
        self._origin = self._pos
        self._target_xy = (self._origin[0] + east, self._origin[1] + north)
        self.get_logger().info(
            f"target is {self._distance:.1f} m away (east {east:.1f} m, north {north:.1f} m)"
        )
        if not self._enable_flight:
            self._finish(
                Phase.DONE,
                "dry run OK: FCU connected, GPS fix, target accepted. "
                "Re-run with -p enable_flight:=true to fly.",
            )
            return
        if self._armed():
            self._fail("vehicle is already armed: refusing to start")
            return
        self._enter(Phase.HOLD_PAYLOAD)

    def _do_hold_payload(self) -> None:
        """Latch the payload and confirm the servo output follows, BEFORE arming."""
        if not self._issued:
            self._start(self._cli_command, self._servo_command(self._limits.hold_pwm))
            return
        if not self._acked:
            response = self._poll()
            if response is _PENDING:
                return
            if not response.success:
                raise RuntimeError("payload servo hold command rejected")
            self._acked = True
            self._ack_time = self._now()
            self._t0 = self._ack_time
        if self._servo_at(self._limits.hold_pwm):
            self._enter(Phase.SET_GUIDED)
        elif self._now() - self._ack_time > self._confirm_timeout:
            raise RuntimeError(
                "payload servo output did not reach the hold position "
                "(wrong servo_channel, or SERVOn_FUNCTION is not 0?). Not arming."
            )

    def _do_set_guided(self) -> None:
        if not self._issued:
            request = SetMode.Request()
            request.base_mode = 0
            request.custom_mode = self._guided
            self._start(self._cli_mode, request)
            return
        if not self._acked:
            response = self._poll()
            if response is _PENDING:
                return
            if not response.mode_sent:
                raise RuntimeError(f"set_mode {self._guided} was rejected")
            self._acked = True
        if self._state is not None and self._state.mode == self._guided:
            self._enter(Phase.ARM)

    def _do_arm(self) -> None:
        if not self._issued:
            request = CommandBool.Request()
            request.value = True
            self._start(self._cli_arm, request)
            return
        if not self._acked:
            response = self._poll()
            if response is _PENDING:
                return
            if not response.success:
                raise RuntimeError("arming rejected (check ArduPilot pre-arm messages)")
            self._acked = True
        if self._armed():
            self._t_armed = self._now()
            self._enter(Phase.TAKEOFF_CMD)

    def _do_takeoff_cmd(self) -> None:
        if not self._issued:
            request = CommandTOL.Request()
            request.min_pitch = 0.0
            request.yaw = 0.0
            request.latitude = 0.0
            request.longitude = 0.0
            request.altitude = float(self._limits.cruise_altitude)
            self._start(self._cli_takeoff, request)
            return
        response = self._poll()
        if response is _PENDING:
            return
        if not response.success:
            raise RuntimeError("takeoff command rejected")
        self._enter(Phase.CLIMB)

    def _do_climb(self) -> None:
        if self._pos is None:
            return
        target_z = self._origin[2] + self._limits.cruise_altitude
        if mission.altitude_reached(self._pos[2], target_z, self._limits.vertical_tolerance):
            self._enter(Phase.TRANSIT)

    def _do_transit(self) -> None:
        target = self._above_target(self._limits.cruise_altitude)
        self._publish_setpoint(target)
        if self._at(target):
            self._enter(Phase.DESCEND)

    def _do_descend(self) -> None:
        target = self._above_target(self._limits.drop_altitude)
        self._publish_setpoint(target)
        if self._at(target):
            self._enter(Phase.RELEASE_CMD)

    def _do_release_cmd(self) -> None:
        self._publish_setpoint(self._above_target(self._limits.drop_altitude))
        if not self._issued:
            self._start(self._cli_command, self._servo_command(self._limits.release_pwm))
            return
        if not self._acked:
            response = self._poll()
            if response is _PENDING:
                return
            if not response.success:
                raise RuntimeError("payload release command rejected: bringing the payload home")
            self._acked = True
            self._ack_time = self._now()
            self._t0 = self._ack_time
        if self._servo_at(self._limits.release_pwm):
            self.delivered = True
            self.get_logger().info("payload RELEASED (servo output confirmed)")
            self._enter(Phase.RELEASE_WAIT)
        elif self._now() - self._ack_time > self._confirm_timeout:
            raise RuntimeError(
                "RELEASE NOT CONFIRMED: servo output did not change. Delivery not recorded; "
                "bringing the payload home"
            )

    def _do_release_wait(self) -> None:
        self._publish_setpoint(self._above_target(self._limits.drop_altitude))
        if self._now() - self._t0 >= self._limits.release_time:
            self._enter(Phase.ASCEND)

    def _do_ascend(self) -> None:
        target = self._above_target(self._limits.cruise_altitude)
        self._publish_setpoint(target)
        if self._at(target):
            self._enter(Phase.RETURN)

    def _do_return(self) -> None:
        target = self._above_origin(self._limits.cruise_altitude)
        self._publish_setpoint(target)
        if self._at(target):
            self._begin_land(failsafe=False)

    def _do_land_cmd(self) -> None:
        if not self._issued:
            request = SetMode.Request()
            request.base_mode = 0
            request.custom_mode = self._land_plan[self._land_idx]
            self._start(self._cli_mode, request)
            return
        response = self._poll()
        if response is _PENDING:
            return
        if not response.mode_sent:
            self._advance_land("set_mode rejected")
            return
        self._enter(Phase.WAIT_DISARM)

    def _do_wait_disarm(self) -> None:
        if self._state is not None and not self._state.armed:
            if self.delivered and not self.failed:
                self._finish(Phase.DONE, "delivered, returned, landed and disarmed")
            else:
                self._finish(
                    Phase.DONE,
                    "landed and disarmed (payload "
                    f"{'DELIVERED' if self.delivered else 'NOT delivered'}; run had a failure)",
                )

    def _publish_setpoint(self, target: tuple[float, float, float]) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = target
        (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ) = self._orientation
        self._setpoint_pub.publish(msg)


def main(args=None) -> int:
    # NO: let Ctrl-C raise KeyboardInterrupt so we can still send RTL/LAND afterwards.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = DeliveryMission()
    try:
        while rclpy.ok() and node.phase not in TERMINAL:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().warn("Ctrl-C received")
        node.emergency_failsafe()
    finally:
        ok = node.phase == Phase.DONE and not node.failed and (node.delivered or not node._enable_flight)
        exit_code = 0 if ok else 1
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
