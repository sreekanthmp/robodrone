"""Fly a small square in ArduPilot GUIDED mode through MAVROS.

Sequence: wait for FCU -> GUIDED -> arm -> takeoff -> square -> LAND -> disarm.

Safety behaviour (all of it is covered by test/test_offboard_demo.py):

* **Dry run by default.** Without ``enable_flight:=true`` the node only checks
  that the FCU is connected and a local position is arriving, then exits.
* **Pilot always wins.** If the flight mode ever changes away from GUIDED while
  we are flying, the node assumes the pilot took over, sends nothing further
  and exits.
* **Fail toward the ground.** Any timeout, rejected command or service error
  while armed and flying commands LAND (retried a few times). Ctrl-C does the
  same.
* **Hard limits.** Altitude and square size are clamped, see ``mission.py``.

Frames: MAVROS local positions are ENU (x east, y north, z up) relative to the
EKF origin. The drone must have a valid position estimate (GPS lock, or an
external-nav / optical-flow setup) before GUIDED will accept takeoff.
"""

from __future__ import annotations

import enum
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions

from robodrone_control import mission

TICK_HZ = 10.0
MAX_LAND_ATTEMPTS = 3
DISARM_TIMEOUT_S = 120.0
LAND_MODE = "LAND"


class Phase(enum.Enum):
    WAIT_FCU = enum.auto()
    WAIT_POSE = enum.auto()
    SET_GUIDED = enum.auto()
    ARM = enum.auto()
    TAKEOFF_CMD = enum.auto()
    CLIMB = enum.auto()
    SQUARE = enum.auto()
    LAND_CMD = enum.auto()
    WAIT_DISARM = enum.auto()
    DONE = enum.auto()
    ABORTED = enum.auto()


TERMINAL = frozenset({Phase.DONE, Phase.ABORTED})
# Phases where we are armed (or about to be) in GUIDED and the pilot may override.
FLYING = frozenset({Phase.ARM, Phase.TAKEOFF_CMD, Phase.CLIMB, Phase.SQUARE})

_PENDING = object()


class OffboardDemo(Node):
    def __init__(self) -> None:
        super().__init__("offboard_demo")

        self.declare_parameter("enable_flight", False)
        self.declare_parameter("takeoff_altitude", 2.0)
        self.declare_parameter("square_side", 2.0)
        self.declare_parameter("position_tolerance", 0.3)
        self.declare_parameter("step_timeout", 30.0)
        self.declare_parameter("guided_mode", "GUIDED")
        self.declare_parameter("mavros_ns", "/mavros")

        self._enable_flight = bool(self.get_parameter("enable_flight").value)
        self._timeout = float(self.get_parameter("step_timeout").value)
        self._guided = str(self.get_parameter("guided_mode").value)
        ns = str(self.get_parameter("mavros_ns").value).rstrip("/")

        self._state: State | None = None
        self._pos: tuple[float, float, float] | None = None
        self._orientation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
        self._ground: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._waypoints: list[mission.Vec3] = []
        self._wp_index = 0
        self._land_attempts = 0
        self._future = None
        self._issued = False
        self._acked = False
        self._t0 = 0.0
        self.phase = Phase.WAIT_FCU
        # True once anything went wrong, even if we then landed safely. Drives the
        # exit code so a script never mistakes "failed but landed" for success.
        self.failed = False

        self._limits: mission.DemoLimits | None = None
        try:
            self._limits = mission.validate_limits(
                self.get_parameter("takeoff_altitude").value,
                self.get_parameter("square_side").value,
                self.get_parameter("position_tolerance").value,
            )
        except mission.InvalidParameter as exc:
            self.get_logger().error(f"bad parameter: {exc}")
            self.failed = True
            self.phase = Phase.ABORTED
            return

        self.create_subscription(State, f"{ns}/state", self._on_state, 10)
        self.create_subscription(
            PoseStamped, f"{ns}/local_position/pose", self._on_pose, qos_profile_sensor_data
        )
        # MAVROS' setpoint subscriber is BEST_EFFORT (checked with `ros2 topic info -v`
        # on MAVROS 2.x), so the default RELIABLE publisher is compatible with it.
        self._setpoint_pub = self.create_publisher(
            PoseStamped, f"{ns}/setpoint_position/local", 10
        )
        self._cli_mode = self.create_client(SetMode, f"{ns}/set_mode")
        self._cli_arm = self.create_client(CommandBool, f"{ns}/cmd/arming")
        self._cli_takeoff = self.create_client(CommandTOL, f"{ns}/cmd/takeoff")

        self._t0 = self._now()
        self._timer = self.create_timer(1.0 / TICK_HZ, self._tick)
        self.get_logger().info(
            f"offboard_demo ready ({'FLIGHT ENABLED' if self._enable_flight else 'dry run'}): "
            f"alt={self._limits.takeoff_altitude} m, side={self._limits.square_side} m"
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

    # -- failure handling ---------------------------------------------------

    def _fail(self, reason: str) -> None:
        self.get_logger().error(reason)
        self.failed = True
        if self.phase in FLYING and self._armed():
            self.get_logger().warn("armed and flying: commanding LAND")
            self._enter(Phase.LAND_CMD)
        elif (
            self.phase == Phase.LAND_CMD
            and self._armed()
            and self._land_attempts < MAX_LAND_ATTEMPTS
        ):
            self.get_logger().warn(f"retrying LAND ({self._land_attempts}/{MAX_LAND_ATTEMPTS})")
            self._enter(Phase.LAND_CMD)
        elif self.phase == Phase.LAND_CMD and self._armed():
            self._finish(
                Phase.ABORTED, "COULD NOT COMMAND LAND - TAKE OVER MANUALLY NOW", error=True
            )
        else:
            self._finish(Phase.ABORTED, f"aborted in {self.phase.name}", error=True)

    def emergency_land(self) -> None:
        """Best-effort LAND on Ctrl-C. Does nothing if we are not the ones flying."""
        if self.phase not in FLYING or not self._armed():
            return
        self.get_logger().warn("interrupted while flying: commanding LAND")
        request = SetMode.Request()
        request.base_mode = 0
        request.custom_mode = LAND_MODE
        future = self._cli_mode.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

    # -- state machine ------------------------------------------------------

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

        # 2. Per-step timeout.
        limit = DISARM_TIMEOUT_S if self.phase == Phase.WAIT_DISARM else self._timeout
        if self._now() - self._t0 > limit:
            self._fail(f"timeout in {self.phase.name} after {limit:.0f} s")
            return

        # 3. Do this phase's work.
        try:
            getattr(self, f"_do_{self.phase.name.lower()}")()
        except RuntimeError as exc:
            self._fail(str(exc))

    def _do_wait_fcu(self) -> None:
        if self._state is not None and self._state.connected:
            self._enter(Phase.WAIT_POSE)

    def _do_wait_pose(self) -> None:
        if self._pos is None:
            return
        if not self._enable_flight:
            self._finish(
                Phase.DONE,
                "dry run OK: FCU connected and local position streaming. "
                "Re-run with -p enable_flight:=true to fly.",
            )
            return
        if self._armed():
            self._fail("vehicle is already armed: refusing to start")
            return
        self._ground = self._pos
        self._enter(Phase.SET_GUIDED)

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
            self._enter(Phase.TAKEOFF_CMD)

    def _do_takeoff_cmd(self) -> None:
        if not self._issued:
            request = CommandTOL.Request()
            request.min_pitch = 0.0
            request.yaw = 0.0
            request.latitude = 0.0
            request.longitude = 0.0
            request.altitude = float(self._limits.takeoff_altitude)
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
        target_z = self._ground[2] + self._limits.takeoff_altitude
        if mission.altitude_reached(self._pos[2], target_z, self._limits.tolerance):
            self._waypoints = mission.square_waypoints(
                self._pos, self._limits.square_side, 0.0
            )
            self._wp_index = 0
            self._enter(Phase.SQUARE)

    def _do_square(self) -> None:
        if self._pos is None:
            return
        target = self._waypoints[self._wp_index]
        self._publish_setpoint(target)
        if mission.reached(self._pos, target, self._limits.tolerance):
            self._wp_index += 1
            self.get_logger().info(f"waypoint {self._wp_index}/{len(self._waypoints)} reached")
            if self._wp_index >= len(self._waypoints):
                self._enter(Phase.LAND_CMD)
            else:
                self._t0 = self._now()  # fresh timeout for the next leg

    def _do_land_cmd(self) -> None:
        if not self._issued:
            request = SetMode.Request()
            request.base_mode = 0
            request.custom_mode = LAND_MODE
            if self._start(self._cli_mode, request):
                self._land_attempts += 1
            return
        response = self._poll()
        if response is _PENDING:
            return
        if not response.mode_sent:
            raise RuntimeError("set_mode LAND was rejected")
        self._enter(Phase.WAIT_DISARM)

    def _do_wait_disarm(self) -> None:
        if self._state is not None and not self._state.armed:
            self._finish(Phase.DONE, "landed and disarmed")

    def _publish_setpoint(self, target: mission.Vec3) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = target
        # Hold the heading we had at takeoff instead of snapping to "east".
        (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ) = self._orientation
        self._setpoint_pub.publish(msg)


def main(args=None) -> int:
    # NO: let Ctrl-C raise KeyboardInterrupt so we can still send LAND afterwards.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = OffboardDemo()
    try:
        while rclpy.ok() and node.phase not in TERMINAL:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().warn("Ctrl-C received")
        node.emergency_land()
    finally:
        exit_code = 0 if node.phase == Phase.DONE and not node.failed else 1
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
