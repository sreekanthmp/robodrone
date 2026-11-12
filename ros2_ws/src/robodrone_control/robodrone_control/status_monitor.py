"""One-line-per-second health summary of the MAVROS link.

Handy during bench setup: it tells you at a glance whether MAVROS is talking to
the Pixhawk, whether data streams are actually flowing, and what the battery
and position estimate look like.
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, NavSatFix

from robodrone_control import params

# If connected but nothing else arrives, ArduPilot is probably not streaming.
STREAM_WARN_AFTER_S = 10.0


class StatusMonitor(Node):
    def __init__(self) -> None:
        super().__init__("status_monitor")
        self.declare_parameter("mavros_ns", "/mavros")
        params.declare_number(self, "period", 1.0)  # integer or decimal (see params.py)
        ns = str(self.get_parameter("mavros_ns").value).rstrip("/")

        self._state: State | None = None
        self._battery: BatteryState | None = None
        self._pose: PoseStamped | None = None
        self._fix: NavSatFix | None = None
        self._connected_since: float | None = None
        self._warned = False

        self.create_subscription(State, f"{ns}/state", self._on_state, 10)
        self.create_subscription(
            BatteryState, f"{ns}/battery", self._on_battery, qos_profile_sensor_data
        )
        self.create_subscription(
            PoseStamped, f"{ns}/local_position/pose", self._on_pose, qos_profile_sensor_data
        )
        self.create_subscription(
            NavSatFix, f"{ns}/global_position/raw/fix", self._on_fix, qos_profile_sensor_data
        )
        self.create_timer(params.number(self, "period"), self._report)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_state(self, msg: State) -> None:
        if msg.connected and self._connected_since is None:
            self._connected_since = self._now()
        elif not msg.connected:
            self._connected_since = None
        self._state = msg

    def _on_battery(self, msg: BatteryState) -> None:
        self._battery = msg

    def _on_pose(self, msg: PoseStamped) -> None:
        self._pose = msg

    def _on_fix(self, msg: NavSatFix) -> None:
        self._fix = msg

    def _report(self) -> None:
        if self._state is None:
            self.get_logger().info("no /mavros/state yet: is mavros running?")
            return
        if not self._state.connected:
            self.get_logger().warn(
                "MAVROS is up but the FCU is not connected: check fcu_url, baud rate, "
                "wiring and SERIAL2_* parameters"
            )
            return

        parts = [f"mode={self._state.mode}", f"armed={self._state.armed}"]
        if self._battery is not None and not math.isnan(self._battery.voltage):
            parts.append(f"batt={self._battery.voltage:.2f}V")
        if self._fix is not None:
            parts.append(f"gps_status={self._fix.status.status}")
        if self._pose is not None:
            p = self._pose.pose.position
            parts.append(f"pos(ENU)=({p.x:.2f}, {p.y:.2f}, {p.z:.2f})")
        self.get_logger().info(" | ".join(parts))

        no_data = self._battery is None and self._pose is None and self._fix is None
        if (
            no_data
            and not self._warned
            and self._connected_since is not None
            and self._now() - self._connected_since > STREAM_WARN_AFTER_S
        ):
            self._warned = True
            self.get_logger().warn(
                "connected but no telemetry streams: is request_stream_rates running "
                "(bringup starts it unless stream_rate:=0)? Otherwise set the SR2_* "
                "stream-rate parameters on the autopilot"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StatusMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
