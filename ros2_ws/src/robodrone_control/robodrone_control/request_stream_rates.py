"""Ask ArduPilot to stream telemetry once MAVROS reports the FCU is connected.

Why this exists: ArduPilot only sends the data streams it has been asked for.
With MAVROS alone on the link (no ground station and no ``SRn_*`` stream-rate
parameters), the heartbeat and status text arrive but ``/mavros/local_position/pose``,
``/mavros/battery`` etc. stay silent, and nothing downstream can start. This was
observed against ArduPilot SITL and fixed by exactly this request.

It calls ``/mavros/set_stream_rate`` (stream_id 0 = all streams) when the FCU
connects, again after every reconnect, and then every ``refresh_period``
seconds. The request is idempotent, so repeating it is harmless and also
recovers from an autopilot that rebooted without MAVROS noticing.

On real hardware, also set the ``SRn_*`` parameters on the autopilot (see
``params/``); they survive without any companion computer running.
"""

from __future__ import annotations

import rclpy
from mavros_msgs.msg import State
from mavros_msgs.srv import StreamRate
from rclpy.node import Node

TICK_S = 1.0
STUCK_CALL_S = 5.0
STREAM_ALL = 0


class StreamRateRequester(Node):
    def __init__(self) -> None:
        super().__init__("request_stream_rates")
        self.declare_parameter("rate", 10.0)
        self.declare_parameter("refresh_period", 30.0)
        self.declare_parameter("mavros_ns", "/mavros")

        self._rate = int(round(float(self.get_parameter("rate").value)))
        self._refresh = float(self.get_parameter("refresh_period").value)
        ns = str(self.get_parameter("mavros_ns").value).rstrip("/")

        self._state: State | None = None
        self._last_request: float | None = None  # None => request on next tick
        self._future = None
        self._future_started = 0.0
        self.requests_sent = 0

        if self._rate <= 0:
            self.get_logger().info("rate <= 0: not requesting telemetry streams")
            return

        self.create_subscription(State, f"{ns}/state", self._on_state, 10)
        self._client = self.create_client(StreamRate, f"{ns}/set_stream_rate")
        self.create_timer(TICK_S, self._tick)
        self.get_logger().info(f"will request all telemetry streams at {self._rate} Hz")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_state(self, msg: State) -> None:
        self._state = msg

    def _tick(self) -> None:
        if self._state is None or not self._state.connected:
            self._last_request = None  # re-request when the link comes back
            self._future = None
            return

        now = self._now()
        if self._future is not None:
            if self._future.done():
                self._future = None
            elif now - self._future_started > STUCK_CALL_S:
                self.get_logger().warn("set_stream_rate did not answer; will retry")
                self._future = None
                self._last_request = None
            return

        due = self._last_request is None or now - self._last_request >= self._refresh
        if not due or not self._client.service_is_ready():
            return

        request = StreamRate.Request()
        request.stream_id = STREAM_ALL
        request.message_rate = self._rate
        request.on_off = True
        self._future = self._client.call_async(request)
        self._future_started = now
        self._last_request = now
        self.requests_sent += 1
        self.get_logger().info(f"requested all telemetry streams at {self._rate} Hz")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StreamRateRequester()
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
