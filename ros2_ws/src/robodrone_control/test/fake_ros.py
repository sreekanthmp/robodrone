"""Minimal stand-ins for rclpy / mavros_msgs so the state machine in
``offboard_demo`` can be unit-tested without a ROS installation.

This only models what the node touches. It is not a ROS simulator: it cannot
catch QoS mismatches, wrong topic names against a real MAVROS, or message
field typos that the stubs happen to share. Run the node against ArduPilot
SITL (docs/ros2.md) before trusting it on hardware.
"""

from __future__ import annotations

import sys
import types
from types import SimpleNamespace


class FakeLogger:
    def __init__(self) -> None:
        self.records: list[tuple[str, str]] = []

    def _log(self, level: str):
        return lambda msg: self.records.append((level, str(msg)))

    def __getattr__(self, name: str):
        if name in ("debug", "info", "warn", "warning", "error"):
            return self._log(name)
        raise AttributeError(name)

    def messages(self, level: str) -> list[str]:
        return [m for lvl, m in self.records if lvl == level]


class FakeFuture:
    def __init__(self) -> None:
        self._done = False
        self._result = None
        self._exc = None

    def done(self) -> bool:
        return self._done

    def result(self):
        return self._result

    def exception(self):
        return self._exc

    def resolve(self, result=None, exc=None) -> None:
        self._done, self._result, self._exc = True, result, exc


class FakeClient:
    def __init__(self, name: str) -> None:
        self.name = name
        self.ready = True
        self.requests: list = []
        self.futures: list[FakeFuture] = []
        self.responder = None  # optional: request -> response, answers at once

    def service_is_ready(self) -> bool:
        return self.ready

    def call_async(self, request) -> FakeFuture:
        future = FakeFuture()
        self.requests.append(request)
        self.futures.append(future)
        if self.responder is not None:
            future.resolve(self.responder(request))
        return future

    def pending(self) -> FakeFuture | None:
        for f in self.futures:
            if not f.done():
                return f
        return None


class FakePublisher:
    def __init__(self) -> None:
        self.published: list = []

    def publish(self, msg) -> None:
        self.published.append(msg)


class FakeTimer:
    def __init__(self) -> None:
        self.cancelled = False

    def cancel(self) -> None:
        self.cancelled = True


class FakeClock:
    def __init__(self) -> None:
        self.t = 100.0  # seconds

    def now(self):
        t = self.t
        return SimpleNamespace(
            nanoseconds=int(t * 1e9), to_msg=lambda: SimpleNamespace(sec=int(t), nanosec=0)
        )


class FakeNode:
    """Parameters come from ``FakeNode.overrides`` (set by the test)."""

    overrides: dict = {}

    def __init__(self, name: str) -> None:
        self.node_name = name
        self._params: dict = {}
        self.descriptors: dict = {}
        self._logger = FakeLogger()
        self._clock = FakeClock()
        self.subscriptions: dict = {}
        self.publishers: dict = {}
        self.clients: dict = {}
        self.timer: FakeTimer | None = None

    def declare_parameter(self, name, default, descriptor=None):
        self._params[name] = type(self).overrides.get(name, default)
        self.descriptors[name] = descriptor

    def get_parameter(self, name):
        return SimpleNamespace(value=self._params[name])

    def get_logger(self) -> FakeLogger:
        return self._logger

    def get_clock(self) -> FakeClock:
        return self._clock

    def create_subscription(self, msg_type, topic, callback, qos):
        self.subscriptions[topic] = callback

    def create_publisher(self, msg_type, topic, qos):
        pub = FakePublisher()
        self.publishers[topic] = pub
        return pub

    def create_client(self, srv_type, name):
        client = FakeClient(name)
        self.clients[name] = client
        return client

    def create_timer(self, period, callback):
        self.timer = FakeTimer()
        return self.timer

    def destroy_node(self) -> None:
        pass


# -- message stubs ------------------------------------------------------------


def _ns(**kw):
    return SimpleNamespace(**kw)


class State:
    def __init__(self, connected=False, armed=False, mode="STABILIZE", guided=False):
        self.connected, self.armed, self.mode, self.guided = connected, armed, mode, guided


class PoseStamped:
    def __init__(self) -> None:
        self.header = _ns(stamp=None, frame_id="")
        self.pose = _ns(
            position=_ns(x=0.0, y=0.0, z=0.0),
            orientation=_ns(x=0.0, y=0.0, z=0.0, w=1.0),
        )


class ParameterDescriptor:
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)


class NavSatFix:
    def __init__(self, latitude=0.0, longitude=0.0, altitude=0.0, status=0):
        self.latitude, self.longitude, self.altitude = latitude, longitude, altitude
        self.status = _ns(status=status)  # 0 = fix, -1 = no fix


class RCOut:
    def __init__(self, channels=None):
        self.channels = list(channels or [])


class _Srv:
    class Request:
        def __init__(self) -> None:
            for field, default in type(self).FIELDS.items():
                setattr(self, field, default)


def _srv(name: str, **fields):
    req = type("Request", (_Srv.Request,), {"FIELDS": fields})
    return type(name, (), {"Request": req})


SetMode = _srv("SetMode", base_mode=0, custom_mode="")
CommandBool = _srv("CommandBool", value=False)
CommandLong = _srv(
    "CommandLong", broadcast=False, command=0, confirmation=0,
    param1=0.0, param2=0.0, param3=0.0, param4=0.0, param5=0.0, param6=0.0, param7=0.0,
)
StreamRate = _srv("StreamRate", stream_id=0, message_rate=0, on_off=False)
CommandTOL = _srv(
    "CommandTOL", min_pitch=0.0, yaw=0.0, latitude=0.0, longitude=0.0, altitude=0.0
)


def build_modules() -> dict[str, types.ModuleType]:
    """sys.modules entries that make ``import offboard_demo`` work without ROS."""

    def mod(name: str, **attrs) -> types.ModuleType:
        m = types.ModuleType(name)
        m.__dict__.update(attrs)
        return m

    rclpy = mod(
        "rclpy",
        ok=lambda: True,
        init=lambda **kw: None,
        shutdown=lambda: None,
        spin_once=lambda *a, **k: None,
        spin_until_future_complete=lambda *a, **k: None,
    )
    return {
        "rclpy": rclpy,
        "rclpy.node": mod("rclpy.node", Node=FakeNode),
        "rclpy.qos": mod("rclpy.qos", qos_profile_sensor_data=object()),
        "rclpy.signals": mod(
            "rclpy.signals", SignalHandlerOptions=SimpleNamespace(NO="NO")
        ),
        "geometry_msgs": mod("geometry_msgs"),
        "geometry_msgs.msg": mod("geometry_msgs.msg", PoseStamped=PoseStamped),
        "mavros_msgs": mod("mavros_msgs"),
        "rcl_interfaces": mod("rcl_interfaces"),
        "rcl_interfaces.msg": mod("rcl_interfaces.msg", ParameterDescriptor=ParameterDescriptor),
        "sensor_msgs": mod("sensor_msgs"),
        "sensor_msgs.msg": mod("sensor_msgs.msg", NavSatFix=NavSatFix),
        "mavros_msgs.msg": mod("mavros_msgs.msg", State=State, RCOut=RCOut),
        "mavros_msgs.srv": mod(
            "mavros_msgs.srv",
            SetMode=SetMode,
            CommandBool=CommandBool,
            CommandTOL=CommandTOL,
            CommandLong=CommandLong,
            StreamRate=StreamRate,
        ),
    }


def install() -> None:
    sys.modules.update(build_modules())
