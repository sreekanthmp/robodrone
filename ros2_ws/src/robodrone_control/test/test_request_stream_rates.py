"""Tests for request_stream_rates against the fake rclpy (see fake_ros.py)."""

from __future__ import annotations

import importlib
import sys

import fake_ros
import pytest

STATE = "/mavros/state"
SERVICE = "/mavros/set_stream_rate"


@pytest.fixture
def mod(monkeypatch):
    for name, module in fake_ros.build_modules().items():
        monkeypatch.setitem(sys.modules, name, module)
    sys.modules.pop("robodrone_control.request_stream_rates", None)
    fake_ros.FakeNode.overrides = {}
    yield importlib.import_module("robodrone_control.request_stream_rates")
    fake_ros.FakeNode.overrides = {}
    sys.modules.pop("robodrone_control.request_stream_rates", None)


class Rig:
    def __init__(self, mod, **params):
        fake_ros.FakeNode.overrides = params
        self.node = mod.StreamRateRequester()

    @property
    def client(self):
        return self.node.clients[SERVICE]

    def connect(self, connected=True):
        self.node.subscriptions[STATE](fake_ros.State(connected=connected))

    def tick(self, dt=1.0, answer=True):
        self.node.get_clock().t += dt
        self.node._tick()
        if answer and (f := self.client.pending()) is not None:
            f.resolve(object())


def test_does_nothing_until_the_fcu_is_connected(mod):
    r = Rig(mod)
    for _ in range(5):
        r.tick()
    r.connect(False)
    r.tick()
    assert not r.client.requests


def test_requests_all_streams_once_connected(mod):
    r = Rig(mod, rate=10.0)
    r.connect()
    r.tick()
    assert len(r.client.requests) == 1
    req = r.client.requests[0]
    assert (req.stream_id, req.message_rate, req.on_off) == (0, 10, True)  # 0 = STREAM_ALL


def test_does_not_spam_inside_the_refresh_period(mod):
    r = Rig(mod, refresh_period=30.0)
    r.connect()
    for _ in range(29):
        r.tick()
    assert len(r.client.requests) == 1


def test_repeats_after_the_refresh_period(mod):
    r = Rig(mod, refresh_period=30.0)
    r.connect()
    for _ in range(31):
        r.tick()
    assert len(r.client.requests) == 2


def test_rerequests_after_a_reconnect_even_inside_the_period(mod):
    r = Rig(mod, refresh_period=300.0)
    r.connect()
    r.tick()
    r.connect(False)  # link drops (e.g. autopilot rebooted)
    r.tick()
    r.connect(True)
    r.tick()
    assert len(r.client.requests) == 2


def test_rate_zero_disables_everything(mod):
    r = Rig(mod, rate=0.0)
    assert not r.node.clients and not r.node.subscriptions and r.node.timer is None


def test_rate_is_rounded_to_an_integer_for_the_uint16_field(mod):
    r = Rig(mod, rate=7.6)
    r.connect()
    r.tick()
    assert r.client.requests[0].message_rate == 8


def test_waits_for_the_service_and_then_sends(mod):
    r = Rig(mod)
    r.client.ready = False
    r.connect()
    for _ in range(3):
        r.tick()
    assert not r.client.requests
    r.client.ready = True
    r.tick()
    assert len(r.client.requests) == 1


def test_a_call_that_never_answers_is_dropped_and_retried(mod):
    r = Rig(mod, refresh_period=300.0)
    r.connect()
    for _ in range(8):  # never resolves the future
        r.tick(answer=False)
    assert len(r.client.requests) >= 2
    assert any("did not answer" in m for m in r.node.get_logger().messages("warn"))
