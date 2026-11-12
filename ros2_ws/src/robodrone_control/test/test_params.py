"""Tests for params.py with the fake rclpy. The real-rclpy behaviour is in
test_params_real_rclpy.py."""

from __future__ import annotations

import importlib
import sys

import fake_ros
import pytest


@pytest.fixture
def params(monkeypatch):
    for name, module in fake_ros.build_modules().items():
        monkeypatch.setitem(sys.modules, name, module)
    fake_ros.FakeNode.overrides = {}
    yield importlib.import_module("robodrone_control.params")
    fake_ros.FakeNode.overrides = {}


def _node(**overrides):
    fake_ros.FakeNode.overrides = overrides
    return fake_ros.FakeNode("t")


def test_numbers_are_returned_as_floats(params):
    node = _node(a=60, b=2.5, c=float("nan"))
    for name in "abc":
        params.declare_number(node, name, 1.0)
    assert params.number(node, "a") == 60.0 and isinstance(params.number(node, "a"), float)
    assert params.number(node, "b") == 2.5
    assert params.number(node, "c") != params.number(node, "c")  # NaN survives (means "unset")


@pytest.mark.parametrize("bad", ["abc", True, None, [1]])
def test_non_numbers_are_rejected_with_a_clear_message(params, bad):
    node = _node(x=bad)
    params.declare_number(node, "x", 1.0)
    with pytest.raises(ValueError, match="parameter x must be a number"):
        params.number(node, "x")


@pytest.mark.parametrize("given, expected", [(9, 9), (9.0, 9), (9.5, 9.5), ("9", "9")])
def test_integer_accepts_whole_numbers_and_leaves_the_rest_for_validation(params, given, expected):
    node = _node(ch=given)
    params.declare_number(node, "ch", 5)
    got = params.integer(node, "ch")
    assert got == expected and type(got) is type(expected)


def test_numeric_parameters_are_declared_with_dynamic_typing(params):
    # Without this, rclpy rejects "-p x:=60" for a parameter whose default is 60.0.
    node = _node()
    params.declare_number(node, "x", 1.0)
    assert node.descriptors["x"].dynamic_typing is True


@pytest.mark.parametrize("module", ["delivery_mission", "offboard_demo", "request_stream_rates"])
def test_every_node_declares_its_numbers_dynamically(monkeypatch, module):
    # Guards against a node going back to a plain declare_parameter for a number.
    for name, mod in fake_ros.build_modules().items():
        monkeypatch.setitem(sys.modules, name, mod)
    fake_ros.FakeNode.overrides = {}
    node_mod = importlib.import_module(f"robodrone_control.{module}")
    cls = {"delivery_mission": "DeliveryMission", "offboard_demo": "OffboardDemo",
           "request_stream_rates": "StreamRateRequester"}[module]
    node = getattr(node_mod, cls)()
    numeric = [n for n, d in node._params.items() if isinstance(d, (int, float)) and not isinstance(d, bool)]
    assert numeric, "no numeric parameters found"
    for name in numeric:
        assert node.descriptors[name] is not None and node.descriptors[name].dynamic_typing, name
