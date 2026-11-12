"""ROS 2 parameter helpers shared by the nodes.

Why this exists: rclpy gives a parameter the type of its default value and then
*rejects* an override of a different type. A node that declares
``target_north_m`` with the default ``60.0`` (a double) crashes with
``InvalidParameterTypeException`` when started with ``-p target_north_m:=60``
(an integer). People type ``60`` all the time, so numeric parameters are declared
here with dynamic typing and converted on read.
"""

from __future__ import annotations

from rcl_interfaces.msg import ParameterDescriptor

_DYNAMIC = ParameterDescriptor(dynamic_typing=True)


def declare_number(node, name: str, default) -> None:
    """Declare a numeric parameter that accepts either an integer or a decimal."""
    node.declare_parameter(name, default, _DYNAMIC)


def number(node, name: str) -> float:
    """The parameter as a float. Raises ValueError for anything that isn't a number."""
    value = node.get_parameter(name).value
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"parameter {name} must be a number, got {value!r}")
    return float(value)


def integer(node, name: str):
    """The parameter as an int when it is a whole number (so ``9`` and ``9.0`` both work).

    Anything else is returned unchanged, so the caller's own validation can reject
    it with a clear message instead of this raising something cryptic.
    """
    value = node.get_parameter(name).value
    if isinstance(value, float) and value.is_integer():
        return int(value)
    return value
