# Presence of this file puts this directory on sys.path, so
# `python3 -m pytest` (or plain `pytest`) can import `robodrone_control`
# without building the workspace.

import sys

import pytest


@pytest.fixture(autouse=True)
def _isolate_robodrone_modules():
    """Node modules are bound to fake ROS in some tests and real ROS in others.

    Python caches a submodule both in sys.modules and as an attribute of the
    package, and ``from robodrone_control import params`` trusts the attribute,
    so clearing sys.modules alone would still hand a later test a stale,
    fake-bound module. Scrub both, before and after every test.
    """

    def scrub():
        pkg = sys.modules.get("robodrone_control")
        for name in [m for m in list(sys.modules) if m.startswith("robodrone_control.")]:
            sys.modules.pop(name, None)
            short = name.rsplit(".", 1)[1]
            if pkg is not None and hasattr(pkg, short):
                delattr(pkg, short)

    scrub()
    yield
    scrub()
