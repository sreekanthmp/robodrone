"""State-machine tests for offboard_demo against a fake rclpy + fake autopilot.

See fake_ros.py for what this does and does not prove.
"""

from __future__ import annotations

import importlib
import sys
from types import SimpleNamespace

import fake_ros
import pytest

NS = "/mavros"


@pytest.fixture
def demo(monkeypatch):
    for name, module in fake_ros.build_modules().items():
        monkeypatch.setitem(sys.modules, name, module)
    sys.modules.pop("robodrone_control.offboard_demo", None)
    fake_ros.FakeNode.overrides = {}
    yield importlib.import_module("robodrone_control.offboard_demo")
    fake_ros.FakeNode.overrides = {}
    sys.modules.pop("robodrone_control.offboard_demo", None)


class Drone:
    """Plays executor + a simple ArduPilot: answers service calls and moves."""

    def __init__(self, mod, *, reject=(), silent=False, no_climb=False,
                 armed=False, orientation=(0.0, 0.0, 0.0, 1.0), **params):
        params.setdefault("enable_flight", True)
        fake_ros.FakeNode.overrides = params
        self.mod = mod
        self.node = mod.OffboardDemo()
        self.reject, self.silent, self.no_climb = set(reject), silent, no_climb
        self.orientation = orientation
        self.mode, self.armed, self.connected = "STABILIZE", armed, True
        self.pos = [0.0, 0.0, 0.0]
        self.land_countdown = None

    # shortcuts
    def client(self, short):
        return self.node.clients[f"{NS}/{short}"]

    @property
    def setpoints(self):
        return self.node.publishers[f"{NS}/setpoint_position/local"].published

    def modes_requested(self):
        return [r.custom_mode for r in self.client("set_mode").requests]

    def errors(self):
        return self.node.get_logger().messages("error")

    # simulation
    def _answer(self, short, make_response):
        client = self.client(short)
        future = client.pending()
        if future is None or self.silent:
            return
        request = client.requests[client.futures.index(future)]
        future.resolve(make_response(request))

    def _autopilot(self):
        def set_mode(req):
            if req.custom_mode == "LAND" and "land" in self.reject:
                return SimpleNamespace(mode_sent=False)
            if req.custom_mode == "GUIDED" and "guided" in self.reject:
                return SimpleNamespace(mode_sent=False)
            self.mode = req.custom_mode
            if req.custom_mode == "LAND":
                self.land_countdown = 5
            return SimpleNamespace(mode_sent=True)

        def arming(req):
            if "arm" in self.reject:
                return SimpleNamespace(success=False)
            self.armed = req.value
            return SimpleNamespace(success=True)

        def takeoff(req):
            if "takeoff" in self.reject:
                return SimpleNamespace(success=False)
            if not self.no_climb:
                self.pos[2] = req.altitude
            return SimpleNamespace(success=True)

        self._answer("set_mode", set_mode)
        self._answer("cmd/arming", arming)
        self._answer("cmd/takeoff", takeoff)

        if self.mode == "GUIDED" and self.armed and self.setpoints:
            p = self.setpoints[-1].pose.position
            self.pos = [p.x, p.y, p.z]
        if self.land_countdown is not None:
            self.land_countdown -= 1
            if self.land_countdown <= 0:
                self.armed, self.pos[2], self.land_countdown = False, 0.0, None

    def step(self, dt=0.1):
        self.node.get_clock().t += dt
        self._autopilot()
        q = self.orientation
        pose = fake_ros.PoseStamped()
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = self.pos
        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = q
        self.node.subscriptions[f"{NS}/state"](
            fake_ros.State(connected=self.connected, armed=self.armed, mode=self.mode)
        )
        self.node.subscriptions[f"{NS}/local_position/pose"](pose)
        self.node._tick()

    def run(self, until, max_steps=2000):
        for _ in range(max_steps):
            if until():
                return
            self.step()
        raise AssertionError(f"condition never met; phase={self.node.phase.name}")

    def run_to_end(self):
        self.run(lambda: self.node.phase in self.mod.TERMINAL)

    def run_to_phase(self, phase):
        self.run(lambda: self.node.phase == phase)


# -- tests --------------------------------------------------------------------


def test_dry_run_checks_link_and_touches_nothing(demo):
    d = Drone(demo, enable_flight=False)
    d.run_to_end()
    assert d.node.phase == demo.Phase.DONE and not d.node.failed
    assert all(not c.requests for c in d.node.clients.values())
    assert not d.setpoints


def test_bad_parameter_aborts_before_anything_is_created(demo):
    d = Drone(demo, takeoff_altitude=-1.0)
    assert d.node.phase == demo.Phase.ABORTED and d.node.failed
    assert not d.node.clients and not d.node.publishers


def test_happy_path_flies_the_square_and_lands(demo):
    d = Drone(demo, takeoff_altitude=2.0, square_side=2.0)
    d.run_to_end()

    assert d.node.phase == demo.Phase.DONE and not d.node.failed
    assert d.modes_requested() == ["GUIDED", "LAND"]
    assert [r.value for r in d.client("cmd/arming").requests] == [True]
    assert [r.altitude for r in d.client("cmd/takeoff").requests] == [2.0]

    corners = {(round(s.pose.position.x, 3), round(s.pose.position.y, 3), s.pose.position.z)
               for s in d.setpoints}
    assert corners == {(2.0, 0.0, 2.0), (2.0, 2.0, 2.0), (0.0, 2.0, 2.0), (0.0, 0.0, 2.0)}
    assert {s.header.frame_id for s in d.setpoints} == {"map"}
    assert not d.errors()


def test_setpoints_hold_the_heading_the_drone_had(demo):
    q = (0.0, 0.0, 0.7071, 0.7071)  # facing north, not the identity "east"
    d = Drone(demo, orientation=q)
    d.run_to_end()
    o = d.setpoints[0].pose.orientation
    assert (o.x, o.y, o.z, o.w) == q


def test_altitude_request_is_clamped_to_the_hard_ceiling(demo):
    d = Drone(demo, takeoff_altitude=100.0)
    d.run_to_phase(demo.Phase.CLIMB)
    assert d.client("cmd/takeoff").requests[0].altitude == 10.0


def test_pilot_changing_mode_stops_everything_without_landing(demo):
    d = Drone(demo)
    d.run(lambda: d.node.phase == demo.Phase.SQUARE and len(d.setpoints) >= 1)

    d.mode = "LOITER"  # pilot flips the switch
    d.step()
    assert d.node.phase == demo.Phase.ABORTED and d.node.failed

    n_setpoints = len(d.setpoints)
    for _ in range(10):
        d.step()
    assert len(d.setpoints) == n_setpoints  # no more setpoints
    assert d.modes_requested() == ["GUIDED"]  # and we never fought back with LAND
    d.node.emergency_land()  # Ctrl-C afterwards must not fight the pilot either
    assert d.modes_requested() == ["GUIDED"]


def test_rejected_arming_aborts_without_landing(demo):
    d = Drone(demo, reject={"arm"})
    d.run_to_end()
    assert d.node.phase == demo.Phase.ABORTED and d.node.failed
    assert d.modes_requested() == ["GUIDED"]
    assert any("arming rejected" in e for e in d.errors())


def test_refuses_to_start_if_already_armed(demo):
    d = Drone(demo, armed=True)
    t_start = d.node.get_clock().t
    d.run_to_end()
    assert d.node.phase == demo.Phase.ABORTED
    assert all(not c.requests for c in d.node.clients.values())
    # Must be a deliberate refusal, not a 30 s timeout that happens to end the same way.
    assert any("already armed" in e for e in d.errors())
    assert d.node.get_clock().t - t_start < 2.0


def test_rejected_takeoff_lands_and_reports_failure(demo):
    d = Drone(demo, reject={"takeoff"})
    d.run_to_end()
    assert d.modes_requested() == ["GUIDED", "LAND"]
    assert d.node.phase == demo.Phase.DONE  # it did land...
    assert d.node.failed  # ...but the run is still a failure (non-zero exit)


def test_never_reaching_altitude_times_out_into_land(demo):
    d = Drone(demo, no_climb=True, step_timeout=5.0)
    d.run_to_end()
    assert d.modes_requested() == ["GUIDED", "LAND"]
    assert d.node.failed
    assert any("timeout in CLIMB" in e for e in d.errors())


def test_land_is_retried_then_pilot_is_told_loudly(demo):
    d = Drone(demo, reject={"takeoff", "land"})
    d.run_to_end()
    assert d.modes_requested().count("LAND") == demo.MAX_LAND_ATTEMPTS
    assert d.node.phase == demo.Phase.ABORTED and d.node.failed
    assert any("TAKE OVER MANUALLY" in e for e in d.errors())


def test_service_error_while_flying_lands(demo):
    d = Drone(demo)
    d.run_to_phase(demo.Phase.TAKEOFF_CMD)
    d.step()  # takeoff request issued
    d.client("cmd/takeoff").pending().resolve(exc=RuntimeError("boom"))
    d.step()
    assert d.node.phase == demo.Phase.LAND_CMD
    assert any("service call failed" in e for e in d.errors())


def test_unready_services_time_out_without_sending_anything(demo):
    d = Drone(demo, step_timeout=3.0)
    for c in d.node.clients.values():
        c.ready = False
    d.run_to_end()
    assert d.node.phase == demo.Phase.ABORTED
    assert all(not c.requests for c in d.node.clients.values())


def test_silent_fcu_never_lets_us_arm(demo):
    d = Drone(demo, silent=True, step_timeout=3.0)
    d.run_to_end()
    assert d.node.phase == demo.Phase.ABORTED
    assert not d.client("cmd/arming").requests


def test_ctrl_c_while_flying_commands_land(demo):
    d = Drone(demo)
    d.run(lambda: d.node.phase == demo.Phase.SQUARE)
    d.node.emergency_land()
    assert d.modes_requested()[-1] == "LAND"


def test_ctrl_c_on_the_ground_does_nothing(demo):
    d = Drone(demo, enable_flight=False)
    d.run_to_end()
    d.node.emergency_land()
    assert not d.client("set_mode").requests
