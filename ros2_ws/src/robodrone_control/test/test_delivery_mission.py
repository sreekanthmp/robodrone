"""State-machine tests for delivery_mission against a fake rclpy + fake autopilot.

See fake_ros.py for what this does and does not prove.
"""

from __future__ import annotations

import importlib
import math
import sys
from types import SimpleNamespace

import fake_ros
import pytest

from robodrone_control import mission

NS = "/mavros"
ORIGIN_LAT, ORIGIN_LON = -35.363261, 149.165230
NAN = float("nan")


@pytest.fixture
def mod(monkeypatch):
    for name, module in fake_ros.build_modules().items():
        monkeypatch.setitem(sys.modules, name, module)
    sys.modules.pop("robodrone_control.delivery_mission", None)
    fake_ros.FakeNode.overrides = {}
    yield importlib.import_module("robodrone_control.delivery_mission")
    fake_ros.FakeNode.overrides = {}
    sys.modules.pop("robodrone_control.delivery_mission", None)


class Drone:
    """Executor + a simple ArduPilot that answers calls, moves, and drives rc/out."""

    def __init__(self, mod, *, reject=(), stuck_hold=False, stuck_release=False, stale_rc=False,
                 frozen=False, no_climb=False, armed=False, gps=True, speed=5.0,
                 initial_servo=1000, silent=False, orientation=(0.0, 0.0, 0.0, 1.0), **params):
        params.setdefault("enable_flight", True)
        if "target_lat" not in params:
            params.setdefault("target_north_m", 60.0)
            params.setdefault("target_east_m", 40.0)
        fake_ros.FakeNode.overrides = params
        self.mod = mod
        self.node = mod.DeliveryMission()
        self.reject, self.stuck_hold, self.stuck_release = set(reject), stuck_hold, stuck_release
        self.stale_rc, self.frozen, self.no_climb, self.gps = stale_rc, frozen, no_climb, gps
        self.speed, self.silent, self.orientation = speed, silent, orientation
        self.mode, self.armed, self.connected = "STABILIZE", armed, True
        self.pos = [0.0, 0.0, 0.0]
        self.rc = [1000] * 16
        self.rc[8] = initial_servo
        self.rc_published = False
        self.land_countdown = None
        self.events: list[tuple] = []
        self.servo_log: list[tuple[int, int]] = []
        self.t_in_phase: dict = {}

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

    def now(self):
        return self.node.get_clock().t

    # ---- simulated autopilot ----
    def _answer(self, short, fn):
        client = self.client(short)
        future = client.pending()
        if future is None or self.silent:
            return
        future.resolve(fn(client.requests[client.futures.index(future)]))

    def _autopilot(self, dt):
        def set_mode(req):
            mode = req.custom_mode
            key = {"GUIDED": "guided", "LAND": "land", "RTL": "rtl"}.get(mode, mode.lower())
            ok = key not in self.reject
            self.events.append(("mode", mode, ok))
            if ok:
                self.mode = mode
                if mode in ("LAND", "RTL"):
                    self.land_countdown = 5 if mode == "LAND" else 8
            return SimpleNamespace(mode_sent=ok)

        def arming(req):
            ok = "arm" not in self.reject
            self.events.append(("arm", req.value, ok))
            if ok:
                self.armed = req.value
            return SimpleNamespace(success=ok)

        def takeoff(req):
            ok = "takeoff" not in self.reject
            self.events.append(("takeoff", req.altitude, ok))
            if ok and not self.no_climb:
                self.pos[2] = req.altitude
            return SimpleNamespace(success=ok)

        def command(req):
            ch, pwm = int(req.param1), int(req.param2)
            release = pwm > 1500
            ok = not ("servo_release" in self.reject if release else "servo_hold" in self.reject)
            self.servo_log.append((ch, pwm))
            self.events.append(("servo", ch, pwm, req.command, ok))
            stuck = self.stuck_release if release else self.stuck_hold
            if ok and not stuck:
                self.rc[ch - 1] = pwm
            return SimpleNamespace(success=ok)

        self._answer("set_mode", set_mode)
        self._answer("cmd/arming", arming)
        self._answer("cmd/takeoff", takeoff)
        self._answer("cmd/command", command)

        if self.mode == "GUIDED" and self.armed and self.setpoints and not self.frozen:
            p = self.setpoints[-1].pose.position
            dx, dy, dz = p.x - self.pos[0], p.y - self.pos[1], p.z - self.pos[2]
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            step = self.speed * dt
            if dist <= step or dist == 0:
                self.pos = [p.x, p.y, p.z]
            else:
                self.pos = [self.pos[0] + dx / dist * step, self.pos[1] + dy / dist * step,
                            self.pos[2] + dz / dist * step]
        if self.land_countdown is not None:
            self.land_countdown -= 1
            if self.land_countdown <= 0:
                if self.mode == "RTL":
                    self.pos[0] = self.pos[1] = 0.0
                self.armed, self.pos[2], self.land_countdown = False, 0.0, None

    def _publish(self):
        n = self.node
        n.subscriptions[f"{NS}/state"](
            fake_ros.State(connected=self.connected, armed=self.armed, mode=self.mode))
        pose = fake_ros.PoseStamped()
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = self.pos
        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = self.orientation
        n.subscriptions[f"{NS}/local_position/pose"](pose)
        n.subscriptions[f"{NS}/global_position/global"](
            fake_ros.NavSatFix(ORIGIN_LAT, ORIGIN_LON, 584.0, status=0 if self.gps else -1))
        if not (self.stale_rc and self.rc_published):
            n.subscriptions[f"{NS}/rc/out"](fake_ros.RCOut(self.rc))
            self.rc_published = True

    def step(self, dt=0.1):
        self.node.get_clock().t += dt
        self._autopilot(dt)
        self._publish()
        before = self.node.phase
        self.node._tick()
        if self.node.phase != before:
            self.t_in_phase.setdefault(self.node.phase, self.now())

    def run(self, until, max_steps=6000):
        for _ in range(max_steps):
            if until():
                return
            self.step()
        raise AssertionError(f"condition never met; phase={self.node.phase.name}")

    def run_to_end(self):
        self.run(lambda: self.node.phase in self.mod.TERMINAL)

    def run_to_phase(self, phase):
        self.run(lambda: self.node.phase == phase)

    def setpoint_path(self):
        """Setpoints with consecutive duplicates removed, rounded to 0.1 m."""
        out = []
        for s in self.setpoints:
            p = s.pose.position
            t = (round(p.x, 1), round(p.y, 1), round(p.z, 1))
            if not out or out[-1] != t:
                out.append(t)
        return out

    def nothing_sent(self):
        return all(not c.requests for c in self.node.clients.values()) and not self.setpoints


# ---------------------------------------------------------------- happy path


def test_dry_run_validates_the_target_and_touches_nothing(mod):
    d = Drone(mod, enable_flight=False)
    d.run_to_end()
    assert d.node.phase == mod.Phase.DONE and not d.node.failed
    assert d.nothing_sent()
    assert any("target is 72.1 m away" in m for m in d.node.get_logger().messages("info"))


def test_happy_path_delivers_and_comes_home(mod):
    d = Drone(mod)
    d.run_to_end()
    n = d.node
    assert n.phase == mod.Phase.DONE and not n.failed and n.delivered
    assert d.servo_log == [(9, 1100), (9, 1900)]            # latch, then release
    assert d.modes_requested() == ["GUIDED", "LAND"]
    assert [e[1] for e in d.events if e[0] == "arm"] == [True]
    assert [e[1] for e in d.events if e[0] == "takeoff"] == [15.0]
    # east 40, north 60 from launch; cruise 15, drop 3; then home at cruise
    assert d.setpoint_path() == [(40.0, 60.0, 15.0), (40.0, 60.0, 3.0),
                                 (40.0, 60.0, 15.0), (0.0, 0.0, 15.0)]
    assert not d.errors()


def test_payload_is_latched_before_the_motors_are_armed(mod):
    d = Drone(mod)
    d.run_to_end()
    kinds = [e[0] for e in d.events]
    assert kinds.index("servo") < kinds.index("arm")
    assert d.events[kinds.index("servo")][3] == 183         # MAV_CMD_DO_SET_SERVO


def test_dwells_at_the_drop_point_after_release(mod):
    d = Drone(mod, release_time=3.0)
    d.run_to_end()
    at_drop = [s for s in d.setpoints if round(s.pose.position.z, 1) == 3.0]
    assert len(at_drop) >= 30                              # >= 3 s at 10 Hz


def test_gps_target_is_converted_to_the_right_offset(mod):
    lat, lon = ORIGIN_LAT + 0.0005, ORIGIN_LON + 0.0003
    d = Drone(mod, target_lat=lat, target_lon=lon, target_north_m=NAN, target_east_m=NAN)
    d.run_to_end()
    east, north = mission.enu_offset(ORIGIN_LAT, ORIGIN_LON, lat, lon)
    first = d.setpoint_path()[0]
    assert first[0] == pytest.approx(east, abs=0.1) and first[1] == pytest.approx(north, abs=0.1)
    assert d.node.delivered and not d.node.failed


def test_setpoints_hold_the_heading_the_drone_had(mod):
    q = (0.0, 0.0, 0.7071, 0.7071)
    d = Drone(mod, orientation=q)
    d.run_to_end()
    o = d.setpoints[0].pose.orientation
    assert (o.x, o.y, o.z, o.w) == q


# ---------------------------------------------------------------- refusals


@pytest.mark.parametrize(
    "params",
    [
        {"servo_channel": 3},                       # motor output
        {"servo_channel": 1},
        {"drop_altitude": 1.0},                     # below the safe minimum
        {"drop_altitude": 20.0, "cruise_altitude": 15.0},
        {"release_pwm": 1150},                      # too close to hold
        {"failsafe_mode": "STABILIZE"},
        {"max_flight_time": 0.0},
    ],
)
def test_bad_parameters_abort_before_anything_is_created(mod, params):
    d = Drone(mod, **params)
    assert d.node.phase == mod.Phase.ABORTED and d.node.failed
    assert not d.node.clients and not d.node.publishers


@pytest.mark.parametrize(
    "params, text",
    [
        ({"target_north_m": 500.0, "target_east_m": 0.0}, "beyond max_range"),
        ({"target_north_m": 1.0, "target_east_m": 1.0}, "minimum"),
        ({"target_north_m": NAN, "target_east_m": NAN}, "either"),
        ({"target_lat": ORIGIN_LAT + 0.0003}, "together"),
    ],
)
def test_bad_targets_are_refused_before_any_command(mod, params, text):
    d = Drone(mod, **params)
    d.run_to_end()
    assert d.node.phase == mod.Phase.ABORTED and d.node.failed
    assert d.nothing_sent()
    assert any(text in e for e in d.errors())


def test_no_gps_fix_never_arms(mod):
    d = Drone(mod, gps=False, step_timeout=5.0)
    d.run_to_end()
    assert d.node.phase == mod.Phase.ABORTED
    assert d.nothing_sent()


def test_refuses_to_start_if_already_armed(mod):
    d = Drone(mod, armed=True)
    t0 = d.now()
    d.run_to_end()
    assert d.node.phase == mod.Phase.ABORTED
    assert d.nothing_sent()
    assert any("already armed" in e for e in d.errors())
    assert d.now() - t0 < 2.0                              # deliberate refusal, not a timeout


def test_servo_that_does_not_follow_blocks_takeoff(mod):
    d = Drone(mod, stuck_hold=True, confirm_timeout=1.0)
    d.run_to_end()
    assert d.node.phase == mod.Phase.ABORTED and d.node.failed
    assert d.modes_requested() == [] and not [e for e in d.events if e[0] == "arm"]
    assert any("did not reach the hold position" in e for e in d.errors())


def test_stale_servo_telemetry_is_not_accepted_as_confirmation(mod):
    # rc/out already shows the hold value but never updates after the command.
    d = Drone(mod, initial_servo=1100, stale_rc=True, confirm_timeout=1.0)
    d.run_to_end()
    assert d.node.phase == mod.Phase.ABORTED
    assert not [e for e in d.events if e[0] == "arm"]


def test_rejected_hold_command_blocks_takeoff(mod):
    d = Drone(mod, reject={"servo_hold"})
    d.run_to_end()
    assert d.node.phase == mod.Phase.ABORTED
    assert not [e for e in d.events if e[0] == "arm"]


def test_rejected_arming_aborts_without_failsafe(mod):
    d = Drone(mod, reject={"arm"})
    d.run_to_end()
    assert d.node.phase == mod.Phase.ABORTED
    assert d.modes_requested() == ["GUIDED"]


# ---------------------------------------------------------------- delivery failures


def test_rejected_release_brings_the_payload_home_via_rtl(mod):
    d = Drone(mod, reject={"servo_release"})
    d.run_to_end()
    assert d.modes_requested() == ["GUIDED", "RTL"]
    assert not d.node.delivered and d.node.failed
    assert d.node.phase == mod.Phase.DONE                  # it did get home and disarm
    assert any("payload release command rejected" in e for e in d.errors())


def test_unconfirmed_release_is_not_recorded_as_delivered(mod):
    d = Drone(mod, stuck_release=True, confirm_timeout=1.0)
    d.run_to_end()
    assert d.modes_requested() == ["GUIDED", "RTL"]
    assert not d.node.delivered and d.node.failed
    assert any("RELEASE NOT CONFIRMED" in e for e in d.errors())


def test_rtl_rejected_falls_back_to_land(mod):
    d = Drone(mod, reject={"servo_release", "rtl"})
    d.run_to_end()
    assert d.modes_requested() == ["GUIDED", "RTL", "LAND"]
    assert d.node.phase == mod.Phase.DONE and d.node.failed


def test_failsafe_mode_land_is_honoured(mod):
    d = Drone(mod, reject={"servo_release"}, failsafe_mode="LAND")
    d.run_to_end()
    assert d.modes_requested() == ["GUIDED", "LAND"]


def test_rtl_and_land_both_rejected_shouts_for_a_human(mod):
    d = Drone(mod, reject={"servo_release", "rtl", "land"})
    d.run_to_end()
    assert d.modes_requested() == ["GUIDED", "RTL", "LAND", "LAND", "LAND"]
    assert d.node.phase == mod.Phase.ABORTED and d.node.failed
    assert any("TAKE OVER MANUALLY" in e for e in d.errors())


def test_never_reaching_cruise_altitude_goes_home(mod):
    d = Drone(mod, no_climb=True, step_timeout=5.0)
    d.run_to_end()
    assert d.modes_requested() == ["GUIDED", "RTL"]
    assert any("timeout in CLIMB" in e for e in d.errors())


def test_flight_time_limit_brings_it_home(mod):
    d = Drone(mod, speed=1.0, max_flight_time=20.0)
    d.run_to_end()
    assert d.modes_requested()[:2] == ["GUIDED", "RTL"]
    assert any("flight time limit" in e for e in d.errors())
    assert not d.node.delivered


def test_transit_timeout_scales_with_distance(mod):
    d = Drone(mod, frozen=True, step_timeout=10.0)         # target is 72.1 m away
    d.run_to_phase(mod.Phase.TRANSIT)
    t0 = d.now()
    d.run(lambda: d.node.phase != mod.Phase.TRANSIT)
    waited = d.now() - t0
    assert 80.0 <= waited <= 85.0                          # 10 s + 72.1 m / 1 m/s
    assert d.node.phase == mod.Phase.LAND_CMD              # failsafe entered...
    for _ in range(3):
        d.step()                                           # ...and the request goes out next tick
    assert d.modes_requested()[-1] == "RTL"


def test_silent_autopilot_never_arms(mod):
    d = Drone(mod, silent=True, step_timeout=3.0)
    d.run_to_end()
    assert d.node.phase == mod.Phase.ABORTED
    assert not [e for e in d.events if e[0] == "arm"]


# ---------------------------------------------------------------- pilot override / Ctrl-C


def _into_transit(d, mod):
    d.run(lambda: d.node.phase == mod.Phase.TRANSIT and len(d.setpoints) >= 3)


def test_pilot_changing_mode_stops_everything(mod):
    d = Drone(mod)
    _into_transit(d, mod)
    d.mode = "LOITER"
    d.step()
    assert d.node.phase == mod.Phase.ABORTED and d.node.failed
    n_sp, n_modes = len(d.setpoints), len(d.modes_requested())
    for _ in range(20):
        d.step()
    assert len(d.setpoints) == n_sp                         # no more setpoints
    assert len(d.modes_requested()) == n_modes              # never fights back
    assert d.modes_requested() == ["GUIDED"]
    assert d.servo_log == [(9, 1100)]                       # never released anything
    d.node.emergency_failsafe()                             # Ctrl-C afterwards: still hands off
    assert d.modes_requested() == ["GUIDED"]


def test_pilot_takeover_during_the_release_does_not_release_again(mod):
    d = Drone(mod)
    d.run_to_phase(mod.Phase.DESCEND)
    d.mode = "STABILIZE"
    d.step()
    assert d.node.phase == mod.Phase.ABORTED
    assert d.servo_log == [(9, 1100)]


def test_ctrl_c_in_flight_commands_rtl(mod):
    d = Drone(mod)
    _into_transit(d, mod)
    d.client("set_mode").responder = lambda r: SimpleNamespace(mode_sent=True)
    d.node.emergency_failsafe()
    assert d.modes_requested() == ["GUIDED", "RTL"]


def test_ctrl_c_falls_back_to_land_only_if_rtl_is_rejected(mod):
    d = Drone(mod)
    _into_transit(d, mod)
    d.client("set_mode").responder = lambda r: SimpleNamespace(mode_sent=r.custom_mode != "RTL")
    d.node.emergency_failsafe()
    assert d.modes_requested() == ["GUIDED", "RTL", "LAND"]


def test_ctrl_c_with_no_answer_does_not_override_an_accepted_rtl(mod):
    d = Drone(mod)
    _into_transit(d, mod)
    d.node.emergency_failsafe()                             # fake never answers: RTL may be slow
    assert d.modes_requested() == ["GUIDED", "RTL"]


def test_ctrl_c_on_the_ground_does_nothing(mod):
    d = Drone(mod, enable_flight=False)
    d.run_to_end()
    d.node.emergency_failsafe()
    assert not d.client("set_mode").requests


def test_servo_channel_and_pwm_values_come_from_the_parameters(mod):
    d = Drone(mod, servo_channel=7, hold_pwm=1250, release_pwm=1750)
    d.run_to_end()
    assert d.servo_log == [(7, 1250), (7, 1750)]
    assert d.node.delivered
