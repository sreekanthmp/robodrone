# Delivery mission

`delivery_mission` flies one delivery: take off, fly to a target, lower to a
drop height, release a payload with a servo, climb, fly back to where it
launched, and land. It builds on the same GUIDED-mode plumbing as the square
demo ([ros2.md](ros2.md)).

```
latch payload -> GUIDED -> arm -> takeoff -> TRANSIT to target (cruise altitude)
   -> DESCEND to drop height -> RELEASE (servo) -> wait -> ASCEND
   -> RETURN to the launch point -> LAND -> disarm
```

Read [preflight.md](preflight.md) and the **limits** section below before
flying anything real. It has been exercised in a simulator only.

## Run it

**1. Dry run.** Connects, checks GPS and the target, prints where it would go,
and touches nothing. Always do this first.

```bash
ros2 run robodrone_control delivery_mission --ros-args \
  -p target_north_m:=60.0 -p target_east_m:=40.0
```
```
target is 72.1 m away (east 40.0 m, north 60.0 m)
dry run OK: FCU connected, GPS fix, target accepted. ...
```

**2. In the simulator** (no hardware): one command, cold start.

```bash
MISSION=delivery scripts/sim_e2e.sh                 # default target 60 m N, 40 m E
MISSION=delivery NORTH=30 EAST=-20 scripts/sim_e2e.sh
```

**3. Fly it** (simulator, or hardware once the staged plan in
[preflight.md](preflight.md) has been followed):

```bash
ros2 run robodrone_control delivery_mission --ros-args -p enable_flight:=true \
  -p target_north_m:=60.0 -p target_east_m:=40.0
```

The exit code is 0 **only** if the payload was released and confirmed, the
drone got back and landed, and nothing went wrong on the way.

## Choosing the target

Give it either a GPS position or an offset from where it launches, never both
and never half of one. Anything else is refused, because guessing which was
meant is how a delivery ends up in the wrong place.

| Way | Parameters |
|---|---|
| GPS | `target_lat`, `target_lon` (decimal degrees) |
| Offset from launch | `target_north_m`, `target_east_m` (metres; negative = south/west) |

The target must be at least 5 m and at most `max_range` from the launch point.
GPS targets are converted with a WGS84 local-tangent-plane calculation
(centimetre accurate over a few kilometres; checked against textbook degree
lengths and an independent haversine calculation).

## Parameters

| Parameter | Default | Limit enforced in code | Meaning |
|---|---|---|---|
| `enable_flight` | `false` | | Without it, dry run only |
| `cruise_altitude` | 15.0 m | cap 30 m | Height above the launch point for transit and return |
| `drop_altitude` | 3.0 m | 1.5 m ... `cruise_altitude` | Height above the launch point for the release |
| `max_range` | 100 m | cap 500 m | Furthest allowed target distance |
| `horizontal_tolerance` | 2.0 m | > 0 | "Arrived" radius. GPS wanders by metres: do not make it tight |
| `vertical_tolerance` | 0.5 m | > 0, at most `drop_altitude` - 0.75 m | Altitude "arrived" band. The cap stops it settling below a safe drop height |
| `servo_channel` | 9 | **5 to 16** | Servo output for the release. Outputs 1-4 are motors and are refused |
| `hold_pwm` / `release_pwm` | 1100 / 1900 | 800-2200 µs, at least 200 apart | Latched and released positions |
| `release_time` | 2.0 s | 0 < t <= 30 | Hover at the drop point after releasing |
| `confirm_timeout` | 3.0 s | | How long to wait for the servo output to show the new value |
| `step_timeout` | 60 s | | Per-step timeout. Transit and return add `distance / 1 m/s` |
| `max_flight_time` | 600 s | | Total flight time from arming before it heads home |
| `failsafe_mode` | `RTL` | `RTL` or `LAND` | What to command when something goes wrong in the air |

A parameter can ask for less than a cap, never more. Raising a cap means
editing `mission.py` on purpose.

## The payload servo

This board's `Pixhawk1-1M` firmware has no gripper or winch support (compiled
out; see [firmware.md](firmware.md)), so the release is a plain servo or relay
driven with `MAV_CMD_DO_SET_SERVO`.

* **Wiring:** servo signal on a spare output, ground shared, and **power from
  its own BEC, never from the Pixhawk's servo rail**. On a Pixhawk 1 the first
  AUX output is servo output 9 (check your board's labelling). Keep it away
  from outputs 1-4.
* **Configuration:** set `SERVOn_FUNCTION` for that output to `0` (disabled).
  Measured against ArduPilot: with function `0` (or `1`, RC passthrough) the
  autopilot accepts `DO_SET_SERVO` and the output follows; with the output
  assigned to something else (tried 7 and 33) ArduPilot **rejects** the command
  (`result=4`) and the output does not move. The node treats a rejection as a
  fault.
* **Latch before flight:** before arming, the node commands `hold_pwm` and
  checks that `/mavros/rc/out` shows it. If the command is rejected or the
  output doesn't follow, it refuses to take off.
* **A release only counts if the output moved:** after the release command it
  watches `/mavros/rc/out` (using only samples taken *after* the command was
  acknowledged, so stale data can't confirm it). If the new value never shows
  up, the delivery is not recorded and the drone brings the payload home.

> **What "confirmed" means.** It confirms the autopilot's servo **output**, not
> that the payload physically left the aircraft. If that matters, fit a
> sensor and extend the node.

## What it does when things go wrong

| Situation | Behaviour |
|---|---|
| Parameter invalid (e.g. servo channel 1-4, drop above cruise) | Refuses at start, creates nothing |
| Target ambiguous, too close, or beyond `max_range` | Refuses before any command is sent |
| No GPS fix, link down, vehicle already armed | Waits, then refuses; never arms |
| Payload latch rejected or not confirmed | Aborts **before arming** |
| **Pilot changes mode away from GUIDED** | Stops at once, sends nothing further, exits non-zero |
| Release rejected, or not confirmed | Commands `failsafe_mode` (RTL), payload still aboard, exit non-zero |
| Timeout in any flying phase, or `max_flight_time` exceeded | Commands `failsafe_mode` |
| Ctrl-C while flying | Commands `failsafe_mode` |
| `RTL` rejected | Falls back to `LAND` (up to 3 attempts) |
| Neither can be commanded | Shouts `TAKE OVER MANUALLY NOW` and exits |

Failsafe defaults to **RTL, not LAND**: a fault 300 m out should bring the drone
home, not drop it in a field. RTL uses the autopilot's own settings (notably
`RTL_ALT`); set it above anything on your route.

"Home" here means two things. The delivery's own return flies back to **the
point this run started from**. RTL returns to **ArduPilot's home position**.
They are normally the same place. In the simulator they were: a flight started
16 m from the previous one, and RTL brought it back to *that* flight's arming
point (0.03 m off), not the earlier origin. On a real vehicle, confirm where
ArduPilot's home is set before relying on RTL.

## Limits: what it does not do

* **No obstacle avoidance.** It flies straight lines at a fixed altitude. Fly a
  route you have checked.
* **Altitudes are relative to the launch point.** No terrain following and no
  rangefinder: a target on a hill or in a hollow is not compensated, and the
  drop height is only as good as the ground is level.
* **GPS only.** There is no GPS-denied mode on this board's firmware
  ([firmware.md](firmware.md)). GPS accuracy is a few metres, so this is not
  precision delivery.
* **Battery:** only the flight-time limit and ArduPilot's own battery failsafes
  (configure `BATT_*` and `FS_BATT_*`). It does not estimate energy for the
  route.
* **No wind handling** beyond what the autopilot's position controller does.
* **One payload, one drop, one way out and back.** The release servo stays open
  after the drop; nothing re-latches it.
* **A pilot with a transmitter must be ready to take over.** On take-over your
  stick positions decide what the aircraft does (see
  [preflight.md](preflight.md)).
* **Rules.** Delivering cargo, flying beyond line of sight or over people, and
  dropping things from the air usually need permission. Check your local
  regulations before any real use.

## What has and hasn't been verified

**Unit tests.** 121 tests (no ROS needed) cover the geometry, the limits and
the whole state machine against a fake autopilot. The safety logic was
mutation-checked: 20 deliberate breakages, such as disabling the pilot
override, skipping the release confirmation, removing the failsafe, or allowing
motor outputs, were each caught by a failing test.

**Against ArduPilot SITL** (Copter 4.7.2, a full-feature simulator build, not
the `Pixhawk1-1M` firmware), through real MAVROS, measured from recordings and
not just log messages:

| Check | Result |
|---|---|
| Full delivery, target 60 m N / 40 m E (72.1 m) | Climbed to **15.01 m**; reached **0.00 m** from the target; released at **0.02 m** from it, **3.24 m** up; landed **0.01 m** from the launch point; exit 0 |
| Latch before arming | Servo reached the hold PWM at t=7.0 s, before arming at 8.3 s |
| Ctrl-C 35 m out | Commanded RTL; drone flew home and landed **0.02 m** from launch |
| Pilot switches to LOITER mid-transit | Node stopped, said the pilot took control, mode stayed LOITER, payload never released |
| Release command rejected mid-flight | Delivery **not** recorded; RTL; landed 0.03 m from that flight's start; exit 1 |
| Latch command rejected before start | Refused; never reached `SET_GUIDED`; vehicle never armed; exit 1 |
| `DO_SET_SERVO` vs `SERVOn_FUNCTION` | Accepted for 0 and 1, rejected (`result=4`) for 7 and 33 |

SITL has perfect GPS and no wind, so the position accuracy above is the best
case. Real GPS wanders by a few metres.

**Not verified:**

* The real Pixhawk 2.4.8, the `Pixhawk1-1M` firmware, a real servo and
  release mechanism, and any real flight.
* The "command accepted but the output never moved" path. It is unit-tested,
  but the simulator always moves the output after an accepted command, so I
  could not produce it live.
* Live: the flight-time limit, and the RTL-rejected fallback to LAND.
* Wind, GPS noise, terrain, battery behaviour over a long route.
