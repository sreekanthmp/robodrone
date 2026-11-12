# robodrone

A 10-inch quadcopter on a **Pixhawk 2.4.8** flight controller, with a
**companion computer running ROS 2** that talks to it over **MAVLink**
(through MAVROS).

```
 ROS 2 (Jazzy) on companion ──MAVROS── MAVLink 2 / TELEM2 ──► Pixhawk 2.4.8 + ArduCopter
```

## The one decision that shaped this repo

The Pixhawk 2.4.8 is an old FMUv2 board, usually with 1 MB of usable flash.
Current PX4 doesn't officially support it, and no native DDS fits. So:

* firmware: **ArduPilot ArduCopter**, `Pixhawk1-1M` build
* ROS 2 link: **MAVROS over MAVLink** on TELEM2, not micro-ROS/DDS

Details and the upgrade path to a newer board: [docs/firmware.md](docs/firmware.md).

## Layout

| Path | What |
|---|---|
| [docs/hardware.md](docs/hardware.md) | Parts, sizing, wiring (TELEM2 pinout, motor order) |
| [docs/firmware.md](docs/firmware.md) | Flashing, calibration, parameters, mode switch, failsafes |
| [docs/ros2.md](docs/ros2.md) | Install, bring-up, simulate, topics, tests |
| [docs/delivery.md](docs/delivery.md) | The delivery mission: run it, parameters, payload servo, what happens when things fail, limits |
| [docs/preflight.md](docs/preflight.md) | Staged testing plan and pre-flight checklist |
| [params/](params/pixhawk248_copter_quad.param) | Baseline ArduCopter parameters (review before loading) |
| [ros2_ws/src/robodrone_bringup](ros2_ws/src/robodrone_bringup) | Launch file: MAVROS + stream requester + monitor |
| [ros2_ws/src/robodrone_control](ros2_ws/src/robodrone_control) | `delivery_mission`, `offboard_demo`, `request_stream_rates`, `status_monitor`, tests |
| [scripts/sim_e2e.sh](scripts/sim_e2e.sh) | Headless ArduPilot SITL + MAVROS + flight; prints PASS/FAIL. `MISSION=square` (default) passed live before the `MISSION` option was added; both modes since changed are only stub-tested until re-run |
| [scripts/sitl.sh](scripts/sitl.sh) | Interactive SITL with MAVProxy console/map, for use with a ground station (**not yet run**) |
| [scripts/check_mavros_interfaces.sh](scripts/check_mavros_interfaces.sh) | Checks MAVROS exposes every topic/service the nodes use; needs no FC or simulator |
| [docker/Dockerfile](docker/Dockerfile) | ROS 2 Jazzy + MAVROS + this workspace |

## Quick start (simulation)

```bash
# ROS 2 + MAVROS sourced, workspace built, ArduPilot SITL built (see docs/ros2.md)
scripts/sim_e2e.sh     # starts SITL + MAVROS, flies the demo, prints PASS/FAIL
```

Step-by-step, by hand, and on the real vehicle: [docs/ros2.md](docs/ros2.md).

## What has and hasn't been verified

**Verified**

* The plain-Python logic and the state machines of `offboard_demo`,
  `request_stream_rates` and `delivery_mission` pass 121 unit tests
  (`cd ros2_ws/src/robodrone_control && python3 -m pytest`), including
  pilot-override, rejected commands, timeouts, failsafe and Ctrl-C paths. The
  tests were mutation-checked: deliberately breaking the safety logic (e.g.
  disabling the pilot override, skipping the release confirmation, allowing
  motor outputs as the payload servo) makes them fail.
* **Against ArduPilot SITL (Copter 4.7.2) through real MAVROS**, run from a cold
  start by `scripts/sim_e2e.sh`: the demo goes GUIDED, arms, takes off, flies a
  square, lands, disarms and exits 0. Measured, not just reported: it climbed to
  3.08 m when asked for 3.0 and traced a 3 m square. Ctrl-C mid-flight makes it
  command LAND and the vehicle lands. Leaving GUIDED mid-flight makes it stop
  and send nothing further. This run also found that ArduPilot streams nothing
  to MAVROS unless asked, which is why `request_stream_rates` exists.
* **The delivery mission against the same SITL**, measured from recordings: it
  flew 72 m to the target at 15 m altitude, lowered to 3 m, released the payload
  servo (confirmed from the servo output) 0.02 m from the target, returned and
  landed 0.01 m from launch. Failure paths also run live: Ctrl-C 35 m out gives
  RTL and a landing 0.02 m from launch; a pilot mode switch stops it; a rejected
  release brings the payload home; a rejected latch refuses to arm. Details and
  the limits of that evidence: [docs/delivery.md](docs/delivery.md).
* Against a **real ROS 2 Jazzy + MAVROS 2.x** install, with no flight
  controller: both packages build with colcon; the launch file resolves
  MAVROS' `apm.launch`; MAVROS starts; every topic and service the nodes use
  exists under `/mavros/...` with compatible QoS; message and service fields
  match the stubs used in the unit tests. Repeat it with
  `scripts/check_mavros_interfaces.sh`.
* The `Pixhawk1-1M` feature cuts, read from the ArduPilot **4.7.2** source
  (see [docs/firmware.md](docs/firmware.md)).

**Not verified**

* **The real Pixhawk 2.4.8.** SITL is a full-feature simulator build, not the
  `Pixhawk1-1M` firmware, so it cannot show the 1 MB feature cuts or whether the
  firmware fits in flash. It also has no RC transmitter: flipping from GUIDED to
  LOITER in SITL made the vehicle descend at ~2.5 m/s because the simulated
  throttle stick sat at minimum (see [docs/preflight.md](docs/preflight.md)).
* The Dockerfile (never built), `scripts/sitl.sh` (the MAVProxy route), and the
  `SR2_*`/TELEM2 parameter values.
* That the `Pixhawk1-1M` build compiles and fits in 1 MB at 4.7.2.
* **Delivery, beyond the simulator:** a real servo and release mechanism, any
  real flight, GPS noise and wind (SITL has perfect GPS and none), terrain, and
  battery behaviour on a long route. Also the "release accepted but the output
  never moved" path, which is unit-tested but cannot be produced in SITL.

Run [Stage 0 of the test plan](docs/preflight.md) in simulation before putting
props on anything.

## Safety

The demo and the delivery mission fly a real aircraft if you tell them to. Both
are dry-run by default, capped in altitude and distance, and yield to the
pilot's mode switch, but they are **not** a substitute for a pilot with a
transmitter in hand, a clear area, and the staged plan in
[docs/preflight.md](docs/preflight.md). Delivering cargo from the air usually
needs permission: check your local rules first.
