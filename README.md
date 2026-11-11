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
| [docs/preflight.md](docs/preflight.md) | Staged testing plan and pre-flight checklist |
| [params/](params/pixhawk248_copter_quad.param) | Baseline ArduCopter parameters (review before loading) |
| [ros2_ws/src/robodrone_bringup](ros2_ws/src/robodrone_bringup) | Launch file: MAVROS + monitor |
| [ros2_ws/src/robodrone_control](ros2_ws/src/robodrone_control) | `status_monitor`, `offboard_demo`, tests |
| [scripts/sitl.sh](scripts/sitl.sh) | ArduPilot SITL wired to MAVROS |
| [docker/Dockerfile](docker/Dockerfile) | ROS 2 Jazzy + MAVROS + this workspace |

## Quick start (simulation)

```bash
scripts/sitl.sh                                                          # terminal 1
ros2 launch robodrone_bringup bringup.launch.py fcu_url:=udp://:14560@   # terminal 2
ros2 run robodrone_control offboard_demo                                 # terminal 3: dry run
```

Full steps, including the real vehicle: [docs/ros2.md](docs/ros2.md).

## What has and hasn't been verified

**Verified:** the plain-Python logic and the `offboard_demo` state machine pass
33 unit tests (`cd ros2_ws/src/robodrone_control && python3 -m pytest`),
including pilot-override, rejected commands, timeouts, failed-LAND and Ctrl-C
paths. The tests were mutation-checked: deliberately breaking the safety logic
makes them fail.

**Not verified:** anything that needs ROS 2, MAVROS, ArduPilot or hardware.
The launch file, the Dockerfile, the `SR2_*`/TELEM2 parameter values, and the
current feature limits of the `Pixhawk1-1M` ArduPilot build were written from
documentation and not run. Run [Stage 0 of the test plan](docs/preflight.md)
in simulation before putting props on anything.

## Safety

The demo flies a real aircraft if you tell it to. It is dry-run by default,
capped in altitude and size, and yields to the pilot's mode switch, but it is
**not** a substitute for a pilot with a transmitter in hand, a clear area, and
the staged plan in [docs/preflight.md](docs/preflight.md).
