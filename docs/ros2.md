# ROS 2 + MAVLink (MAVROS)

```
 ROS 2 nodes ◄──topics/services──► MAVROS ◄──MAVLink 2, 921600 baud──► Pixhawk 2.4.8
 (this repo)                     (ros-jazzy-mavros)    TELEM2 (UART)     ArduCopter
```

MAVROS is the bridge: it turns MAVLink messages into ROS 2 topics and services
and back. On this board it is the only sensible ROS 2 path (see
[firmware.md](firmware.md) for why there's no direct DDS).

Targets **Ubuntu 24.04 + ROS 2 Jazzy + MAVROS 2.x**. Other ROS 2 distros should
work wherever MAVROS is packaged, but are untested here.

> **Status:** the Python logic and the node state machine are unit-tested
> (see "Tests" below). The launch file, Dockerfile, and the nodes' behaviour
> against real MAVROS were **not run** where this was written (no ROS 2
> installation, ArduPilot site unreachable). Stage 0 in
> [preflight.md](preflight.md) is how you close that gap, in simulation, before
> any hardware flies.

## Install

```bash
# ROS 2 Jazzy: follow https://docs.ros.org/en/jazzy/Installation.html, then:
sudo apt install ros-jazzy-mavros ros-jazzy-mavros-extras python3-colcon-common-extensions

# MAVROS will not start without the GeographicLib geoid datasets:
sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh

# Serial access (log out and in afterwards). If /dev/ttyACM0 gets grabbed by
# ModemManager on a desktop install, remove or mask ModemManager.
sudo usermod -aG dialout "$USER"
```

## Build this workspace

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Simulate first (no hardware)

Three terminals.

```bash
# 1. ArduPilot SITL (simulated ArduCopter) -> MAVLink on UDP 14560
scripts/sitl.sh
#    wait for "EKF3 IMU0 is using GPS" (30-60 s)

# 2. MAVROS + status monitor
ros2 launch robodrone_bringup bringup.launch.py fcu_url:=udp://:14560@

# 3. Dry run: checks the link and position stream, touches nothing
ros2 run robodrone_control offboard_demo
#    then fly the square in the simulator
ros2 run robodrone_control offboard_demo --ros-args -p enable_flight:=true
```

Expected: `phase: ... -> SET_GUIDED -> ARM -> TAKEOFF_CMD -> CLIMB -> SQUARE ->
LAND_CMD -> WAIT_DISARM`, then `landed and disarmed` and exit code 0.

## On the real vehicle

```bash
# Pixhawk on the companion's USB-serial adapter wired to TELEM2
ros2 launch robodrone_bringup bringup.launch.py fcu_url:=/dev/ttyUSB0:921600

# Pixhawk directly on micro-USB (bench work)
ros2 launch robodrone_bringup bringup.launch.py fcu_url:=/dev/ttyACM0:115200
```

You should see a line per second from `status_monitor` like
`mode=STABILIZE | armed=False | batt=16.1V | gps_status=0 | pos(ENU)=(0.0, 0.0, 0.0)`.

| You see | Likely cause |
|---|---|
| `FCU is not connected` | wrong `fcu_url`/baud, TX/RX not crossed, no shared GND, `SERIAL2_*` not set, FC not rebooted after loading params |
| connected but no telemetry (warns after 10 s) | `SR2_*` stream rates; or call `/mavros/set_stream_rate` |
| MAVROS dies at start about GeographicLib | datasets not installed (see above) |
| arming rejected | read the ArduPilot pre-arm message (`ros2 topic echo /mavros/statustext/recv`); fix the cause, never disable `ARMING_CHECK` |

## Topics and services used

| Name | Type | Notes |
|---|---|---|
| `/mavros/state` | `mavros_msgs/State` | connected, armed, mode |
| `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | **ENU**: x east, y north, z up, from the EKF origin |
| `/mavros/battery` | `sensor_msgs/BatteryState` | |
| `/mavros/global_position/raw/fix` | `sensor_msgs/NavSatFix` | raw GPS |
| `/mavros/setpoint_position/local` | `geometry_msgs/PoseStamped` | position setpoint, published by `offboard_demo` |
| `/mavros/set_mode` | `mavros_msgs/SetMode` | `custom_mode: "GUIDED"`, `"LAND"`, ... |
| `/mavros/cmd/arming` | `mavros_msgs/CommandBool` | |
| `/mavros/cmd/takeoff` | `mavros_msgs/CommandTOL` | `altitude` in metres above the home position |

QoS gotcha: MAVROS publishes telemetry as **best-effort**. `ros2 topic echo`
needs `--qos-reliability best_effort` on those topics, and your own subscribers
should use `qos_profile_sensor_data`. The demo's *setpoint publisher* is the
opposite: it must stay default/reliable, because MAVROS' setpoint subscribers
are reliable and a best-effort publisher will never connect.

## The nodes

* **`status_monitor`** - one line a second: mode, armed, battery, GPS, position.
  Warns if connected but nothing is streaming.
* **`offboard_demo`** - GUIDED takeoff, a square, land. **Dry run unless
  `enable_flight:=true`.** Parameters: `takeoff_altitude` (m, default 2.0, hard
  cap 10), `square_side` (m, default 2.0, hard cap 5), `position_tolerance`
  (m, 0.3), `step_timeout` (s, 30), `mavros_ns`, `guided_mode`.
  Pilot override, fail-to-land and Ctrl-C behaviour are described at the top of
  [`offboard_demo.py`](../ros2_ws/src/robodrone_control/robodrone_control/offboard_demo.py).
  Exit code is non-zero if *anything* went wrong, even if it landed fine.

`robodrone_control/mission.py` is plain Python (waypoints, limits, tolerances)
with no ROS imports: reuse it when you write your own mission nodes.

## Tests

```bash
cd ros2_ws/src/robodrone_control
python3 -m pip install pytest
python3 -m pytest
```

These need no ROS installation. `test/fake_ros.py` stands in for `rclpy` and
`mavros_msgs` so the node's state machine runs against a scripted fake
autopilot that can reject commands, go silent, or hand control to the pilot.
What the fakes cannot prove: QoS compatibility, real topic names, ArduPilot's
actual responses. That is what SITL is for.

## PX4 or a newer board later

The MAVROS topics above are autopilot-agnostic, but flight-mode names are not
(`GUIDED` is ArduPilot; PX4 uses `OFFBOARD`), so `offboard_demo` would need a
mode-name change and a different launch file (`px4.launch`). On a newer board
you can also use ArduPilot or PX4's native DDS bridge instead of MAVROS.
