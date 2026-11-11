# Firmware: ArduPilot on the Pixhawk 2.4.8

## Why ArduPilot, not PX4

Current PX4 does not officially support FMUv2 boards (the original Pixhawk 1
family), and PX4's native ROS 2 path (uXRCE-DDS) needs more flash than a 1 MB
board has anyway. ArduPilot still ships a Pixhawk 1 build, which is what the
rest of this repo assumes.

### What was checked against the source

Read directly from ArduPilot's source at tag **`Copter-4.7.2`** (the newest
stable Copter tag when this was written), under
`libraries/AP_HAL_ChibiOS/hwdef/`:

* `Pixhawk1-1M` **still exists** in 4.7.2 (`Pixhawk1-1M/hwdef.dat`).
* It sets `FLASH_SIZE_KB 1024` and pulls in `include/minimize_features.inc`
  and `include/minimize_common.inc`, which switch off, among other things:
  `GUIDED_NOGPS`, `FOLLOW`, `FLOWHOLD`, `ZIGZAG`, `SYSTEMID` and `TURTLE` modes;
  OSD, MSP and several telemetry protocols; gimbal mounts; the gripper, winch
  and sprayer; all GPS drivers except u-blox (+ NMEA on this board); all frame
  types except quad, hexa and octa.
* **Not** cut: `GUIDED`, `LOITER`, `LAND`, `STABILIZE`, `ALT_HOLD`, analog
  battery monitoring, MAVLink 2. Everything the demo needs is present.
  (`AC_NAV_GUIDED`, which is switched off, only gates the `NAV_GUIDED_ENABLE`
  mission item inside AUTO mode.)
* It contains `BOARD_CHECK_F427_USE_2M`: **on a real 2 MB chip the 1M firmware
  stops with "2M flash - use Pixhawk1 firmware".**

### Still not verified

* That this exact build **compiles and fits in 1 MB** at 4.7.2 (it should, as
  ArduPilot publishes it, but it was not built here).
* The ArduPilot website's own limited-firmware page and the
  `firmware.ardupilot.org` listing were unreachable from the authoring
  environment. Check both before you commit to a version:
  <https://ardupilot.org/copter/docs/common-limited-firmware.html>.
  If the latest stable ever drops `Pixhawk1-1M`, use the newest release that
  still has it.

## 1. Flash

Use **Mission Planner** (Windows) or **QGroundControl** to install ArduCopter.

* **Try `Pixhawk1-1M` first.** If your chip really is 1 MB it just works. If
  you get `2M flash - use Pixhawk1 firmware`, your chip has the full 2 MB:
  flash the plain **`Pixhawk1`** build instead. (The reverse mistake, the
  full build on a 1 MB chip, is the bad one: boot errors and random USB
  disconnects. That is why 1M goes first.)
* If your tool picks the build automatically and you want to be explicit,
  download `arducopter.apj` for the build you want from firmware.ardupilot.org
  and use the tool's *custom firmware* option.
* Stay on a stable release. Do not use `latest`/beta builds on a 1 MB board;
  they are the ones most likely to be out of flash.

## 2. Calibrate (in this order)

1. Frame type: Quad **X** (`FRAME_CLASS=1`, `FRAME_TYPE=1`).
2. Accelerometer (6-position).
3. Compass: **external** one on the GPS mast, outdoors, away from metal. Turn
   off/disable the internal compass if it disagrees.
4. Radio calibration, then flight modes on a switch (see below).
5. ESC calibration (PWM ESCs), props off.
6. Battery monitor: set it up and **check the reading against a multimeter**.
7. Motor test, props off. Verify the order and spin in
   [hardware.md](hardware.md).

## 3. Load the baseline parameters

[`params/pixhawk248_copter_quad.param`](../params/pixhawk248_copter_quad.param)
sets the airframe, the TELEM2 companion link, stream rates and a couple of
failsafes. Read it first. Then reboot the FC. The key lines:

| Parameter | Value | Why |
|---|---|---|
| `SERIAL2_PROTOCOL` | `2` | TELEM2 speaks MAVLink 2 |
| `SERIAL2_BAUD` | `921` | 921600 baud to the companion |
| `SR2_*` | 2-10 Hz | Without these ArduPilot streams almost nothing and MAVROS topics stay empty |

If `/mavros/state` shows `connected: true` but the other topics are silent,
it's the stream rates. The monitor node warns about this. As a fallback MAVROS
can request them itself:

```bash
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate \
  "{stream_id: 0, message_rate: 10, on_off: true}"
```

## 4. Flight-mode switch layout

Put these on a switch you can reach without looking:

| Position | Mode | Use |
|---|---|---|
| 1 | `STABILIZE` or `ALT_HOLD` | Manual; your safe default and the mode to flip to if anything looks wrong |
| 2 | `LOITER` | GPS position hold |
| 3 | `GUIDED` | What the ROS 2 demo uses |

And a separate switch for **Land**, plus a **motor emergency stop** on its own
switch (assign it with the `RCx_OPTION` parameter for that channel; see
ArduPilot's *RC Auxiliary Function Switches* page for the current option
numbers).

**You can always take over by leaving GUIDED.** The demo node treats any mode
change away from GUIDED as "the pilot has taken control" and stops sending
anything. (Checked against ArduPilot SITL: it stops, does not command LAND or
GUIDED again, and the mode stays where you put it.) What the *aircraft* then
does depends on your sticks; keep the throttle stick near centre, see
[preflight.md](preflight.md).

## 5. GUIDED mode needs a position estimate

GUIDED takeoff and position setpoints need a valid EKF position, normally from
**GPS** (wait for 3D fix, 8+ satellites, "EKF3 IMU0 is using GPS" in the
messages). Indoors without GPS you would need optical flow + rangefinder or an
external-navigation source, and **`GUIDED_NOGPS` and `FLOWHOLD` are compiled
out of the `Pixhawk1-1M` build**, so on this board treat the demo as
**outdoor, GPS-only**. This repo does not cover GPS-denied flight.

## 6. GCS failsafe, deliberately left off

`FS_GCS_ENABLE` is `0` in the baseline file. Whether the companion's MAVLink
heartbeats count as "the ground station is alive" depends on how MAVROS's
system id relates to `SYSID_MYGCS`. Decide that on the bench, with props off,
by unplugging the link and watching what the vehicle does, before you enable
it. Until then your protection against a dead companion is: your hand on the
transmitter, the RC failsafe (`FS_THR_ENABLE`), and the mode switch.
