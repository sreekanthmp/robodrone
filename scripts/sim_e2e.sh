#!/usr/bin/env bash
# End-to-end simulation test, no hardware needed:
#
#   ArduPilot SITL (simulated ArduCopter)  <--MAVLink-->  MAVROS  <--ROS 2-->  offboard_demo
#
# It starts SITL and MAVROS, waits until the simulated vehicle has a position
# estimate, flies the square demo, and checks that it landed and disarmed with
# exit code 0. PASS/FAIL on the last line; logs are kept on failure.
#
# Prerequisites
#   * ROS 2 + MAVROS sourced and this workspace built and sourced:
#       source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash
#   * GeographicLib datasets installed (MAVROS will not start without them)
#   * An ArduPilot checkout with SITL already built (see scripts/sitl.sh header):
#       cd ~/ardupilot && ./waf configure --board sitl && ./waf copter
#
# Environment (all optional)
#   ARDUPILOT_DIR  ArduPilot checkout            (default ~/ardupilot)
#   SITL_PYTHON    python that has ArduPilot's   (default python3)
#                  dependencies, e.g. pexpect
#   FCU_URL        how MAVROS reaches SITL       (default tcp://127.0.0.1:5760)
#   ALTITUDE, SIDE demo size in metres           (default 2.0 / 2.0)
#   EKF_TIMEOUT    seconds to wait for a position estimate (default 180)
#   FLIGHT_TIMEOUT seconds allowed for the flight itself   (default 240)
#   MISSION        square (default) or delivery
#   NORTH, EAST    delivery target, metres from launch    (default 60 / 40)
#                  A delivery ignores ALTITUDE and SIDE: it flies at its own
#                  cruise altitude (15 m by default).

set -uo pipefail

ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/ardupilot}"
SITL_PYTHON="${SITL_PYTHON:-python3}"
FCU_URL="${FCU_URL:-tcp://127.0.0.1:5760}"
ALTITUDE="${ALTITUDE:-2.0}"
SIDE="${SIDE:-2.0}"
EKF_TIMEOUT="${EKF_TIMEOUT:-180}"
FLIGHT_TIMEOUT="${FLIGHT_TIMEOUT:-240}"
MISSION="${MISSION:-square}"
NORTH="${NORTH:-60}"
EAST="${EAST:-40}"

die() { echo "FAIL: $*" >&2; exit 1; }

# Which node to run, how to dry-run it (link + position/GPS ready?), how to fly
# it, and what a successful run prints.
case "$MISSION" in
  square)
    NODE=offboard_demo
    DRY_ARGS=(-p step_timeout:=6.0)
    FLY_ARGS=(-p enable_flight:=true -p takeoff_altitude:="$ALTITUDE" -p square_side:="$SIDE"
              -p step_timeout:=60.0)
    PASS_TEXT="landed and disarmed"
    PASS_MSG="SITL flew the square, landed and disarmed (offboard_demo exit 0)."
    ;;
  delivery)
    NODE=delivery_mission
    DRY_ARGS=(-p target_north_m:="$NORTH" -p target_east_m:="$EAST" -p step_timeout:=6.0)
    FLY_ARGS=(-p enable_flight:=true -p target_north_m:="$NORTH" -p target_east_m:="$EAST"
              -p step_timeout:=60.0)
    PASS_TEXT="delivered, returned, landed and disarmed"
    PASS_MSG="SITL delivered to ${NORTH} m N / ${EAST} m E, returned, landed and disarmed (delivery_mission exit 0)."
    ;;
  *) echo "MISSION must be 'square' or 'delivery', not '$MISSION'" >&2; exit 2 ;;
esac

command -v ros2 >/dev/null || { echo "ros2 not found: source your ROS 2 setup.bash first" >&2; exit 2; }
SIM_VEHICLE="$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py"
[[ -f "$SIM_VEHICLE" ]] || { echo "no ArduPilot checkout at $ARDUPILOT_DIR (set ARDUPILOT_DIR)" >&2; exit 2; }
[[ -x "$ARDUPILOT_DIR/build/sitl/bin/arducopter" ]] || {
  echo "SITL is not built: run './waf configure --board sitl && ./waf copter' in $ARDUPILOT_DIR" >&2; exit 2; }

WORK="$(mktemp -d)"
PIDS=()
KEEP_LOGS=0

# Everything we start runs in its own session so the whole tree can be stopped.
# (Background jobs in a script ignore SIGINT, so use TERM, then KILL.)
cleanup() {
  for pid in "${PIDS[@]}"; do kill -TERM -- "-$pid" 2>/dev/null; done
  for _ in $(seq 20); do
    alive=0; for pid in "${PIDS[@]}"; do kill -0 -- "-$pid" 2>/dev/null && alive=1; done
    (( alive )) || break; sleep 0.5
  done
  for pid in "${PIDS[@]}"; do kill -KILL -- "-$pid" 2>/dev/null; done
  # sim_vehicle starts the simulator in its own session; stop that one too.
  pkill -f "$ARDUPILOT_DIR/build/sitl/bin/arducopter" 2>/dev/null
  if (( KEEP_LOGS )); then echo "logs kept in $WORK"; else rm -rf "$WORK"; fi
}
trap cleanup EXIT

echo "== starting ArduPilot SITL (headless, no MAVProxy)"
# exec inside the subshell: $! must be the real process (= its own session and
# process-group leader). Backgrounding a "cd && setsid ..." list instead gives
# the PID of a wrapper subshell, and cleanup then misses the simulator.
( cd "$WORK" && exec setsid "$SITL_PYTHON" "$SIM_VEHICLE" -v ArduCopter -f quad \
    --no-mavproxy --no-rebuild ) >"$WORK/sitl.log" 2>&1 &
PIDS+=("$!")

echo "== starting MAVROS ($FCU_URL)"
setsid ros2 launch robodrone_bringup bringup.launch.py fcu_url:="$FCU_URL" monitor:=false \
  >"$WORK/mavros.log" 2>&1 &
PIDS+=("$!")

echo "== waiting for the vehicle to connect and get a position estimate (up to ${EKF_TIMEOUT}s)"
deadline=$(( SECONDS + EKF_TIMEOUT ))
ready=0
while (( SECONDS < deadline )); do
  if timeout 30 ros2 run robodrone_control "$NODE" --ros-args "${DRY_ARGS[@]}" \
       >"$WORK/dryrun.log" 2>&1; then ready=1; break; fi
  sleep 3
done
if (( ! ready )); then
  KEEP_LOGS=1
  echo "--- last dry-run output"; tail -5 "$WORK/dryrun.log"
  echo "--- mavros.log"; sed 's/\x1b\[[0-9;]*m//g' "$WORK/mavros.log" | tail -8
  echo "--- sitl.log";  tail -8 "$WORK/sitl.log"
  die "never reached connected + position streaming"
fi
echo "== link and position estimate OK (after $(( EKF_TIMEOUT - (deadline - SECONDS) ))s), flying"

timeout "$FLIGHT_TIMEOUT" ros2 run robodrone_control "$NODE" --ros-args "${FLY_ARGS[@]}" \
  2>&1 | tee "$WORK/flight.log" | sed 's/\x1b\[[0-9;]*m//g'
rc=${PIPESTATUS[0]}

if (( rc == 0 )) && grep -q "$PASS_TEXT" "$WORK/flight.log"; then
  echo "PASS: $PASS_MSG"
  exit 0
fi
KEEP_LOGS=1
die "$NODE exit code $rc (see $WORK/flight.log, mavros.log, sitl.log)"
