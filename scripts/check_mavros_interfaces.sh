#!/usr/bin/env bash
# Start MAVROS with NO flight controller attached and check that every topic and
# service the robodrone_control nodes use actually exists. Needs no simulator
# and no hardware, so it is a quick regression check after upgrading MAVROS/ROS.
#
# Prerequisites: ROS 2 + MAVROS sourced, this workspace built and sourced, and
# the GeographicLib datasets installed (MAVROS aborts at start without them).
#
#   source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash
#   scripts/check_mavros_interfaces.sh
#
# Env: FCU_URL (default udp://:14599@, nothing listens there), WAIT seconds
# before checking (default 15).

set -uo pipefail

command -v ros2 >/dev/null || { echo "ros2 not found: source your ROS 2 setup.bash first" >&2; exit 2; }

FCU_URL="${FCU_URL:-udp://:14599@}"
WAIT="${WAIT:-15}"
LOG="$(mktemp)"
PID=""

# MAVROS runs in its own session so we can stop the whole tree (launch + nodes).
# Background jobs in a non-interactive script ignore SIGINT, so use TERM, wait a
# bounded time, then KILL. Otherwise this script can hang on exit and leave
# mavros_node running.
cleanup() {
  if [[ -n "$PID" ]]; then
    kill -TERM -- "-$PID" 2>/dev/null
    for _ in $(seq 20); do kill -0 -- "-$PID" 2>/dev/null || break; sleep 0.5; done
    kill -KILL -- "-$PID" 2>/dev/null
  fi
  rm -f "$LOG"
}
trap cleanup EXIT

setsid ros2 launch robodrone_bringup bringup.launch.py fcu_url:="$FCU_URL" monitor:=false >"$LOG" 2>&1 &
PID=$!
sleep "$WAIT"

if ! kill -0 "$PID" 2>/dev/null || grep -q "process has died" "$LOG"; then
  echo "FAIL: MAVROS did not stay up. Last log lines:" >&2
  sed 's/\x1b\[[0-9;]*m//g' "$LOG" | tail -8 >&2
  echo "(a 'GeographicLib ... File not readable' error means the datasets are missing:" >&2
  echo " run install_geographiclib_datasets.sh from the mavros package)" >&2
  exit 1
fi

topics="$(ros2 topic list)"
services="$(ros2 service list)"
fail=0

check() {  # kind name list
  if grep -qx -- "$2" <<<"$3"; then echo "ok       $1 $2"; else echo "MISSING  $1 $2"; fail=1; fi
}

# Keep in step with robodrone_control/{offboard_demo,status_monitor}.py
for t in /mavros/state /mavros/local_position/pose /mavros/battery \
         /mavros/global_position/raw/fix /mavros/setpoint_position/local; do
  check topic "$t" "$topics"
done
for s in /mavros/set_mode /mavros/cmd/arming /mavros/cmd/takeoff /mavros/set_stream_rate; do
  check service "$s" "$services"
done

if (( fail )); then echo "FAIL: MAVROS interface differs from what the nodes expect." >&2; exit 1; fi
echo "PASS: all expected MAVROS topics and services are present."
