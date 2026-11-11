#!/usr/bin/env bash
# Run ArduPilot SITL (simulated ArduCopter quad) and forward MAVLink to UDP
# 14560 on localhost so MAVROS can attach with:
#
#   ros2 launch robodrone_bringup bringup.launch.py fcu_url:=udp://:14560@
#
# 14560 (not the default 14550) so MAVROS and a ground station such as
# QGroundControl, which listens on 14550, don't fight over one port.
#
# Needs an ArduPilot checkout with its prerequisites installed. If you do not
# have one:
#   git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git ~/ardupilot
#   cd ~/ardupilot && Tools/environment_install/install-prereqs-ubuntu.sh -y
#   (then log out and back in so PATH picks up the tools)
#
# SITL needs ~30-60 s after start before the EKF has a position and GUIDED
# takeoff is accepted. Wait for "EKF3 IMU0 is using GPS" in the console.

set -euo pipefail

ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/ardupilot}"
OUT_PORT="${OUT_PORT:-14560}"

if [[ ! -x "${ARDUPILOT_DIR}/Tools/autotest/sim_vehicle.py" ]]; then
  echo "ArduPilot not found at ${ARDUPILOT_DIR}." >&2
  echo "Clone it there, or set ARDUPILOT_DIR. See the header of this script." >&2
  exit 1
fi

cd "${ARDUPILOT_DIR}"
exec Tools/autotest/sim_vehicle.py -v ArduCopter -f quad --console --map \
  --out "udp:127.0.0.1:${OUT_PORT}" "$@"
