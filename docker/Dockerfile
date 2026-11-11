# ROS 2 Jazzy + MAVROS + this workspace. Used on the companion computer or on a
# dev machine. ArduPilot SITL runs outside this image (scripts/sitl.sh).
#
# Build (from the repo root):
#   docker build -f docker/Dockerfile -t robodrone .
#
# Run against SITL running on the same host:
#   docker run --rm -it --net=host robodrone \
#     ros2 launch robodrone_bringup bringup.launch.py fcu_url:=udp://:14560@
#
# Run against a real Pixhawk on a serial port:
#   docker run --rm -it --net=host --device=/dev/ttyUSB0 robodrone \
#     ros2 launch robodrone_bringup bringup.launch.py fcu_url:=/dev/ttyUSB0:921600
#
# NOTE: written but not built or run in the environment where it was authored.
FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-mavros ros-jazzy-mavros-extras ros-jazzy-mavros-msgs \
        python3-colcon-common-extensions python3-pytest \
    && rm -rf /var/lib/apt/lists/*

# MAVROS refuses to run without the GeographicLib geoid datasets.
RUN /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh

WORKDIR /ws
COPY ros2_ws/src ./src
RUN . /opt/ros/jazzy/setup.sh && colcon build --symlink-install

RUN echo '. /opt/ros/jazzy/setup.bash && . /ws/install/setup.bash' >> /root/.bashrc
CMD ["bash"]
