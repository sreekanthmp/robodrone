"""Start MAVROS (ArduPilot flavour) and, optionally, the status monitor.

Real Pixhawk on a companion computer's UART or a USB-serial adapter:

    ros2 launch robodrone_bringup bringup.launch.py fcu_url:=/dev/ttyUSB0:921600

Pixhawk plugged straight into the computer over micro-USB (bench work only):

    ros2 launch robodrone_bringup bringup.launch.py fcu_url:=/dev/ttyACM0:115200

ArduPilot SITL (see scripts/sitl.sh, which forwards MAVLink to UDP 14560):

    ros2 launch robodrone_bringup bringup.launch.py fcu_url:=udp://:14560@

This wraps MAVROS' own ``apm.launch`` so its ArduPilot plugin list and config
are used unchanged. The ``fcu_url`` / ``gcs_url`` argument names are those of
MAVROS 2.x; confirm against your install with
``ros2 launch mavros apm.launch --show-args``.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    fcu_url = LaunchConfiguration("fcu_url")
    gcs_url = LaunchConfiguration("gcs_url")
    monitor = LaunchConfiguration("monitor")

    mavros = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("mavros"), "launch", "apm.launch"])
        ),
        launch_arguments={"fcu_url": fcu_url, "gcs_url": gcs_url}.items(),
    )

    status_monitor = Node(
        package="robodrone_control",
        executable="status_monitor",
        output="screen",
        condition=IfCondition(monitor),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "fcu_url",
                default_value="/dev/ttyUSB0:921600",
                description="MAVROS FCU connection URL (serial 'device:baud' or udp://...)",
            ),
            DeclareLaunchArgument(
                "gcs_url",
                default_value="",
                description="Optional MAVLink forward to a ground station, "
                "e.g. udp://@192.168.1.50:14550",
            ),
            DeclareLaunchArgument(
                "monitor",
                default_value="true",
                description="Also start the status_monitor node",
            ),
            mavros,
            status_monitor,
        ]
    )
