from setuptools import find_packages, setup

package_name = "robodrone_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="robodrone",
    maintainer_email="maintainer@example.com",
    description="MAVROS-based monitoring and guided-flight demo for a Pixhawk/ArduPilot quadcopter.",
    license="TODO: choose a license",
    entry_points={
        "console_scripts": [
            "status_monitor = robodrone_control.status_monitor:main",
            "offboard_demo = robodrone_control.offboard_demo:main",
            "delivery_mission = robodrone_control.delivery_mission:main",
            "request_stream_rates = robodrone_control.request_stream_rates:main",
        ],
    },
)
