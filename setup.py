from glob import glob
from setuptools import find_packages, setup

package_name = "ros_slam_robot"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/urdf", glob("urdf/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Furkan Gonen",
    maintainer_email="furkangnn@users.noreply.github.com",
    description="ROS 2 differential-drive robot with SLAM and Nav2.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "serial_base = ros_slam_robot.serial_base:main",
        ],
    },
)
