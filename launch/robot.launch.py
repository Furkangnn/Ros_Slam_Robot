from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("ros_slam_robot")
    model = PathJoinSubstitution([share, "urdf", "robot.urdf.xacro"])
    parameters = PathJoinSubstitution([share, "config", "robot.yaml"])

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": Command(["xacro ", model])}],
                output="screen",
            ),
            Node(
                package="ros_slam_robot",
                executable="serial_base",
                parameters=[parameters],
                output="screen",
            ),
            Node(
                package="rplidar_ros",
                executable="rplidar_composition",
                name="rplidar",
                parameters=[
                    {
                        "serial_port": "/dev/ttyUSB_LIDAR",
                        "serial_baudrate": 115200,
                        "frame_id": "laser",
                        "inverted": False,
                        "angle_compensate": True,
                    }
                ],
                output="screen",
            ),
        ]
    )
