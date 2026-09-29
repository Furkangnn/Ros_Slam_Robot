from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_slam_robot"), "launch", "robot.launch.py"])
        )
    )
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("slam_toolbox"), "launch", "online_async_launch.py"]
            )
        ),
        launch_arguments={
            "slam_params_file": PathJoinSubstitution(
                [FindPackageShare("ros_slam_robot"), "config", "slam.yaml"]
            ),
            "use_sim_time": "false",
        }.items(),
    )
    return LaunchDescription([robot, slam])
