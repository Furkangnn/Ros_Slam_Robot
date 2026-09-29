from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_file = LaunchConfiguration("map")
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_slam_robot"), "launch", "robot.launch.py"])
        )
    )
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("nav2_bringup"), "launch", "bringup_launch.py"])
        ),
        launch_arguments={
            "map": map_file,
            "params_file": PathJoinSubstitution(
                [FindPackageShare("ros_slam_robot"), "config", "nav2.yaml"]
            ),
            "use_sim_time": "false",
        }.items(),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("map", description="Absolute path to the map YAML file"),
            robot,
            nav2,
        ]
    )
