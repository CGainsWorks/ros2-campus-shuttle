"""SLAM: drive the world with teleop and build a map.

    ros2 launch shuttle_bringup slam.launch.py headless:=false rviz:=true
    ros2 run teleop_twist_keyboard teleop_twist_keyboard \
        --ros-args -r cmd_vel:=cmd_vel_in

Save the result with:
    ros2 run nav2_map_server map_saver_cli -f data/maps/campus_map
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup = FindPackageShare("shuttle_bringup")

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="true"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([bringup, "launch", "sim.launch.py"])),
            launch_arguments={
                "headless": LaunchConfiguration("headless"),
                "rviz": LaunchConfiguration("rviz"),
                "x": "3.0",
                "y": "0.0",
                "yaw": "0.0",
            }.items(),
        ),

        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[
                PathJoinSubstitution([bringup, "config", "slam_toolbox.yaml"]),
                {"use_sim_time": True},
            ],
        ),
    ])
