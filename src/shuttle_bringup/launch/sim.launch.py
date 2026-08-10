"""Gazebo simulation bringup: world, robot, TF, EKF and optionally RViz.

This is the base every other launch file builds on. On its own it gives a robot
you can drive with teleop.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration, PathJoinSubstitution, PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup = FindPackageShare("shuttle_bringup")
    description = FindPackageShare("shuttle_description")

    use_rviz = LaunchConfiguration("rviz")
    headless = LaunchConfiguration("headless")
    world = LaunchConfiguration("world")
    x_pose = LaunchConfiguration("x")
    y_pose = LaunchConfiguration("y")
    yaw_pose = LaunchConfiguration("yaw")

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument(
            "headless", default_value="true",
            description="run gzserver only; the automated comparison needs no GUI"),
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution(
                [bringup, "worlds", "campus.world"])),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("yaw", default_value="0.0"),

        # gzserver always; gzclient only when not headless.
        ExecuteProcess(
            cmd=["gzserver", "--verbose", "-s", "libgazebo_ros_init.so",
                 "-s", "libgazebo_ros_factory.so", world],
            output="screen",
        ),
        ExecuteProcess(
            cmd=["gzclient"],
            output="screen",
            condition=IfCondition(
                PythonExpression(["'", headless, "' == 'false'"])),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [description, "launch", "description.launch.py"])),
            launch_arguments={"use_sim_time": "true", "rviz": use_rviz}.items(),
        ),

        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            output="screen",
            arguments=[
                "-topic", "robot_description",
                "-entity", "shuttle",
                "-x", x_pose, "-y", y_pose, "-z", "0.05", "-Y", yaw_pose,
            ],
        ),

        # Local EKF: odom -> base_footprint, fusing wheel odometry and IMU.
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node_odom",
            output="screen",
            parameters=[
                PathJoinSubstitution([bringup, "config", "ekf.yaml"]),
                {"use_sim_time": True},
            ],
            remappings=[("odometry/filtered", "odometry/filtered")],
        ),
    ])
