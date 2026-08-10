"""Publish the robot description and TF tree.

Included by the simulation bringup; also useful on its own for inspecting the
URDF in RViz without starting Gazebo.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("shuttle_description")
    xacro_file = PathJoinSubstitution([pkg, "urdf", "shuttle.urdf.xacro"])

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("rviz")

    # value_type=str matters: without it the launch system tries to parse the
    # generated URDF as YAML and the parameter arrives mangled.
    robot_description = ParameterValue(
        Command(["xacro ", xacro_file]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="false"),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": use_sim_time,
            }],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            condition=IfCondition(use_rviz),
            arguments=[
                "-d",
                PathJoinSubstitution([pkg, "rviz", "shuttle.rviz"]),
            ],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])
