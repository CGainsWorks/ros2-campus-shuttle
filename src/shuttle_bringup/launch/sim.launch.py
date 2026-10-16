"""Gazebo simulation bringup: world, robot, bridge, TF, safety gate and EKF.

This is the base every other launch file builds on. On its own it gives a robot
you can drive with teleop, through the safety gate:

    ros2 run teleop_twist_keyboard teleop_twist_keyboard \
        --ros-args -r cmd_vel:=cmd_vel_in

The simulator is modern Gazebo (gz-sim, Harmonic), the version paired with
ROS 2 Jazzy, started through ros_gz_sim and connected to ROS by ros_gz_bridge.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription,
    RegisterEventHandler, SetEnvironmentVariable, TimerAction,
)
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare


def generate_launch_description():
    bringup = FindPackageShare("shuttle_bringup")
    description = FindPackageShare("shuttle_description")

    use_rviz = LaunchConfiguration("rviz")
    headless = LaunchConfiguration("headless")
    world = LaunchConfiguration("world")
    x_pose = LaunchConfiguration("x")
    y_pose = LaunchConfiguration("y")
    yaw_pose = LaunchConfiguration("yaw")
    cmd_timeout = LaunchConfiguration("cmd_timeout")

    # Server: -r starts the physics immediately, -s runs without the GUI and
    # --headless-rendering gives the LiDAR and camera an offscreen (EGL) render
    # context, so they work with no display at all. Started through a wrapper
    # that takes the whole server process group down on shutdown; `gz sim`
    # forks the real server and a launch otherwise only stops the front end.
    gz_server = ExecuteProcess(
        cmd=[
            PathJoinSubstitution(
                [FindPackagePrefix("shuttle_bringup"), "lib", "shuttle_bringup",
                 "gz_server"]),
            "-r", "-s", "--headless-rendering", "-v", "2", world,
        ],
        name="gz_server",
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument(
            "headless", default_value="true",
            description="run the Gazebo server only; the automated runs need no GUI"),
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution([bringup, "worlds", "campus.sdf"])),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        DeclareLaunchArgument(
            "cmd_timeout", default_value="0.5",
            description="safety gate command timeout, s of simulated time"),

        # Keep this simulator's transport to itself. Without a partition, two
        # Gazebo instances on one machine see each other's topics and a stale
        # server left over from a previous run would answer the spawn request
        # meant for the new one.
        SetEnvironmentVariable(
            "GZ_PARTITION",
            EnvironmentVariable("GZ_PARTITION", default_value="shuttle")),

        gz_server,
        # Without the simulator nothing else means anything, so its exit ends
        # the launch.
        RegisterEventHandler(OnProcessExit(
            target_action=gz_server,
            on_exit=[EmitEvent(event=Shutdown(reason="Gazebo server exited"))],
        )),
        # The GUI is a separate client process that attaches to the server.
        ExecuteProcess(
            cmd=["gz", "sim", "-g", "-v", "2"],
            output="screen",
            condition=UnlessCondition(headless),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [description, "launch", "description.launch.py"])),
            launch_arguments={"use_sim_time": "true", "rviz": use_rviz}.items(),
        ),

        # Spawn from the robot_description topic. The create tool waits for the
        # world's entity factory, so a slow cold start delays the spawn rather
        # than losing it; the short delay only lets robot_state_publisher latch
        # the description first.
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package="ros_gz_sim",
                    executable="create",
                    output="screen",
                    arguments=[
                        "-topic", "robot_description",
                        "-name", "shuttle",
                        "-x", x_pose, "-y", y_pose, "-z", "0.05", "-Y", yaw_pose,
                    ],
                ),
            ],
        ),

        # LiDAR, IMU, camera, wheel odometry, ground truth, joint states, clock
        # and cmd_vel. The list, with what is deliberately left out, is in
        # config/gz_bridge.yaml.
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="gz_bridge",
            output="screen",
            parameters=[{
                "config_file": PathJoinSubstitution(
                    [bringup, "config", "gz_bridge.yaml"]),
                "use_sim_time": True,
            }],
        ),

        # Every velocity source publishes on /cmd_vel_in. Only the gate
        # publishes /cmd_vel, which is the topic the base listens to.
        Node(
            package="shuttle_control",
            executable="safety_gate_node",
            name="safety_gate",
            output="screen",
            parameters=[{
                "cmd_timeout": cmd_timeout,
                "use_sim_time": True,
            }],
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
