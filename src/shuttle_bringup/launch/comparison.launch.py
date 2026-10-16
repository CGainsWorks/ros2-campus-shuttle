"""Run one controller over one scenario in Gazebo and record the metrics.

Driven by scripts/run_comparison.sh, which invokes this once per controller so
both face the identical world, the identical latched plan and the identical
start pose. The path comes from the fixed scenario generator rather than from
the Nav2 planner, because a replanned route would differ between runs and
confound the comparison.

Example:
    ros2 launch shuttle_bringup comparison.launch.py \
        controller:=stanley scenario:=campus_loop speed:=1.0
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup = FindPackageShare("shuttle_bringup")

    controller = LaunchConfiguration("controller")
    scenario = LaunchConfiguration("scenario")
    speed = LaunchConfiguration("speed")
    output_dir = LaunchConfiguration("output_dir")
    x_pose = LaunchConfiguration("x")
    y_pose = LaunchConfiguration("y")
    yaw_pose = LaunchConfiguration("yaw")
    exit_on_finish = LaunchConfiguration("exit_on_finish")

    # The recorder decides when the run is over: it exits once the controller
    # has reported the goal and the robot has settled.
    metrics_recorder = Node(
        package="shuttle_control",
        executable="metrics_recorder_node",
        name="metrics_recorder",
        output="screen",
        parameters=[{
            "controller": controller,
            "scenario": scenario,
            "target_speed": speed,
            "output_dir": output_dir,
            "use_sim_time": True,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("controller", default_value="pure_pursuit",
                              description="pure_pursuit or stanley"),
        DeclareLaunchArgument("scenario", default_value="campus_loop"),
        DeclareLaunchArgument("speed", default_value="1.0"),
        DeclareLaunchArgument("output_dir", default_value="results/gazebo"),
        # The start pose must match the scenario's start pose in
        # scenario_paths.py; run_comparison.sh passes the right one.
        DeclareLaunchArgument("x", default_value="3.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        # The integration test keeps the launch up after the recorder exits and
        # ends it itself, once it has checked the outputs.
        DeclareLaunchArgument("exit_on_finish", default_value="true"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([bringup, "launch", "sim.launch.py"])),
            launch_arguments={
                "headless": "true",
                "rviz": "false",
                # Obstacle-free world, deliberately. The benchmark routes are
                # laid out in their own coordinates and several of them pass
                # through the campus buildings; the robot then drives into a
                # wall and the run records a stall that says nothing about the
                # control law. Obstacle avoidance is the Nav2 demo's job.
                "world": PathJoinSubstitution(
                    [bringup, "worlds", "empty_test.sdf"]),
                "x": x_pose,
                "y": y_pose,
                "yaw": yaw_pose,
            }.items(),
        ),

        # Ground truth from Gazebo is published in the world frame. For the
        # comparison the map frame IS the world frame, so map -> odom is the
        # spawn pose (the odom frame starts wherever the robot starts) rather
        # than the output of AMCL: localisation error would otherwise be mixed
        # into the controller comparison and the question here is which
        # controller tracks better, not which localiser drifts less.
        #
        # This used to be the identity, which was only right for scenarios
        # starting at the origin. From any other start pose the controller saw
        # itself at the origin while the robot was elsewhere.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_odom_spawn_pose",
            arguments=[
                "--x", x_pose, "--y", y_pose, "--yaw", yaw_pose,
                "--frame-id", "map", "--child-frame-id", "odom",
            ],
            parameters=[{"use_sim_time": True}],
        ),

        Node(
            package="shuttle_localisation",
            executable="path_publisher",
            name="path_publisher",
            output="screen",
            parameters=[{
                "scenario": scenario,
                "frame_id": "map",
                "use_sim_time": True,
            }],
        ),

        Node(
            package="shuttle_control",
            executable="path_tracker_node",
            name="path_tracker",
            output="screen",
            parameters=[
                PathJoinSubstitution([bringup, "config", "controllers.yaml"]),
                {
                    "controller_type": controller,
                    "max_speed": speed,
                    "global_frame": "map",
                    "base_frame": "base_footprint",
                    "use_sim_time": True,
                },
            ],
            # Commands go through the safety gate like every other source.
            remappings=[("odom", "/odometry/filtered"),
                        ("cmd_vel", "cmd_vel_in")],
        ),

        metrics_recorder,

        Node(
            package="shuttle_localisation",
            executable="ekf_evaluator",
            name="ekf_evaluator",
            output="screen",
            parameters=[{
                "output_csv": PathJoinSubstitution(
                    [output_dir, "ekf_evaluation.csv"]),
                # The estimate lives in the odom frame, which starts at the
                # spawn pose; ground truth is in the world frame.
                "odom_origin_x": x_pose,
                "odom_origin_y": y_pose,
                "odom_origin_yaw": yaw_pose,
                "use_sim_time": True,
            }],
        ),

        # End the run as soon as the recorder is done.
        #
        # Without this, nothing tells the launch the experiment is finished: the
        # other nodes keep running and the wrapper script's timeout eventually
        # kills everything. That wastes the whole remaining timeout on every
        # run and the shutdown then arrives as a hard signal partway through
        # the EKF evaluator's write, which is how its results were being lost.
        RegisterEventHandler(
            OnProcessExit(
                target_action=metrics_recorder,
                on_exit=[EmitEvent(event=Shutdown(reason="run complete"))],
            ),
            condition=IfCondition(exit_on_finish),
        ),
    ])
