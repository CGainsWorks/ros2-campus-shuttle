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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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
                    [bringup, "worlds", "empty_test.world"]),
                "x": x_pose,
                "y": y_pose,
                "yaw": yaw_pose,
            }.items(),
        ),

        # Ground truth from Gazebo is published in the world frame. For the
        # comparison the map frame is the world frame, so publish the identity
        # rather than running AMCL: localisation error would otherwise be mixed
        # into the controller comparison, and the question here is which
        # controller tracks better, not which localiser drifts less.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_odom_identity",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
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
            remappings=[("odom", "/odometry/filtered")],
        ),

        Node(
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
        ),

        Node(
            package="shuttle_localisation",
            executable="ekf_evaluator",
            name="ekf_evaluator",
            output="screen",
            parameters=[{
                "output_csv": "results/gazebo/ekf_evaluation.csv",
                "use_sim_time": True,
            }],
        ),
    ])
