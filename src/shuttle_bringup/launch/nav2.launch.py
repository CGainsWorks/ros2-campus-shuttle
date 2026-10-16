"""Full autonomy: simulation, AMCL localisation, the global EKF and Nav2.

    ros2 launch shuttle_bringup nav2.launch.py headless:=false rviz:=true

Then send goals from RViz. Which path tracker Nav2 uses is set by the
`FollowPath` / `StanleyPath` plugin ids in config/nav2_params.yaml; both are
registered, so switching is a parameter change rather than a rebuild.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup = FindPackageShare("shuttle_bringup")
    nav2_params = PathJoinSubstitution([bringup, "config", "nav2_params.yaml"])
    map_yaml = LaunchConfiguration("map")

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument(
            "map",
            default_value=PathJoinSubstitution(
                [bringup, "maps", "campus_map.yaml"]),
            description="occupancy grid produced by slam.launch.py"),

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
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[nav2_params, {"yaml_filename": map_yaml,
                                      "use_sim_time": True}],
        ),
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=[nav2_params, {"use_sim_time": True}],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="screen",
            parameters=[nav2_params],
        ),

        # Global EKF: map -> odom, fusing the AMCL pose with wheel odometry and
        # the IMU. AMCL has tf_broadcast disabled so only one node owns the
        # transform.
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node_map",
            output="screen",
            parameters=[
                PathJoinSubstitution([bringup, "config", "ekf_global.yaml"]),
                {"use_sim_time": True},
            ],
            remappings=[("odometry/filtered", "odometry/filtered_map")],
        ),

        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[nav2_params],
            remappings=[("cmd_vel", "cmd_vel_nav")],
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[nav2_params],
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[nav2_params],
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[nav2_params],
        ),
        Node(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            name="velocity_smoother",
            output="screen",
            parameters=[nav2_params],
            # The smoothed command goes to the safety gate, which owns
            # /cmd_vel: Nav2 is one command source among several and an
            # e-stop or a dead controller has to stop it like any other.
            remappings=[("cmd_vel", "cmd_vel_nav"),
                        ("cmd_vel_smoothed", "cmd_vel_in")],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[nav2_params],
        ),
    ])
