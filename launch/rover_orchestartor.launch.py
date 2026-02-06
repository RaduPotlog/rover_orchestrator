#!/usr/bin/env python3

from rover_utils.logging import limit_log_level_to_info
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    common_dir_path = LaunchConfiguration("common_dir_path")
    declare_common_dir_path_arg = DeclareLaunchArgument(
        "common_dir_path",
        default_value="",
        description="Path to the common configuration directory.",
    )
    rover_orchestrator_common_dir = PythonExpression(
        [
            "'",
            common_dir_path,
            "/rover_orchestrator' if '",
            common_dir_path,
            "' else '",
            FindPackageShare("rover_orchestrator"),
            "'",
        ]
    )

    rover_orchestrator_pkg = FindPackageShare("rover_orchestrator")

    log_level = LaunchConfiguration("log_level")
    declare_log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR", "FATAL"],
        description="Logging level",
    )

    namespace = LaunchConfiguration("namespace")
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        description="Add namespace to all launched nodes.",
    )

    safety_bt_project_path = LaunchConfiguration("safety_bt_project_path")
    declare_safety_bt_project_path_arg = DeclareLaunchArgument(
        "safety_bt_project_path",
        default_value=PathJoinSubstitution(
            [rover_orchestrator_pkg, "behavior_trees", "SafetyBT.btproj"]
        ),
        description="Path to BehaviorTree project file, responsible for safety and shutdown orchestrator.",
    )

    shutdown_hosts_config_path = LaunchConfiguration("shutdown_hosts_config_path")
    declare_shutdown_hosts_config_path_arg = DeclareLaunchArgument(
        "shutdown_hosts_config_path",
        default_value=PathJoinSubstitution(
            [
                rover_orchestrator_common_dir,
                "config",
                "shutdown_hosts.yaml",
            ]
        ),
        description="Path to file with list of hosts to request shutdown.",
    )

    use_sim = LaunchConfiguration("use_sim")
    declare_use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="False",
        description="Whether simulation is used",
    )

    safety_orchestrator_node = Node(
        package="rover_orchestrator",
        executable="safety_orchestrator_node",
        name="safety_orchestrator_node",
        parameters=[
            PathJoinSubstitution([rover_orchestrator_pkg, "config", "safety_manager.yaml"]),
            {
                "bt_project_path": safety_bt_project_path,
                "shutdown_hosts_path": shutdown_hosts_config_path,
            },
        ],
        namespace=namespace,
        arguments=[
            "--ros-args",
            "--log-level",
            log_level,
            "--log-level",
            limit_log_level_to_info("rcl", log_level),
        ],
        emulate_tty=True,
        condition=UnlessCondition(use_sim),
    )

    actions = [
        declare_common_dir_path_arg,
        declare_log_level_arg,
        declare_safety_bt_project_path_arg,
        declare_namespace_arg,
        declare_shutdown_hosts_config_path_arg,
        declare_use_sim_arg,
        safety_orchestrator_node,
    ]

    return LaunchDescription(actions)