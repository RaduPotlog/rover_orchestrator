#!/usr/bin/env python3

# Copyright 2025 Mechatronics Academy
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rover_mission_manager = FindPackageShare("rover_mission_manager")

    bt_project_path = LaunchConfiguration("bt_project_path")
    declare_bt_project_path_arg = DeclareLaunchArgument(
        "bt_project_path",
        default_value=PathJoinSubstitution(
            [rover_mission_manager, "behavior_trees", "rover_mission.xml"]
        ),
        description="Path to the BehaviorTree project the mission manager ticks.",
    )

    log_level = LaunchConfiguration("log_level")
    declare_log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level.",
        choices=["debug", "info", "warning", "error"],
    )

    namespace = LaunchConfiguration("namespace")
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable(
            "ROVER_NAMESPACE",
            default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        ),
        description="Add namespace to all launched nodes.",
    )

    params_file = LaunchConfiguration("params_file")
    declare_params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [rover_mission_manager, "config", "mission_manager.yaml"]
        ),
        description="Parameter file for the mission manager.",
    )

    localization_source = LaunchConfiguration("localization_source")
    declare_localization_source_arg = DeclareLaunchArgument(
        "localization_source",
        default_value="odom",
        description=(
            "Must match the value rover_navigation was launched with: it decides which frame "
            "mission waypoints are expressed in (<namespace>/odom, or <namespace>/map for "
            "gps, slam and amcl)."
        ),
        choices=["odom", "gps", "slam", "amcl"],
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    declare_use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if true.",
    )

    namespace_ext = PythonExpression(
        ["'", namespace, "' + '/' if '", namespace, "' else ''"]
    )
    goal_frame_id = PythonExpression(
        [
            "'",
            namespace_ext,
            "' + ('odom' if '",
            localization_source,
            "' == 'odom' else 'map')",
        ]
    )

    mission_manager_node = Node(
        package="rover_mission_manager",
        executable="mission_manager_node",
        name="mission_manager",
        namespace=namespace,
        parameters=[
            params_file,
            {
                "bt_project_path": bt_project_path,
                "goal_frame_id": goal_frame_id,
                "use_sim_time": use_sim_time,
            },
        ],
        arguments=["--ros-args", "--log-level", log_level],
        output="screen",
    )

    return LaunchDescription(
        [
            declare_bt_project_path_arg,
            declare_localization_source_arg,
            declare_log_level_arg,
            declare_namespace_arg,
            declare_params_file_arg,
            declare_use_sim_time_arg,
            mission_manager_node,
        ]
    )
