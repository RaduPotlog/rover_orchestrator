#!/usr/bin/env python3

# Copyright 2026 Mechatronics Academy
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

"""The follow_me node. Needs Nav 2 with the Following server (rover_navigation), and a person
tracker publishing tracked_person (rover_perception: use_person_tracking)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
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
            [FindPackageShare("rover_follow_me"), "config", "follow_me.yaml"]
        ),
        description="Parameter file for the follow_me node.",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    declare_use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if true.",
    )

    follow_me_node = Node(
        package="rover_follow_me",
        executable="follow_me_node",
        name="follow_me",
        namespace=namespace,
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", log_level],
        output="screen",
    )

    return LaunchDescription(
        [
            declare_log_level_arg,
            declare_namespace_arg,
            declare_params_file_arg,
            declare_use_sim_time_arg,
            follow_me_node,
        ]
    )
