#!/usr/bin/env python3

# Copyright 2026 Rover A1 contributors
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

"""
The swappable half of localization_source:=indoor.

rover_indoor_nav_manager runs this as a child process and replaces it whenever the
operator starts mapping or loads a saved map:

- mode:=mapping       slam_toolbox + map_saver (slam_launch.py)
- mode:=localization  map_server + AMCL on `map` (localization.launch.py, AMCL enabled)

Both publish <namespace>/map -> <namespace>/odom, so the manager stops one before starting
the other. Nav 2 itself (bringup.launch.py's nav2_container) keeps running throughout.
Composition is off so stopping this launch never touches nav2_container. `params_file` must be
the file bringup.launch.py already namespaced (<namespace>/ replaced); the manager receives
that path from bringup.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    launch_dir = PathJoinSubstitution([FindPackageShare("rover_navigation"), "launch"])

    mode = LaunchConfiguration("mode")
    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")
    map_yaml = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")
    autostart = LaunchConfiguration("autostart")

    args = [
        DeclareLaunchArgument(
            "mode", description="mapping (slam_toolbox) or localization (map_server + AMCL).",
            choices=["mapping", "localization"]),
        DeclareLaunchArgument("namespace", default_value="", description="Rover namespace."),
        DeclareLaunchArgument(
            "params_file",
            description="rover_nav_params.yaml with <namespace>/ already substituted."),
        DeclareLaunchArgument(
            "map", default_value="", description="Map yaml; required for mode:=localization."),
        DeclareLaunchArgument("initial_pose_x", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_y", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_yaw", default_value="0.0"),
        DeclareLaunchArgument("use_sim_time", default_value="False"),
        DeclareLaunchArgument("log_level", default_value="info"),
        DeclareLaunchArgument("autostart", default_value="True"),
    ]

    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([launch_dir, "slam_launch.py"])),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'mapping'"])),
        launch_arguments={
            "autostart": autostart,
            "log_level": log_level,
            "params_file": params_file,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([launch_dir, "localization.launch.py"])),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'localization'"])),
        launch_arguments={
            "autostart": autostart,
            "container_name": "nav2_container",
            "initial_pose_x": LaunchConfiguration("initial_pose_x"),
            "initial_pose_y": LaunchConfiguration("initial_pose_y"),
            "initial_pose_yaw": LaunchConfiguration("initial_pose_yaw"),
            "localization_source": "indoor",
            "log_level": log_level,
            "map": map_yaml,
            "namespace": namespace,
            "params_file": params_file,
            "use_composition": "False",
            "use_sim_time": use_sim_time,
        }.items(),
    )

    return LaunchDescription(
        args + [GroupAction([PushRosNamespace(namespace), mapping, localization])]
    )
