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

"""
Online SLAM for the rover: slam_toolbox plus a map_saver_server to persist the result.

Started by bringup.launch.py in place of localization.launch.py when
`localization_source:=slam`. In this mode slam_toolbox publishes
<namespace>/map -> <namespace>/odom, so it must not run alongside the GPS global EKF
(rover_ekf_global_node, enabled by ROVER_USE_GPS) or AMCL -- all three publish that
same transform and would fight over it. GPS fusion can stay on if the global EKF's TF is
off (ROVER_GPS_PUBLISH_MAP_TF=false / publish_global_tf:=false).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    autostart = LaunchConfiguration("autostart")
    log_level = LaunchConfiguration("log_level")
    params_file = LaunchConfiguration("params_file")
    use_respawn = LaunchConfiguration("use_respawn")
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_autostart_arg = DeclareLaunchArgument(
        "autostart",
        default_value="True",
        description="Automatically startup the SLAM stack.",
    )
    declare_log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level.",
        choices=["debug", "info", "warning", "error"],
    )
    declare_params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("rover_navigation"), "config", "rover_nav_params.yaml"]
        ),
        description="Path to the parameters file to use for slam_toolbox and map_saver.",
    )
    declare_use_respawn_arg = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes.",
    )
    declare_use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if true.",
    )

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            param_rewrites={"use_sim_time": use_sim_time},
            convert_types=True,
        ),
        allow_substs=True,
    )

    # Resolved lazily via FindPackageShare rather than get_package_share_directory() so that
    # simply importing this file does not require slam_toolbox to be installed.
    slam_toolbox_launch = PathJoinSubstitution(
        [FindPackageShare("slam_toolbox"), "launch", "online_sync_launch.py"]
    )

    map_saver_server = Node(
        package="nav2_map_server",
        executable="map_saver_server",
        name="map_saver",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", log_level],
    )

    lifecycle_manager_slam = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": autostart},
            {"node_names": ["map_saver"]},
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_launch),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam_params_file": params_file,
        }.items(),
    )

    slam_group = GroupAction(
        [
            # slam_toolbox publishes on absolute topics; pull them into the namespace this
            # group is pushed into so map_server/map_autosaver and the global costmap's
            # static layer all agree on <namespace>/map.
            SetRemap("/map", "map"),
            SetRemap("/map_metadata", "map_metadata"),
            SetRemap("/trajectories", "trajectories"),
            map_saver_server,
            lifecycle_manager_slam,
            slam_toolbox,
        ]
    )

    return LaunchDescription(
        [
            declare_autostart_arg,
            declare_log_level_arg,
            declare_params_file_arg,
            declare_use_respawn_arg,
            declare_use_sim_time_arg,
            slam_group,
        ]
    )
