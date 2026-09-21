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

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    rover_dir = get_package_share_directory("rover_navigation")

    namespace = LaunchConfiguration("namespace")
    map_yaml_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")
    use_composition = LaunchConfiguration("use_composition")
    container_name = LaunchConfiguration("container_name")
    container_name_full = (namespace, "/", container_name)
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")
    localization_source = LaunchConfiguration("localization_source")
    initial_pose_x = LaunchConfiguration("initial_pose_x")
    initial_pose_y = LaunchConfiguration("initial_pose_y")
    initial_pose_yaw = LaunchConfiguration("initial_pose_yaw")

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {"use_sim_time": use_sim_time, "yaml_filename": map_yaml_file}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    stdout_linebuf_envvar = SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1")

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map", description="Path to map yaml file to load"
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation (Gazebo) clock if true"
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(rover_dir, "config", "rover_nav_params.yaml"),
        description="Path to the parameters file to use for all nav2 related nodes.",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart", default_value="true", description="Automatically startup the nav2 stack"
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        "use_composition", default_value="False", description="Use composed bringup if True"
    )

    declare_container_name_cmd = DeclareLaunchArgument(
        "container_name",
        default_value="nav2_container",
        description="the name of container that nodes will load in if use composition",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    # Kept identical to bringup.launch.py's and rover_mission_manager's lists on purpose --
    # test/launch/test_localization_launch.py asserts all three match, because a silent
    # divergence between them is a field bug rather than a startup error. 'slam' is never
    # routed here (bringup includes slam_launch.py instead) but stays in the list for that.
    # 'indoor' reaches here only through indoor_localization.launch.py, which
    # rover_indoor_nav_manager runs when it switches to a saved map: it means AMCL too.
    declare_localization_source_cmd = DeclareLaunchArgument(
        "localization_source",
        default_value="odom",
        description=(
            "Only 'amcl' and 'indoor' start nav2_amcl; every other value brings up map_server "
            "alone. "
            "See bringup.launch.py for what each value means."
        ),
        choices=["odom", "gps", "slam", "amcl", "indoor"],
    )

    declare_initial_pose_x_cmd = DeclareLaunchArgument(
        "initial_pose_x",
        default_value="0.0",
        description="X of AMCL's startup pose in <namespace>/map. Only used with amcl.",
    )

    declare_initial_pose_y_cmd = DeclareLaunchArgument(
        "initial_pose_y",
        default_value="0.0",
        description="Y of AMCL's startup pose in <namespace>/map. Only used with amcl.",
    )

    declare_initial_pose_yaw_cmd = DeclareLaunchArgument(
        "initial_pose_yaw",
        default_value="0.0",
        description="Yaw (rad) of AMCL's startup pose. Only used with amcl.",
    )

    def localization_setup(context, *args, **kwargs):
        """Build the localization nodes once localization_source is known.

        This is an OpaqueFunction rather than plain declarative actions because
        lifecycle_manager's `node_names` is a string_array whose *element count* has to
        vary with the mode, and no substitution can do that: PythonExpression always
        evaluates to a str, which the parameter declaration rejects, and ParameterValue
        builds one array element per substitution, so it cannot add or drop one.
        ComposableNode has no `condition` argument either. Resolving the configuration
        here turns both problems back into ordinary Python.
        """
        amcl_enabled = context.perform_substitution(localization_source) in ("amcl", "indoor")

        # map_server first: lifecycle_manager transitions in list order, and AMCL blocks
        # waiting for the map, so activating the publisher first avoids a startup stall.
        lifecycle_nodes = ["map_server"] + (["amcl"] if amcl_enabled else [])

        # Dotted keys override the nested initial_pose block in rover_nav_params.yaml.
        # Resolved to floats here because AMCL declares them as doubles and a launch
        # substitution would hand it strings.
        amcl_overrides = {
            "initial_pose.x": float(context.perform_substitution(initial_pose_x)),
            "initial_pose.y": float(context.perform_substitution(initial_pose_y)),
            "initial_pose.yaw": float(context.perform_substitution(initial_pose_yaw)),
        }

        # AMCL runs only with localization_source:=amcl, and is then the sole owner of
        # <namespace>/map -> <namespace>/odom. rover_ekf_global_node (rover_localization)
        # publishes that same transform when ROVER_GPS_PUBLISH_MAP_TF=true, and
        # slam_toolbox when localization_source:=slam. Exactly one of the three may run --
        # two owners do not error, they fight, and the pose visibly jitters between them.
        plain_nodes = [
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
            ),
        ]
        if amcl_enabled:
            plain_nodes.append(
                Node(
                    package="nav2_amcl",
                    executable="amcl",
                    name="amcl",
                    output="screen",
                    respawn=use_respawn,
                    respawn_delay=2.0,
                    parameters=[configured_params, amcl_overrides],
                    arguments=["--ros-args", "--log-level", log_level],
                )
            )
        plain_nodes.append(
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_localization",
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": autostart},
                    {"node_names": lifecycle_nodes},
                ],
            )
        )

        composable_nodes = [
            ComposableNode(
                package="nav2_map_server",
                plugin="nav2_map_server::MapServer",
                name="map_server",
                parameters=[configured_params],
            ),
        ]
        if amcl_enabled:
            composable_nodes.append(
                ComposableNode(
                    package="nav2_amcl",
                    plugin="nav2_amcl::AmclNode",
                    name="amcl",
                    parameters=[configured_params, amcl_overrides],
                )
            )
        composable_nodes.append(
            ComposableNode(
                package="nav2_lifecycle_manager",
                plugin="nav2_lifecycle_manager::LifecycleManager",
                name="lifecycle_manager_localization",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": autostart,
                        "node_names": lifecycle_nodes,
                    }
                ],
            )
        )

        return [
            GroupAction(
                condition=IfCondition(PythonExpression(["not ", use_composition])),
                actions=plain_nodes,
            ),
            LoadComposableNodes(
                condition=IfCondition(use_composition),
                target_container=container_name_full,
                composable_node_descriptions=composable_nodes,
            ),
        ]

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_localization_source_cmd)
    ld.add_action(declare_initial_pose_x_cmd)
    ld.add_action(declare_initial_pose_y_cmd)
    ld.add_action(declare_initial_pose_yaw_cmd)

    # Add the actions to launch all of the localiztion nodes
    ld.add_action(OpaqueFunction(function=localization_setup))

    return ld
