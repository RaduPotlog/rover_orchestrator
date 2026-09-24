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
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString, RewrittenYaml

def generate_launch_description():
    rover_navigation = FindPackageShare("rover_navigation")
    launch_dir = PathJoinSubstitution([rover_navigation, "launch"])

    autostart = LaunchConfiguration("autostart")
    log_level = LaunchConfiguration("log_level")
    map = LaunchConfiguration("map")
    maps_dir = LaunchConfiguration("maps_dir")
    namespace = LaunchConfiguration("namespace")
    observation_topic = LaunchConfiguration("observation_topic")
    observation_topic_type = LaunchConfiguration("observation_topic_type")
    localization_source = LaunchConfiguration("localization_source")
    initial_pose_x = LaunchConfiguration("initial_pose_x")
    initial_pose_y = LaunchConfiguration("initial_pose_y")
    initial_pose_yaw = LaunchConfiguration("initial_pose_yaw")
    params_file = LaunchConfiguration("params_file")
    robot_model = LaunchConfiguration("robot_model")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_autostart_arg = DeclareLaunchArgument(
        "autostart",
        default_value="True",
        description="Automatically startup the nav2 stack.",
    )
    declare_log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level.",
        choices=["debug", "info", "warning", "error"],
    )
    declare_map_arg = DeclareLaunchArgument(
        "map",
        # map_server resolves the path as given, so the default must be absolute.
        default_value=PathJoinSubstitution([rover_navigation, "map", "empty_world.yaml"]),
        description="Path to map yaml file to load.",
    )
    declare_maps_dir_arg = DeclareLaunchArgument(
        "maps_dir",
        default_value="/maps",
        description=(
            "Where maps are kept (the rover-maps volume in rover-a1-orchestrator). With "
            "localization_source:=indoor, rover_indoor_nav_manager keeps its maps, places and "
            "last pose here. With slam, map_autosaver writes <maps_dir>/map.{yaml,pgm}."
        ),
    )
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable(
            "ROVER_NAMESPACE",
            default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        ),
        description="Add namespace to all launched nodes.",
    )
    declare_observation_topic_arg = DeclareLaunchArgument(
        "observation_topic",
        default_value="scan",
        description=(
            "Topic feeding the costmaps' stvl_layer. With observation_topic_type:=laserscan "
            "this is rover_rs16_lidar's LaserScan ('scan'). With pointcloud it is the raw cloud "
            "('rslidar_points'), which pointcloud_crop_box self-filters into "
            "<observation_topic>_filtered."
        ),
    )
    declare_observation_topic_type_arg = DeclareLaunchArgument(
        "observation_topic_type",
        default_value="laserscan",
        description=(
            "Observation topic type. 'laserscan' consumes rover_rs16_lidar's flattened scan "
            "directly; 'pointcloud' runs pointcloud_crop_box over the raw RS16 cloud first."
        ),
        choices=["laserscan", "pointcloud"],
    )
    declare_params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [rover_navigation, "config", "rover_nav_params.yaml"]
        ),
        description="Path to the parameters file to use for all nav2 related nodes",
    )

    declare_robot_model_arg = DeclareLaunchArgument(
        "robot_model",
        default_value=EnvironmentVariable(name="ROBOT_MODEL_NAME", default_value="rover_a1"),
        description="Specify robot model",
        choices=["rover_a1"],
    )
    declare_localization_source_arg = DeclareLaunchArgument(
        "localization_source",
        default_value="odom",
        description=(
            "Where the Nav 2 global frame comes from. Exactly one process may publish\n"
            "map -> odom, so these are mutually exclusive:\n"
            "\t- 'odom': no map -> odom at all. The global frame IS <namespace>/odom, so\n"
            "\t  navigation is odometry-relative and drifts with the odometry. The global\n"
            "\t  costmap's static layer will not line up with the map. Default; matches the\n"
            "\t  behaviour of this package before GPS fusion existed.\n"
            "\t- 'gps': the global frame is <namespace>/map, published by rover_ekf_global_node\n"
            "\t  (rover_localization, started when ROVER_USE_GPS is set). Do NOT publish a\n"
            "\t  static map -> odom and do NOT enable AMCL in this mode.\n"
            "\t- 'slam': the global frame is <namespace>/map, published by slam_toolbox.\n"
            "\t  Requires ROVER_USE_GPS to be off.\n"
            "\t- 'amcl': the global frame is <namespace>/map, published by nav2_amcl,\n"
            "\t  which matches the lidar scan against the static map from map_server.\n"
            "\t  Indoor mode. Needs a real map (map:=, NOT the default empty_world.yaml -\n"
            "\t  every particle scores identically on an empty map and AMCL never\n"
            "\t  converges), ROVER_USE_LIDAR=true, and ROVER_GPS_PUBLISH_MAP_TF=false.\n"
            "\t- 'indoor': the global frame is <namespace>/map, and rover_indoor_nav_manager\n"
            "\t  owns localization at runtime: it runs slam_toolbox while a map is being\n"
            "\t  built and map_server + AMCL on a saved map (from /maps), and switches\n"
            "\t  between them on request from the drive UI (rover_drive_interface). `map`\n"
            "\t  and initial_pose_* are ignored; the manager remembers the last map and pose.\n"
            "\t  Same requirements as 'amcl'."
        ),
        choices=["odom", "gps", "slam", "amcl", "indoor"],
    )
    declare_initial_pose_x_arg = DeclareLaunchArgument(
        "initial_pose_x",
        default_value=EnvironmentVariable("ROVER_AMCL_INITIAL_POSE_X", default_value="0.0"),
        description=(
            "X of the pose AMCL is seeded with at startup, in <namespace>/map. Only used "
            "with localization_source:=amcl. The default 0.0 is correct only when the map "
            "origin is the rover's parking spot, i.e. the slam run started there."
        ),
    )
    declare_initial_pose_y_arg = DeclareLaunchArgument(
        "initial_pose_y",
        default_value=EnvironmentVariable("ROVER_AMCL_INITIAL_POSE_Y", default_value="0.0"),
        description="Y of AMCL's startup pose. See initial_pose_x.",
    )
    declare_initial_pose_yaw_arg = DeclareLaunchArgument(
        "initial_pose_yaw",
        default_value=EnvironmentVariable("ROVER_AMCL_INITIAL_POSE_YAW", default_value="0.0"),
        description="Yaw (rad) of AMCL's startup pose. See initial_pose_x.",
    )
    declare_use_composition_arg = DeclareLaunchArgument(
        "use_composition",
        default_value="True",
        description="Whether to use composed bringup.",
    )
    declare_use_respawn_arg = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )
    declare_use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if true.",
    )

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {"use_sim_time": use_sim_time, "yaml_filename": map}

    namespace_ext = PythonExpression(["'", namespace, "' + '/' if '", namespace, "' else ''"])
    # What amcl and slam_toolbox subscribe to. Both need a LaserScan, so in pointcloud mode
    # observation_topic names a PointCloud2 and cannot be used here -- fall back to 'scan',
    # which rover_rs16_lidar publishes in both modes (its scan projection always runs).
    scan_topic = PythonExpression(
        [
            "'scan' if '",
            observation_topic_type,
            "' == 'pointcloud' else '",
            observation_topic,
            "'",
        ]
    )

    slam = PythonExpression(["'", localization_source, "' == 'slam'"])
    indoor = PythonExpression(["'", localization_source, "' == 'indoor'"])
    # localization.launch.py (map_server, + AMCL for 'amcl') runs for every fixed mode; slam
    # has its own launch, and in 'indoor' rover_indoor_nav_manager starts one or the other.
    static_localization = PythonExpression(
        ["'", localization_source, "' not in ('slam', 'indoor')"]
    )

    # Nav 2's global frame. In 'odom' mode it stays <namespace>/odom (no map -> odom exists);
    # in 'gps', 'slam' and 'amcl' it is <namespace>/map, and the transform comes from whichever
    # single source localization_source names -- rover_ekf_global_node, slam_toolbox or
    # nav2_amcl respectively. The local costmap is a rolling window and always stays on
    # <namespace>/odom -- it is deliberately not tokenised.
    global_frame = PythonExpression(
        [
            "'",
            namespace_ext,
            "' + ('odom' if '",
            localization_source,
            "' == 'odom' else 'map')",
        ]
    )

    bb_padding = 0.04
    robot_bounding_box = {
        "rover_a1": {
            "min_x": -0.45 - bb_padding,
            "min_y": -0.45 - bb_padding,
            "min_z": 0.05,
            "max_x": 0.45 + bb_padding,
            "max_y": 0.45 + bb_padding,
            "max_z": 0.5,
        },
    }
    # Output of pointcloud_crop_box, and the topic the stvl_layer's `pointcloud` source
    # reads. Only produced in pointcloud mode.
    observation_topic_filtered = PythonExpression(
        ["'", observation_topic, "_filtered'"],
    )

    def override_params_file(robot_model_name):
        bounding_box = robot_bounding_box[robot_model_name]
        params = ReplaceString(
            source_file=params_file,
            replacements={
                "<namespace>/": namespace_ext,
                "<min_x>": str(bounding_box["min_x"]),
                "<max_x>": str(bounding_box["max_x"]),
                "<min_y>": str(bounding_box["min_y"]),
                "<max_y>": str(bounding_box["max_y"]),
                "<min_z>": str(bounding_box["min_z"]),
                "<max_z>": str(bounding_box["max_z"]),
                "<global_frame>": global_frame,
                "<observation_topic>": observation_topic,
                "<observation_topic_type>": observation_topic_type,
                "<scan_topic>": scan_topic,
            },
            condition=IfCondition(
                PythonExpression(["'", robot_model, f"' == '{robot_model_name}'"])
            ),
        )

        return params

    params_file = override_params_file("rover_a1")

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    bringup_cmd_group = GroupAction(
        [
            PushRosNamespace(namespace),
            # Strips the rover's own body out of the raw RS16 cloud before it reaches the
            # costmaps. Only needed on the pointcloud path: in laserscan mode the stvl_layer
            # clears the footprint itself via update_footprint_enabled.
            #
            # There is deliberately NO cloud-to-scan conversion here -- rover_rs16_lidar owns
            # that conversion and already publishes <ns>/scan, so a second one would
            # double-publish the topic.
            Node(
                condition=IfCondition(
                    PythonExpression(["'", observation_topic_type, "' == 'pointcloud'"])
                ),
                package="pointcloud_crop_box",
                executable="pointcloud_crop_box_node",
                name="pointcloud_crop_box",
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                output="screen",
            ),
            Node(
                condition=IfCondition(use_composition),
                name="nav2_container",
                package="rclcpp_components",
                executable="component_container_isolated",
                parameters=[configured_params, {"autostart": autostart}],
                arguments=["--ros-args", "--log-level", log_level],
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([launch_dir, "slam_launch.py"])
                ),
                condition=IfCondition(slam),
                launch_arguments={
                    "autostart": autostart,
                    "log_level": log_level,
                    "params_file": params_file,
                    "use_respawn": use_respawn,
                    "use_sim_time": use_sim_time,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([launch_dir, "localization.launch.py"])
                ),
                condition=IfCondition(static_localization),
                launch_arguments={
                    "autostart": autostart,
                    "container_name": "nav2_container",
                    "initial_pose_x": initial_pose_x,
                    "initial_pose_y": initial_pose_y,
                    "initial_pose_yaw": initial_pose_yaw,
                    "localization_source": localization_source,
                    "log_level": log_level,
                    "map": map,
                    "namespace": namespace,
                    "params_file": params_file,
                    "use_composition": use_composition,
                    "use_respawn": use_respawn,
                    "use_sim_time": use_sim_time,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([launch_dir, "rover_nav.launch.py"])
                ),
                launch_arguments={
                    "namespace": namespace,
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "params_file": params_file,
                    "use_composition": use_composition,
                    "use_respawn": use_respawn,
                    "container_name": "nav2_container",
                }.items(),
            ),
            # 'indoor': the manager gets the namespaced params file so the localization
            # sub-stack it launches sees the same <namespace>/ substitutions as Nav 2. It is
            # not a declared dependency of rover_navigation (it depends on this package's
            # launch files); rover_autonomy pulls both in.
            Node(
                condition=IfCondition(indoor),
                package="rover_indoor_nav_manager",
                executable="indoor_nav_manager_node",
                name="indoor_nav_manager",
                parameters=[
                    {
                        "localization_params_file": params_file,
                        "maps_dir": maps_dir,
                        "use_sim_time": use_sim_time,
                        "log_level": log_level,
                    }
                ],
                arguments=["--ros-args", "--log-level", log_level],
                output="screen",
            ),
            Node(
                condition=IfCondition(slam),
                name="map_autosaver",
                package="rover_navigation",
                executable="map_autosaver_node",
                parameters=[
                    configured_params,
                    {"map_directory": PathJoinSubstitution([maps_dir, "map"])},
                ],
                arguments=["--ros-args", "--log-level", log_level],
                output="screen",
            ),
        ]
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            declare_autostart_arg,
            declare_log_level_arg,
            declare_map_arg,
            declare_maps_dir_arg,
            declare_namespace_arg,
            declare_observation_topic_arg,
            declare_observation_topic_type_arg,
            declare_params_file_arg,
            declare_localization_source_arg,
            declare_initial_pose_x_arg,
            declare_initial_pose_y_arg,
            declare_initial_pose_yaw_arg,
            declare_robot_model_arg,
            declare_use_composition_arg,
            declare_use_respawn_arg,
            declare_use_sim_time_arg,
            bringup_cmd_group,
        ]
    )
