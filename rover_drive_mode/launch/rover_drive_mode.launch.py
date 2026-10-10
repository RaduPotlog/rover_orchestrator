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

"""Driving modes: the mode manager plus the collision monitor that guards ASSISTED driving.

Independent of Nav 2 on purpose: MANUAL and ASSISTED must work on a rover that does not run
navigation. AUTOMATIC additionally needs rover_navigation (whose collision monitor feeds
nav_cmd_vel_guarded) and rover_mission_manager (set_mission).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString

TELEOP_GUARD_NODE = "teleop_collision_monitor"
TELEOP_GUARD_LIFECYCLE_MANAGER = "lifecycle_manager_teleop_guard"


def generate_launch_description():
    rover_drive_mode = FindPackageShare("rover_drive_mode")

    namespace = LaunchConfiguration("namespace")
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable(
            "ROVER_SYSTEM_NAMESPACE",
            default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        ),
        description="Add namespace to all launched nodes.",
    )

    default_mode = LaunchConfiguration("default_mode")
    declare_default_mode_arg = DeclareLaunchArgument(
        "default_mode",
        default_value=EnvironmentVariable(
            "ROVER_ORCH_DRIVE_DEFAULT_MODE", default_value="assisted"),
        description="Driving mode at startup.",
        choices=["manual", "assisted"],
    )

    use_teleop_guard = LaunchConfiguration("use_teleop_guard")
    declare_use_teleop_guard_arg = DeclareLaunchArgument(
        "use_teleop_guard",
        default_value="True",
        description=(
            "Run the teleop collision monitor. Without it ASSISTED is refused and the rover "
            "boots in MANUAL."
        ),
    )

    manager_params_file = LaunchConfiguration("manager_params_file")
    declare_manager_params_file_arg = DeclareLaunchArgument(
        "manager_params_file",
        default_value=PathJoinSubstitution([rover_drive_mode, "config", "drive_mode_manager.yaml"]),
        description="Parameter file for the drive mode manager.",
    )

    guard_params_file = LaunchConfiguration("guard_params_file")
    declare_guard_params_file_arg = DeclareLaunchArgument(
        "guard_params_file",
        default_value=PathJoinSubstitution(
            [rover_drive_mode, "config", "teleop_collision_monitor.yaml"]
        ),
        description="Parameter file for the teleop collision monitor.",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    declare_use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if true.",
    )

    log_level = LaunchConfiguration("log_level")
    declare_log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level.",
        choices=["debug", "info", "warning", "error"],
    )

    # Frame ids are "rover/base_link", not "/rover/base_link", and plain "base_link" when the
    # rover runs without a namespace. Same substitution as rover_navigation's params.
    namespace_ext = PythonExpression(["'", namespace, "' + '/' if '", namespace, "' else ''"])
    guard_params = ParameterFile(
        ReplaceString(source_file=guard_params_file, replacements={"<namespace>/": namespace_ext}),
        allow_substs=True,
    )

    drive_mode_manager_node = Node(
        package="rover_drive_mode",
        executable="drive_mode_manager",
        name="drive_mode_manager",
        namespace=namespace,
        parameters=[
            manager_params_file,
            {
                "default_mode": default_mode,
                "use_teleop_guard": use_teleop_guard,
                "use_sim_time": use_sim_time,
            },
        ],
        arguments=["--ros-args", "--log-level", log_level],
        respawn=True,
        respawn_delay=2.0,
        output="screen",
    )

    teleop_guard_node = Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        name=TELEOP_GUARD_NODE,
        namespace=namespace,
        parameters=[guard_params, {"use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", log_level],
        respawn=True,
        respawn_delay=2.0,
        output="screen",
        condition=IfCondition(use_teleop_guard),
    )

    teleop_guard_lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name=TELEOP_GUARD_LIFECYCLE_MANAGER,
        namespace=namespace,
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": True,
                "node_names": [TELEOP_GUARD_NODE],
            }
        ],
        arguments=["--ros-args", "--log-level", log_level],
        output="screen",
        condition=IfCondition(use_teleop_guard),
    )

    return LaunchDescription(
        [
            declare_namespace_arg,
            declare_default_mode_arg,
            declare_use_teleop_guard_arg,
            declare_manager_params_file_arg,
            declare_guard_params_file_arg,
            declare_use_sim_time_arg,
            declare_log_level_arg,
            drive_mode_manager_node,
            teleop_guard_node,
            teleop_guard_lifecycle_manager_node,
        ]
    )
