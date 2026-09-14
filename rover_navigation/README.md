# rover_navigation

Package containing the Nav 2 configuration of the Rover A1 — costmaps, MPPI controller,
Smac 2D planner, recovery behaviors, behavior trees and map server.

It is a pure configuration package: it builds no nodes, it only installs launch files,
parameters, behavior trees and a map. It is meant to run on the orchestrator computer and
drives the rover by publishing `nav_cmd_vel_stamped`, which `rover_twist_mux` arbitrates
against the teleop sources.

## Prerequisites

ROS 2 **Jazzy**, with the workspace built and sourced. `rover_navigation` is not part of
`rover_metapackage`, so build it explicitly:

```bash
cd ~/ros2_ws/rover_a1
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install --packages-select rover_navigation
source install/setup.bash
```

Navigation only consumes data — the rest of the rover stack must already be running and
providing:

| What | Provided by |
|---|---|
| TF `odom -> base_link` | `rover_localization` (EKF) |
| TF `base_link -> lidar_link` | `rover_description` (URDF / robot_state_publisher) |
| `odom` (`nav_msgs/Odometry`) | `rover_localization`, remapped from `/odometry/filtered` |
| `/scan` (`sensor_msgs/LaserScan`) | LiDAR driver, or the Gazebo bridge in simulation |
| `cmd_vel` arbitration | `rover_twist_mux` (input `nav_cmd_vel_stamped`, priority 5) |

When navigation runs on a different machine than the rover, both must share the same
`ROS_DOMAIN_ID`.

## Config Files

- `config/rover_nav_params.yaml` - parameters for every Nav 2 node, in a single `/**:`
  block. The `<namespace>`, `<min_x>`/`<max_x>`/`<min_y>`/`<max_y>`/`<min_z>`/`<max_z>`,
  `<observation_topic>`, `<observation_topic_type>`, `<scan_topic>` and `<stvl_layer>`
  placeholders are substituted at launch time by `bringup.launch.py`.
- `map/empty_world.yaml` + `map/empty_world.png` - default empty map, 50 x 50 m at
  0.1 m/px.
- `map/rover_map_server.yaml` - a `map_server` parameter snippet, not a map.
- `behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml` - tree used for
  `NavigateToPose`.
- `behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml` - tree used for
  `NavigateThroughPoses`.

## Launch Files

- `bringup.launch.py` - top-level entry point. Starts the `nav2_container` component
  container and includes the two launch files below.
- `rover_nav.launch.py` - the Nav 2 navigation servers plus
  `lifecycle_manager_navigation`.
- `localization.launch.py` - `map_server` plus `lifecycle_manager_localization`.

## Running

#### Simulated rover:

```bash
# terminal 1 - simulator, URDF, RViz, ros2_control and EKF
ros2 launch rover_gazebo simulation.launch.py

# terminal 2 - navigation
ros2 launch rover_navigation bringup.launch.py \
  use_sim_time:=True \
  observation_topic_type:=laserscan \
  observation_topic:=scan \
  map:=$(ros2 pkg prefix rover_navigation)/share/rover_navigation/map/empty_world.yaml
```

#### Real rover:

```bash
# on the rover
ros2 launch rover_bringup rover_bringup.launch.py

# on the rover or on the orchestrator computer
ros2 launch rover_navigation bringup.launch.py \
  use_sim_time:=False \
  observation_topic_type:=laserscan \
  observation_topic:=scan \
  map:=$(ros2 pkg prefix rover_navigation)/share/rover_navigation/map/empty_world.yaml
```

#### Namespaced:

```bash
ros2 launch rover_navigation bringup.launch.py namespace:=rover_a1 ...
```

`namespace` defaults to the `ROBOT_NAMESPACE` environment variable. Note that the rest of
the rover stack reads `ROVER_NAMESPACE` instead, so set both if you namespace the robot.

## Launch Arguments

Arguments of `bringup.launch.py` (`ros2 launch rover_navigation bringup.launch.py
--show-args`):

| Argument | Default | Description |
|---|---|---|
| `autostart` | `True` | Automatically start up the Nav 2 stack. |
| `log_level` | `info` | Logging level: `debug`, `info`, `warning`, `error`. |
| `map` | `empty_world.yaml` | Map yaml file to load. Pass an absolute path. |
| `namespace` | `$ROBOT_NAMESPACE`, else empty | Namespace applied to all launched nodes. |
| `observation_topic` | `` (empty) | LaserScan or PointCloud2 topic feeding the costmaps. |
| `observation_topic_type` | `pointcloud` | `laserscan` or `pointcloud`. Use `laserscan`. |
| `params_file` | `<share>/rover_navigation/config/rover_nav_params.yaml` | Parameter file for all Nav 2 nodes. |
| `robot_model` | `$ROBOT_MODEL_NAME`, else `rover_a1` | Robot model; selects the footprint bounding box. |
| `slam` | `False` | Run SLAM instead of the map server. See limitations below. |
| `use_composition` | `True` | Load all servers into one component container. |
| `use_respawn` | `False` | Respawn a crashed node. Only when composition is disabled. |
| `use_sim_time` | `False` | Use the Gazebo clock. |
| `container_name` | `nav2_container` | Container the composable nodes are loaded into. |

The two sub-launch files can also be run on their own, but both default `params_file` to a
path inside `nav2_bringup`, so always pass it explicitly. `localization.launch.py` has no
default for `map`:

```bash
# navigation servers only
ros2 launch rover_navigation rover_nav.launch.py \
  params_file:=$(ros2 pkg prefix rover_navigation)/share/rover_navigation/config/rover_nav_params.yaml \
  use_composition:=False

# map server only
ros2 launch rover_navigation localization.launch.py \
  params_file:=$(ros2 pkg prefix rover_navigation)/share/rover_navigation/config/rover_nav_params.yaml \
  map:=$(ros2 pkg prefix rover_navigation)/share/rover_navigation/map/empty_world.yaml
```

## Nodes, Topics and Frames

Lifecycle nodes managed by `lifecycle_manager_navigation`: `controller_server`,
`smoother_server`, `planner_server`, `behavior_server`, `bt_navigator`,
`waypoint_follower`, `velocity_smoother`. `map_server` is managed separately by
`lifecycle_manager_localization`.

| Direction | Topic | Type |
|---|---|---|
| in | `odom` | `nav_msgs/Odometry` |
| in | `/scan` | `sensor_msgs/LaserScan` |
| in | `/map` | `nav_msgs/OccupancyGrid` (global costmap static layer) |
| out | `nav_cmd_vel_stamped` | `geometry_msgs/TwistStamped` |
| out | `local_costmap/costmap`, `global_costmap/costmap` | `nav_msgs/OccupancyGrid` |

`nav_cmd_vel_stamped` comes from the `velocity_smoother` (`cmd_vel_smoothed` is remapped to
it). `rover_twist_mux` gives it priority 5 — below both teleop sources — and masks it
whenever the `motion_lock` E-Stop is active.

All Nav 2 frames are `odom` / `base_link`, not `map` (see limitations). Plugins in use:

- Controller: `nav2_mppi_controller::MPPIController` (`DiffDrive` motion model,
  `vx_max 0.8`, `wz_max 1.0`, 10 Hz).
- Planner: `nav2_smac_planner::SmacPlanner2D`.
- Costmap layers: `spatio_temporal_voxel_layer` + inflation on both costmaps, plus a static
  layer on the global costmap.
- Behaviors: `spin`, `backup`, `wait`.
- Footprint: a 0.98 x 0.98 m square, built from the `rover_a1` bounding box hardcoded in
  `bringup.launch.py`.

## Sending a Goal

```bash
ros2 lifecycle get /bt_navigator     # expect: active [3]

ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: odom}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"

ros2 topic echo /nav_cmd_vel_stamped
```

The goal `frame_id` must be **`odom`**, not `map`.

## Known Limitations and Troubleshooting

- **`slam:=True` does not work.** `bringup.launch.py` includes `launch/slam_launch.py` and
  starts a `map_autosaver_node`; neither exists in this package.
- **There is no `map -> odom` transform.** AMCL is commented out in
  `localization.launch.py`, and every Nav 2 global frame is set to `odom`, so navigation is
  odometry-relative and drifts with the odometry. The global costmap's static layer has no
  usable transform to the map frame — expect transform warnings, or publish a `map -> odom`
  static transform if you want the static layer to contribute:
  ```bash
  ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id odom
  ```
- **Do not use the default `observation_topic_type:=pointcloud`.** It expects a
  `<observation_topic>_filtered` PointCloud2 produced by the crop-box and
  `pointcloud_to_laserscan` nodes, both of which are commented out in `bringup.launch.py`.
  Nothing publishes that topic, so the costmaps stay empty. Use `laserscan`.
- **Pass an absolute `map:=` path.** The bare-filename default is resolved against the
  working directory of the process, not the package share directory.
- **Servers stay `inactive` / `unconfigured` and the log repeats `Timed out waiting for
  transform from base_link to odom`:** the rover stack is not running. The costmaps block
  until `odom -> base_link` exists, so `controller_server` never finishes activating and
  `bt_navigator` stays inactive. Only `map_server` comes up on its own. Start
  `rover_bringup` or `rover_gazebo` first.
- **Rover does not move but `nav_cmd_vel_stamped` is publishing:** check `rover_twist_mux`
  — a higher-priority teleop input may be active, or `motion_lock` may be engaged. A stale
  `motion_lock` topic is treated as locked, so the mux closes if `rover_motion_lock_node` dies.
- **Debugging:** `use_composition:=False use_respawn:=True log_level:=debug` runs one
  process per server, which makes crashes and parameter errors far easier to read.
