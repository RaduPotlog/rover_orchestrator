# rover_navigation

Package containing the Nav 2 configuration of the Rover A1 — costmaps, MPPI controller,
Smac 2D planner, recovery behaviors, behavior trees and map server.

Mostly configuration — launch files, parameters, behavior trees and a map — plus two small
pieces of C++: the `IsMotionLocked` behavior-tree condition and the SLAM `map_autosaver_node`.
It runs on the orchestrator computer and drives the rover by publishing
`nav_cmd_vel_stamped`, which `rover_twist_mux` arbitrates against the teleop sources.

## Prerequisites

ROS 2 **Lyrical**, with the workspace built and sourced. `rover_navigation` is not part of
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
| TF `<namespace>/odom -> <namespace>/base_link` | `rover_localization` (EKF) |
| TF `<namespace>/base_link -> <namespace>/lidar_link` | `rover_description` (URDF / robot_state_publisher) |
| `odom` (`nav_msgs/Odometry`) | `rover_localization`, remapped from `odometry/filtered` |
| `<namespace>/scan` (`sensor_msgs/LaserScan`) | `rover_rs16_lidar` on hardware (sensor payload, `rover-a1-sensors`, `ROVER_USE_LIDAR=true`), or the Gazebo bridge in simulation |
| `<namespace>/diagnostics` (`diagnostic_msgs/DiagnosticArray`) | `rover_rs16_lidar` — watched by the `IsLidarHealthy` BT condition |

Both costmaps mark and clear from the lidar, so without it the local costmap stays empty and
the rover plans blind. `rover_rs16_lidar` also owns the cloud-to-scan projection — this
package deliberately does not run a second one.
| `cmd_vel` arbitration | `rover_twist_mux` (input `nav_cmd_vel_stamped`, priority 5) |

When navigation runs on a different machine than the rover, both must share the same
`ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION` (`rmw_zenoh_cpp`, see `rover_docker`'s README) and
`ROVER_NAMESPACE` (`rover`).

## Config Files

- `config/rover_nav_params.yaml` - parameters for every Nav 2 node, in a single `/**:`
  block. The `<namespace>`, `<min_x>`/`<max_x>`/`<min_y>`/`<max_y>`/`<min_z>`/`<max_z>`,
  `<observation_topic>`, `<observation_topic_type>`, `<scan_topic>` and `<global_frame>`
  placeholders are substituted at launch time by `bringup.launch.py`. The
  `<min_x>`..`<max_z>` box sizes both the Nav 2 footprint and `pointcloud_crop_box`'s
  self-filter, so the two cannot drift apart.
- `map/empty_world.yaml` + `map/empty_world.png` - default empty map, 50 x 50 m at
  0.1 m/px.
- `map/rover_map_server.yaml` - a `map_server` parameter snippet, not a map.
- `behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml` - tree used for
  `NavigateToPose`.
- `behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml` - tree used for
  `NavigateThroughPoses`.

## Launch Files

- `bringup.launch.py` - top-level entry point. Starts the `nav2_container` component
  container and includes the launch files below.
- `rover_nav.launch.py` - the Nav 2 navigation servers plus
  `lifecycle_manager_navigation`.
- `localization.launch.py` - `map_server` plus `lifecycle_manager_localization`. Used for
  `localization_source` `odom` and `gps`.
- `slam_launch.py` - `slam_toolbox` plus `map_saver` and `lifecycle_manager_slam`. Used for
  `localization_source:=slam`, and the only mode in which `map_autosaver_node` runs.

## Running

#### Simulated rover:

```bash
# terminal 1 - simulator, URDF, RViz, ros2_control and EKF
ros2 launch rover_gazebo simulation.launch.py

# terminal 2 - navigation
ros2 launch rover_navigation bringup.launch.py \
  use_sim_time:=True \
  map:=$(ros2 pkg prefix rover_navigation)/share/rover_navigation/map/empty_world.yaml
```

#### Real rover:

```bash
# on the rover: platform + sensor payload
ros2 launch rover_bringup rover_bringup.launch.py
ros2 launch rover_sensors_bringup rover_sensors.launch.py use_lidar:=true

# on the rover or on the orchestrator computer
ros2 launch rover_navigation bringup.launch.py \
  use_sim_time:=False \
  map:=$(ros2 pkg prefix rover_navigation)/share/rover_navigation/map/empty_world.yaml
```

#### Namespaced:

```bash
ros2 launch rover_navigation bringup.launch.py namespace:=rover ...
```

`namespace` defaults to the `ROVER_NAMESPACE` environment variable (falling back to the
legacy `ROBOT_NAMESPACE`), the same variable the rover stack reads. The rover runs under
`rover` (`rover_docker/docker-compose.yml`), so export `ROVER_NAMESPACE=rover` on this
computer too: otherwise Nav 2 publishes `/nav_cmd_vel_stamped` while the rover's mux listens on
`/rover/nav_cmd_vel_stamped`, and looks up `odom`/`base_link` instead of `rover/odom`/`rover/base_link`.

## Launch Arguments

Arguments of `bringup.launch.py` (`ros2 launch rover_navigation bringup.launch.py
--show-args`):

| Argument | Default | Description |
|---|---|---|
| `autostart` | `True` | Automatically start up the Nav 2 stack. |
| `log_level` | `info` | Logging level: `debug`, `info`, `warning`, `error`. |
| `map` | `empty_world.yaml` | Map yaml file to load. Pass an absolute path. |
| `namespace` | `$ROVER_NAMESPACE`, else `$ROBOT_NAMESPACE`, else empty | Namespace applied to all launched nodes. |
| `observation_topic` | `scan` | Topic feeding the costmaps' `stvl_layer`. `scan` with `observation_topic_type:=laserscan`; `rslidar_points` with `pointcloud`. |
| `observation_topic_type` | `laserscan` | `laserscan` consumes `rover_rs16_lidar`'s flattened scan directly; `pointcloud` runs `pointcloud_crop_box` over the raw RS16 cloud first. |
| `params_file` | `<share>/rover_navigation/config/rover_nav_params.yaml` | Parameter file for all Nav 2 nodes. |
| `robot_model` | `$ROBOT_MODEL_NAME`, else `rover_a1` | Robot model; selects the footprint bounding box. |
| `localization_source` | `odom` | Where the Nav 2 global frame comes from: `odom`, `gps` or `slam`. Replaces the old `slam` boolean. See below. |
| `use_composition` | `True` | Load all servers into one component container. |
| `use_respawn` | `False` | Respawn a crashed node. Only when composition is disabled. |
| `use_sim_time` | `False` | Use the Gazebo clock. |
| `container_name` | `nav2_container` | Container the composable nodes are loaded into. |

The two sub-launch files can also be run on their own; both default `params_file` to this
package's own `config/rover_nav_params.yaml`, the same file `bringup.launch.py` passes down.
`localization.launch.py` has no default for `map`:

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
| in | `<observation_topic>` (default `scan`) | `sensor_msgs/LaserScan` — the `stvl_layer`'s observation source |
| in | `diagnostics` | `diagnostic_msgs/DiagnosticArray` — read by `IsLidarHealthy` |
| in | `/<namespace>/map` | `nav_msgs/OccupancyGrid` (global costmap static layer) |
| out | `nav_cmd_vel_stamped` | `geometry_msgs/TwistStamped` |
| out | `local_costmap/costmap`, `global_costmap/costmap` | `nav_msgs/OccupancyGrid` |

`nav_cmd_vel_stamped` comes from the `velocity_smoother` (`cmd_vel_smoothed` is remapped to
it). `rover_twist_mux` gives it priority 5 — below both teleop sources — and masks it
whenever the `motion_lock` E-Stop is active.

The Nav 2 global frame depends on `localization_source` (see below); the robot frame is
always `<namespace>/base_link`, and the local costmap always stays on `<namespace>/odom`
because it is a rolling window. Plugins in use:

- Controller: `nav2_mppi_controller::MPPIController` (`DiffDrive` motion model,
  `vx_max 0.8`, `wz_max 1.0`, 10 Hz).
- Planner: `nav2_smac_planner::SmacPlanner2D`.
- Costmap layers: `spatio_temporal_voxel_layer` (`stvl_layer`) + inflation on both costmaps,
  plus a static layer on the global costmap. `stvl_layer` is what marks and clears from the
  lidar; `obstacle_range` is 3.0 m locally and 8.0 m globally.
- Behaviors: `spin`, `backup`, `wait`.
- BT plugins: `is_motion_locked_bt_node` (`IsMotionLocked`) and `is_lidar_healthy_bt_node`
  (`IsLidarHealthy`), both listed in `bt_navigator`'s `plugin_lib_names` and used as guards at
  the top of both navigation trees. Note their polarity is opposite: `IsMotionLocked` returns
  SUCCESS when *locked* and is wrapped in an `Inverter`, while `IsLidarHealthy` returns SUCCESS
  when *healthy* and is not.
- Footprint: a 0.98 x 0.98 m square, built from the `rover_a1` bounding box hardcoded in
  `bringup.launch.py`.

## Localization source

`localization_source` decides **which frame Nav 2 plans in, and who publishes
`map -> odom`**. Exactly one process may own that transform, so the three modes are mutually
exclusive. This argument replaces the old `slam` boolean.

| Mode | Nav 2 global frame | `map -> odom` published by | Use when |
|---|---|---|---|
| `odom` (default) | `<namespace>/odom` | nobody | No GPS, no SLAM. Navigation is odometry-relative and **drifts**; the global costmap's static layer will not line up with the map. |
| `gps` | `<namespace>/map` | `rover_ekf_global_node` (`rover_localization`) | `ROVER_USE_GPS` is set on the rover. |
| `slam` | `<namespace>/map` | `slam_toolbox` | Mapping a new area. Requires `ROVER_USE_GPS` **off**. |

```bash
# GPS-backed navigation, global frame rover/map
ros2 launch rover_navigation bringup.launch.py \
  localization_source:=gps \
  map:=$(ros2 pkg prefix rover_navigation)/share/rover_navigation/map/empty_world.yaml
```

Check there is exactly one publisher before trusting a goal:

```bash
ros2 run tf2_ros tf2_monitor rover/map rover/odom
```

In `gps` mode the map's origin and the EKF datum (`rover/localization/datum`) must describe
the same place, or the static layer will be offset from the world by that difference. This is
a configuration trap, not a bug — the transform will look perfectly healthy.

## Sending a Goal

```bash
ros2 lifecycle get /rover/bt_navigator     # expect: active [3]

ros2 action send_goal /rover/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: rover/odom}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"

ros2 topic echo /rover/nav_cmd_vel_stamped
```

The goal `frame_id` must match the mode: **`rover/odom`** with `localization_source:=odom`
(as above), **`rover/map`** with `gps` or `slam`.

## Known Limitations and Troubleshooting

- **Never publish a static `map -> odom`.** Earlier revisions of this README suggested
  `static_transform_publisher --frame-id map --child-frame-id odom` to make the global
  costmap's static layer usable. **Do not do that.** Since GPS fusion was integrated,
  `rover_ekf_global_node` (started by `rover_localization` whenever `ROVER_USE_GPS` is
  set) publishes `<namespace>/map -> <namespace>/odom` at 50 Hz. A static publisher would be a
  second owner of that transform and the two would fight. Use `localization_source:=gps`
  instead — see [Localization source](#localization-source).
- **Only one process may own `map -> odom`.** The three producers are mutually exclusive:
  `rover_ekf_global_node` (GPS), `slam_toolbox` (SLAM) and AMCL (disabled, and commented in
  `localization.launch.py` with the reason). Pick one with `localization_source`.
- **`observation_topic_type` picks between two working pipelines.** `laserscan` (the
  default) feeds the `stvl_layer` from `rover_rs16_lidar`'s `<namespace>/scan` directly. `pointcloud`
  runs `pointcloud_crop_box` over the raw `<namespace>/rslidar_points`, publishing
  `<observation_topic>_filtered` for the layer's `pointcloud` source — so pass
  `observation_topic:=rslidar_points` with it, and `vcs import` the crop-box repo first (see
  `rover_autonomy/autonomy_deps.repos`). Prefer `laserscan` unless you specifically want the
  full 3D cloud: the RS16 cloud is XYZI with no `ring`/`time` fields, and it costs noticeably
  more CPU on the orchestrator computer.
- **The local costmap stays empty and the rover drives into things.** Check, in order:
  `ros2 topic hz /<ns>/scan` (is `rover_rs16_lidar` up? It runs in `rover-a1-sensors` with
  `ROVER_USE_LIDAR=true`; see `/tmp/rover_sensors.log` there); then the `stvl_layer`'s `min_z`. `min_z`/`max_z`
  are in the **global** frame, not the sensor frame. `lidar_link` sits
  `ROVER_LIDAR_LOCALIZATION_Z` above `body_link` and that variable defaults to `0.0`, so a
  scan ring lands near z=0 — the upstream STVL default of `min_z: 0.1` silently discarded
  every single point. It is `-0.5` here. Raise `ROVER_LIDAR_LOCALIZATION_Z` to the real mount
  height rather than re-tuning `min_z` blindly.
- **`consider_footprint: true` costs CPU.** MPPI's footprint sweep over 800x40 trajectory
  points is the dominant cost in the control loop. If `ros2 topic hz /<ns>/nav_cmd_vel_stamped`
  falls below the 10 Hz `controller_frequency`, set `CostCritic.consider_footprint` back to
  `false` and accept point-check collision detection.
- **Pass an absolute `map:=` path.** The bare-filename default is resolved against the
  working directory of the process, not the package share directory.
- **Servers stay `inactive` / `unconfigured` and the log repeats `Timed out waiting for
  transform from base_link to odom`:** the rover stack is not running. The costmaps block
  until `odom -> base_link` exists, so `controller_server` never finishes activating and
  `bt_navigator` stays inactive. Only `map_server` comes up on its own. Start
  `rover_bringup` or `rover_gazebo` first.
- **Rover does not move and nothing is published on `nav_cmd_vel_stamped`:** the
  `IsMotionLocked` guard at the top of both navigation trees is aborting them. It is
  fail-safe — it reports "locked" before the first `motion_lock` message and whenever the
  last one is older than its `timeout` (0.5 s; the publisher runs at 10 Hz), matching
  `rover_twist_mux`'s own rule that a dead `rover_motion_lock_node` closes the mux. Check
  `ros2 topic hz /rover/motion_lock` and `ros2 topic echo /rover/motion_lock`.
- **Rover does not move but `nav_cmd_vel_stamped` is publishing:** check `rover_twist_mux`
  — a higher-priority teleop input may be active, or `motion_lock` may be engaged.
- **Trees abort and `motion_lock` is fine:** the `IsLidarHealthy` guard is failing. Check
  `ros2 topic echo /<ns>/diagnostics` for the status `rover_rs16_lidar_node: Lidar status` — `ERROR`
  is a cloud timeout, `STALE` is "no data yet", and a status older than the node's `timeout`
  (3 s) also fails. Unlike `IsMotionLocked`, this guard passes when the status has **never**
  been seen, so that `ROVER_USE_LIDAR=false` operation keeps working; set
  `require_present="true"` in both tree XMLs on a rover that always carries a lidar.
- **`plugin_lib_names` semantics.** `bt_navigator` is configured with
  `plugin_lib_names: [is_motion_locked_bt_node]`, on the assumption that Nav 2 loads its
  built-in BT nodes unconditionally from `nav2_behavior_tree/plugins_list.hpp` and treats
  this list as *additional*. If navigation instead fails at startup with unknown-node errors
  for stock nodes like `ComputePathToPose`, this Nav 2 build treats the list as an override:
  prepend the built-in list (`BT_BUILTIN_PLUGINS` in that header) to it. This could not be
  verified here — `nav2_bt_navigator` is not installed on this machine.
- **Debugging:** `use_composition:=False use_respawn:=True log_level:=debug` runs one
  process per server, which makes crashes and parameter errors far easier to read.
