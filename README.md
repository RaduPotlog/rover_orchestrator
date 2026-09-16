# rover_orchestrator

Autonomy packages for the Mechatronics Academy Rover A1, intended to run on the rover's
orchestrator computer alongside the base system from
[`rover_ros`](https://github.com/RaduPotlog/rover_ros).

## Packages

| Package | Description |
|---|---|
| [`rover_autonomy`](rover_autonomy/README.md) | Metapackage grouping the two below. |
| [`rover_navigation`](rover_navigation/README.md) | Nav 2 configuration — costmaps, MPPI controller, Smac 2D planner, behavior trees, map server — plus the `IsMotionLocked` and `IsLidarHealthy` BT conditions and the SLAM map autosaver. |
| [`rover_mission_manager`](rover_mission_manager/README.md) | Behavior-tree-driven mission supervision, dispatching Nav 2 actions. |

## Quick start

Build and source the workspace:

```bash
cd ~/ros2_ws/rover_a1
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install --packages-up-to rover_autonomy
source install/setup.bash
```

Start the rover (real hardware or simulation), then start navigation:

On hardware the rover must be started with `ROVER_USE_LIDAR=true`: both Nav 2 costmaps mark
and clear from `rover_lidar`'s `<namespace>/scan`, and the navigation trees stop driving when
its diagnostics go bad. Without a lidar navigation still comes up, but drives blind.

```bash
# real rover
ROVER_USE_LIDAR=true ros2 launch rover_bringup rover_bringup.launch.py
# or simulation (the Gazebo bridge publishes the same <namespace>/scan)
ros2 launch rover_gazebo simulation.launch.py

ros2 launch rover_navigation bringup.launch.py \
  use_sim_time:=False \
  localization_source:=gps \
  map:=$(ros2 pkg prefix rover_navigation)/share/rover_navigation/map/empty_world.yaml

# optional: mission supervision on top of Nav 2
ros2 launch rover_mission_manager rover_mission_manager.launch.py localization_source:=gps
```

Use `use_sim_time:=True` in simulation.

`localization_source` picks who owns the `map -> odom` transform — `gps`
(`rover_ekf_global_node`, when `ROVER_USE_GPS` is set), `slam` (`slam_toolbox`), or
`odom` (nobody; navigation is odometry-relative and drifts). **Exactly one may run**, and it
must be the same value for both launch files above. See
[`rover_navigation/README.md`](rover_navigation/README.md) for the full argument list, topics,
frames and known limitations.

`observation_topic_type` defaults to `laserscan` / `observation_topic` to `scan`, which is the
`rover_lidar` path and needs no extra arguments. `pointcloud` runs `pointcloud_crop_box` over
the raw RS16 cloud instead — `vcs import` it from
[`rover_autonomy/autonomy_deps.repos`](rover_autonomy/autonomy_deps.repos) first.
