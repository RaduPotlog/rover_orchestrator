# rover_autonomy

Metapackage for the autonomy stack that runs on the Rover A1 **orchestrator computer**. It
builds nothing itself; it exists so the whole stack can be built and installed with one
`--packages-up-to`.

| Package | Description |
|---|---|
| [`rover_navigation`](../rover_navigation/README.md) | Nav 2 configuration, the `IsMotionLocked` and `IsLidarHealthy` BT condition plugins and the SLAM map autosaver. |
| [`rover_mission_manager`](../rover_mission_manager/README.md) | Behavior-tree-driven mission supervision, dispatching Nav 2 actions. |

```bash
cd ~/ros2_ws/rover_a1
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install --packages-up-to rover_autonomy
source install/setup.bash
```

`autonomy_deps.repos` is the extension point for external sources, in the same spirit as
`rover_ros`'s `hardware_deps.repos` / `simulation_deps.repos`. It pins
[`rover_pointcloud_crop_box`](https://github.com/RaduPotlog/rover_pointcloud_crop_box), the
self-filter `rover_navigation` uses on its pointcloud path:

```bash
vcs import src < src/rover_orchestrator/rover_autonomy/autonomy_deps.repos
```

That is only needed for `observation_topic_type:=pointcloud`. The default laserscan path
consumes `rover_lidar`'s `<namespace>/scan` directly and needs nothing extra.

Note `rover_navigation` is deliberately **not** part of `rover_ros`'s `rover_metapackage`:
the rover computer and the orchestrator computer build different things.
