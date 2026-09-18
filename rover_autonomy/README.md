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
`rover_ros`'s `hardware_deps.repos` / `simulation_deps.repos`. It pins:

- [`rover_navigation`](https://github.com/RaduPotlog/rover_navigation) at `1.5.1`, our fork
  of navigation2, for `nav2_smac_planner` - `planner_server`'s `GridBased` plugin and the
  one `nav2_*` package with no arm64 binary on lyrical/resolute. **Always needed.** The
  import lands in `src/rover_navigation`; that is the Nav 2 fork, not this repository's
  `rover_navigation` package.
- [`rover_pointcloud_crop_box`](https://github.com/RaduPotlog/rover_pointcloud_crop_box), the
  self-filter `rover_navigation` uses on its pointcloud path. Only used with
  `observation_topic_type:=pointcloud`; the default laserscan path consumes
  `rover_rs16_lidar`'s `<namespace>/scan` directly.

Import it, then immediately prune the fork to the one package we build (required, not a
size saving - see the comment in `autonomy_deps.repos`):

```bash
vcs import src < src/rover_orchestrator/rover_autonomy/autonomy_deps.repos
git -C src/rover_navigation sparse-checkout init --cone
git -C src/rover_navigation sparse-checkout set nav2_smac_planner
```

Note `rover_navigation` is deliberately **not** part of `rover_ros`'s `rover_metapackage`:
the rover computer and the orchestrator computer build different things.
