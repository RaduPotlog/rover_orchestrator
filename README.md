# rover_orchestrator

Mechatronics Academy's Rover A1 autonomy stack, running on the orchestrator computer.

- [`rover_navigation`](rover_navigation/README.md) - Nav 2 configuration, the `IsMotionLocked`
  and `IsLidarHealthy` behavior-tree conditions and the SLAM map autosaver.
- [`rover_mission_manager`](rover_mission_manager/README.md) - behavior-tree mission
  supervision on top of Nav 2.
- [`rover_indoor_nav_manager`](rover_indoor_nav_manager/README.md) - indoor map library,
  places and runtime SLAM ↔ AMCL switching (`localization_source:=indoor`) for the
  rover_drive_interface web UI.
- [`rover_autonomy`](rover_autonomy/README.md) - metapackage grouping them.

## Quick start

### Create workspace

```bash
mkdir -p ~/ros2_ws/rover_a1
cd ~/ros2_ws/rover_a1
git clone -b master https://github.com/RaduPotlog/rover_ros.git src/rover_ros
git clone -b master https://github.com/RaduPotlog/rover_orchestrator.git src/rover_orchestrator
```

`rover_ros` is required: `rover_mission_manager` builds against `rover_utils`.

### Setup environment variables

```bash
# Every $ROS_DISTRO below is expanded before ROS is sourced, so set it explicitly.
export ROS_DISTRO=lyrical

# The namespace the rover runs under. Both computers must agree, otherwise Nav 2 publishes
# /nav_cmd_vel_stamped while the rover's mux listens on /rover/nav_cmd_vel_stamped.
export ROVER_NAMESPACE=rover
```

### Clone dependency

```bash
vcs import src < src/rover_orchestrator/rover_autonomy/autonomy_deps.repos
```

Only needed for `observation_topic_type:=pointcloud`, which runs `pointcloud_crop_box` over
the raw lidar cloud. Every `nav2_*` package, `nav2_smac_planner` included, comes from apt.

### Build

```bash
sudo rosdep init
rosdep update --rosdistro $ROS_DISTRO
rosdep install --from-paths src -y -i

source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install --packages-up-to rover_autonomy

source install/setup.bash
```

### Running

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

# on the rover or on the orchestrator computer: navigation
ros2 launch rover_navigation bringup.launch.py \
  use_sim_time:=False \
  localization_source:=gps \
  map:=$(ros2 pkg prefix rover_navigation)/share/rover_navigation/map/empty_world.yaml
```

#### Mission manager:

```bash
# localization_source must match what rover_navigation was launched with
ros2 launch rover_mission_manager rover_mission_manager.launch.py \
  localization_source:=gps \
  namespace:=rover
```

### Testing

```bash
colcon test --packages-select rover_navigation rover_mission_manager
colcon test-result --all
```

Launch arguments, parameters, localization sources and troubleshooting are documented in the
package READMEs linked above.

## Related repositories

A complete rover is three repositories, one per container:

- [`rover_ros`](https://github.com/RaduPotlog/rover_ros) - the platform
  (`rover-a1-platform`).
- [`rover_sensors`](https://github.com/RaduPotlog/rover_sensors) - the sensor payload,
  GNSS and lidar drivers (`rover-a1-sensors`).
- [`rover_orchestrator`](https://github.com/RaduPotlog/rover_orchestrator) - this one, the
  autonomy stack (`rover-a1-orchestrator`).
