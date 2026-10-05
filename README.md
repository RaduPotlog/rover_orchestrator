<p align="center">
  <img src="icons/Logo-Arm-WhiteOrange-372x372-1.png" alt="Mechatronics Academy" width="140">
</p>

# rover_orchestrator

Mechatronics Academy's Rover A1 autonomy stack, running on the orchestrator computer.

- [`rover_navigation`](rover_navigation/README.md) - Nav 2 configuration, the `IsMotionLocked`
  and `IsLidarHealthy` behavior-tree conditions and the SLAM map autosaver.
- [`rover_mission_manager`](rover_mission_manager/README.md) - behavior-tree mission
  supervision on top of Nav 2.
- [`rover_indoor_nav_manager`](rover_indoor_nav_manager/README.md) - indoor map library,
  places and runtime SLAM ↔ AMCL switching (`localization_source:=indoor`) for the
  rover_drive_interface web UI.
- [`rover_drive_mode`](rover_drive_mode/README.md) - the driving modes (Manual, Assisted with
  lidar slow-down/stop, Automatic): routes the web UI's joystick and Nav 2's commands to the
  platform according to the mode.
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

One script per localization source starts everything, in order and each part only once the
previous one is up: a local Zenoh router, `rover_gazebo` (simulator, URDF, RViz, ros2_control,
EKF, twist_mux + motion_lock), `rover_navigation`, `rover_drive_mode` and `rover_mission_manager`.
It then switches the drive mode to AUTOMATIC, so Nav 2 goals (RViz "Nav2 Goal") drive the rover.
One Ctrl+C stops everything, last-started first.

```bash
src/rover_orchestrator/scripts/sim/sim_nav_odom.sh     # odometry only, no map
src/rover_orchestrator/scripts/sim/sim_nav_slam.sh     # slam_toolbox; autosaves ~/rover_sim_maps/slam/map.yaml
src/rover_orchestrator/scripts/sim/sim_nav_amcl.sh     # AMCL on that map (or: sim_nav_amcl.sh <map.yaml>)
src/rover_orchestrator/scripts/sim/sim_nav_indoor.sh   # rover_indoor_nav_manager, maps in ~/rover_sim_maps/indoor
```

- Logs: one file per part under `~/.ros/rover_sim/<mode>-<time>/`; on failure the script prints
  the tail of the part that died.
- Every process runs as a Zenoh **client** of the local router (`tcp/localhost:7447`), whatever
  `ZENOH_CONFIG_OVERRIDE` says, so the simulated `/rover` never joins the real rover's router.
  Other terminals need the same:
  `export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/localhost:7447"]'`.
- Outside AUTOMATIC a Nav 2 goal plans but the rover does not move: Nav 2's commands reach the
  platform only through `rover_drive_mode`, and only in AUTOMATIC, which needs
  `rover_mission_manager` running.
- The rotation shim turns at 0.7 rad/s in simulation instead of the rover's 1.5: the simulated
  wheels have no static friction to break, and at 1.5 the rover rocked back and forth in place
  before driving off (`ROVER_SIM_SHIM_TURN_RATE` overrides it).
- Options (environment): `ROVER_SIM_RVIZ`, `ROVER_SIM_HEADLESS`, `ROVER_SIM_MAPS_DIR`,
  `ROVER_SIM_LOG_DIR`, `ROVER_SIM_TIMEOUT`, `ROVER_SIM_SHIM_TURN_RATE` - see
  `scripts/sim/sim_nav_common.sh`.

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
