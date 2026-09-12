# rover_orchestrator

Autonomy packages for the Mechatronics Academy Rover A1, intended to run on the rover's
orchestrator computer alongside the base system from
[`rover_ros`](https://github.com/RaduPotlog/rover_ros).

## Packages

| Package | Description |
|---|---|
| [`rover_navigation`](rover_navigation/README.md) | Nav 2 configuration — costmaps, MPPI controller, Smac 2D planner, behavior trees and map server. |

## Quick start

Build and source the workspace:

```bash
cd ~/ros2_ws/rover_a1
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install --packages-select rover_navigation
source install/setup.bash
```

Start the rover (real hardware or simulation), then start navigation:

```bash
# real rover
ros2 launch rover_bringup rover_bringup.launch.py
# or simulation
ros2 launch rover_gazebo simulation.launch.py

ros2 launch rover_navigation bringup.launch.py \
  use_sim_time:=False \
  observation_topic_type:=laserscan \
  observation_topic:=scan \
  map:=$(ros2 pkg prefix rover_navigation)/share/rover_navigation/map/empty_world.yaml
```

Use `use_sim_time:=True` in simulation. See
[`rover_navigation/README.md`](rover_navigation/README.md) for launch arguments, topics,
frames and known limitations.
