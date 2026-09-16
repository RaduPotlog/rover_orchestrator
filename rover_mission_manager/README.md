# rover_mission_manager

Behavior-tree-driven mission supervision for the Rover A1, running on the orchestrator
computer next to `rover_navigation`.

The shape is the "manager" pattern used throughout this fleet — `rover_safety` in `rover_ros`,
and Husarion's `lights_manager` / `safety_manager` / `docking_manager`: a plain node owns a
BehaviorTree, ticks it once per wall-timer period, loads its leaf nodes from `.so` names given
as parameters, and exposes the live tree to Groot2.

## Two layers, on purpose

| Layer | What it owns | Change it by |
|---|---|---|
| The **behavior tree** (`behavior_trees/rover_mission.xml`) | Operator-facing policy: guards, sequencing, which leaf nodes run in what order. | Editing XML / Groot2. No rebuild. |
| **`RunMissionUseCase`** (`application/`) | The invariants: waypoint cursor, hold/resume, abort rules. | C++, with unit tests. |

**The shipped tree does not call Nav 2.** Waypoint dispatch belongs to `RunMissionUseCase`,
which owns the mission cursor; adding a `NavigateToPose` node to the tree as well would give
the rover two independent things sending goals to the same action server. If you do want the
tree to drive Nav 2, list `nav2_behavior_tree`'s prebuilt plugins in `ros_plugin_libs` and
read the note at the top of the tree file first.

## Architecture

Clean Architecture, matching `rover_battery` and `rover_hardware_interface`:

```
include/rover_mission_manager/
├── domain/                     # pure C++: no ROS, no BehaviorTree, no I/O
│   ├── mission.hpp             #   Mission, Waypoint, MissionState
│   ├── mission_policy.hpp      #   when it is safe to drive
│   └── ports/                  #   NavigationPort, MissionStatusPublisherPort
├── application/
│   └── run_mission_use_case.hpp
└── infrastructure/             # ROS 2 + BehaviorTree adapters
    ├── mission_manager_node.hpp
    ├── behavior_tree_runner.hpp
    ├── nav2_navigation_adapter.hpp
    └── ros_mission_status_publisher.hpp
```

`CMakeLists.txt` splits this into `rover_mission_manager_core` (domain + application, links no
ROS) and `rover_mission_manager_ros`, and `scripts/check_domain_purity.sh` runs as a CTest
that fails if a ROS or BehaviorTree header ever appears under `domain/`.

`infrastructure/behavior_tree_runner.hpp` is a generalised copy of
`rover_safety::BehaviorTreeSafety`. It is duplicated rather than reused because `rover_safety`
ships in the separate `rover_ros` repository, its class is safety-specific, and its
`CMakeLists.txt` exports `include/rover_safety` rather than `include`, so its headers do not
resolve from a downstream package. If that class is ever promoted to `rover_utils`, delete
this header in favour of it.

## Running

```bash
ros2 launch rover_mission_manager rover_mission_manager.launch.py \
  localization_source:=gps \
  namespace:=rover
```

`localization_source` **must match what `rover_navigation` was launched with** — it is what
decides whether waypoints are interpreted in `<namespace>/odom` or `<namespace>/map`.

## Interfaces

| Direction | Name | Type |
|---|---|---|
| in | `motion_lock` | `std_msgs/Bool` — from `rover_motion_lock_node` |
| in | `battery` | `sensor_msgs/BatteryState` |
| in | `diagnostics` | `diagnostic_msgs/DiagnosticArray` — `rover_lidar`'s health task |
| out | `mission_status` | `std_msgs/String`, latched |
| service | `run_mission` | `std_srvs/SetBool` — `false` cancels |
| action client | `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` |

### Mission source — not finished

`run_mission` currently only implements **cancel** (`data: false`). There is no way to hand
the manager a list of waypoints yet, because `rover_msgs` has no mission message and it lives
in the `rover_ros` repository, so adding one is a cross-repo change. `RunMissionUseCase` is
complete and tested; what is missing is only the inbound interface. The two options are a
`rover_msgs/Mission` message plus a `SetMission` service, or an orchestrator-side interfaces
package. Deliberately left as a decision rather than guessed at.

## Safety behaviour

The motion lock is a **hold**, not an abort: it is how an operator pauses the rover, so the
mission survives it and resumes on the same waypoint. It is fail-safe in both places that
read it — the `IsMotionLocked` BT condition and `MissionManagerNode::currentConditions()`
both treat "no message yet" and "last message older than `motion_lock_timeout`" as locked,
the same rule `rover_twist_mux` applies so that a dead `rover_motion_lock_node` closes the
mux rather than opening it.

A battery below `abort_battery_fraction` is an **abort**, checked before the lock: holding
would only keep discharging, and an engaged lock must not mask why the mission stopped. The
battery is ignored entirely until the first `BatteryState` with a non-NaN `percentage`.

A dead lidar is a **hold** too, for the same reason as the lock: Nav 2's costmaps stop being
trustworthy without it, but the sensor can come back and the mission should resume on the same
waypoint. `rover_lidar` exposes no service, no lifecycle transition and no boolean "lidar ok"
topic, so its `diagnostic_updater` task `rover_lidar_node: Lidar status` is the only signal
there is — the manager watches the raw `diagnostics` topic rather than `diagnostics_agg` so it
does not depend on `rover_diag_manager`'s aggregator running. `OK` and `WARN` both count as
usable (a sparse cloud or a low rate degrades the costmaps but does not invalidate them);
`ERROR`, `STALE` and a status older than `lidar_health_timeout` do not.

Unlike the motion lock, this is **not** fail-safe on "never heard": `require_lidar` defaults to
`false` so a rover booted with `ROVER_USE_LIDAR=false` still runs missions, and so that
`rover_lidar`'s 10 s startup delay in `rover_bringup` does not block the first mission. Set
`require_lidar: true` on a rover that always carries one. The same reasoning and the same
default apply to `IsLidarHealthy`'s `require_present` port in `rover_navigation`.

## Parameters

Generated by `generate_parameter_library` from `src/mission_manager_parameters.yaml`; defaults
live there, and `config/mission_manager.yaml` carries the deployed values. Notable ones:

| Parameter | Default | Notes |
|---|---|---|
| `bt_project_path` | *(required)* | The node refuses to start without it. |
| `tree_name` | `RoverMission` | Tree ID inside the project. |
| `timer_frequency` | `20.0` Hz | Tick rate. |
| `bt_server_port` | `4444` | Groot2. The next free port is used if taken, so several managers coexist. |
| `plugin_libs` | `[is_motion_locked_bt_node]` | Plain BT.CPP plugins. Built by `rover_navigation`. |
| `ros_plugin_libs` | *(unset)* | See the note in `config/mission_manager.yaml` — an empty YAML list is rejected by rcl, so leave it unset rather than writing `[]`. |
| `goal_frame_id` | `""` | Empty derives `<namespace>/odom`; the launch file sets it from `localization_source`. |
| `motion_lock_timeout` | `0.5` s | Publisher runs at 10 Hz. |
| `lidar_health_topic` | `diagnostics` | Raw `DiagnosticArray` topic, not `diagnostics_agg`. |
| `lidar_status_name` | `rover_lidar_node: Lidar status` | Exact `DiagnosticStatus` name to match. |
| `lidar_health_timeout` | `3.0` s | `diagnostic_updater` publishes at 1 Hz. |
| `require_lidar` | `false` | When true, a never-seen lidar status holds the mission. |
| `abort_battery_fraction` | `0.10` | 0..1, matching `BatteryState::percentage`. |

## Testing

```bash
colcon test --packages-select rover_mission_manager
colcon test-result --all
```

33 unit tests covering the mission state machine, the policy (including the lidar-health
rules) and the use case, plus the domain-purity check. All run without a ROS graph, which is the point of the `_core` split.
