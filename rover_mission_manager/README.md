# rover_mission_manager

Behavior-tree-driven mission supervision for the Rover A1, running on the orchestrator
computer next to `rover_navigation`.

The shape is the "manager" pattern used throughout this fleet — `rover_safety` in `rover_ros`,
and Husarion's `lights_manager` / `safety_manager` / `docking_manager`: a node owns a
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
decides whether waypoints are interpreted in `<namespace>/odom` (`odom`) or
`<namespace>/map` (`gps`, `slam`, `amcl`). `rover_navigation`'s
`test_localization_launch.py` asserts the two argument declarations stay in sync.

### Lifecycle

`mission_manager` is a lifecycle node, and by default (`autostart: true`) it configures and
activates itself at startup. If it cannot configure (a missing tree or plugin), the process
exits non-zero so the container restarts it.

| Transition | What happens |
|---|---|
| configure | Builds the tree and the Nav 2 / status adapters, and subscribes. `set_mission` and `run_mission` exist from here on but refuse requests until the node is active. |
| activate | Starts the tick timer. |
| deactivate | Stops ticking, **cancels the running mission and its Nav 2 goal**, and halts the tree. This is how to pause the manager without leaving the rover driving: `ros2 lifecycle set /<ns>/mission_manager deactivate`. |
| cleanup | Releases everything configure built, including the Groot2 port. |
| shutdown | Runs from rclcpp's pre-shutdown hook on Ctrl+C / SIGTERM, while the context is still valid, so a goal in flight is cancelled instead of abandoned. |

To hand the node to an external manager, set `autostart: false`. The node does not open a
bond, so a `nav2_lifecycle_manager` driving it needs `bond_timeout: 0.0`.

## Interfaces

| Direction | Name | Type |
|---|---|---|
| in | `motion_lock` | `std_msgs/Bool` — from `rover_motion_lock_node` |
| in | `rover_battery/battery_status` | `sensor_msgs/BatteryState` (`battery_topic`) |
| in | `diagnostics` | `diagnostic_msgs/DiagnosticArray` — `rover_rs16_lidar`'s health task |
| out | `mission_status` | `std_msgs/String`, latched — one line, for logs |
| out | `mission_state` | `rover_msgs/MissionState`, latched — for UIs (rover_drive_interface) |
| service | `set_mission` | `rover_msgs/SetMission` — replace the active mission with these waypoints and start it |
| service | `run_mission` | `std_srvs/SetBool` — `false` cancels; `true` is refused (use `set_mission`) |
| action client | `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` |

### Mission source

`set_mission` (`rover_msgs/srv/SetMission`) takes a list of `geometry_msgs/PoseStamped`
waypoints and hands them to `RunMissionUseCase::accept`. That cancels anything in flight, so a
GoTo from the drive UI is simply a one-waypoint mission that replaces the previous one.

- **Frames:** every waypoint must leave `frame_id` empty or use the manager's goal frame. The
  manager has no TF buffer, so a pose in another frame is rejected rather than driven to in
  the wrong frame.
- **Rejected requests:** an empty list, or a non-finite position or zero quaternion.
- **Ids:** an empty `mission_id` becomes `mission-<n>`.
- **Cancelling:** `run_mission` with `data: false`.

The request translation lives in `infrastructure/mission_request.cpp` and is unit tested in
`test/unit/test_mission_request.cpp`.

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
waypoint. `rover_rs16_lidar` exposes no service, no lifecycle transition and no boolean "lidar ok"
topic, so its `diagnostic_updater` task `rover_rs16_lidar_node: Lidar status` is the only signal
there is — the manager watches the raw `diagnostics` topic rather than `diagnostics_agg` so it
does not depend on `rover_diag_manager`'s aggregator running. `OK` and `WARN` both count as
usable (a sparse cloud or a low rate degrades the costmaps but does not invalidate them);
`ERROR`, `STALE` and a status older than `lidar_health_timeout` do not.

Unlike the motion lock, this is **not** fail-safe on "never heard": `require_lidar` defaults to
`false` so a rover booted with `ROVER_USE_LIDAR=false` still runs missions, and so that
a lidar driver that starts after the mission manager (it runs in a separate container,
`rover-a1-sensors`) does not block the first mission. Set
`require_lidar: true` on a rover that always carries one. The same reasoning and the same
default apply to `IsLidarHealthy`'s `require_present` port in `rover_navigation`.

## Parameters

Generated by `generate_parameter_library` from `src/mission_manager_parameters.yaml`; defaults
live there, and `config/mission_manager.yaml` carries the deployed values. Notable ones:

| Parameter | Default | Notes |
|---|---|---|
| `autostart` | `true` | Configure and activate at startup. `false` leaves the node unconfigured for an external lifecycle manager. |
| `bt_project_path` | *(required)* | The node refuses to start without it. |
| `tree_name` | `RoverMission` | Tree ID inside the project. |
| `timer_frequency` | `20.0` Hz | Tick rate. |
| `bt_server_port` | `4444` | Groot2. The next free port is used if taken, so several managers coexist. |
| `plugin_libs` | `[is_motion_locked_bt_node]` | Plain BT.CPP plugins. Built by `rover_navigation`. |
| `ros_plugin_libs` | *(unset)* | See the note in `config/mission_manager.yaml` — an empty YAML list is rejected by rcl, so leave it unset rather than writing `[]`. Leaves get a helper `nav2::LifecycleNode` that is on no executor, so they must spin their own callback group, as `nav2_behavior_tree`'s action, service and topic leaves do. |
| `goal_frame_id` | `""` | Empty derives `<namespace>/odom`; the launch file sets it to `<namespace>/map` for `gps`, `slam` and `amcl`. |
| `motion_lock_timeout` | `0.5` s | Publisher runs at 10 Hz. |
| `lidar_health_topic` | `diagnostics` | Raw `DiagnosticArray` topic, not `diagnostics_agg`. |
| `lidar_status_name` | `rover_rs16_lidar_node: Lidar status` | Exact `DiagnosticStatus` name to match. |
| `lidar_health_timeout` | `3.0` s | `diagnostic_updater` publishes at 1 Hz. |
| `require_lidar` | `false` | When true, a never-seen lidar status holds the mission. |
| `abort_battery_fraction` | `0.10` | 0..1, matching `BatteryState::percentage`. |

## Testing

```bash
colcon test --packages-select rover_mission_manager
colcon test-result --all
```

Unit tests cover the mission state machine, the policy (including the lidar-health rules)
and the use case. They run without a ROS graph, which is the point of the `_core` split. The
domain-purity check runs alongside them. There are two single-process ROS tests:

- `test_nav2_navigation_adapter` runs the adapter against an in-process fake `navigate_to_pose`
  server. It covers a cancel issued before the goal response arrives, which must still cancel
  the goal the server accepts. It also checks that our own CANCELED is ignored while another
  client's CANCELED fails the goal.
- `test_mission_manager_initialize` builds the node from the shipped config and tree, then
  drives it configure → activate → `set_mission` (HELD, since there is no motion lock) →
  deactivate (CANCELLED) → cleanup → configure again.

Off the rover, run them on FastDDS with `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, and unset
the Zenoh client config.
