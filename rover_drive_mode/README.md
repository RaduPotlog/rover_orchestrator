# rover_drive_mode

The Rover A1's driving modes. The drive UI (`rover_drive_interface`) switches between them;
this package owns the mode and routes commands according to it.

| Mode | Web joystick | Nav 2 (GoTo) |
|---|---|---|
| **MANUAL** | straight to the platform, no obstacle check | blocked |
| **ASSISTED** (boot default) | through the teleop collision monitor: slows down, then stops in front of obstacles | blocked |
| **AUTOMATIC** | a moving stick takes over: switch to ASSISTED, the mission is cancelled | through Nav 2's collision monitor, to the platform |

The RC transmitter (`rover_crsf_teleop`) and the Foxglove joystick do not pass through here:
they stay unfiltered overrides above every mode, so the rover can always be driven out of a
spot where the lidar blocks it.

## Architecture

```
web UI ── teleop_web_cmd_vel_stamped ──► drive_mode_manager
   MANUAL    ─────────────────────────────────────────────────────────┐
   ASSISTED  ─► teleop_guard_in ─► teleop_collision_monitor ─► teleop_guard_out ─┤
   AUTOMATIC: moving stick = takeover → ASSISTED; centred stick dropped         │
                                        teleop_driver_interface_cmd_vel_stamped ◄┘ ─► platform
                                        (rover_command_freshness_node → twist_mux, priority 8)

Nav 2 ─► cmd_vel_smoothed ─► collision_monitor (rover_navigation) ─► nav_cmd_vel_guarded
       ─► drive_mode_manager, AUTOMATIC only ─► nav_cmd_vel_stamped ─► platform (twist_mux, priority 5)
```

`drive_mode_manager` is the only writer of the two platform inputs, and it gates the last hop
as well as the first, so a collision monitor still flushing commands after a mode change cannot
drive the rover. If the manager dies, neither web teleop nor Nav 2 reaches the platform (twist_mux
times both inputs out); RC and Foxglove are unaffected. The platform (`rover_ros`) knows nothing
about modes.

The manager is a plain `rclcpp::Node`, not a lifecycle node, on purpose: it must route commands
from the moment it starts (booting into a mode that waits for an external `activate` would leave
the drive UI dead), and it owns no hardware. Its fail-safe is silence: both platform inputs time
out in twist_mux when it is not running. The collision monitors it depends on are lifecycle
nodes, activated by their lifecycle managers.

The package follows the workspace's Clean Architecture: `domain/` (mode, transition policy,
command routing, guard state — plain C++, checked by `scripts/check_domain_purity.sh`),
`application/drive_mode_use_case`, and `infrastructure/drive_mode_node` + the output adapter.

## Interfaces

| Direction | Name | Type |
|---|---|---|
| in | `teleop_web_cmd_vel_stamped` | `geometry_msgs/TwistStamped` — the drive UI's joystick |
| in | `teleop_guard_out` | `geometry_msgs/TwistStamped` — from the teleop collision monitor |
| in | `nav_cmd_vel_guarded` | `geometry_msgs/TwistStamped` — from Nav 2's collision monitor |
| in | `teleop_collision_monitor_state`, `collision_monitor_state` | `nav2_msgs/CollisionMonitorState` |
| out | `teleop_driver_interface_cmd_vel_stamped` | `geometry_msgs/TwistStamped` — platform web-teleop input |
| out | `teleop_guard_in` | `geometry_msgs/TwistStamped` — to the teleop collision monitor |
| out | `nav_cmd_vel_stamped` | `geometry_msgs/TwistStamped` — platform Nav 2 input |
| out | `drive_mode` | `rover_msgs/DriveMode`, latched — mode, guard state, reason of the last change |
| service | `set_drive_mode` | `rover_msgs/SetDriveMode` — refused with a reason when the mode's prerequisites are missing |
| client (liveness) | `set_mission` | `rover_msgs/SetMission` — AUTOMATIC needs `rover_mission_manager` |

Headers pass through untouched, so the platform's freshness check still measures the whole
browser-to-platform latency, collision monitor included.

## Rules

- **MANUAL** is always reachable.
- **ASSISTED** needs the teleop collision monitor to be launched (`use_teleop_guard`). A monitor
  that is running but has no lidar data is not a reason to refuse: it stops the rover, which is
  the fail-safe outcome, and the guard state reads `NO_DATA`.
- **AUTOMATIC** needs `rover_mission_manager` (its `set_mission` service on the graph). If the
  manager disappears for more than `mission_manager_loss_grace` (2 s), the mode falls back to
  ASSISTED ("mission manager lost").
- **Takeover:** in AUTOMATIC a web joystick command above `takeover_threshold` (0.02 m/s or
  rad/s) switches to ASSISTED ("operator takeover") and is routed as such. A centred stick is
  dropped, so the UI's release burst of zeros does not take over by itself.
- **Leaving AUTOMATIC** sends one zero on `nav_cmd_vel_stamped`, so twist_mux does not hold Nav
  2's last command until its timeout. `rover_mission_manager` sees the mode change on
  `drive_mode` and cancels the mission.
- **Boot:** `default_mode` (`assisted` or `manual`, never `automatic`), falling back to MANUAL
  when ASSISTED is unavailable.

## Guard state

`nav2_collision_monitor` reports its state only when the active zone changes, and only while
commands flow through it. So the manager keeps the last report, treats "no report yet" from a
running monitor as clear, and learns whether a monitor runs at all from the graph (a publisher on
its state topic). A monitor that stops on stale lidar data reports the zone `invalid source`,
shown as `NO_DATA` rather than as an obstacle.

| `guard` | Meaning |
|---|---|
| `GUARD_BYPASSED` | MANUAL: no collision monitor in the path |
| `GUARD_CLEAR` | nothing in the slow-down or stop zones |
| `GUARD_SLOWING` | obstacle in the slow-down zone |
| `GUARD_STOPPED` | obstacle in the stop zone |
| `GUARD_NO_DATA` | monitor not running, or no lidar data — motion blocked |

## Teleop collision monitor

`config/teleop_collision_monitor.yaml`, node `teleop_collision_monitor`, activated by its own
`lifecycle_manager_teleop_guard`. Two `velocity_polygon`s (`teleop_stop`, `teleop_slow`) whose
zone is picked from the **commanded** velocity: turning in place, driving forward, driving
backward. An obstacle ahead therefore does not block backing away. The forward and reverse zones
start at the footprint edge; the turn-in-place zone covers the swept corner radius (0.608 m).

- **Self-hits:** the turn-in-place zone cannot exclude the rover's own body (polygons have no
  holes), so any part of the rover the lidar sees inside it would block turning in place. The
  current lidar mount sees none (2026-09-28). If a future mount does, hide it with
  `rover_rs16_lidar`'s `scan.self_filter`.
- **All zone sizes and `slowdown_ratio` are tunables.** Measure the stop distance on the rover.
- `source_timeout: 0.5` s: a scan older than that stops the rover (`NO_DATA`).
- `base_shift_correction: false`: the guard needs only the static `lidar_link → base_link`
  transform, not odometry from another container.
- It is not a safety function: a single scan slice misses low and overhanging obstacles.
  See `rover_ros/rover_arch/SAFETY_CHAIN.md`.

## Running

```bash
ros2 launch rover_drive_mode rover_drive_mode.launch.py namespace:=rover
```

| Argument | Default | Notes |
|---|---|---|
| `namespace` | `$ROVER_NAMESPACE` | |
| `default_mode` | `$ROVER_DRIVE_DEFAULT_MODE` or `assisted` | `manual` or `assisted` |
| `use_teleop_guard` | `True` | `False` skips the teleop collision monitor; ASSISTED is then refused |
| `use_sim_time` | `False` | `True` in Gazebo |

In `rover_docker` it runs in `rover-a1-orchestrator` whenever `ROVER_START_DRIVE_MODE=true`
(default), independent of `ROVER_START_NAVIGATION`: without it the drive UI cannot drive.

```bash
ros2 topic echo /rover/drive_mode
ros2 service call /rover/set_drive_mode rover_msgs/srv/SetDriveMode "{mode: 1}"   # MANUAL
ros2 service call /rover/set_drive_mode rover_msgs/srv/SetDriveMode "{mode: 3}"   # AUTOMATIC
```

## Testing

```bash
colcon build --packages-select rover_drive_mode --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select rover_drive_mode --parallel-workers 1
```

- Unit (gtest): routing, transition policy, guard state, use case.
- Integration (`test_drive_mode_node`): latched boot mode, routing to each output, the Nav 2
  gate, takeover, lost mission manager, guard state from a fake monitor.
- Config (`test_collision_monitor_config.py`): every command selects a zone, zones stay clear of
  the footprint, the turn zone covers the swept radius, stop ⊂ slow, names do not clash with Nav
  2's monitor.
- Launch (`test_launch_structure.py`) and `check_domain_purity`.
