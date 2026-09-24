# rover_indoor_nav_manager

Indoor map library, places, and runtime SLAM ↔ AMCL switching for rover_a1. It is the robot
side of the Clearpath IndoorNav-style **Facility** and **Places** features in
[rover_drive_interface](https://github.com/RaduPotlog/rover_drive_interface).

`rover_navigation`'s `bringup.launch.py` starts it with `localization_source:=indoor`
(`ROVER_LOCALIZATION_SOURCE=indoor` on the orchestrator):
- Nav 2 runs with `global_frame=<ns>/map`.
- This node owns whatever publishes `map → odom`, running
  `rover_navigation/indoor_localization.launch.py` as a child process group:

| Mode | Child | Publishes `map → odom` |
|------|-------|------------------------|
| `MAPPING` | slam_toolbox + map_saver (`slam_launch.py`) | slam_toolbox |
| `LOCALIZATION` | map_server + AMCL on a saved map (`localization.launch.py`, uncomposed) | AMCL |

A switch between modes stops and reaps the old group (SIGINT → SIGTERM → SIGKILL on the process
group) before starting the new one, so the two never publish `map → odom` at the same time.
`nav2_container` is never touched. The group runs as Zenoh clients (`zenoh_client_mode`), so
stopping it cannot stall the other containers' processes through direct peer links.

Loading another saved map while already localizing restarts nothing: the manager calls
map_server's `load_map` (which republishes `map`; AMCL has `first_map_only: false`) and
re-seeds AMCL. If that fails, it falls back to restarting the group.

## Interfaces (all relative to the rover namespace)

| Kind | Name | Type | Notes |
|------|------|------|-------|
| service | `start_mapping` | `std_srvs/Trigger` | Start a new SLAM session |
| service | `save_map` | `rover_msgs/SaveMap` | Save the SLAM map as `/maps/<name>/map.{yaml,pgm}` (MAPPING only) |
| service | `load_map` | `rover_msgs/LoadMap` | Switch to AMCL on a saved map, optionally seeding the pose |
| service | `delete_map` | `rover_msgs/DeleteMap` | Not the map in use |
| service | `save_place` | `rover_msgs/SavePlace` | Create (empty id) or update a place on the active map |
| service | `delete_place` | `rover_msgs/DeletePlace` | |
| topic (latched) | `localization_state` | `rover_msgs/LocalizationState` | `MAPPING` / `LOCALIZATION` / `SWITCHING` / `UNAVAILABLE` |
| topic (latched) | `maps` | `rover_msgs/MapList` | Stored maps plus the active one |
| topic (latched) | `places` | `rover_msgs/PlaceList` | Places of the active map |
| client | `map_saver/save_map` | `nav2_msgs/SaveMap` | Provided by slam_launch.py |

`start_mapping` and `load_map` answer within about 2 s, even though a switch can take longer.
Follow `localization_state` for the outcome.

## Behaviour

- **Startup.** The manager resumes on the last loaded map (`/maps/active`). If that map has a
  remembered pose (`/maps/<name>/last_pose.yaml`), AMCL starts there with a **wider spread**:
  - σ 1.5 m / 45° (`restore_sigma_xy` / `restore_sigma_yaw`) instead of AMCL's built-in
    ~0.5 m / 15°;
  - so it still converges if the rover was nudged or the pose is slightly stale;
  - the manager publishes this `initialpose` once AMCL is up, meaning it has a subscriber and the
    map → base_link TF resolves;
  - with no remembered pose it starts mapping, or uses the map origin.
- **When the pose is saved:**
  - the moment the rover comes to rest, from `odometry/wheels`: below 0.03 m/s and 0.05 rad/s for
    0.5 s after moving;
  - on shutdown;
  - before switching to another map or to mapping;
  - every 5 s while localized, as a backstop.
- **Find me.** The drive interface's **Find me** button calls AMCL's
  `reinitialize_global_localization` directly (particles over the whole map), then
  `request_nomotion_update` three times. It's a last resort: in repetitive areas AMCL can settle on
  a look-alike spot, so use Set pose when you know where the rover is.
- **Save, then load.** Loading a map right after saving it from the running SLAM session seeds
  AMCL with SLAM's current pose, so the rover stays localized.
- **Places.** Places live per map in `/maps/<name>/places.yaml`. Names are unique per map
  (case-insensitive). Places can only be edited while a map is loaded.
- **Map names.** `[A-Za-z0-9_-]{1,64}`, used directly as directory names. An existing name is
  refused (delete it first), as VDA 5050 does with `DUPLICATE_MAP`.
- **Crashes.** If the child stack exits on its own, the state becomes `UNAVAILABLE`. It is not
  restarted automatically.

## Layout (Clean Architecture)

```
rover_indoor_nav_manager/
├── domain/          # Place, PlaceBook, MapRecord, LocalizationState, ports - no ROS
├── application/     # IndoorNavService: every use case, over the ports
├── infrastructure/  # FileMapRepository, LaunchLocalizationController, ROS adapters, node
└── presentation/    # main()
```

`scripts/check_domain_purity.sh` (a CTest) fails the build if `domain/` or `application/`
imports ROS or an outer layer.

## Parameters

| Name | Default | |
|------|---------|--|
| `localization_params_file` | *(required)* | bringup's namespaced `rover_nav_params.yaml` |
| `maps_dir` | `/maps` | The `rover-maps` volume in rover-a1-orchestrator |
| `save_map_timeout` | `5.0` | s, map_saver |
| `load_map_timeout` | `5.0` | s, map_server `load_map` (in-place map switch) |
| `zenoh_client_mode` | `true` | Run the child launch as Zenoh clients (rmw_zenoh only) |
| `pose_record_period` | `5.0` | s, backstop for the stop-triggered save |
| `motion_topic` | `odometry/wheels` | Speed source for detecting a stop |
| `stop_linear_threshold` / `stop_angular_threshold` / `stop_hold_time` | `0.03` / `0.05` / `0.5` | m/s, rad/s, s |
| `restore_sigma_xy` / `restore_sigma_yaw` | `1.5` / `0.785` | m / rad, AMCL spread around a remembered pose |
| `auto_start` | `true` | Resume / start mapping on startup |
| `launch_package`, `launch_file` | `rover_navigation`, `indoor_localization.launch.py` | |

## Tests

```bash
colcon test --packages-select rover_indoor_nav_manager
```

The unit tests are pytest with fakes for every port: domain rules, every use case, the file
repository on a temp dir, the launch command line, and process-group start/stop. They need no
ROS graph.

`test/integration/test_indoor_nav_node.py` runs the real node on an rclpy graph with
`auto_start` off, so no SLAM/AMCL launch is spawned. It calls the services the drive UI uses
and checks that a late subscriber still gets the latched `maps` list. The test is pinned to
FastDDS on localhost, so it needs neither a Zenoh router nor the rover.
