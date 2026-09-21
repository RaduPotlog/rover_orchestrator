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

A switch stops and reaps the old group (SIGINT → SIGTERM → SIGKILL on the process group)
before starting the new one, so the two never publish `map → odom` at the same time.
`nav2_container` is never touched.

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

- **Startup.** The manager resumes on the last loaded map (`/maps/active`), seeding AMCL at the
  last recorded pose (`/maps/<name>/last_pose.yaml`, written every 5 s while localized). With no
  map yet, it starts mapping.
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
| `pose_record_period` | `5.0` | s |
| `auto_start` | `true` | Resume / start mapping on startup |
| `launch_package`, `launch_file` | `rover_navigation`, `indoor_localization.launch.py` | |

## Tests

```bash
colcon test --packages-select rover_indoor_nav_manager
```

The tests are pytest with fakes for every port: domain rules, every use case, the file
repository on a temp dir, the launch command line, and process-group start/stop. They need no
ROS graph.
