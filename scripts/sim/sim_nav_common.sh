#!/usr/bin/env bash
# Copyright 2026 Mechatronics Academy
# Licensed under the Apache License, Version 2.0.
#
# Shared by sim_nav_{odom,amcl,slam,indoor}.sh; not meant to be run on its own.
#
# Starts the simulated Rover A1 and the orchestrator stack for one localization source, in
# order, each one only once the previous one is up:
#   Zenoh router -> rover_gazebo simulation -> rover_navigation bringup -> rover_drive_mode
#   -> rover_mission_manager -> drive mode AUTOMATIC
# then waits. One Ctrl+C stops everything, last-started first. If any part dies, the rest is
# stopped too and the tail of its log is printed.
#
# Each launch runs in its own session (setsid), so a Ctrl+C reaches only this script, which then
# stops them in order. Output goes to one log file per part.
#
# Environment (all optional):
#   ROVER_NAMESPACE      rover namespace                        (default: rover)
#   ROVER_SIM_MAPS_DIR   where SLAM/indoor maps are written     (default: ~/rover_sim_maps)
#   ROVER_SIM_LOG_DIR    where the per-run log folders go       (default: ~/.ros/rover_sim)
#   ROVER_SIM_RVIZ       start RViz with the simulation         (default: True)
#   ROVER_SIM_HEADLESS   Gazebo without its GUI                 (default: False)
#   ROVER_SIM_TIMEOUT    seconds to wait for each part          (default: 180)

set -o pipefail

SIM_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_WS_DIR="$(cd "$SIM_SCRIPT_DIR/../../../.." && pwd)"

NS="${ROVER_NAMESPACE:-rover}"
MAPS_DIR="${ROVER_SIM_MAPS_DIR:-$HOME/rover_sim_maps}"
RVIZ="${ROVER_SIM_RVIZ:-True}"
HEADLESS="${ROVER_SIM_HEADLESS:-False}"
TIMEOUT="${ROVER_SIM_TIMEOUT:-180}"

# rover_msgs/DriveMode
MODE_AUTOMATIC=3

# Zenoh session config for every process of the stack (and for other terminals, see the end).
SIM_ZENOH_CONFIG='mode="client";connect/endpoints=["tcp/localhost:7447"]'

PIDS=()
NAMES=()
LOG_DIR=""
CLEANING_UP=0

log() { printf '\033[1m[rover_sim %s]\033[0m %s\n' "$(date +%H:%M:%S)" "$*"; }
die() { log "ERROR: $*"; exit 1; }

# Starts "$@" in the background in its own session and process group (pgid = pid), output to
# $LOG_DIR/<name>.log. A non-interactive shell starts background commands with SIGINT ignored;
# env --default-signal restores it so cleanup can stop the launch like a Ctrl+C would.
start_bg() {
  local name="$1"
  shift
  setsid env --default-signal=INT "$@" < /dev/null > "$LOG_DIR/$name.log" 2>&1 &
  PIDS+=("$!")
  NAMES+=("$name")
  log "started $name (pid $!, log $LOG_DIR/$name.log)"
}

# Dies with the tail of the log of the first started part that is no longer running.
check_alive() {
  local i
  for i in "${!PIDS[@]}"; do
    if ! kill -0 "${PIDS[$i]}" 2> /dev/null; then
      log "${NAMES[$i]} exited unexpectedly; last lines of $LOG_DIR/${NAMES[$i]}.log:"
      tail -n 25 "$LOG_DIR/${NAMES[$i]}.log" | sed 's/^/    /'
      exit 1
    fi
  done
}

# wait_for <description> <command...>: retries the command every 2 s until it succeeds,
# failing after $TIMEOUT seconds or as soon as a started part dies.
wait_for() {
  local what="$1"
  shift
  local start=$SECONDS
  log "waiting for $what..."
  until "$@" > /dev/null 2>&1; do
    check_alive
    if ((SECONDS - start > TIMEOUT)); then
      die "$what not ready after ${TIMEOUT} s"
    fi
    sleep 2
  done
  log "$what: ready ($((SECONDS - start)) s)"
}

# Process group of a started part still has members (the launch can exit before its children).
group_alive() { pgrep -g "$1" > /dev/null 2>&1; }

stop_group() {
  local pid="$1" name="$2" t
  group_alive "$pid" || return 0
  log "stopping $name"
  kill -INT -- "-$pid" 2> /dev/null
  for t in $(seq 40); do
    group_alive "$pid" || return 0
    sleep 0.5
  done
  log "$name still running after 20 s, sending SIGTERM"
  kill -TERM -- "-$pid" 2> /dev/null
  for t in $(seq 10); do
    group_alive "$pid" || return 0
    sleep 0.5
  done
  log "$name still running, sending SIGKILL"
  kill -KILL -- "-$pid" 2> /dev/null
}

cleanup() {
  ((CLEANING_UP)) && return
  CLEANING_UP=1
  trap '' INT TERM
  local i
  for ((i = ${#PIDS[@]} - 1; i >= 0; i--)); do
    stop_group "${PIDS[$i]}" "${NAMES[$i]}"
  done
  [[ -n "$LOG_DIR" ]] && log "stopped; logs in $LOG_DIR"
}

# Every probe has a hard timeout: some ros2 CLI calls never return while a node is coming up.
topic_has_message() { timeout 15 ros2 topic echo --no-daemon --once --timeout 5 "$1"; }
service_available() { timeout 15 ros2 service list --no-daemon | grep -qx "$1"; }
# Nav 2's lifecycle manager answers success=True once all of its nodes are active.
lifecycle_manager_active() {
  timeout 15 ros2 service call "$1/is_active" std_srvs/srv/Trigger | grep -q 'success=True'
}
router_listening() { ss -ltn | grep -qE ':7447\b'; }

set_automatic() {
  local reply
  reply="$(timeout 20 ros2 service call "/$NS/set_drive_mode" rover_msgs/srv/SetDriveMode \
    "{mode: $MODE_AUTOMATIC}" 2>&1)"
  if grep -q 'success=True' <<< "$reply"; then
    return 0
  fi
  # Prints the refusal reason (e.g. a missing prerequisite) while wait_for keeps retrying.
  grep -o "message='[^']*'" <<< "$reply" | head -1 | sed 's/^/    refused: /' >&2
  return 1
}

# run_stack <localization_source> [extra bringup.launch.py arguments...]
run_stack() {
  local localization="$1"
  shift
  local nav_args=("$@")

  # Anchored to the process itself, so a shell whose command line merely mentions these names
  # does not count.
  local running_re='^[^ ]*/gz-sim-main |^[^ ]*python3[^ ]* [^ ]*/ros2 launch rover_(gazebo|navigation|drive_mode|mission_manager) '
  if pgrep -f "$running_re" > /dev/null; then
    die "a simulation or orchestrator launch is already running; stop it first:
$(pgrep -af "$running_re")"
  fi

  [[ -f "$SIM_WS_DIR/install/setup.bash" ]] ||
    die "$SIM_WS_DIR/install/setup.bash not found; build the workspace first"
  # shellcheck disable=SC1091
  source "$SIM_WS_DIR/install/setup.bash"

  # The simulation runs on this machine: never join the rover's (or any remote) Zenoh router,
  # or the simulated /<namespace> would share topics with the real rover. Client mode, as on the
  # rover: every process talks only to the router. In the default peer mode each new process
  # (every later ros2 CLI call, RViz goal tool, ...) must also connect directly to every other
  # one; once the Nav 2 component container stops accepting those connections, nothing started
  # afterwards can reach Nav 2's services or actions.
  export ZENOH_CONFIG_OVERRIDE="$SIM_ZENOH_CONFIG"
  export ROVER_NAMESPACE="$NS"
  # Only 'gps' mode wants the global EKF; every mode here owns map -> odom itself.
  export ROVER_USE_GPS=false ROVER_GPS_PUBLISH_MAP_TF=false
  # A ros2 daemon started from a shell with a different Zenoh config answers stale graphs.
  ros2 daemon stop > /dev/null 2>&1

  LOG_DIR="${ROVER_SIM_LOG_DIR:-$HOME/.ros/rover_sim}/${localization}-$(date +%Y%m%d-%H%M%S)"
  mkdir -p "$LOG_DIR" "$MAPS_DIR" || die "cannot create $LOG_DIR or $MAPS_DIR"

  # The SLAM map autosaver writes to /maps/map on the rover; point it at $MAPS_DIR here.
  local params="$LOG_DIR/rover_nav_params.yaml"
  sed "s|^\(\s*map_directory:\).*|\1 $MAPS_DIR/slam/map|" \
    "$(ros2 pkg prefix rover_navigation)/share/rover_navigation/config/rover_nav_params.yaml" \
    > "$params" || die "cannot write $params"
  mkdir -p "$MAPS_DIR/slam"

  trap cleanup EXIT
  trap 'exit 130' INT TERM

  log "localization_source=$localization, namespace=/$NS, logs in $LOG_DIR"

  if router_listening; then
    log "Zenoh router already listening on :7447, using it"
  else
    # The override is for sessions; applied to the router it would make it a client too.
    start_bg router env -u ZENOH_CONFIG_OVERRIDE ros2 run rmw_zenoh_cpp rmw_zenohd
    wait_for "Zenoh router" router_listening
  fi

  start_bg simulation ros2 launch rover_gazebo simulation.launch.py \
    namespace:="$NS" use_rviz:="$RVIZ" gz_headless_mode:="$HEADLESS"
  wait_for "simulation clock" topic_has_message /clock

  start_bg navigation ros2 launch rover_navigation bringup.launch.py \
    namespace:="$NS" use_sim_time:=True localization_source:="$localization" \
    params_file:="$params" "${nav_args[@]}"
  wait_for "Nav 2 (all navigation nodes active)" \
    lifecycle_manager_active "/$NS/lifecycle_manager_navigation"

  start_bg drive_mode ros2 launch rover_drive_mode rover_drive_mode.launch.py \
    namespace:="$NS" use_sim_time:=True
  wait_for "drive mode manager" service_available "/$NS/set_drive_mode"

  start_bg mission_manager ros2 launch rover_mission_manager rover_mission_manager.launch.py \
    namespace:="$NS" localization_source:="$localization" use_sim_time:=True
  wait_for "mission manager" service_available "/$NS/set_mission"

  wait_for "drive mode AUTOMATIC" set_automatic

  log "all up (localization_source=$localization). Send goals with 'Nav2 Goal' in RViz."
  log "follow a part: tail -f $LOG_DIR/<router|simulation|navigation|drive_mode|mission_manager>.log"
  log "other terminals (ros2 CLI, ...): export ZENOH_CONFIG_OVERRIDE='$SIM_ZENOH_CONFIG'"
  log "Ctrl+C stops everything."
  while true; do
    check_alive
    sleep 2
  done
}
