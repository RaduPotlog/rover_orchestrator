#!/usr/bin/env bash
# Copyright 2026 Mechatronics Academy
# Licensed under the Apache License, Version 2.0.
#
# Simulated Rover A1 + orchestrator, slam_toolbox owning map -> odom (localization_source:=slam).
# The map origin is the spawn pose; drive the whole area (closing a loop) and the autosaver
# writes $ROVER_SIM_MAPS_DIR/slam/map.yaml + its image (default ~/rover_sim_maps) every 15 s, which
# sim_nav_amcl.sh then loads. Ctrl+C stops everything.

source "$(dirname "${BASH_SOURCE[0]}")/sim_nav_common.sh"
log "map autosaved every 15 s to $MAPS_DIR/slam/map.yaml"
run_stack slam
