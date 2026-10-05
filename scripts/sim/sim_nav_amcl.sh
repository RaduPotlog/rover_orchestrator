#!/usr/bin/env bash
# Copyright 2026 Mechatronics Academy
# Licensed under the Apache License, Version 2.0.
#
# Simulated Rover A1 + orchestrator, AMCL on a saved map (localization_source:=amcl).
# Usage: sim_nav_amcl.sh [map.yaml]   (default: $ROVER_SIM_MAPS_DIR/slam/map.yaml, the map
# sim_nav_slam.sh saves). AMCL never converges on the empty default map, so a real one is
# required. The initial pose 0,0,0 is the spawn pose, which is also the SLAM map's origin.

source "$(dirname "${BASH_SOURCE[0]}")/sim_nav_common.sh"
map="${1:-$MAPS_DIR/slam/map.yaml}"
[[ -f "$map" ]] || die "no map at $map. Build one first: run sim_nav_slam.sh, drive the rover
around the world (close a loop), wait 15 s for the autosave, then stop it."
run_stack amcl map:="$(realpath "$map")"
