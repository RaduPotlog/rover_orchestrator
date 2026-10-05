#!/usr/bin/env bash
# Copyright 2026 Mechatronics Academy
# Licensed under the Apache License, Version 2.0.
#
# Simulated Rover A1 + orchestrator, Nav 2 on odometry only (localization_source:=odom):
# the global frame is <namespace>/odom, no map. Ctrl+C stops everything.
# Options: see the environment variables in sim_nav_common.sh.

source "$(dirname "${BASH_SOURCE[0]}")/sim_nav_common.sh"
run_stack odom
