# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.

"""Runs rover_navigation's indoor_localization.launch.py as a child process group.

One group at a time: slam_toolbox (+ map_saver) for mapping, or map_server + AMCL for
localization. Exactly one of them may publish map -> odom, so the old group is stopped and
reaped before the new one starts. Nav 2 itself (nav2_container) is never touched.
Switching between saved maps while localizing restarts nothing: switch_map() hands map_server
the new yaml and re-seeds AMCL.

Under rmw_zenoh the group runs as Zenoh clients (router link only). As peers, every process
would hold a direct link to every other ROS process on the host, platform included, and
stopping the group made those processes stall for seconds (EKF, LED frames, measured on the
rover); as clients, only the router sees the links go.
"""

import math
import os
import signal
import subprocess
import threading
import time
from typing import Dict, List, Mapping, Optional

from ..domain.model import Pose2D

ZENOH_CLIENT_OVERRIDE = 'mode="client"'

# AMCL's own default start spread (standard deviations), used for an in-place map switch the
# same way a fresh AMCL uses it with the launch's initial_pose.
AMCL_DEFAULT_SIGMA_XY = 0.5
AMCL_DEFAULT_SIGMA_YAW = math.pi / 12


def child_env(base: Mapping[str, str], zenoh_client: bool) -> Dict[str, str]:
    """The environment for the child launch: `base`, plus Zenoh client mode when it applies."""
    env = dict(base)
    if zenoh_client and env.get('RMW_IMPLEMENTATION') == 'rmw_zenoh_cpp':
        # rmw_zenoh applies the ';'-separated keys in order, so appending wins over any earlier
        # mode in the override while keeping its other keys (e.g. connect/endpoints).
        existing = env.get('ZENOH_CONFIG_OVERRIDE', '')
        env['ZENOH_CONFIG_OVERRIDE'] = (
            f'{existing};{ZENOH_CLIENT_OVERRIDE}' if existing else ZENOH_CLIENT_OVERRIDE)
    return env


class LaunchLocalizationController:

    def __init__(self, logger, namespace: str, params_file: str, use_sim_time: bool,
                 log_level: str = 'info', launch_package: str = 'rover_navigation',
                 launch_file: str = 'indoor_localization.launch.py',
                 stop_timeout: float = 15.0, initial_pose_seeder=None,
                 zenoh_client: bool = True, map_loader=None):
        self._logger = logger
        self._namespace = namespace
        self._params_file = params_file
        self._use_sim_time = use_sim_time
        self._log_level = log_level
        self._launch = [launch_package, launch_file]
        self._stop_timeout = stop_timeout
        self._process: Optional[subprocess.Popen] = None
        self._lock = threading.Lock()
        # Publishes AMCL's initialpose (RosInitialPoseSeeder); None in process-only tests.
        self._seeder = initial_pose_seeder
        self._zenoh_client = zenoh_client
        # Calls map_server's load_map (RosMapLoader); None in process-only tests.
        self._map_loader = map_loader

    def command(self, mode: str, map_yaml: str = '', pose: Optional[Pose2D] = None) -> List[str]:
        cmd = ['ros2', 'launch', *self._launch,
               f'mode:={mode}',
               f'namespace:={self._namespace}',
               f'params_file:={self._params_file}',
               f'use_sim_time:={self._use_sim_time}',
               f'log_level:={self._log_level}']
        if mode == 'localization':
            p = pose or Pose2D(0.0, 0.0, 0.0)
            cmd += [f'map:={map_yaml}', f'initial_pose_x:={p.x}', f'initial_pose_y:={p.y}',
                    f'initial_pose_yaw:={p.theta}']
        return cmd

    def start_mapping(self) -> None:
        self._start(self.command('mapping'))

    def start_localization(self, map_yaml: str, initial_pose: Pose2D) -> None:
        self._start(self.command('localization', map_yaml, initial_pose))

    def switch_map(self, map_yaml: str, initial_pose: Pose2D) -> None:
        if self._map_loader is None or self._seeder is None:
            raise RuntimeError('no map loader configured')
        if not self.running():
            raise RuntimeError('localization is not running')
        self._map_loader.load(map_yaml)
        # AMCL only swaps its map on a new /map; its particles stay where the old map put them
        # until an initialpose arrives, so the seed is not optional here.
        self._seeder.seed(initial_pose, AMCL_DEFAULT_SIGMA_XY, AMCL_DEFAULT_SIGMA_YAW)

    def widen_initial_estimate(self, pose: Pose2D, sigma_xy: float, sigma_yaw: float) -> None:
        if self._seeder is None:
            raise RuntimeError('no initial pose seeder configured')
        self._seeder.seed(pose, sigma_xy, sigma_yaw)

    def running(self) -> bool:
        with self._lock:
            return self._process is not None and self._process.poll() is None

    def stop(self) -> None:
        if self._seeder is not None:
            self._seeder.cancel()  # a pending seed must not land on the next stack
        with self._lock:
            process, self._process = self._process, None
        if process is None:
            return
        if process.poll() is None:
            self._logger.info(f'Stopping localization stack (pid {process.pid})')
            # SIGINT lets launch shut its nodes down cleanly; escalate if it hangs.
            for sig, wait in ((signal.SIGINT, self._stop_timeout), (signal.SIGTERM, 5.0),
                              (signal.SIGKILL, 5.0)):
                self._signal_group(process, sig)
                try:
                    process.wait(timeout=wait)
                    break
                except subprocess.TimeoutExpired:
                    self._logger.warning(f'Localization stack ignored {sig.name}')
        # Nodes launch leaves behind stay in the group; make sure none survives.
        self._signal_group(process, signal.SIGKILL)

    def _start(self, cmd: List[str]) -> None:
        with self._lock:
            if self._process is not None and self._process.poll() is None:
                raise RuntimeError('A localization stack is already running.')
            self._logger.info('Starting: ' + ' '.join(cmd))
            self._process = subprocess.Popen(
                cmd, start_new_session=True, stdin=subprocess.DEVNULL,
                env=child_env(os.environ, self._zenoh_client))
            process = self._process
        # Fail fast on an immediate crash (bad arguments, missing package).
        time.sleep(1.0)
        if process.poll() is not None:
            raise RuntimeError(f'ros2 launch exited with code {process.returncode}')

    @staticmethod
    def _signal_group(process: subprocess.Popen, sig) -> None:
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            pass
