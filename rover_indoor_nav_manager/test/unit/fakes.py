# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.

"""In-memory implementations of the ports, for the application tests."""

from rover_indoor_nav_manager.domain.model import MapRecord
from rover_indoor_nav_manager.domain.ports import (
    IndoorNavObserver,
    LocalizationController,
    MapRepository,
    MapSaver,
    RobotPoseSource,
)


class FakeMaps(MapRepository):
    def __init__(self, names=(), active=''):
        self.maps = {n: {'places': [], 'pose': None} for n in names}
        self.active = active

    def list_maps(self):
        return [MapRecord(name=n) for n in sorted(self.maps)]

    def exists(self, name):
        return name in self.maps

    def map_yaml_path(self, name):
        return f'/maps/{name}/map.yaml'

    def map_url(self, name):
        return f'/maps/{name}/map'

    def prepare(self, name):
        self.maps.setdefault(name, {'places': [], 'pose': None})

    def delete(self, name):
        self.maps.pop(name, None)

    def load_places(self, name):
        return list(self.maps[name]['places'])

    def save_places(self, name, places):
        self.maps[name]['places'] = list(places)

    def active_map(self):
        return self.active

    def set_active_map(self, name):
        self.active = name

    def last_pose(self, name):
        return self.maps.get(name, {}).get('pose')

    def save_last_pose(self, name, pose):
        self.maps[name]['pose'] = pose


class FakeLocalization(LocalizationController):
    def __init__(self):
        self.calls = []
        self.alive = False
        self.fail_next = False
        self.widened = []
        self.widen_error = None
        self.switch_error = None

    def start_mapping(self):
        if self.fail_next:
            self.fail_next = False
            raise RuntimeError('boom')
        self.calls.append(('mapping',))
        self.alive = True

    def start_localization(self, map_yaml, initial_pose):
        self.calls.append(('localization', map_yaml, initial_pose))
        self.alive = True

    def switch_map(self, map_yaml, initial_pose):
        if self.switch_error:
            raise RuntimeError(self.switch_error)
        self.calls.append(('switch', map_yaml, initial_pose))

    def stop(self):
        self.calls.append(('stop',))
        self.alive = False

    def widen_initial_estimate(self, pose, sigma_xy, sigma_yaw):
        if self.widen_error:
            raise RuntimeError(self.widen_error)
        self.widened.append((pose, sigma_xy, sigma_yaw))

    def running(self):
        return self.alive


class FakeSaver(MapSaver):
    def __init__(self):
        self.saved = []
        self.fail = False

    def save(self, map_url):
        if self.fail:
            raise RuntimeError('map_saver timed out')
        self.saved.append(map_url)


class FakeRobot(RobotPoseSource):
    def __init__(self, pose=None):
        self.pose = pose

    def current_pose(self):
        return self.pose


class RecordingObserver(IndoorNavObserver):
    def __init__(self):
        self.states = []
        self.places = []
        self.maps = []

    def on_state(self, state):
        self.states.append(state)

    def on_places(self, map_name, places):
        self.places.append((map_name, places))

    def on_maps(self, maps, active_map):
        self.maps.append(([m.name for m in maps], active_map))
