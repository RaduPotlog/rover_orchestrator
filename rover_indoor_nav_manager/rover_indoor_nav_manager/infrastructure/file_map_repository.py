# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.

"""Maps on disk, one directory per map under the maps volume (/maps in the container):

    /maps/<name>/map.yaml, map.pgm   written by nav2 map_saver, read by map_server
    /maps/<name>/places.yaml         the map's places
    /maps/<name>/last_pose.yaml      where the rover was last localized
    /maps/active                     name of the map to resume on after a restart
"""

import os
import shutil
import tempfile
from typing import List, Optional

import yaml

from ..domain.model import MapRecord, Place, Pose2D, validate_map_name

MAP_BASENAME = 'map'


def _atomic_write(path: str, text: str) -> None:
    directory = os.path.dirname(path)
    fd, tmp = tempfile.mkstemp(dir=directory, prefix='.tmp-')
    try:
        with os.fdopen(fd, 'w') as handle:
            handle.write(text)
        os.replace(tmp, path)
    except BaseException:
        if os.path.exists(tmp):
            os.unlink(tmp)
        raise


def read_pgm_size(path: str):
    """Width and height from a binary/ASCII PGM header, or (0, 0)."""
    try:
        with open(path, 'rb') as handle:
            tokens = []
            while len(tokens) < 3:
                line = handle.readline()
                if not line:
                    return 0, 0
                line = line.split(b'#', 1)[0]
                tokens += line.split()
            if tokens[0] not in (b'P5', b'P2'):
                return 0, 0
            return int(tokens[1]), int(tokens[2])
    except (OSError, ValueError, IndexError):
        return 0, 0


class FileMapRepository:

    def __init__(self, root: str):
        self._root = root
        os.makedirs(root, exist_ok=True)

    def _dir(self, name: str) -> str:
        return os.path.join(self._root, validate_map_name(name))

    def list_maps(self) -> List[MapRecord]:
        records = []
        for name in sorted(os.listdir(self._root)):
            directory = os.path.join(self._root, name)
            yaml_path = os.path.join(directory, f'{MAP_BASENAME}.yaml')
            if not os.path.isdir(directory) or not os.path.isfile(yaml_path):
                continue
            try:
                with open(yaml_path) as handle:
                    meta = yaml.safe_load(handle) or {}
                image = os.path.join(directory, str(meta.get('image', f'{MAP_BASENAME}.pgm')))
                width, height = read_pgm_size(image)
                records.append(MapRecord(
                    name=name,
                    resolution=float(meta.get('resolution', 0.0)),
                    width=width,
                    height=height,
                    saved_unix=os.path.getmtime(yaml_path)))
            except (OSError, yaml.YAMLError, ValueError):
                continue
        return records

    def exists(self, name: str) -> bool:
        return os.path.isfile(self.map_yaml_path(name))

    def map_yaml_path(self, name: str) -> str:
        return os.path.join(self._dir(name), f'{MAP_BASENAME}.yaml')

    def map_url(self, name: str) -> str:
        return os.path.join(self._dir(name), MAP_BASENAME)

    def prepare(self, name: str) -> None:
        os.makedirs(self._dir(name), exist_ok=True)

    def delete(self, name: str) -> None:
        shutil.rmtree(self._dir(name), ignore_errors=True)

    def load_places(self, name: str) -> List[Place]:
        path = os.path.join(self._dir(name), 'places.yaml')
        if not os.path.isfile(path):
            return []
        with open(path) as handle:
            data = yaml.safe_load(handle) or {}
        places = []
        for entry in data.get('places', []):
            try:
                places.append(Place(
                    id=str(entry['id']), name=str(entry['name']), map_name=name,
                    pose=Pose2D(float(entry['x']), float(entry['y']), float(entry['theta']))))
            except (KeyError, TypeError, ValueError):
                continue  # a hand-edited bad entry must not hide the others
        return places

    def save_places(self, name: str, places: List[Place]) -> None:
        data = {'places': [
            {'id': p.id, 'name': p.name, 'x': p.pose.x, 'y': p.pose.y, 'theta': p.pose.theta}
            for p in places]}
        _atomic_write(os.path.join(self._dir(name), 'places.yaml'),
                      yaml.safe_dump(data, sort_keys=False))

    def active_map(self) -> str:
        try:
            with open(os.path.join(self._root, 'active')) as handle:
                return handle.read().strip()
        except OSError:
            return ''

    def set_active_map(self, name: str) -> None:
        if name:
            validate_map_name(name)
        _atomic_write(os.path.join(self._root, 'active'), f'{name}\n' if name else '')

    def last_pose(self, name: str) -> Optional[Pose2D]:
        path = os.path.join(self._dir(name), 'last_pose.yaml')
        try:
            with open(path) as handle:
                data = yaml.safe_load(handle) or {}
            return Pose2D(float(data['x']), float(data['y']), float(data['theta']))
        except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError):
            return None

    def save_last_pose(self, name: str, pose: Pose2D) -> None:
        if not os.path.isdir(self._dir(name)):
            return
        _atomic_write(os.path.join(self._dir(name), 'last_pose.yaml'),
                      yaml.safe_dump({'x': pose.x, 'y': pose.y, 'theta': pose.theta}))
