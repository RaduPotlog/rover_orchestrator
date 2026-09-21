# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.

import os

import pytest

from rover_indoor_nav_manager.domain.model import DomainError, Place, Pose2D
from rover_indoor_nav_manager.infrastructure.file_map_repository import (
    FileMapRepository,
    read_pgm_size,
)


def write_map(root, name, width=40, height=30):
    directory = os.path.join(root, name)
    os.makedirs(directory, exist_ok=True)
    with open(os.path.join(directory, 'map.yaml'), 'w') as handle:
        handle.write('image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\nnegate: 0\n'
                     'occupied_thresh: 0.65\nfree_thresh: 0.25\n')
    with open(os.path.join(directory, 'map.pgm'), 'wb') as handle:
        handle.write(b'P5\n# CREATOR: map_saver\n%d %d\n255\n' % (width, height))
        handle.write(bytes(width * height))


def test_lists_only_directories_with_a_map(tmp_path):
    root = str(tmp_path)
    write_map(root, 'lab', 40, 30)
    os.makedirs(os.path.join(root, 'empty_dir'))
    repo = FileMapRepository(root)
    maps = repo.list_maps()
    assert [m.name for m in maps] == ['lab']
    assert (maps[0].width, maps[0].height, maps[0].resolution) == (40, 30, 0.05)
    assert maps[0].saved_unix > 0
    assert repo.exists('lab') and not repo.exists('empty_dir')
    assert repo.map_yaml_path('lab') == os.path.join(root, 'lab', 'map.yaml')
    assert repo.map_url('lab') == os.path.join(root, 'lab', 'map')


def test_rejects_path_traversal(tmp_path):
    repo = FileMapRepository(str(tmp_path))
    with pytest.raises(DomainError):
        repo.map_yaml_path('../etc')


def test_places_round_trip_and_skip_bad_entries(tmp_path):
    root = str(tmp_path)
    write_map(root, 'lab')
    repo = FileMapRepository(root)
    dock = Place.create('Dock', 'lab', Pose2D(1.5, -2.0, 0.3), 'abc')
    repo.save_places('lab', [dock])
    assert repo.load_places('lab') == [dock]
    with open(os.path.join(root, 'lab', 'places.yaml'), 'a') as handle:
        handle.write('- {id: broken}\n')  # appended to the list
    assert repo.load_places('lab') == [dock]


def test_active_map_and_last_pose(tmp_path):
    root = str(tmp_path)
    write_map(root, 'lab')
    repo = FileMapRepository(root)
    assert repo.active_map() == ''
    repo.set_active_map('lab')
    assert repo.active_map() == 'lab'
    repo.set_active_map('')
    assert repo.active_map() == ''
    assert repo.last_pose('lab') is None
    repo.save_last_pose('lab', Pose2D(1, 2, 0.5))
    assert repo.last_pose('lab') == Pose2D(1, 2, 0.5)
    repo.save_last_pose('gone', Pose2D(0, 0))  # no directory: ignored


def test_delete_removes_everything(tmp_path):
    root = str(tmp_path)
    write_map(root, 'lab')
    repo = FileMapRepository(root)
    repo.delete('lab')
    assert not os.path.exists(os.path.join(root, 'lab'))


def test_pgm_header_parsing(tmp_path):
    path = tmp_path / 'x.pgm'
    path.write_bytes(b'P5 # comment\n12\n# more\n7 255\n' + bytes(84))
    assert read_pgm_size(str(path)) == (12, 7)
    path.write_bytes(b'P6\n1 1\n255\n')
    assert read_pgm_size(str(path)) == (0, 0)
    assert read_pgm_size(str(tmp_path / 'missing.pgm')) == (0, 0)
