# Copyright 2026 Mechatronics Academy
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math

import pytest

from rover_indoor_nav_manager.domain.model import (
    DomainError,
    Place,
    PlaceBook,
    Pose2D,
    validate_map_name,
)


def test_map_names_are_directory_safe():
    assert validate_map_name('warehouse_1-b') == 'warehouse_1-b'
    for bad in ('', '../etc', 'a b', 'x' * 65, 'map.yaml'):
        with pytest.raises(DomainError):
            validate_map_name(bad)


def test_pose_normalizes_theta_and_rejects_nan():
    assert Pose2D(0, 0, 3 * math.pi).theta == pytest.approx(math.pi)
    with pytest.raises(DomainError):
        Pose2D(float('nan'), 0, 0)


def test_place_create_trims_and_assigns_id():
    place = Place.create('  Dock  ', 'lab', Pose2D(1, 2, 0))
    assert place.name == 'Dock'
    assert len(place.id) == 12
    with pytest.raises(DomainError):
        Place.create('   ', 'lab', Pose2D(0, 0))


def test_place_book_rejects_duplicate_names_and_foreign_maps():
    book = PlaceBook('lab')
    dock = book.upsert(Place.create('Dock', 'lab', Pose2D(0, 0)))
    with pytest.raises(DomainError):
        book.upsert(Place.create('dock', 'lab', Pose2D(1, 1)))
    with pytest.raises(DomainError):
        book.upsert(Place.create('Other', 'hall', Pose2D(1, 1)))
    renamed = book.upsert(Place.create('Charger', 'lab', Pose2D(0, 0), dock.id))
    assert [p.name for p in book.places] == ['Charger']
    assert book.remove(renamed.id) == renamed
    with pytest.raises(DomainError):
        book.remove('missing')
