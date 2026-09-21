# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.

"""Entities and value objects. Pure Python: no rclpy, no *_msgs, no file or process I/O."""

import enum
import math
import re
import uuid
from dataclasses import dataclass, field, replace
from typing import Optional

MAP_NAME_PATTERN = re.compile(r'^[A-Za-z0-9_-]{1,64}$')
MAX_PLACE_NAME = 64


class DomainError(ValueError):
    """A request that breaks a domain rule; the message is shown to the operator."""


def validate_map_name(name: str) -> str:
    """Map names become directory names under the maps dir, so keep them boring."""
    if not MAP_NAME_PATTERN.match(name or ''):
        raise DomainError(
            f"Invalid map name '{name}': use 1-64 letters, digits, '_' or '-'.")
    return name


def normalize_angle(theta: float) -> float:
    return math.atan2(math.sin(theta), math.cos(theta))


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    theta: float = 0.0

    def __post_init__(self):
        for value in (self.x, self.y, self.theta):
            if not math.isfinite(value):
                raise DomainError('A pose must be finite.')
        object.__setattr__(self, 'theta', normalize_angle(self.theta))


@dataclass(frozen=True)
class Place:
    """A named pose on one map (an IndoorNav "endpoint")."""

    id: str
    name: str
    map_name: str
    pose: Pose2D

    @staticmethod
    def create(name: str, map_name: str, pose: Pose2D, place_id: str = '') -> 'Place':
        clean = (name or '').strip()
        if not clean:
            raise DomainError('A place needs a name.')
        if len(clean) > MAX_PLACE_NAME:
            raise DomainError(f'Place names are at most {MAX_PLACE_NAME} characters.')
        validate_map_name(map_name)
        return Place(id=place_id or uuid.uuid4().hex[:12], name=clean, map_name=map_name,
                     pose=pose)


@dataclass(frozen=True)
class MapRecord:
    name: str
    resolution: float = 0.0
    width: int = 0
    height: int = 0
    saved_unix: float = 0.0


class SeedSource(enum.Enum):
    """Where AMCL's starting pose came from - decides how much to trust it."""

    SLAM = 'slam'            # hand-off from the SLAM session just saved: exact
    EXPLICIT = 'explicit'    # the operator gave the pose (LoadMap with set_initial_pose)
    LAST_POSE = 'last_pose'  # last_pose.yaml, e.g. after a reboot: may be stale or moved
    ORIGIN = 'origin'        # nothing known: the map origin


class LocalizationMode(enum.Enum):
    UNAVAILABLE = 0
    MAPPING = 1
    LOCALIZATION = 2
    SWITCHING = 3


@dataclass(frozen=True)
class LocalizationState:
    mode: LocalizationMode = LocalizationMode.UNAVAILABLE
    map_name: str = ''
    message: str = ''

    def with_(self, **changes) -> 'LocalizationState':
        return replace(self, **changes)


@dataclass
class PlaceBook:
    """The places of one map, with the uniqueness rule for names."""

    map_name: str
    places: list = field(default_factory=list)

    def upsert(self, place: Place) -> Place:
        if place.map_name != self.map_name:
            raise DomainError(
                f"Place '{place.name}' belongs to map '{place.map_name}', "
                f"not the active map '{self.map_name}'.")
        for other in self.places:
            if other.id != place.id and other.name.lower() == place.name.lower():
                raise DomainError(f"A place named '{place.name}' already exists.")
        self.places = [p for p in self.places if p.id != place.id] + [place]
        self.places.sort(key=lambda p: p.name.lower())
        return place

    def remove(self, place_id: str) -> Place:
        for place in self.places:
            if place.id == place_id:
                self.places = [p for p in self.places if p.id != place_id]
                return place
        raise DomainError(f"No place with id '{place_id}'.")

    def get(self, place_id: str) -> Optional[Place]:
        return next((p for p in self.places if p.id == place_id), None)
