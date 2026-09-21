# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.

"""Ports the application layer depends on; infrastructure implements them."""

from abc import ABC, abstractmethod
from typing import List, Optional

from .model import MapRecord, Place, Pose2D


class MapRepository(ABC):
    """Maps stored on the rover, their places, and which one is active."""

    @abstractmethod
    def list_maps(self) -> List[MapRecord]: ...

    @abstractmethod
    def exists(self, name: str) -> bool: ...

    @abstractmethod
    def map_yaml_path(self, name: str) -> str:
        """Where the map's yaml lives (used by map_server)."""

    @abstractmethod
    def map_url(self, name: str) -> str:
        """Path without extension that nav2's map_saver writes <url>.yaml/.pgm to."""

    @abstractmethod
    def prepare(self, name: str) -> None:
        """Create the map's directory before map_saver writes into it."""

    @abstractmethod
    def delete(self, name: str) -> None: ...

    @abstractmethod
    def load_places(self, name: str) -> List[Place]: ...

    @abstractmethod
    def save_places(self, name: str, places: List[Place]) -> None: ...

    @abstractmethod
    def active_map(self) -> str:
        """Last map localized on, '' if none."""

    @abstractmethod
    def set_active_map(self, name: str) -> None: ...

    @abstractmethod
    def last_pose(self, name: str) -> Optional[Pose2D]:
        """Where the rover was last seen on this map, to seed AMCL after a restart."""

    @abstractmethod
    def save_last_pose(self, name: str, pose: Pose2D) -> None: ...


class LocalizationController(ABC):
    """Starts and stops the localization sub-stack (slam_toolbox, or map_server + AMCL).

    Calls block until the switch is done; the application runs them off the ROS executor.
    """

    @abstractmethod
    def start_mapping(self) -> None: ...

    @abstractmethod
    def start_localization(self, map_yaml: str, initial_pose: Pose2D) -> None: ...

    @abstractmethod
    def stop(self) -> None: ...

    @abstractmethod
    def widen_initial_estimate(self, pose: Pose2D, sigma_xy: float, sigma_yaw: float) -> None:
        """Re-seed the running AMCL at `pose` with a wider spread (standard deviations).

        Used when the seed is only a remembered pose, so AMCL can still converge if the rover
        was moved a little while it was off. Blocks until AMCL is up or gives up.
        """

    @abstractmethod
    def running(self) -> bool:
        """False once the sub-stack exited on its own (crash)."""


class MapSaver(ABC):
    """Writes the map currently being built (nav2 map_saver)."""

    @abstractmethod
    def save(self, map_url: str) -> None:
        """Raise on failure."""


class RobotPoseSource(ABC):
    @abstractmethod
    def current_pose(self) -> Optional[Pose2D]:
        """The rover in the map frame, or None when not localized."""


class IndoorNavObserver(ABC):
    """Outbound: state changes the rest of the system should see (latched ROS topics)."""

    @abstractmethod
    def on_state(self, state) -> None: ...

    @abstractmethod
    def on_places(self, map_name: str, places: List[Place]) -> None: ...

    @abstractmethod
    def on_maps(self, maps: List[MapRecord], active_map: str) -> None: ...
