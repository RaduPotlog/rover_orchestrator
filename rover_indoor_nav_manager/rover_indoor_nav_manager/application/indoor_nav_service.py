# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.

"""Use cases of the indoor navigation manager.

One class, because every use case reads or changes the same small state: which
localization mode runs, on which map, and that map's places. Calls that switch the
localization sub-stack block for seconds; the caller runs them off the ROS executor.
"""

import threading
from typing import List, Optional

from ..domain.model import (
    DomainError,
    LocalizationMode,
    LocalizationState,
    MapRecord,
    Place,
    PlaceBook,
    Pose2D,
    validate_map_name,
)
from ..domain.ports import (
    IndoorNavObserver,
    LocalizationController,
    MapRepository,
    MapSaver,
    RobotPoseSource,
)

ORIGIN = Pose2D(0.0, 0.0, 0.0)


class IndoorNavService:

    def __init__(self, maps: MapRepository, localization: LocalizationController,
                 saver: MapSaver, robot: RobotPoseSource, observer: IndoorNavObserver):
        self._maps = maps
        self._localization = localization
        self._saver = saver
        self._robot = robot
        self._observer = observer
        self._lock = threading.RLock()
        self._state = LocalizationState()
        self._book = PlaceBook(map_name='')
        # Name the running SLAM session was last saved under: loading that map right after
        # saving it seeds AMCL with the pose SLAM had, so the rover stays localized.
        self._saved_from_mapping = ''

    # --- queries ---------------------------------------------------------------------
    @property
    def state(self) -> LocalizationState:
        return self._state

    def places(self) -> List[Place]:
        return list(self._book.places)

    def maps(self) -> List[MapRecord]:
        return self._maps.list_maps()

    # --- lifecycle -------------------------------------------------------------------
    def startup(self) -> None:
        """Resume on the last active map, or start mapping when there is none."""
        self._publish_maps()
        active = self._maps.active_map()
        if active and self._maps.exists(active):
            self.load_map(active)
        else:
            self.start_mapping()

    def shutdown(self) -> None:
        with self._lock:
            self._localization.stop()
            self._set_state(LocalizationState(message='Stopped.'))

    def check_health(self) -> None:
        """Report a sub-stack that died on its own; it is not restarted automatically."""
        with self._lock:
            if self._state.mode in (LocalizationMode.MAPPING, LocalizationMode.LOCALIZATION) \
                    and not self._localization.running():
                self._set_state(self._state.with_(
                    mode=LocalizationMode.UNAVAILABLE,
                    message='The localization stack exited unexpectedly. Load a map or start '
                            'mapping again.'))

    # --- mapping / localization ------------------------------------------------------
    def start_mapping(self) -> None:
        with self._lock:
            self._switching('Starting SLAM...')
            try:
                self._localization.stop()
                self._localization.start_mapping()
            except Exception as error:  # noqa: BLE001 - reported to the operator
                self._set_state(LocalizationState(message=f'Could not start SLAM: {error}'))
                raise
            self._saved_from_mapping = ''
            self._set_places(PlaceBook(map_name=''))
            self._set_state(LocalizationState(
                mode=LocalizationMode.MAPPING, message='Drive the rover around to build the map.'))

    def save_map(self, name: str) -> None:
        with self._lock:
            validate_map_name(name)
            if self._state.mode != LocalizationMode.MAPPING:
                raise DomainError('Start mapping before saving a map.')
            if self._maps.exists(name):
                raise DomainError(f"A map named '{name}' already exists; delete it first or "
                                  'pick another name.')
            self._maps.prepare(name)
            try:
                self._saver.save(self._maps.map_url(name))
            except Exception as error:  # noqa: BLE001
                self._maps.delete(name)
                raise DomainError(f'Saving the map failed: {error}') from error
            pose = self._robot.current_pose()
            if pose is not None:
                self._maps.save_last_pose(name, pose)
            self._saved_from_mapping = name
            self._set_state(self._state.with_(map_name=name, message=f"Saved as '{name}'."))
            self._publish_maps()

    def load_map(self, name: str, initial_pose: Optional[Pose2D] = None) -> None:
        with self._lock:
            validate_map_name(name)
            if not self._maps.exists(name):
                raise DomainError(f"No map named '{name}'.")
            pose = initial_pose or self._seed_pose(name)
            self._switching(f"Loading map '{name}'...")
            try:
                self._localization.stop()
                self._localization.start_localization(self._maps.map_yaml_path(name), pose)
            except Exception as error:  # noqa: BLE001
                self._set_state(LocalizationState(message=f'Could not start AMCL: {error}'))
                raise
            self._maps.set_active_map(name)
            self._saved_from_mapping = ''
            self._set_places(PlaceBook(map_name=name, places=self._maps.load_places(name)))
            self._set_state(LocalizationState(
                mode=LocalizationMode.LOCALIZATION, map_name=name,
                message='Localized on the saved map. Use Set pose if the rover is elsewhere.'))
            self._publish_maps()

    def delete_map(self, name: str) -> None:
        with self._lock:
            validate_map_name(name)
            if not self._maps.exists(name):
                raise DomainError(f"No map named '{name}'.")
            if self._state.mode == LocalizationMode.LOCALIZATION and self._state.map_name == name:
                raise DomainError(f"'{name}' is the map in use; load another map or start "
                                  'mapping first.')
            self._maps.delete(name)
            if self._maps.active_map() == name:
                self._maps.set_active_map('')
            self._publish_maps()

    def record_pose(self) -> None:
        """Remember where the rover is, so AMCL starts there after a restart."""
        with self._lock:
            if self._state.mode != LocalizationMode.LOCALIZATION:
                return
            pose = self._robot.current_pose()
            if pose is not None:
                self._maps.save_last_pose(self._state.map_name, pose)

    # --- places ----------------------------------------------------------------------
    def save_place(self, name: str, pose: Pose2D, place_id: str = '') -> Place:
        with self._lock:
            if self._state.mode != LocalizationMode.LOCALIZATION:
                raise DomainError('Places belong to a saved map: load one first.')
            if place_id and self._book.get(place_id) is None:
                raise DomainError(f"No place with id '{place_id}'.")
            place = self._book.upsert(Place.create(name, self._state.map_name, pose, place_id))
            self._maps.save_places(self._book.map_name, self._book.places)
            self._observer.on_places(self._book.map_name, self.places())
            return place

    def delete_place(self, place_id: str) -> Place:
        with self._lock:
            place = self._book.remove(place_id)
            self._maps.save_places(self._book.map_name, self._book.places)
            self._observer.on_places(self._book.map_name, self.places())
            return place

    # --- helpers ---------------------------------------------------------------------
    def _seed_pose(self, name: str) -> Pose2D:
        if self._state.mode == LocalizationMode.MAPPING and self._saved_from_mapping == name:
            current = self._robot.current_pose()
            if current is not None:
                return current
        return self._maps.last_pose(name) or ORIGIN

    def _switching(self, message: str) -> None:
        self._set_state(self._state.with_(mode=LocalizationMode.SWITCHING, message=message))

    def _set_state(self, state: LocalizationState) -> None:
        self._state = state
        self._observer.on_state(state)

    def _set_places(self, book: PlaceBook) -> None:
        self._book = book
        self._observer.on_places(book.map_name, self.places())

    def _publish_maps(self) -> None:
        self._observer.on_maps(self._maps.list_maps(), self._maps.active_map())
