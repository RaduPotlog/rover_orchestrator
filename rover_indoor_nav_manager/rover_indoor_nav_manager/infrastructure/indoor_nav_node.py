# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.

"""The ROS node: services for the drive UI, timers, and the worker that runs switches."""

from concurrent.futures import ThreadPoolExecutor, TimeoutError as FutureTimeout

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from rover_msgs.srv import DeleteMap, DeletePlace, LoadMap, SaveMap, SavePlace
from std_srvs.srv import Trigger

from ..application.indoor_nav_service import IndoorNavService
from ..domain.model import DomainError, Pose2D
from .file_map_repository import FileMapRepository
from .launch_localization_controller import LaunchLocalizationController
from .ros_adapters import place_to_msg, RosMapSaver, RosObserver, TfPoseSource

# How long a service call waits for a switch before answering "in progress". The drive UI
# follows localization_state for the outcome, so it does not need to block for seconds.
SWITCH_REPLY_WAIT = 2.0


def ns_frame(namespace: str, frame: str) -> str:
    prefix = namespace.strip('/')
    return f'{prefix}/{frame}' if prefix else frame


class IndoorNavNode(Node):

    def __init__(self):
        super().__init__('indoor_nav_manager')
        self.declare_parameter('maps_dir', '/maps')
        self.declare_parameter('localization_params_file', '')
        self.declare_parameter('log_level', 'info')
        self.declare_parameter('launch_package', 'rover_navigation')
        self.declare_parameter('launch_file', 'indoor_localization.launch.py')
        self.declare_parameter('save_map_timeout', 5.0)
        self.declare_parameter('pose_record_period', 5.0)
        self.declare_parameter('auto_start', True)

        params_file = self.get_parameter('localization_params_file').value
        if not params_file:
            raise RuntimeError('localization_params_file is required (bringup.launch.py '
                               'passes its namespaced rover_nav_params.yaml)')

        namespace = self.get_namespace()
        use_sim_time = bool(self.get_parameter('use_sim_time').value)
        self._services_group = MutuallyExclusiveCallbackGroup()
        io_group = ReentrantCallbackGroup()

        self._worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix='indoor_nav')
        self._controller = LaunchLocalizationController(
            self.get_logger(), namespace.strip('/'), params_file, use_sim_time,
            log_level=self.get_parameter('log_level').value,
            launch_package=self.get_parameter('launch_package').value,
            launch_file=self.get_parameter('launch_file').value)
        self.service = IndoorNavService(
            maps=FileMapRepository(self.get_parameter('maps_dir').value),
            localization=self._controller,
            saver=RosMapSaver(self, 'map_saver/save_map',
                              self.get_parameter('save_map_timeout').value, io_group),
            robot=TfPoseSource(self, ns_frame(namespace, 'map'),
                               ns_frame(namespace, 'base_link')),
            observer=RosObserver(self))

        g = self._services_group
        self.create_service(Trigger, 'start_mapping', self._start_mapping, callback_group=g)
        self.create_service(SaveMap, 'save_map', self._save_map, callback_group=g)
        self.create_service(LoadMap, 'load_map', self._load_map, callback_group=g)
        self.create_service(DeleteMap, 'delete_map', self._delete_map, callback_group=g)
        self.create_service(SavePlace, 'save_place', self._save_place, callback_group=g)
        self.create_service(DeletePlace, 'delete_place', self._delete_place, callback_group=g)

        self.create_timer(1.0, lambda: self._worker.submit(self.service.check_health),
                          callback_group=io_group)
        self.create_timer(self.get_parameter('pose_record_period').value,
                          lambda: self._worker.submit(self.service.record_pose),
                          callback_group=io_group)

        if self.get_parameter('auto_start').value:
            self._worker.submit(self._startup)

    def _startup(self):
        try:
            self.service.startup()
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f'Startup failed: {error}')

    def shutdown(self):
        self._worker.submit(self.service.shutdown).result(timeout=40.0)
        self._worker.shutdown(wait=True)

    # --- service plumbing ----------------------------------------------------------------
    def _run(self, response, fn, wait=None, in_progress='Switching - follow localization_state.'):
        future = self._worker.submit(fn)
        try:
            result = future.result(timeout=wait)
        except FutureTimeout:
            response.success = True
            response.message = in_progress
            return response, None
        except DomainError as error:
            response.success = False
            response.message = str(error)
            return response, None
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f'{fn}: {error}')
            response.success = False
            response.message = str(error)
            return response, None
        response.success = True
        return response, result

    def _start_mapping(self, _request, response):
        response, _ = self._run(response, self.service.start_mapping, SWITCH_REPLY_WAIT)
        response.message = response.message or 'Mapping started.'
        return response

    def _save_map(self, request, response):
        timeout = self.get_parameter('save_map_timeout').value + 10.0
        response, _ = self._run(response, lambda: self.service.save_map(request.name), timeout)
        response.message = response.message or f"Map '{request.name}' saved."
        return response

    def _load_map(self, request, response):
        pose = None
        if request.set_initial_pose:
            try:
                pose = Pose2D(request.x, request.y, request.theta)
            except DomainError as error:
                response.success, response.message = False, str(error)
                return response
        response, _ = self._run(
            response, lambda: self.service.load_map(request.name, pose), SWITCH_REPLY_WAIT)
        response.message = response.message or f"Localizing on '{request.name}'."
        return response

    def _delete_map(self, request, response):
        response, _ = self._run(response, lambda: self.service.delete_map(request.name), 5.0)
        response.message = response.message or f"Map '{request.name}' deleted."
        return response

    def _save_place(self, request, response):
        p = request.place

        def save():
            return self.service.save_place(p.name, Pose2D(p.x, p.y, p.theta), p.id)

        response, place = self._run(response, save, 5.0)
        if place is not None:
            response.place = place_to_msg(place)
            response.message = f"Saved '{place.name}'."
        return response

    def _delete_place(self, request, response):
        response, place = self._run(response, lambda: self.service.delete_place(request.id), 5.0)
        if place is not None:
            response.message = f"Deleted '{place.name}'."
        return response
