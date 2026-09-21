# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.

"""ROS 2 adapters: map_saver client, TF pose source, latched state publishers."""

import math
import threading

from builtin_interfaces.msg import Time
from nav2_msgs.srv import SaveMap
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time as RclpyTime
from rover_msgs.msg import LocalizationState as LocalizationStateMsg
from rover_msgs.msg import MapInfo, MapList, Place as PlaceMsg, PlaceList
import tf2_ros

from ..domain.model import LocalizationMode, LocalizationState, Pose2D
from ..domain.ports import IndoorNavObserver, MapSaver, RobotPoseSource

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def place_to_msg(place) -> PlaceMsg:
    return PlaceMsg(id=place.id, name=place.name, map_name=place.map_name,
                    x=place.pose.x, y=place.pose.y, theta=place.pose.theta)


class RosMapSaver(MapSaver):
    """Calls nav2 map_saver's save_map; runs on the worker thread, never the executor."""

    def __init__(self, node: Node, service: str, timeout: float, callback_group=None):
        self._client = node.create_client(SaveMap, service, callback_group=callback_group)
        self._timeout = timeout
        self._service = service

    def save(self, map_url: str) -> None:
        if not self._client.wait_for_service(timeout_sec=self._timeout):
            raise RuntimeError(f'{self._service} is not available (is SLAM running?)')
        request = SaveMap.Request(map_topic='map', map_url=map_url, image_format='pgm',
                                  map_mode='trinary', free_thresh=0.25, occupied_thresh=0.65)
        done = threading.Event()
        future = self._client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        if not done.wait(self._timeout + 5.0):
            raise RuntimeError('map_saver did not answer')
        if not future.result().result:
            raise RuntimeError('map_saver reported failure (no map received yet?)')


class TfPoseSource(RobotPoseSource):

    def __init__(self, node: Node, map_frame: str, base_frame: str):
        self._buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._buffer, node)
        self._map_frame = map_frame
        self._base_frame = base_frame

    def current_pose(self):
        try:
            t = self._buffer.lookup_transform(
                self._map_frame, self._base_frame, RclpyTime(), timeout=Duration(seconds=0.2))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        tr = t.transform.translation
        return Pose2D(tr.x, tr.y, yaw_from_quaternion(t.transform.rotation))


_MODE = {
    LocalizationMode.UNAVAILABLE: LocalizationStateMsg.UNAVAILABLE,
    LocalizationMode.MAPPING: LocalizationStateMsg.MAPPING,
    LocalizationMode.LOCALIZATION: LocalizationStateMsg.LOCALIZATION,
    LocalizationMode.SWITCHING: LocalizationStateMsg.SWITCHING,
}


class RosObserver(IndoorNavObserver):
    """Latched topics so a browser that connects later still gets the current state."""

    def __init__(self, node: Node):
        self._node = node
        self._state_pub = node.create_publisher(LocalizationStateMsg, 'localization_state', LATCHED)
        self._places_pub = node.create_publisher(PlaceList, 'places', LATCHED)
        self._maps_pub = node.create_publisher(MapList, 'maps', LATCHED)

    def _stamp(self):
        return self._node.get_clock().now().to_msg()

    def _live(self) -> bool:
        # On SIGINT rclpy invalidates the context before main() stops the localization
        # stack; the final "Stopped." state then has nowhere to go.
        return self._node.context.ok()

    def on_state(self, state: LocalizationState) -> None:
        self._node.get_logger().info(
            f'Localization: {state.mode.name} map={state.map_name or "-"} {state.message}')
        if not self._live():
            return
        msg = LocalizationStateMsg(mode=_MODE[state.mode], map_name=state.map_name,
                                   message=state.message)
        msg.header.stamp = self._stamp()
        self._state_pub.publish(msg)

    def on_places(self, map_name, places) -> None:
        if not self._live():
            return
        msg = PlaceList(map_name=map_name, places=[place_to_msg(p) for p in places])
        msg.header.stamp = self._stamp()
        self._places_pub.publish(msg)

    def on_maps(self, maps, active_map) -> None:
        if not self._live():
            return
        infos = []
        for m in maps:
            seconds = int(m.saved_unix)
            infos.append(MapInfo(name=m.name, resolution=m.resolution, width=m.width,
                                 height=m.height,
                                 saved=Time(sec=seconds,
                                            nanosec=int((m.saved_unix - seconds) * 1e9))))
        msg = MapList(maps=infos, active_map=active_map)
        msg.header.stamp = self._stamp()
        self._maps_pub.publish(msg)
