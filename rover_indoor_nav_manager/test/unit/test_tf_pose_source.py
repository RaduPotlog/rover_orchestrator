# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.

"""TfPoseSource listens to TF only during a lookup, and answers from fresh data.

Needs a ROS graph (rclpy); run it serialized, like the other node-level tests.
"""

import math
import threading
import time

from geometry_msgs.msg import TransformStamped
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import tf2_ros

from rover_indoor_nav_manager.infrastructure.ros_adapters import TfPoseSource

MAP, ODOM, BASE = 'test_ns/map', 'test_ns/odom', 'test_ns/base_link'


def transform(parent, child, x, yaw, stamp):
    t = TransformStamped()
    t.header.frame_id = parent
    t.header.stamp = stamp
    t.child_frame_id = child
    t.transform.translation.x = x
    t.transform.rotation.z = math.sin(yaw / 2.0)
    t.transform.rotation.w = math.cos(yaw / 2.0)
    return t


@pytest.fixture
def graph():
    rclpy.init()
    publisher = Node('tf_publisher')
    manager = Node('pose_source_under_test')
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(publisher)
    executor.add_node(manager)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    yield publisher, manager
    executor.shutdown()
    publisher.destroy_node()
    manager.destroy_node()
    rclpy.try_shutdown()


def tf_subscriptions(node):
    return [s for s in node.subscriptions if s.topic_name in ('/tf', '/tf_static')]


def test_returns_the_pose_and_unsubscribes(graph):
    publisher, manager = graph
    static = tf2_ros.StaticTransformBroadcaster(publisher)
    static.sendTransform(transform(MAP, ODOM, 1.0, 0.0, publisher.get_clock().now().to_msg()))
    dynamic = tf2_ros.TransformBroadcaster(publisher)
    publisher.create_timer(0.02, lambda: dynamic.sendTransform(
        transform(ODOM, BASE, 2.0, math.pi / 2, publisher.get_clock().now().to_msg())))

    source = TfPoseSource(manager, MAP, BASE, window=2.0)
    pose = source.current_pose()

    assert pose is not None
    assert pose.x == pytest.approx(3.0)
    assert pose.theta == pytest.approx(math.pi / 2)
    assert tf_subscriptions(manager) == []


def test_gives_up_after_the_window_and_unsubscribes(graph):
    _, manager = graph
    source = TfPoseSource(manager, MAP, BASE, window=0.3)
    start = time.monotonic()
    assert source.current_pose() is None
    assert time.monotonic() - start < 2.0
    assert tf_subscriptions(manager) == []
