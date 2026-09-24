# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.

"""IndoorNavNode over a real rclpy graph: its services, and its latched topics' QoS.

auto_start is off, so no SLAM/AMCL launch is spawned; everything here goes through the
node's services, the worker thread and the file repository, as the drive UI would.
"""

import os
import threading

import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rover_msgs.msg import MapList, Place as PlaceMsg
from rover_msgs.srv import DeleteMap, SavePlace

from rover_indoor_nav_manager.infrastructure.indoor_nav_node import IndoorNavNode

TIMEOUT = 10.0
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


def write_map(root, name):
    directory = os.path.join(root, name)
    os.makedirs(directory, exist_ok=True)
    with open(os.path.join(directory, 'map.yaml'), 'w') as handle:
        handle.write('image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\nnegate: 0\n'
                     'occupied_thresh: 0.65\nfree_thresh: 0.25\n')
    with open(os.path.join(directory, 'map.pgm'), 'wb') as handle:
        handle.write(b'P5\n4 3\n255\n' + bytes(12))


@pytest.fixture
def graph(tmp_path):
    maps_dir = str(tmp_path)
    write_map(maps_dir, 'lab')
    write_map(maps_dir, 'garage')

    rclpy.init(args=[
        '--ros-args',
        '-p', f'maps_dir:={maps_dir}',
        '-p', 'auto_start:=false',
        '-p', 'localization_params_file:=/dev/null',
    ])
    node = IndoorNavNode()
    client = rclpy.create_node('indoor_nav_test_client')
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.add_node(client)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()

    yield maps_dir, node, client

    node.shutdown()
    executor.shutdown()
    spinner.join(timeout=TIMEOUT)
    client.destroy_node()
    node.destroy_node()
    rclpy.try_shutdown()


def call(client_node, srv_type, name, request):
    cli = client_node.create_client(srv_type, name)
    assert cli.wait_for_service(timeout_sec=TIMEOUT), name
    done = threading.Event()
    future = cli.call_async(request)
    future.add_done_callback(lambda _: done.set())
    assert done.wait(TIMEOUT), f'{name} did not answer'
    return future.result()


def latest_map_list(client_node):
    """Subscribe the way the drive UI does, late, and return the first MapList received."""
    received = []
    got_one = threading.Event()

    def on_maps(msg):
        received.append(msg)
        got_one.set()

    sub = client_node.create_subscription(MapList, 'maps', on_maps, LATCHED)
    try:
        assert got_one.wait(TIMEOUT), 'no MapList on maps'
    finally:
        client_node.destroy_subscription(sub)
    return received[0]


def test_delete_map_removes_it_and_republishes_the_latched_list(graph):
    maps_dir, _node, client = graph

    response = call(client, DeleteMap, 'delete_map', DeleteMap.Request(name='lab'))

    assert response.success, response.message
    assert not os.path.exists(os.path.join(maps_dir, 'lab'))
    # Subscribed only after the publish: transient_local is what lets a browser that
    # connects later still see the current list.
    assert [m.name for m in latest_map_list(client).maps] == ['garage']


def test_domain_errors_come_back_as_failed_responses(graph):
    _maps_dir, _node, client = graph

    missing = call(client, DeleteMap, 'delete_map', DeleteMap.Request(name='nowhere'))
    assert not missing.success
    assert 'nowhere' in missing.message

    place = call(client, SavePlace, 'save_place',
                 SavePlace.Request(place=PlaceMsg(name='dock', x=1.0, y=2.0, theta=0.0)))
    assert not place.success
    assert 'load one first' in place.message
