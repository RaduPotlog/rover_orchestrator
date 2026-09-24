# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.

"""RosInitialPoseSeeder waits for AMCL in the background and never seeds a stack it outlived."""

import threading
import time

from rover_indoor_nav_manager.domain.model import Pose2D
from rover_indoor_nav_manager.infrastructure.ros_adapters import RosInitialPoseSeeder


class FakePublisher:
    def __init__(self):
        self.subscribers = 0
        self.published = []
        self.polls = 0      # how often the seeder has checked for AMCL

    def get_subscription_count(self):
        self.polls += 1
        return self.subscribers

    def publish(self, msg):
        self.published.append(msg)


class FakeNode:
    def __init__(self):
        self.pub = FakePublisher()

    def create_publisher(self, *_args, **_kwargs):
        return self.pub

    def get_logger(self):
        class Log:
            def info(self, *_):
                pass

            def warning(self, *_):
                pass
        return Log()

    def get_clock(self):
        class Clock:
            def now(self):
                class T:
                    def to_msg(self):
                        return None
                return T()
        return Clock()


class FakePose:
    def __init__(self):
        self.pose = None

    def current_pose(self):
        return self.pose


def wait_for(cond, timeout=3.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.02)
    return False


def seeder_threads_exit(started_before, timeout=3.0):
    """Wait until every seeder thread started since `started_before` has finished.

    Once they have, nothing more can be published, so a "still nothing" assertion after
    this holds for good rather than just for however long a sleep happened to last.
    """
    def alive():
        return [t for t in threading.enumerate()
                if t.name == 'amcl_seed' and t not in started_before and t.is_alive()]
    return wait_for(lambda: not alive(), timeout)


def test_seed_returns_at_once_and_publishes_when_amcl_is_up():
    node, pose = FakeNode(), FakePose()
    seeder = RosInitialPoseSeeder(node, 'rover/map', pose, timeout=5.0)
    t0 = time.monotonic()
    seeder.seed(Pose2D(1.0, 2.0, 0.5), 1.5, 0.785)
    assert time.monotonic() - t0 < 0.1          # never blocks the manager's worker
    assert wait_for(lambda: node.pub.polls >= 2)  # it has looked, more than once...
    assert node.pub.published == []            # ...and AMCL is not up yet
    node.pub.subscribers = 1
    pose.pose = Pose2D(0.0, 0.0, 0.0)            # map -> base_link resolves
    assert wait_for(lambda: len(node.pub.published) == 1)
    msg = node.pub.published[0]
    assert (msg.pose.pose.position.x, msg.pose.pose.position.y) == (1.0, 2.0)
    assert abs(msg.pose.covariance[0] - 2.25) < 1e-9


def test_cancel_and_a_newer_seed_drop_the_pending_one():
    node, pose = FakeNode(), FakePose()
    seeder = RosInitialPoseSeeder(node, 'rover/map', pose, timeout=5.0)
    before = set(threading.enumerate())
    seeder.seed(Pose2D(1.0, 1.0, 0.0), 1.5, 0.785)
    seeder.cancel()                            # localization stopped before AMCL came up
    seeder.seed(Pose2D(9.0, 9.0, 0.0), 1.5, 0.785)
    node.pub.subscribers = 1
    pose.pose = Pose2D(0.0, 0.0, 0.0)
    assert wait_for(lambda: len(node.pub.published) >= 1)
    assert seeder_threads_exit(before)         # the cancelled seed can no longer publish
    assert [m.pose.pose.position.x for m in node.pub.published] == [9.0]


def test_gives_up_quietly_after_the_timeout():
    node, pose = FakeNode(), FakePose()
    seeder = RosInitialPoseSeeder(node, 'rover/map', pose, timeout=0.3)
    before = set(threading.enumerate())
    seeder.seed(Pose2D(1.0, 1.0, 0.0), 1.5, 0.785)
    assert seeder_threads_exit(before)         # gave up after its 0.3 s
    node.pub.subscribers = 1                   # AMCL comes up too late
    pose.pose = Pose2D(0.0, 0.0, 0.0)
    assert node.pub.published == []
