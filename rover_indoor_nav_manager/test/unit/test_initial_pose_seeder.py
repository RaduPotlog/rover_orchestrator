# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.

"""RosInitialPoseSeeder waits for AMCL in the background and never seeds a stack it outlived."""

import time

from rover_indoor_nav_manager.domain.model import Pose2D
from rover_indoor_nav_manager.infrastructure.ros_adapters import RosInitialPoseSeeder


class FakePublisher:
    def __init__(self):
        self.subscribers = 0
        self.published = []

    def get_subscription_count(self):
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


def test_seed_returns_at_once_and_publishes_when_amcl_is_up():
    node, pose = FakeNode(), FakePose()
    seeder = RosInitialPoseSeeder(node, 'rover/map', pose, timeout=5.0)
    t0 = time.monotonic()
    seeder.seed(Pose2D(1.0, 2.0, 0.5), 1.5, 0.785)
    assert time.monotonic() - t0 < 0.1          # never blocks the manager's worker
    time.sleep(0.3)
    assert node.pub.published == []            # AMCL not up yet
    node.pub.subscribers = 1
    pose.pose = Pose2D(0.0, 0.0, 0.0)            # map -> base_link resolves
    assert wait_for(lambda: len(node.pub.published) == 1)
    msg = node.pub.published[0]
    assert (msg.pose.pose.position.x, msg.pose.pose.position.y) == (1.0, 2.0)
    assert abs(msg.pose.covariance[0] - 2.25) < 1e-9


def test_cancel_and_a_newer_seed_drop_the_pending_one():
    node, pose = FakeNode(), FakePose()
    seeder = RosInitialPoseSeeder(node, 'rover/map', pose, timeout=5.0)
    seeder.seed(Pose2D(1.0, 1.0, 0.0), 1.5, 0.785)
    seeder.cancel()                            # localization stopped before AMCL came up
    seeder.seed(Pose2D(9.0, 9.0, 0.0), 1.5, 0.785)
    node.pub.subscribers = 1
    pose.pose = Pose2D(0.0, 0.0, 0.0)
    assert wait_for(lambda: len(node.pub.published) >= 1)
    time.sleep(0.6)
    assert [m.pose.pose.position.x for m in node.pub.published] == [9.0]


def test_gives_up_quietly_after_the_timeout():
    node, pose = FakeNode(), FakePose()
    seeder = RosInitialPoseSeeder(node, 'rover/map', pose, timeout=0.3)
    seeder.seed(Pose2D(1.0, 1.0, 0.0), 1.5, 0.785)
    time.sleep(0.6)
    node.pub.subscribers = 1
    pose.pose = Pose2D(0.0, 0.0, 0.0)
    time.sleep(0.4)
    assert node.pub.published == []
