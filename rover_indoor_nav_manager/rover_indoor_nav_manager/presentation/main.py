# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.

"""Entry point: indoor_nav_manager_node."""

import signal

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor

from ..infrastructure.indoor_nav_node import IndoorNavNode


def main(args=None):
    # Started as a background job of a non-interactive shell, this process inherits SIGINT
    # as ignored, and so would the ros2 launch it spawns. A handled signal resets to the
    # default on exec, so installing one here lets the child launch shut down on SIGINT.
    if signal.getsignal(signal.SIGINT) is signal.SIG_IGN:
        signal.signal(signal.SIGINT, signal.default_int_handler)
    rclpy.init(args=args)
    node = IndoorNavNode()
    # Two threads: a service handler (one at a time, MutuallyExclusive) may block on the worker,
    # and the other keeps serving the clients, timers and TF lookups that worker waits on. Each
    # extra thread costs CPU on every wake-up in rclpy.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Never leave slam_toolbox / AMCL running without their manager.
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
