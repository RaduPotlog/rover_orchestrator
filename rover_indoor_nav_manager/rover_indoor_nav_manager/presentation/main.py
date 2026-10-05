# Copyright 2026 Mechatronics Academy
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
    executor = MultiThreadedExecutor(num_threads=4)
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
