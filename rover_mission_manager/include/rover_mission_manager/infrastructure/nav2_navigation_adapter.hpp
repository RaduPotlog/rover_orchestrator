// Copyright 2025 Mechatronics Academy
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROVER_MISSION_MANAGER_INFRASTRUCTURE_NAV2_NAVIGATION_ADAPTER_HPP_
#define ROVER_MISSION_MANAGER_INFRASTRUCTURE_NAV2_NAVIGATION_ADAPTER_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "rover_mission_manager/domain/ports/navigation_port.hpp"

namespace rover_mission_manager::infrastructure
{

/**
 * @brief NavigationPort backed by Nav 2's `navigate_to_pose` action.
 *
 * Every goTo() and cancel() starts a new generation. Action-client callbacks carry the
 * generation of the goal they belong to and are dropped once it is no longer current, so a
 * late result from a cancelled or replaced goal cannot overwrite the state of the one that
 * replaced it. Nav 2's own tree aborts on the same motion lock and lidar conditions the
 * manager holds on, so that ABORTED routinely lands after the manager's cancel().
 */
class Nav2NavigationAdapter : public domain::ports::NavigationPort
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    Nav2NavigationAdapter(
        rclcpp::Node * node,
        const std::string & action_name,
        std::string goal_frame_id,
        std::chrono::duration<double> server_timeout);

    bool goTo(const domain::Waypoint & waypoint) override;
    void cancel() override;
    domain::ports::NavigationResult result() const override;

private:
    void goalResponseCb(const GoalHandle::SharedPtr & goal_handle, std::uint64_t generation);
    void resultCb(const GoalHandle::WrappedResult & wrapped_result, std::uint64_t generation);
    std::uint64_t currentGeneration() const;

    rclcpp::Node * node_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
    std::string goal_frame_id_;
    std::chrono::duration<double> server_timeout_;

    // Written from action-client callbacks, read from the manager's timer thread.
    std::atomic<domain::ports::NavigationResult> result_;

    // Guards generation_ and goal_handle_, and makes a callback's "is this still my goal?"
    // check atomic with the result_ write that follows it.
    mutable std::mutex mutex_;
    std::uint64_t generation_{0};
    GoalHandle::SharedPtr goal_handle_;
};

}  // namespace rover_mission_manager::infrastructure

#endif  // ROVER_MISSION_MANAGER_INFRASTRUCTURE_NAV2_NAVIGATION_ADAPTER_HPP_
