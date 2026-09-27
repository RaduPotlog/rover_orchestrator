// Copyright 2026 Mechatronics Academy
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

#include "rover_drive_mode/application/drive_mode_use_case.hpp"

#include <stdexcept>
#include <utility>

namespace rover_drive_mode::application
{

using domain::DriveMode;
using domain::TeleopRoute;

DriveModeUseCase::DriveModeUseCase(
    std::shared_ptr<domain::ports::DriveModeOutputPort> output,
    domain::DriveModePolicy policy,
    domain::TakeoverThreshold takeover_threshold,
    DriveMode initial_mode,
    std::string initial_reason)
: output_(std::move(output)),
  policy_(policy),
  takeover_threshold_(takeover_threshold)
{
    if (output_ == nullptr) {
        throw std::invalid_argument("DriveModeUseCase requires a DriveModeOutputPort");
    }

    status_.mode = initial_mode;
    status_.reason = std::move(initial_reason);
}

void DriveModeUseCase::start(const domain::Prerequisites & prereqs)
{
    // The configured boot mode still has to satisfy its prerequisites: booting into ASSISTED
    // with the guard disabled would route the joystick into a monitor that is not there.
    const auto decision = policy_.decide(policy_.fallbackMode(prereqs), status_.mode, prereqs);

    if (!decision.accepted) {
        status_.mode = policy_.fallbackMode(prereqs);
        status_.reason = "boot default unavailable: " + decision.reason;
    }

    publishIfChanged();
}

domain::TransitionDecision DriveModeUseCase::request(
    DriveMode to, const domain::Prerequisites & prereqs)
{
    const auto decision = policy_.decide(status_.mode, to, prereqs);

    if (decision.accepted && to != status_.mode) {
        changeMode(to, "operator request");
    }

    return decision;
}

TeleopRoute DriveModeUseCase::onTeleop(
    const domain::Velocity & cmd, const domain::Prerequisites & prereqs)
{
    const auto decision = domain::routeTeleop(
        status_.mode, cmd, takeover_threshold_, policy_.fallbackMode(prereqs));

    if (decision.switch_to.has_value()) {
        changeMode(*decision.switch_to, "operator takeover");
    }

    return decision.route;
}

bool DriveModeUseCase::acceptGuarded(domain::GuardedSource source) const
{
    return domain::forwardGuarded(source, status_.mode);
}

void DriveModeUseCase::onGuardReports(
    const domain::GuardReport & teleop_guard, const domain::GuardReport & navigation_guard)
{
    teleop_guard_ = teleop_guard;
    navigation_guard_ = navigation_guard;
    publishIfChanged();
}

void DriveModeUseCase::tick(const domain::Prerequisites & prereqs, double now_s)
{
    if (prereqs.mission_manager_available) {
        mission_manager_missing_since_s_.reset();
    } else if (!mission_manager_missing_since_s_.has_value()) {
        mission_manager_missing_since_s_ = now_s;
    }

    const double missing_s =
        mission_manager_missing_since_s_.has_value() ? now_s - *mission_manager_missing_since_s_
                                                     : 0.0;

    if (const auto forced = policy_.forcedTransition(status_.mode, prereqs, missing_s)) {
        changeMode(*forced, "mission manager lost");
    }

    publishIfChanged();
}

void DriveModeUseCase::changeMode(DriveMode to, std::string reason)
{
    const DriveMode from = status_.mode;

    status_.mode = to;
    status_.reason = std::move(reason);

    if (from == DriveMode::kAutomatic && to != DriveMode::kAutomatic) {
        output_->stopNavigation();
    }

    publishIfChanged();
}

void DriveModeUseCase::publishIfChanged()
{
    status_.guard = domain::guardState(status_.mode, teleop_guard_, navigation_guard_);

    if (last_published_.has_value() && last_published_->mode == status_.mode &&
        last_published_->guard == status_.guard && last_published_->reason == status_.reason)
    {
        return;
    }

    last_published_ = status_;
    output_->publishStatus(status_);
}

}  // namespace rover_drive_mode::application
