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

#ifndef ROVER_DRIVE_MODE_APPLICATION_DRIVE_MODE_USE_CASE_HPP_
#define ROVER_DRIVE_MODE_APPLICATION_DRIVE_MODE_USE_CASE_HPP_

#include <memory>
#include <optional>
#include <string>

#include "rover_drive_mode/domain/command_router.hpp"
#include "rover_drive_mode/domain/drive_mode.hpp"
#include "rover_drive_mode/domain/drive_mode_policy.hpp"
#include "rover_drive_mode/domain/guard_state.hpp"
#include "rover_drive_mode/domain/ports/drive_mode_output_port.hpp"

namespace rover_drive_mode::application
{

/**
 * @brief Owns the rover's driving mode and decides where every command goes.
 *
 * The single writer of the platform's web-teleop and Nav 2 inputs: the caller asks it for a
 * route before publishing anything. Every call returns promptly; the caller supplies the clock
 * (@p now_s, any monotonic seconds) and the cadence of tick().
 */
class DriveModeUseCase
{
public:
    DriveModeUseCase(
        std::shared_ptr<domain::ports::DriveModeOutputPort> output,
        domain::DriveModePolicy policy,
        domain::TakeoverThreshold takeover_threshold,
        domain::DriveMode initial_mode,
        std::string initial_reason);

    /** @brief Publish the initial status. Call once the output port is ready. */
    void start(const domain::Prerequisites & prereqs);

    /** @brief An operator's mode request. Applied only when the policy accepts it. */
    domain::TransitionDecision request(
        domain::DriveMode to, const domain::Prerequisites & prereqs);

    /** @brief Route one web joystick command; a takeover switches the mode first. */
    domain::TeleopRoute onTeleop(
        const domain::Velocity & cmd, const domain::Prerequisites & prereqs);

    /** @brief Whether a command coming out of a collision monitor may reach the platform. */
    bool acceptGuarded(domain::GuardedSource source) const;

    /** @brief Latest reports of the two collision monitors. */
    void onGuardReports(
        const domain::GuardReport & teleop_guard, const domain::GuardReport & navigation_guard);

    /** @brief Apply forced transitions (lost mission manager). */
    void tick(const domain::Prerequisites & prereqs, double now_s);

    domain::DriveMode mode() const { return status_.mode; }
    domain::GuardState guard() const { return status_.guard; }
    const std::string & reason() const { return status_.reason; }

private:
    void changeMode(domain::DriveMode to, std::string reason);

    /** @brief Recompute the guard for the current mode and publish on any change. */
    void publishIfChanged();

    std::shared_ptr<domain::ports::DriveModeOutputPort> output_;
    domain::DriveModePolicy policy_;
    domain::TakeoverThreshold takeover_threshold_;

    domain::ports::DriveModeStatus status_;
    std::optional<domain::ports::DriveModeStatus> last_published_;

    domain::GuardReport teleop_guard_;
    domain::GuardReport navigation_guard_;

    std::optional<double> mission_manager_missing_since_s_;
};

}  // namespace rover_drive_mode::application

#endif  // ROVER_DRIVE_MODE_APPLICATION_DRIVE_MODE_USE_CASE_HPP_
