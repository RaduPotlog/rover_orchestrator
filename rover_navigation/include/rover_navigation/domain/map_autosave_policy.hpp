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

#ifndef ROVER_NAVIGATION_DOMAIN_MAP_AUTOSAVE_POLICY_HPP_
#define ROVER_NAVIGATION_DOMAIN_MAP_AUTOSAVE_POLICY_HPP_

#include <chrono>
#include <string>

namespace rover_navigation::domain
{

/** @brief What to write, and where. Pure data; the adapter turns this into a service request. */
struct MapSaveRequest
{
    std::string map_topic;
    std::string map_url;
    std::string image_format = "png";
    std::string map_mode = "trinary";
    double free_thresh = 0.25;
    double occupied_thresh = 0.65;
};

/**
 * @brief Decides how often a SLAM map may be written to disk, and how to behave when the
 *        map saver is not answering.
 *
 * Saving a map is disk I/O over a service call; asking for it faster than the saver can
 * complete it only queues work. The policy clamps the requested period to a floor and backs
 * off exponentially while saves keep failing, so a missing map_saver does not produce a
 * request every tick.
 */
class MapAutosavePolicy
{
public:
    /** @brief Shortest period a map may be saved at, whatever the parameter says. */
    static constexpr std::chrono::duration<double> kMinPeriod{10.0};

    /** @brief Backoff stops growing here, so the saver is still retried periodically. */
    static constexpr int kMaxBackoffExponent = 4;

    explicit MapAutosavePolicy(std::chrono::duration<double> requested_period);

    /** @brief The requested period, clamped to kMinPeriod. */
    std::chrono::duration<double> period() const { return period_; }

    /** @brief True when the requested period had to be clamped, so the caller can warn. */
    bool periodWasClamped() const { return period_was_clamped_; }

    /**
     * @brief Whether this tick should attempt a save.
     *
     * Returns false while backing off from earlier failures, consuming one skip per call.
     */
    bool shouldSave();

    void recordSuccess();
    void recordFailure();

    int consecutiveFailures() const { return consecutive_failures_; }

private:
    /** @brief Number of ticks to skip before the next attempt: 0, 1, 3, 7, 15. */
    int currentBackoffTicks() const;

    std::chrono::duration<double> period_;
    bool period_was_clamped_;
    int consecutive_failures_;
    int ticks_to_skip_;
};

}  // namespace rover_navigation::domain

#endif  // ROVER_NAVIGATION_DOMAIN_MAP_AUTOSAVE_POLICY_HPP_
