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

#include <chrono>
#include <memory>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "rover_navigation/application/autosave_map_use_case.hpp"

using rover_navigation::application::AutosaveMapUseCase;
using rover_navigation::application::AutosaveOutcome;
using rover_navigation::domain::MapAutosavePolicy;
using rover_navigation::domain::MapSaveRequest;

namespace
{

/** @brief Records every dispatched request and returns a scripted availability. */
class FakeMapSaver : public rover_navigation::domain::ports::MapSaverPort
{
public:
    bool available = true;
    std::vector<MapSaveRequest> requests;

    bool save(const MapSaveRequest & request) override
    {
        if (!available) {
            return false;
        }

        requests.push_back(request);
        return true;
    }
};

AutosaveMapUseCase makeUseCase(std::shared_ptr<FakeMapSaver> saver)
{
    MapSaveRequest request;
    request.map_topic = "/rover/map";
    request.map_url = "/maps/map";

    return AutosaveMapUseCase(
        std::move(saver), MapAutosavePolicy(std::chrono::duration<double>(15.0)), request);
}

}  // namespace

TEST(AutosaveMapUseCaseTest, RejectsANullPort)
{
    EXPECT_THROW(
        AutosaveMapUseCase(
            nullptr, MapAutosavePolicy(std::chrono::duration<double>(15.0)), MapSaveRequest{}),
        std::invalid_argument);
}

TEST(AutosaveMapUseCaseTest, DispatchesTheConfiguredRequest)
{
    auto saver = std::make_shared<FakeMapSaver>();
    auto use_case = makeUseCase(saver);

    EXPECT_EQ(use_case.execute(), AutosaveOutcome::kSaved);

    ASSERT_EQ(saver->requests.size(), 1u);
    EXPECT_EQ(saver->requests.front().map_topic, "/rover/map");
    EXPECT_EQ(saver->requests.front().map_url, "/maps/map");
}

TEST(AutosaveMapUseCaseTest, ReportsAnUnavailableSaverAndThenBacksOff)
{
    auto saver = std::make_shared<FakeMapSaver>();
    saver->available = false;

    auto use_case = makeUseCase(saver);

    EXPECT_EQ(use_case.execute(), AutosaveOutcome::kSaverUnavailable);
    EXPECT_EQ(use_case.execute(), AutosaveOutcome::kSkippedBackingOff);

    EXPECT_TRUE(saver->requests.empty());
}

TEST(AutosaveMapUseCaseTest, ResumesSavingOnceTheSaverComesBack)
{
    auto saver = std::make_shared<FakeMapSaver>();
    saver->available = false;

    auto use_case = makeUseCase(saver);

    ASSERT_EQ(use_case.execute(), AutosaveOutcome::kSaverUnavailable);
    ASSERT_EQ(use_case.execute(), AutosaveOutcome::kSkippedBackingOff);

    saver->available = true;

    EXPECT_EQ(use_case.execute(), AutosaveOutcome::kSaved);
    EXPECT_EQ(use_case.policy().consecutiveFailures(), 0);
    EXPECT_EQ(saver->requests.size(), 1u);
}
