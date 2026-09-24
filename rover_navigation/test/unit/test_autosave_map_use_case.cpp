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
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "rover_navigation/application/autosave_map_use_case.hpp"

using rover_navigation::application::AutosaveMapUseCase;
using rover_navigation::application::AutosaveOutcome;
using rover_navigation::domain::MapAutosavePolicy;
using rover_navigation::domain::MapSaveRequest;

namespace
{

/**
 * @brief Records every dispatched request and returns a scripted availability.
 *
 * Replies are held back until the test calls `answer()`, like the asynchronous service.
 */
class FakeMapSaver : public rover_navigation::domain::ports::MapSaverPort
{
public:
    bool available = true;
    std::vector<MapSaveRequest> requests;
    std::vector<SaveDoneCallback> pending;

    bool save(const MapSaveRequest & request, SaveDoneCallback on_done) override
    {
        if (!available) {
            return false;
        }

        requests.push_back(request);
        pending.push_back(std::move(on_done));
        return true;
    }

    /** @brief Answer the oldest unanswered request. */
    void answer(bool written)
    {
        ASSERT_FALSE(pending.empty());
        auto on_done = std::move(pending.front());
        pending.erase(pending.begin());
        on_done(written);
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

    EXPECT_EQ(use_case.execute(), AutosaveOutcome::kRequested);

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

    EXPECT_EQ(use_case.execute(), AutosaveOutcome::kRequested);
    saver->answer(true);

    EXPECT_EQ(use_case.policy().consecutiveFailures(), 0);
    EXPECT_EQ(saver->requests.size(), 1u);
}

TEST(AutosaveMapUseCaseTest, DispatchAloneIsNotASuccess)
{
    auto saver = std::make_shared<FakeMapSaver>();
    saver->available = false;

    auto use_case = makeUseCase(saver);

    ASSERT_EQ(use_case.execute(), AutosaveOutcome::kSaverUnavailable);
    ASSERT_EQ(use_case.execute(), AutosaveOutcome::kSkippedBackingOff);

    saver->available = true;
    ASSERT_EQ(use_case.execute(), AutosaveOutcome::kRequested);

    // Until the saver answers, the earlier failure still stands.
    EXPECT_EQ(use_case.policy().consecutiveFailures(), 1);
}

TEST(AutosaveMapUseCaseTest, BacksOffWhenTheWriteFails)
{
    auto saver = std::make_shared<FakeMapSaver>();
    auto use_case = makeUseCase(saver);

    ASSERT_EQ(use_case.execute(), AutosaveOutcome::kRequested);
    saver->answer(false);

    EXPECT_EQ(use_case.policy().consecutiveFailures(), 1);
    EXPECT_EQ(use_case.execute(), AutosaveOutcome::kSkippedBackingOff);

    ASSERT_EQ(use_case.execute(), AutosaveOutcome::kRequested);
    saver->answer(false);

    // Two failures in a row: skip 3 ticks, then retry.
    EXPECT_EQ(use_case.policy().consecutiveFailures(), 2);
    for (int i = 0; i < 3; ++i) {
        EXPECT_EQ(use_case.execute(), AutosaveOutcome::kSkippedBackingOff);
    }
    EXPECT_EQ(use_case.execute(), AutosaveOutcome::kRequested);
    EXPECT_EQ(saver->requests.size(), 3u);
}

TEST(AutosaveMapUseCaseTest, ASuccessfulWriteResetsTheBackoff)
{
    auto saver = std::make_shared<FakeMapSaver>();
    auto use_case = makeUseCase(saver);

    ASSERT_EQ(use_case.execute(), AutosaveOutcome::kRequested);
    saver->answer(false);
    ASSERT_EQ(use_case.execute(), AutosaveOutcome::kSkippedBackingOff);

    ASSERT_EQ(use_case.execute(), AutosaveOutcome::kRequested);
    saver->answer(true);

    EXPECT_EQ(use_case.policy().consecutiveFailures(), 0);
    EXPECT_EQ(use_case.execute(), AutosaveOutcome::kRequested);
}

TEST(AutosaveMapUseCaseTest, KeepsOneRequestInFlight)
{
    auto saver = std::make_shared<FakeMapSaver>();
    auto use_case = makeUseCase(saver);

    ASSERT_EQ(use_case.execute(), AutosaveOutcome::kRequested);
    EXPECT_EQ(use_case.execute(), AutosaveOutcome::kSkippedInFlight);
    EXPECT_EQ(saver->requests.size(), 1u);

    saver->answer(true);

    EXPECT_EQ(use_case.execute(), AutosaveOutcome::kRequested);
    EXPECT_EQ(saver->requests.size(), 2u);
}

TEST(AutosaveMapUseCaseTest, AnUnansweredRequestEventuallyCountsAsAFailure)
{
    auto saver = std::make_shared<FakeMapSaver>();
    auto use_case = makeUseCase(saver);

    ASSERT_EQ(use_case.execute(), AutosaveOutcome::kRequested);
    for (int i = 1; i < AutosaveMapUseCase::kMaxTicksInFlight; ++i) {
        ASSERT_EQ(use_case.execute(), AutosaveOutcome::kSkippedInFlight);
    }

    EXPECT_EQ(use_case.execute(), AutosaveOutcome::kSaveTimedOut);
    EXPECT_EQ(use_case.policy().consecutiveFailures(), 1);

    // A reply that turns up after the timeout does not count twice, or as a success.
    saver->answer(true);
    EXPECT_EQ(use_case.policy().consecutiveFailures(), 1);
    EXPECT_EQ(use_case.execute(), AutosaveOutcome::kSkippedBackingOff);
}

TEST(AutosaveMapUseCaseTest, HandlesASaverThatAnswersImmediately)
{
    /** @brief Replies from inside save(), which a port is allowed to do. */
    class ImmediateSaver : public rover_navigation::domain::ports::MapSaverPort
    {
    public:
        bool save(const MapSaveRequest &, SaveDoneCallback on_done) override
        {
            on_done(false);
            return true;
        }
    };

    AutosaveMapUseCase use_case(
        std::make_shared<ImmediateSaver>(),
        MapAutosavePolicy(std::chrono::duration<double>(15.0)), MapSaveRequest{});

    ASSERT_EQ(use_case.execute(), AutosaveOutcome::kRequested);
    EXPECT_EQ(use_case.policy().consecutiveFailures(), 1);
    EXPECT_EQ(use_case.execute(), AutosaveOutcome::kSkippedBackingOff);
}
