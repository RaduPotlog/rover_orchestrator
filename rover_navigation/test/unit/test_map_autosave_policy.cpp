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

#include <gtest/gtest.h>

#include "rover_navigation/domain/map_autosave_policy.hpp"

using rover_navigation::domain::MapAutosavePolicy;

TEST(MapAutosavePolicyTest, KeepsAPeriodAtOrAboveTheFloor)
{
    MapAutosavePolicy policy(std::chrono::duration<double>(30.0));

    EXPECT_DOUBLE_EQ(policy.period().count(), 30.0);
    EXPECT_FALSE(policy.periodWasClamped());
}

TEST(MapAutosavePolicyTest, ClampsATooShortPeriodAndSaysSo)
{
    MapAutosavePolicy policy(std::chrono::duration<double>(1.0));

    EXPECT_DOUBLE_EQ(policy.period().count(), MapAutosavePolicy::kMinPeriod.count());
    EXPECT_TRUE(policy.periodWasClamped());
}

TEST(MapAutosavePolicyTest, SavesEveryTickWhileHealthy)
{
    MapAutosavePolicy policy(std::chrono::duration<double>(15.0));

    for (int i = 0; i < 5; ++i) {
        ASSERT_TRUE(policy.shouldSave()) << "tick " << i;
        policy.recordSuccess();
    }

    EXPECT_EQ(policy.consecutiveFailures(), 0);
}

TEST(MapAutosavePolicyTest, BacksOffExponentiallyAfterFailures)
{
    MapAutosavePolicy policy(std::chrono::duration<double>(15.0));

    // First failure: skip 1 tick before retrying.
    ASSERT_TRUE(policy.shouldSave());
    policy.recordFailure();
    EXPECT_FALSE(policy.shouldSave());
    ASSERT_TRUE(policy.shouldSave());

    // Second consecutive failure: skip 3.
    policy.recordFailure();
    EXPECT_FALSE(policy.shouldSave());
    EXPECT_FALSE(policy.shouldSave());
    EXPECT_FALSE(policy.shouldSave());
    EXPECT_TRUE(policy.shouldSave());

    EXPECT_EQ(policy.consecutiveFailures(), 2);
}

TEST(MapAutosavePolicyTest, BackoffIsCappedSoTheSaverIsStillRetried)
{
    MapAutosavePolicy policy(std::chrono::duration<double>(15.0));

    for (int i = 0; i < 20; ++i) {
        policy.recordFailure();
    }

    // Capped at (1 << 4) - 1 == 15 skipped ticks, not 2^20.
    int skipped = 0;
    while (!policy.shouldSave()) {
        ++skipped;
        ASSERT_LT(skipped, 100) << "backoff never expired";
    }

    EXPECT_EQ(skipped, 15);
}

TEST(MapAutosavePolicyTest, SuccessClearsAnOutstandingBackoff)
{
    MapAutosavePolicy policy(std::chrono::duration<double>(15.0));

    policy.recordFailure();
    policy.recordFailure();
    policy.recordSuccess();

    EXPECT_EQ(policy.consecutiveFailures(), 0);
    EXPECT_TRUE(policy.shouldSave());
}
