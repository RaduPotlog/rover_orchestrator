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

#include <cmath>
#include <limits>
#include <string>

#include <gtest/gtest.h>

#include "rover_mission_manager/infrastructure/mission_request.hpp"

using rover_mission_manager::infrastructure::missionFromRequest;
using rover_mission_manager::infrastructure::stripLeadingSlash;
using rover_msgs::srv::SetMission;

namespace
{

geometry_msgs::msg::PoseStamped pose(const std::string & frame, double x, double y, double yaw)
{
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = frame;
    p.pose.position.x = x;
    p.pose.position.y = y;
    p.pose.orientation.z = std::sin(yaw / 2.0);
    p.pose.orientation.w = std::cos(yaw / 2.0);
    return p;
}

}  // namespace

TEST(MissionRequest, ConvertsPosesInTheGoalFrame)
{
    SetMission::Request request;
    request.mission_id = "goto";
    request.waypoints.push_back(pose("rover/map", 1.0, 2.0, 0.5));
    request.waypoints.push_back(pose("/rover/map", -3.0, 4.0, -1.0));
    request.waypoints.push_back(pose("", 0.0, 0.0, 3.0));

    std::string error;
    const auto mission = missionFromRequest(request, "rover/map", "fallback", error);

    ASSERT_TRUE(mission.has_value()) << error;
    EXPECT_EQ(mission->id(), "goto");
    ASSERT_EQ(mission->waypoints().size(), 3u);
    EXPECT_DOUBLE_EQ(mission->waypoints()[0].x, 1.0);
    EXPECT_DOUBLE_EQ(mission->waypoints()[0].y, 2.0);
    EXPECT_NEAR(mission->waypoints()[0].yaw, 0.5, 1e-9);
    EXPECT_NEAR(mission->waypoints()[1].yaw, -1.0, 1e-9);
    EXPECT_NEAR(mission->waypoints()[2].yaw, 3.0, 1e-9);
}

TEST(MissionRequest, UsesTheFallbackIdWhenEmpty)
{
    SetMission::Request request;
    request.waypoints.push_back(pose("", 1.0, 0.0, 0.0));

    std::string error;
    const auto mission = missionFromRequest(request, "rover/odom", "mission-7", error);

    ASSERT_TRUE(mission.has_value());
    EXPECT_EQ(mission->id(), "mission-7");
}

TEST(MissionRequest, RejectsEmptyMissions)
{
    SetMission::Request request;
    std::string error;
    EXPECT_FALSE(missionFromRequest(request, "rover/map", "x", error).has_value());
    EXPECT_FALSE(error.empty());
}

TEST(MissionRequest, RejectsAForeignFrame)
{
    SetMission::Request request;
    request.waypoints.push_back(pose("rover/map", 0.0, 0.0, 0.0));
    request.waypoints.push_back(pose("rover/odom", 1.0, 0.0, 0.0));

    std::string error;
    EXPECT_FALSE(missionFromRequest(request, "rover/map", "x", error).has_value());
    EXPECT_NE(error.find("Waypoint 1"), std::string::npos);
}

TEST(MissionRequest, RejectsNonFiniteOrZeroOrientation)
{
    std::string error;

    SetMission::Request nan_request;
    nan_request.waypoints.push_back(
        pose("", std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0));
    EXPECT_FALSE(missionFromRequest(nan_request, "rover/map", "x", error).has_value());

    SetMission::Request zero_q;
    zero_q.waypoints.push_back(pose("", 0.0, 0.0, 0.0));
    zero_q.waypoints[0].pose.orientation.w = 0.0;
    EXPECT_FALSE(missionFromRequest(zero_q, "rover/map", "x", error).has_value());
}

TEST(MissionRequest, StripsLeadingSlashesFromFrameIds)
{
    EXPECT_EQ(stripLeadingSlash("/rover/map"), "rover/map");
    EXPECT_EQ(stripLeadingSlash("//rover/map"), "rover/map");
    EXPECT_EQ(stripLeadingSlash("rover/map"), "rover/map");
    EXPECT_EQ(stripLeadingSlash("/"), "");
    EXPECT_EQ(stripLeadingSlash(""), "");
}
