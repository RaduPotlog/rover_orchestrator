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

#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include <nav2_msgs/srv/save_map.hpp>
#include <rclcpp/rclcpp.hpp>

#include "rover_navigation/infrastructure/nav2_map_saver_client.hpp"

namespace rover_navigation
{
namespace
{

using namespace std::chrono_literals;
using infrastructure::Nav2MapSaverClient;
using SaveMapSrv = nav2_msgs::srv::SaveMap;

constexpr char kService[] = "test_map_saver/save_map";

/** A stand-in for nav2_map_server's saver, answering every request with `result`. */
class Nav2MapSaverClientTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }

    void SetUp() override
    {
        node_ = std::make_shared<rclcpp::Node>("nav2_map_saver_client_test_node");
        executor_.add_node(node_);
    }

    void TearDown() override
    {
        executor_.remove_node(node_);
        client_.reset();
        server_.reset();
        node_.reset();
    }

    void startServer(const bool result)
    {
        server_ = node_->create_service<SaveMapSrv>(
            kService,
            [this, result](
                const std::shared_ptr<SaveMapSrv::Request> request,
                std::shared_ptr<SaveMapSrv::Response> response) {
                last_request_ = *request;
                response->result = result;
            });
    }

    void makeClient(const std::chrono::duration<double> connection_timeout)
    {
        client_ = std::make_unique<Nav2MapSaverClient>(node_.get(), kService, connection_timeout);
    }

    bool spinUntil(const std::function<bool()> & done, const std::chrono::milliseconds timeout)
    {
        const auto deadline = std::chrono::steady_clock::now() + timeout;

        while (!done() && std::chrono::steady_clock::now() < deadline) {
            executor_.spin_some(10ms);
        }

        return done();
    }

    static domain::MapSaveRequest request()
    {
        domain::MapSaveRequest request;
        request.map_topic = "/rover/map";
        request.map_url = "/nonexistent/maps/map";
        return request;
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    rclcpp::Service<SaveMapSrv>::SharedPtr server_;
    std::unique_ptr<Nav2MapSaverClient> client_;
    std::optional<SaveMapSrv::Request> last_request_;
};

TEST_F(Nav2MapSaverClientTest, ReportsAnUnreachableSaver)
{
    makeClient(100ms);

    bool called = false;
    EXPECT_FALSE(client_->save(request(), [&called](bool) { called = true; }));

    executor_.spin_some(10ms);
    EXPECT_FALSE(called);
}

TEST_F(Nav2MapSaverClientTest, ReportsAFailedWrite)
{
    startServer(false);
    makeClient(3s);

    std::optional<bool> written;
    ASSERT_TRUE(client_->save(request(), [&written](bool ok) { written = ok; }));

    ASSERT_TRUE(spinUntil([&written] { return written.has_value(); }, 5s));
    EXPECT_FALSE(*written);

    ASSERT_TRUE(last_request_.has_value());
    EXPECT_EQ(last_request_->map_topic, "/rover/map");
    EXPECT_EQ(last_request_->map_url, "/nonexistent/maps/map");
}

TEST_F(Nav2MapSaverClientTest, ReportsASuccessfulWrite)
{
    startServer(true);
    makeClient(3s);

    std::optional<bool> written;
    ASSERT_TRUE(client_->save(request(), [&written](bool ok) { written = ok; }));

    ASSERT_TRUE(spinUntil([&written] { return written.has_value(); }, 5s));
    EXPECT_TRUE(*written);
}

}  // namespace
}  // namespace rover_navigation
