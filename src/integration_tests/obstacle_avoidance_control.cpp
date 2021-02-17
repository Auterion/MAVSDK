#include "integration_test_helper.h"
#include "global_include.h"
#include "mavsdk.h"
#include "plugins/obstacle_avoidance/obstacle_avoidance.h"
#include "plugins/obstacle_avoidance_server/obstacle_avoidance_server.h"

#include <future>

using namespace mavsdk;
using namespace std::placeholders;
using namespace std::chrono_literals;

static int run_cmd(const std::string& cmd_str);

TEST_F(SitlTest, ObstacleAvoidanceControl)
{
    // Configure MAVSDK GCS instance
    Mavsdk mavsdk_gcs;

    ConnectionResult ret_gcs = mavsdk_gcs.add_udp_connection(14550);
    ASSERT_EQ(ret_gcs, ConnectionResult::Success);

    auto gcs_to_autopilot = std::shared_ptr<System>{nullptr};
    {
        LogInfo() << "Waiting to discover vehicle from the GCS side";
        std::promise<void> prom;
        std::future<void> fut = prom.get_future();

        mavsdk_gcs.subscribe_on_new_system([&prom, &mavsdk_gcs, &gcs_to_autopilot]() {
            for (auto& system : mavsdk_gcs.systems()) {
                if (system->get_system_id() == 1) {
                    gcs_to_autopilot = system;
                    prom.set_value();
                    break;
                }
            }
        });

        ASSERT_EQ(fut.wait_for(std::chrono::seconds(10)), std::future_status::ready);
    }
    ASSERT_TRUE(gcs_to_autopilot->is_connected());
    ASSERT_TRUE(gcs_to_autopilot->has_autopilot());

    // Obstacle avoidance control commands are sent from the GCS
    auto obstacle_avoidance = std::make_shared<ObstacleAvoidance>(gcs_to_autopilot);

    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Configure MAVSDK mission computer instance
    Mavsdk mavsdk_companion;

    // Change configuration so the instance is treated as a mission computer
    Mavsdk::Configuration config_cc(Mavsdk::Configuration::UsageType::CompanionComputer);
    mavsdk_companion.set_configuration(config_cc);

    ConnectionResult ret_comp = mavsdk_companion.add_udp_connection(14540);
    ASSERT_EQ(ret_comp, ConnectionResult::Success);

    auto gcs_to_companion = std::shared_ptr<System>{nullptr};
    {
        LogInfo() << "Waiting to discover GCS from the mission computer side";
        std::promise<void> prom;
        std::future<void> fut = prom.get_future();

        mavsdk_companion.subscribe_on_new_system([&prom, &mavsdk_companion, &gcs_to_companion]() {
            for (auto& system : mavsdk_companion.systems()) {
                if (system->get_system_id() == 245) {
                    gcs_to_companion = system;
                    // Avoid setting the promise twice
                    mavsdk_companion.subscribe_on_new_system(nullptr);
                    prom.set_value();
                    break;
                }
            }
        });

        ASSERT_EQ(fut.wait_for(std::chrono::seconds(10)), std::future_status::ready);
    }
    ASSERT_TRUE(gcs_to_companion->is_connected());

    // Obstacle avoidance commands are processed in the mission computer
    auto obstacle_avoidance_server = std::make_shared<ObstacleAvoidanceServer>(gcs_to_companion);

    // Start obstacle avoidance control commands handler
    obstacle_avoidance_server->subscribe_control([](ObstacleAvoidanceServer::ControlType control) {
        switch (control.control_type) {
            case ObstacleAvoidanceServer::ControlType::Type::ControlStart:
                EXPECT_EQ(
                    run_cmd(
                        "python3 src/integration_tests/test_data/obstacle_avoidance_dummy.py &"),
                    0);
                break;
            case ObstacleAvoidanceServer::ControlType::Type::ControlStop:
                EXPECT_EQ(
                    run_cmd("kill `cat obs_avoid_service.pid` && rm -rf obs_avoid_service.pid"), 0);
                break;
            case ObstacleAvoidanceServer::ControlType::Type::ControlRestart:
                EXPECT_EQ(
                    run_cmd("kill `cat obs_avoid_service.pid` && rm -rf obs_avoid_service.pid"), 0);
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                EXPECT_EQ(
                    run_cmd(
                        "python3 src/integration_tests/test_data/obstacle_avoidance_dummy.py &"),
                    0);
                break;
            default:
                LogInfo() << "Unsupported control type";
        }
    });

    // Start controls sending
    {
        LogInfo() << "Starting obstacle avoidance...";
        EXPECT_EQ(obstacle_avoidance->start(), ObstacleAvoidance::Result::Success);
    }

    std::this_thread::sleep_for(std::chrono::seconds(3));

    {
        LogInfo() << "Restarting obstacle avoidance...";
        EXPECT_EQ(obstacle_avoidance->restart(), ObstacleAvoidance::Result::Success);
    }

    std::this_thread::sleep_for(std::chrono::seconds(5));

    {
        LogInfo() << "Stopping obstacle avoidance...";
        EXPECT_EQ(obstacle_avoidance->stop(), ObstacleAvoidance::Result::Success);
    }
}

int run_cmd(const std::string& cmd_str)
{
    const char* cmd = cmd_str.c_str();
    if (!system(NULL)) {
        return -1;
    }

    return system(cmd);
}
