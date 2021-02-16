#include "integration_test_helper.h"
#include "global_include.h"
#include "mavsdk.h"
#include "plugins/obstacle_avoidance/obstacle_avoidance.h"
#include "plugins/obstacle_avoidance_server/obstacle_avoidance_server.h"

#include <future>

using namespace mavsdk;
using namespace std::placeholders;
using namespace std::chrono_literals;

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

    bool gcs_set = false;
    auto gcs_to_companion = std::shared_ptr<System>{nullptr};
    {
        LogInfo() << "Waiting to discover GCS from the mission computer side";
        std::promise<void> prom;
        std::future<void> fut = prom.get_future();

        mavsdk_companion.subscribe_on_new_system(
            [&prom, &mavsdk_companion, &gcs_to_companion, &gcs_set]() {
                for (auto& system : mavsdk_companion.systems()) {
                    if (!gcs_set && system->get_system_id() == 245) {
                        gcs_to_companion = system;
                        gcs_set = true;
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
                LogInfo() << "Obstacle avoidance started!";
                break;
            case ObstacleAvoidanceServer::ControlType::Type::ControlRestart:
                LogInfo() << "Obstacle avoidance restarted!";
                break;
            case ObstacleAvoidanceServer::ControlType::Type::ControlStop:
                LogInfo() << "Obstacle avoidance stopped!";
                break;
            case ObstacleAvoidanceServer::ControlType::Type::ControlEnable:
                LogInfo() << "Obstacle avoidance enabled!";
                break;
            case ObstacleAvoidanceServer::ControlType::Type::ControlDisable:
                LogInfo() << "Obstacle avoidance disabled!";
                break;
            default:
                LogInfo() << "Unknown control type";
        }
    });

    // Start controls sending
    {
        LogInfo() << "Starting obstacle avoidance...";
        std::promise<void> prom1;
        std::future<void> fut1 = prom1.get_future();
        obstacle_avoidance->start_async([&prom1](ObstacleAvoidance::Result result) {
            EXPECT_EQ(result, ObstacleAvoidance::Result::Success);
            prom1.set_value();
        });
        EXPECT_EQ(fut1.wait_for(std::chrono::seconds(2)), std::future_status::ready);
    }

    std::this_thread::sleep_for(std::chrono::seconds(2));

    {
        LogInfo() << "Restarting obstacle avoidance...";
        std::promise<void> prom;
        std::future<void> fut = prom.get_future();
        obstacle_avoidance->restart_async([&prom](ObstacleAvoidance::Result result) {
            EXPECT_EQ(result, ObstacleAvoidance::Result::Success);
            prom.set_value();
        });
        EXPECT_EQ(fut.wait_for(std::chrono::seconds(2)), std::future_status::ready);
    }

    std::this_thread::sleep_for(std::chrono::seconds(2));

    {
        LogInfo() << "Stopping obstacle avoidance...";
        std::promise<void> prom;
        std::future<void> fut = prom.get_future();
        obstacle_avoidance->stop_async([&prom](ObstacleAvoidance::Result result) {
            EXPECT_EQ(result, ObstacleAvoidance::Result::Success);
            prom.set_value();
        });
        EXPECT_EQ(fut.wait_for(std::chrono::seconds(2)), std::future_status::ready);
    }
}
