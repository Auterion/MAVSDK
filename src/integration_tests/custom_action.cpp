#include "integration_test_helper.h"
#include "global_include.h"
#include "mavsdk.h"
#include "plugins/action/action.h"
#include "plugins/custom_action/custom_action.h"
#include "plugins/telemetry/telemetry.h"

#include <future>

using namespace mavsdk;
using namespace std::placeholders;

static bool _received_custom_action = false;
static int _action_progress = 0;
static CustomAction::Result
process_custom_action(std::shared_ptr<System> system, CustomAction::ActionToExecute action);

TEST_F(SitlTest, CustomAction)
{
    Mavsdk mavsdk;

    ConnectionResult ret = mavsdk.add_udp_connection();
    ASSERT_EQ(ret, ConnectionResult::Success);

    auto system = std::shared_ptr<System>{nullptr};
    {
        LogInfo() << "Waiting to discover vehicle";
        std::promise<void> prom;
        std::future<void> fut = prom.get_future();

        mavsdk.subscribe_on_new_system([&prom, &mavsdk, &system]() {
            if (mavsdk.systems().size() == 1) {
                system = mavsdk.systems().at(0);
                prom.set_value();
            }
        });

        ASSERT_EQ(fut.wait_for(std::chrono::seconds(10)), std::future_status::ready);
    }

    ASSERT_TRUE(system->is_connected());

    auto telemetry = std::make_shared<Telemetry>(system);
    auto action = std::make_shared<Action>(system);
    auto custom_action = std::make_shared<CustomAction>(system);

    {
        LogDebug() << "Waiting to be ready...";
        std::promise<void> prom;
        std::future<void> fut = prom.get_future();
        telemetry->subscribe_health_all_ok([&telemetry, &prom](bool all_ok) {
            if (all_ok) {
                // Unregister to prevent fulfilling promise twice.
                telemetry->subscribe_health_all_ok(nullptr);
                prom.set_value();
            }
        });
        ASSERT_EQ(fut.wait_for(std::chrono::seconds(10)), std::future_status::ready);
    }

    {
        LogInfo() << "Arming";
        std::promise<void> prom;
        std::future<void> fut = prom.get_future();
        action->arm_async([&prom](Action::Result result) {
            EXPECT_EQ(result, Action::Result::Success);
            prom.set_value();
        });
        EXPECT_EQ(fut.wait_for(std::chrono::seconds(2)), std::future_status::ready);
    }

    LogInfo() << "Setting takeoff altitude";
    action->set_takeoff_altitude(5.0f);

    {
        LogInfo() << "Taking off";
        std::promise<void> prom;
        std::future<void> fut = prom.get_future();
        action->takeoff_async([&prom](Action::Result result) {
            EXPECT_EQ(result, Action::Result::Success);
            prom.set_value();
        });
        EXPECT_EQ(fut.wait_for(std::chrono::seconds(2)), std::future_status::ready);
    }

    std::this_thread::sleep_for(std::chrono::seconds(5));
    {
        LogInfo() << "Sending custom action";
        std::promise<void> prom;
        std::future<void> fut = prom.get_future();

        // Process custom action 0
        CustomAction::ActionToExecute action_to_execute{};
        action_to_execute.id = 0;
        action_to_execute.timeout = 10;

        custom_action->set_custom_action_async(
            action_to_execute, [&prom](CustomAction::Result result) {
                EXPECT_EQ(result, CustomAction::Result::Success);
                prom.set_value();
            });
        EXPECT_EQ(fut.wait_for(std::chrono::seconds(2)), std::future_status::ready);
    }

    {
        LogInfo() << "Processing custom action";

        // Change configuration so the instance is treated as a Companion Computer
        Mavsdk::Configuration config_cc(Mavsdk::Configuration::UsageType::CompanionComputer);
        mavsdk.set_configuration(config_cc);

        std::promise<CustomAction::ActionToExecute> prom;
        std::future<CustomAction::ActionToExecute> fut = prom.get_future();

        // Process custom_action
        custom_action->subscribe_custom_action(
            [&prom](CustomAction::ActionToExecute action_to_exec) {
                prom.set_value(action_to_exec);
                _received_custom_action = true;
                EXPECT_TRUE(_received_custom_action);
            });

        CustomAction::ActionToExecute action_exec = fut.get();
        std::future<CustomAction::Result> fut2 =
            std::async(process_custom_action, system, action_exec);
        CustomAction::Result action_result = fut2.get();
        LogInfo() << "Custom action #" << action_exec.id << " executed";

        std::promise<void> prom2;
        std::future<void> fut3 = prom2.get_future();

        action_exec.progress = _action_progress;
        // Send response with the result
        custom_action->respond_custom_action_async(
            action_exec, action_result, [&prom2](CustomAction::Result result) {
                EXPECT_EQ(result, CustomAction::Result::Success);
                prom2.set_value();
            });

        std::this_thread::sleep_for(std::chrono::seconds(5));

        // Change configuration back to default Ground Station
        Mavsdk::Configuration config_gcs(Mavsdk::Configuration::UsageType::GroundStation);
        mavsdk.set_configuration(config_gcs);
    }

    {
        LogInfo() << "Landing";
        std::promise<void> prom;
        std::future<void> fut = prom.get_future();
        action->land_async([&prom](Action::Result result) {
            EXPECT_EQ(result, Action::Result::Success);
            prom.set_value();
        });
        EXPECT_EQ(fut.wait_for(std::chrono::seconds(2)), std::future_status::ready);
    }

    {
        LogInfo() << "Waiting to be landed...";
        std::promise<void> prom;
        std::future<void> fut = prom.get_future();
        telemetry->subscribe_in_air([&telemetry, &prom](bool in_air) {
            if (!in_air) {
                // Unregister to prevent fulfilling promise twice.
                telemetry->subscribe_in_air(nullptr);
                prom.set_value();
            }
        });
        EXPECT_EQ(fut.wait_for(std::chrono::seconds(20)), std::future_status::ready);
    }

    {
        LogInfo() << "Disarming";
        std::promise<void> prom;
        std::future<void> fut = prom.get_future();
        action->disarm_async([&prom](Action::Result result) {
            EXPECT_EQ(result, Action::Result::Success);
            prom.set_value();
        });
        EXPECT_EQ(fut.wait_for(std::chrono::seconds(2)), std::future_status::ready);
    }
}

CustomAction::Result
process_custom_action(std::shared_ptr<System> system, CustomAction::ActionToExecute action)
{
    LogInfo() << "Custom action #" << action.id << " to be executed";

    // TODO: add action parsing and execution
    _action_progress = 100;

    return CustomAction::Result::InProgress;
}
