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
static std::atomic<int> _action_progress;
static std::atomic<CustomAction::Result> _action_result;

static void send_progress_status(std::shared_ptr<CustomAction> custom_action);
static void process_custom_action(
    CustomAction::ActionToExecute action, std::shared_ptr<CustomAction> custom_action);

TEST_F(SitlTest, CustomAction)
{
    // Configure MAVSDK GCS instance
    Mavsdk mavsdk_gcs;

    ConnectionResult ret_gcs = mavsdk_gcs.add_udp_connection(14550);
    ASSERT_EQ(ret_gcs, ConnectionResult::Success);

    auto system_to_gcs = std::shared_ptr<System>{nullptr};
    {
        LogInfo() << "Waiting to discover vehicle from the GCS side";
        std::promise<void> prom;
        std::future<void> fut = prom.get_future();

        mavsdk_gcs.subscribe_on_new_system([&prom, &mavsdk_gcs, &system_to_gcs]() {
            if (mavsdk_gcs.systems().size() == 1) {
                system_to_gcs = mavsdk_gcs.systems().at(0);
                prom.set_value();
            }
        });

        ASSERT_EQ(fut.wait_for(std::chrono::seconds(10)), std::future_status::ready);
    }

    ASSERT_TRUE(system_to_gcs->is_connected());

    // Telemetry and actions are received and sent from the GCS respectively
    // The GCS is also capable of sending the custom action commands
    auto telemetry = std::make_shared<Telemetry>(system_to_gcs);
    auto action = std::make_shared<Action>(system_to_gcs);
    auto custom_action_gcs = std::make_shared<CustomAction>(system_to_gcs);

    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Configure MAVSDK mission computer instance
    Mavsdk mavsdk_companion;

    // Change configuration so the instance is treated as a mission computer
    Mavsdk::Configuration config_cc(Mavsdk::Configuration::UsageType::CompanionComputer);
    mavsdk_companion.set_configuration(config_cc);

    ConnectionResult ret_comp = mavsdk_companion.add_udp_connection(14540);
    ASSERT_EQ(ret_comp, ConnectionResult::Success);

    auto system_to_companion = std::shared_ptr<System>{nullptr};
    {
        LogInfo() << "Waiting to discover vehicle from the mission computer side";
        std::promise<void> prom;
        std::future<void> fut = prom.get_future();

        mavsdk_companion.subscribe_on_new_system(
            [&prom, &mavsdk_companion, &system_to_companion]() {
                if (mavsdk_companion.systems().size() == 1) {
                    system_to_companion = mavsdk_companion.systems().at(0);
                    prom.set_value();
                }
            });

        ASSERT_EQ(fut.wait_for(std::chrono::seconds(10)), std::future_status::ready);
    }
    ASSERT_TRUE(system_to_companion->is_connected());

    // Custom actions are processed and executed in the mission computer
    auto custom_action_comp = std::make_shared<CustomAction>(system_to_companion);

    // Start tests
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
        std::promise<void> prom_res;
        std::future<void> fut_res = prom_res.get_future();

        // Send command to start custom action 0
        CustomAction::ActionToExecute action_to_execute{};
        action_to_execute.id = 0;
        action_to_execute.timeout = 10;

        custom_action_gcs->set_custom_action_async(
            action_to_execute, [&prom_res](CustomAction::Result result) {
                EXPECT_EQ(result, CustomAction::Result::Success);
                prom_res.set_value();
            });

        LogInfo() << "Process custom action";

        // Get the custom action to process
        std::promise<CustomAction::ActionToExecute> prom_act;
        std::future<CustomAction::ActionToExecute> fut_act = prom_act.get_future();
        custom_action_comp->subscribe_custom_action(
            [&prom_act](CustomAction::ActionToExecute action_to_exec) {
                prom_act.set_value(action_to_exec);
                _received_custom_action = true;
                EXPECT_TRUE(_received_custom_action);
            });
        EXPECT_EQ(fut_act.wait_for(std::chrono::seconds(1)), std::future_status::ready);
        CustomAction::ActionToExecute action_exec = fut_act.get();

        // Start the progress status report thread
        std::thread status_th(send_progress_status, custom_action_comp);

        // Process the custom action
        process_custom_action(action_exec, custom_action_comp);

        LogInfo() << "Custom action #" << action_exec.id << " executed!";
        std::this_thread::sleep_for(std::chrono::seconds(1));
        // Used to stop the status report thread
        _action_result.store(CustomAction::Result::Unknown, std::memory_order_relaxed);
        status_th.join();

        std::this_thread::sleep_for(std::chrono::seconds(1));
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

void send_progress_status(std::shared_ptr<CustomAction> custom_action_comp)
{
    do {
        CustomAction::ActionToExecute action_exec{};
        action_exec.progress = _action_progress.load();
        auto action_result = _action_result.load();

        std::promise<void> prom;
        std::future<void> fut = prom.get_future();

        // Send response with the result and the progress
        custom_action_comp->respond_custom_action_async(
            action_exec, action_result, [&prom](CustomAction::Result result) {
                EXPECT_EQ(result, CustomAction::Result::Success);
                prom.set_value();
            });

        std::this_thread::sleep_for(std::chrono::seconds(1));
    } while (_action_result.load() != CustomAction::Result::Unknown);
}

void process_custom_action(
    CustomAction::ActionToExecute action, std::shared_ptr<CustomAction> custom_action_comp)
{
    LogInfo() << "Custom action #" << action.id << " being executed";

    // Get the custom action metadata
    std::promise<CustomAction::ActionMetadata> prom;
    std::future<CustomAction::ActionMetadata> fut = prom.get_future();
    custom_action_comp->custom_action_metadata_async(
        action,
        "src/integration_tests/test_data/custom_action.json",
        [&prom](CustomAction::Result result, CustomAction::ActionMetadata action_metadata) {
            prom.set_value(action_metadata);
            EXPECT_EQ(result, CustomAction::Result::Success);
        });
    CustomAction::ActionMetadata action_metadata = fut.get();

    EXPECT_EQ(action_metadata.id, 0);
    EXPECT_EQ(action_metadata.name, "Integration test action");
    EXPECT_EQ(
        action_metadata.description,
        "Example action to use on the integration test, mimicking the Go To Action test");
    LogInfo() << "Custom action #" << action_metadata.id << " is \"" << action_metadata.name
              << "\"";

    // Start
    _action_result.store(CustomAction::Result::InProgress, std::memory_order_relaxed);
    _action_progress.store(0.0, std::memory_order_relaxed);
    LogInfo() << "Custom action #" << action_metadata.id
              << " current progress: " << _action_progress.load() << "%";

    // First stage
    EXPECT_EQ(
        custom_action_comp->execute_custom_action_stage(action_metadata.stages[0]),
        CustomAction::Result::Success);
    std::this_thread::sleep_for(std::chrono::seconds(10));

    _action_progress.store(50.0, std::memory_order_relaxed);
    LogInfo() << "Custom action #" << action_metadata.id
              << " current progress: " << _action_progress.load() << "%";

    // Second stage
    EXPECT_EQ(
        custom_action_comp->execute_custom_action_stage(action_metadata.stages[1]),
        CustomAction::Result::Success);
    std::this_thread::sleep_for(std::chrono::seconds(10));

    // End
    _action_progress.store(100.0, std::memory_order_relaxed);
    _action_result.store(CustomAction::Result::Success, std::memory_order_relaxed);
    LogInfo() << "Custom action #" << action_metadata.id
              << " current progress: " << _action_progress.load() << "%";
}
