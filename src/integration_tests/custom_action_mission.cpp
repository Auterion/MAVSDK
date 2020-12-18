#include <iostream>
#include <functional>
#include <memory>
#include <future>
#include <atomic>
#include <cmath>
#include "integration_test_helper.h"
#include "mavsdk.h"
#include "plugins/telemetry/telemetry.h"
#include "plugins/action/action.h"
#include "plugins/custom_action/custom_action.h"
#include "plugins/mission_raw/mission_raw.h"

using namespace mavsdk;
using namespace std::placeholders; // for `_1`
using namespace std::chrono_literals;

static void test_mission(
    std::shared_ptr<Telemetry> telemetry,
    std::shared_ptr<MissionRaw> mission_raw,
    std::shared_ptr<Action> action,
    std::shared_ptr<CustomAction> custom_action);

static MissionRaw::MissionItem add_mission_raw_item(
    double latitude_deg,
    double longitude_deg,
    float relative_altitude_m,
    uint16_t sequence,
    uint8_t frame,
    uint16_t command,
    uint8_t current,
    uint8_t autocontinue,
    float param1,
    float param2,
    float param3,
    float param4,
    uint8_t mission_type);

static std::atomic<bool> _received_custom_action{false};
static std::atomic<bool> _pause_already_done{false};
static std::atomic<bool> _mission_finished{false};
static std::atomic<bool> _action_stoped{false};
static std::atomic<int> _action_progress{0};
static std::atomic<CustomAction::Result> _action_result{CustomAction::Result::Unknown};

std::mutex mtxAlert;
std::condition_variable sigAlert;

float _progress_current{0};
float _progress_total{0};

static void send_progress_status(std::shared_ptr<CustomAction> custom_action);
static void process_custom_action(
    CustomAction::ActionToExecute action, std::shared_ptr<CustomAction> custom_action);
static void execute_stages(
    CustomAction::ActionMetadata action_metadata, std::shared_ptr<CustomAction> custom_action);

TEST_F(SitlTest, CustomActionMission)
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
            for (auto& system : mavsdk_gcs.systems()) {
                if (system->has_autopilot()) {
                    system_to_gcs = system;
                    prom.set_value();
                    break;
                }
            }
        });

        ASSERT_EQ(fut.wait_for(std::chrono::seconds(10)), std::future_status::ready);
    }
    ASSERT_TRUE(system_to_gcs->is_connected());
    ASSERT_TRUE(system_to_gcs->has_autopilot());

    // Telemetry and actions are received and sent from the GCS respectively
    // GCS sends the mission as well
    auto telemetry = std::make_shared<Telemetry>(system_to_gcs);
    auto mission_raw = std::make_shared<MissionRaw>(system_to_gcs);
    auto action = std::make_shared<Action>(system_to_gcs);

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
                for (auto& system : mavsdk_companion.systems()) {
                    if (system->has_autopilot()) {
                        system_to_companion = system;
                        prom.set_value();
                        break;
                    }
                }
            });

        ASSERT_EQ(fut.wait_for(std::chrono::seconds(10)), std::future_status::ready);
    }
    ASSERT_TRUE(system_to_companion->is_connected());
    ASSERT_TRUE(system_to_companion->has_autopilot());

    // Custom actions are processed and executed in the mission computer
    auto custom_action = std::make_shared<CustomAction>(system_to_companion);

    test_mission(telemetry, mission_raw, action, custom_action);
}

void test_mission(
    std::shared_ptr<Telemetry> telemetry,
    std::shared_ptr<MissionRaw> mission_raw,
    std::shared_ptr<Action> action,
    std::shared_ptr<CustomAction> custom_action)
{
    while (!telemetry->health_all_ok()) {
        LogInfo() << "Waiting for system to be ready";
        LogDebug() << "Health: " << telemetry->health();
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    LogInfo() << "System ready";
    LogInfo() << "Creating and uploading mission";

    std::vector<MissionRaw::MissionItem> mission_raw_items;

    // Normal waypoint
    mission_raw_items.push_back(add_mission_raw_item(
        47.398170327054473,
        8.5456490218639658,
        10.0f,
        0,
        6, // MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
        16, // MAV_CMD_NAV_WAYPOINT
        1,
        1,
        1.0, // Hold
        1.0, // Acceptance Radius
        1.0, // Pass Radius
        NAN, // Yaw
        0 // MAV_MISSION_TYPE_MISSION
        ));

    // Custom action waypoint
    mission_raw_items.push_back(add_mission_raw_item(
        NAN,
        NAN,
        NAN,
        1,
        2, // MAV_FRAME_MISSION
        31000, // MAV_CMD_WAYPOINT_USER_1
        0,
        1,
        0.0, // Action ID
        0.0, // Action execution control
        30.0, // Action timeout (in seconds)
        NAN,
        0 // MAV_MISSION_TYPE_MISSION
        ));

    // Normal waypoint
    mission_raw_items.push_back(add_mission_raw_item(
        47.398241338125118,
        8.5455360114574432,
        10.0f,
        2,
        6, // MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
        16, // MAV_CMD_NAV_WAYPOINT
        0,
        1,
        1.0, // Hold
        1.0, // Acceptance Radius
        1.0, // Pass Radius
        NAN, // Yaw
        0 // MAV_MISSION_TYPE_MISSION
        ));

    {
        LogInfo() << "Uploading mission...";
        // We only have the upload_mission function asynchronous for now, so we wrap it using
        // std::future.
        std::promise<void> prom{};
        std::future<void> fut = prom.get_future();
        mission_raw->upload_mission_async(mission_raw_items, [&prom](MissionRaw::Result result) {
            ASSERT_EQ(result, MissionRaw::Result::Success);
            prom.set_value();
            LogInfo() << "Mission uploaded.";
        });

        auto status = fut.wait_for(std::chrono::seconds(2));
        ASSERT_EQ(status, std::future_status::ready);
        fut.get();
    }

    LogInfo() << "Arming...";
    const Action::Result arm_result = action->arm();
    EXPECT_EQ(arm_result, Action::Result::Success);
    LogInfo() << "Armed.";

    // Before starting the mission, we want to be sure to subscribe to the mission progress.
    mission_raw->subscribe_mission_progress([&mission_raw](MissionRaw::MissionProgress progress) {
        if (!_mission_finished) {
            LogInfo() << "Mission status update: " << progress.current << " / " << progress.total;
        }

        _progress_current = progress.current;
        _progress_total = progress.total;

        if (progress.current == progress.total && !_mission_finished) {
            _mission_finished = true;
        }
    });

    // Subscribe to the cancelation message
    custom_action->subscribe_custom_action_cancellation(
        [](bool canceled) { _action_stoped.store(canceled, std::memory_order_relaxed); });

    {
        LogInfo() << "Starting mission.";
        auto prom = std::make_shared<std::promise<void>>();
        auto future_result = prom->get_future();
        mission_raw->start_mission_async([prom](MissionRaw::Result result) {
            ASSERT_EQ(result, MissionRaw::Result::Success);
            prom->set_value();
            LogInfo() << "Started mission.";
        });

        auto status = future_result.wait_for(std::chrono::seconds(2));
        ASSERT_EQ(status, std::future_status::ready);
        future_result.get();
    }

    {
        // Get the custom action to process
        std::promise<CustomAction::ActionToExecute> prom_act;
        std::future<CustomAction::ActionToExecute> fut_act = prom_act.get_future();
        custom_action->subscribe_custom_action(
            [&prom_act](CustomAction::ActionToExecute action_to_exec) {
                if (!_received_custom_action) {
                    prom_act.set_value(action_to_exec);
                    _received_custom_action = true;
                }
            });

        while (!_received_custom_action) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        LogInfo() << "Process custom action";
        CustomAction::ActionToExecute action_exec = fut_act.get();

        std::this_thread::sleep_for(std::chrono::seconds(1));

        // Process the custom action
        process_custom_action(action_exec, custom_action);
    }

    while (!_mission_finished) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    {
        LogInfo() << "Return-to-launch";
        std::promise<void> prom;
        std::future<void> fut = prom.get_future();
        action->return_to_launch_async([&prom](Action::Result result) {
            EXPECT_EQ(result, Action::Result::Success);
            prom.set_value();
        });
        EXPECT_EQ(fut.wait_for(std::chrono::seconds(2)), std::future_status::ready);
    }

    while (telemetry->armed()) {
        // Wait until we're done.
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    LogInfo() << "Disarmed.";

    {
        LogInfo() << "Clearing mission.";
        auto prom = std::make_shared<std::promise<void>>();
        auto future_result = prom->get_future();
        mission_raw->clear_mission_async([prom](MissionRaw::Result result) {
            ASSERT_EQ(result, MissionRaw::Result::Success);
            prom->set_value();
            LogInfo() << "Cleared mission, exiting.";
        });

        auto status = future_result.wait_for(std::chrono::seconds(2));
        ASSERT_EQ(status, std::future_status::ready);
        future_result.get();
    }
}

MissionRaw::MissionItem add_mission_raw_item(
    double latitude_deg,
    double longitude_deg,
    float relative_altitude_m,
    uint16_t sequence,
    uint8_t frame,
    uint16_t command,
    uint8_t current,
    uint8_t autocontinue,
    float param1,
    float param2,
    float param3,
    float param4,
    uint8_t mission_type)
{
    MissionRaw::MissionItem new_raw_item_nav{};
    new_raw_item_nav.seq = sequence;
    new_raw_item_nav.frame = frame;
    new_raw_item_nav.command = command;
    new_raw_item_nav.current = current;
    new_raw_item_nav.autocontinue = autocontinue;
    new_raw_item_nav.param1 = param1;
    new_raw_item_nav.param2 = param2;
    new_raw_item_nav.param3 = param3;
    new_raw_item_nav.param4 = param4;
    new_raw_item_nav.x = int32_t(std::round(latitude_deg * 1e7));
    new_raw_item_nav.y = int32_t(std::round(longitude_deg * 1e7));
    new_raw_item_nav.z = relative_altitude_m;
    new_raw_item_nav.mission_type = mission_type;

    return new_raw_item_nav;
}

void send_progress_status(std::shared_ptr<CustomAction> custom_action)
{
    while (!_action_stoped.load() && _action_result.load() != CustomAction::Result::Unknown) {
        CustomAction::ActionToExecute action_exec{};
        action_exec.progress = _action_progress.load();
        auto action_result = _action_result.load();

        std::promise<void> prom;
        std::future<void> fut = prom.get_future();

        // Send response with the result and the progress
        custom_action->respond_custom_action_async(
            action_exec, action_result, [&prom](CustomAction::Result result) {
                EXPECT_EQ(result, CustomAction::Result::Success);
                prom.set_value();
            });

        std::this_thread::sleep_for(std::chrono::seconds(1));
    };
}

void process_custom_action(
    CustomAction::ActionToExecute action, std::shared_ptr<CustomAction> custom_action)
{
    // Note that this (client) function is not custom action generic, in the sense that
    // it a priori knows the number stages the action being executed is composed of
    LogInfo() << "Custom action #" << action.id << " being executed";

    // Get the custom action metadata
    std::promise<CustomAction::ActionMetadata> prom;
    std::future<CustomAction::ActionMetadata> fut = prom.get_future();
    custom_action->custom_action_metadata_async(
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

    // Start the progress status report thread
    std::thread status_th(send_progress_status, custom_action);

    // Start the custom action execution
    execute_stages(action_metadata, custom_action);
    EXPECT_EQ(_action_progress, 100);

    status_th.join();
}

void execute_stages(
    CustomAction::ActionMetadata action_metadata, std::shared_ptr<CustomAction> custom_action)
{
    // First stage
    {
        if (!_action_stoped.load()) {
            CustomAction::Result stage1_res =
                custom_action->execute_custom_action_stage(action_metadata.stages[0]);
            EXPECT_EQ(stage1_res, CustomAction::Result::Success);
            _action_result.store(CustomAction::Result::InProgress, std::memory_order_relaxed);
        }

        auto wait_time = 10s;
        std::unique_lock<std::mutex> lock(mtxAlert);
        sigAlert.wait_for(lock, wait_time, []() { return _action_stoped.load(); });
    }

    // Second stage
    {
        if (!_action_stoped.load()) {
            _action_progress.store(50.0, std::memory_order_relaxed);
            LogInfo() << "Custom action #" << action_metadata.id
                      << " current progress: " << _action_progress.load() << "%";

            CustomAction::Result stage2_res =
                custom_action->execute_custom_action_stage(action_metadata.stages[1]);
            EXPECT_EQ(stage2_res, CustomAction::Result::Success);
            _action_result.store(CustomAction::Result::InProgress, std::memory_order_relaxed);
        }

        auto wait_time = 10s;
        std::unique_lock<std::mutex> lock(mtxAlert);
        sigAlert.wait_for(lock, wait_time, []() { return _action_stoped.load(); });
    }

    // End
    {
        if (!_action_stoped.load()) {
            _action_progress.store(100.0, std::memory_order_relaxed);
            _action_result.store(CustomAction::Result::Success, std::memory_order_relaxed);
            LogInfo() << "Custom action #" << action_metadata.id
                      << " current progress: " << _action_progress.load() << "%";
        }

        auto wait_time = 3s;
        std::unique_lock<std::mutex> lock(mtxAlert);
        sigAlert.wait_for(lock, wait_time, []() { return _action_stoped.load(); });

        // Used to stop the threads
        _action_result.store(CustomAction::Result::Unknown, std::memory_order_relaxed);
    }

    if (_action_stoped.load()) {
        LogWarn() << "Custom action #" << action_metadata.id << " canceled!";
    } else if (!_action_stoped.load() && _action_progress == 100) {
        LogInfo() << "Custom action #" << action_metadata.id << " executed!";
    }
}
