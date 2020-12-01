#include "custom_action_impl.h"
#include "mavsdk_impl.h"

#include <fstream>

namespace mavsdk {

CustomActionImpl::CustomActionImpl(System& system) : PluginImplBase(system)
{
    _parent->register_plugin(this);
}

CustomActionImpl::CustomActionImpl(std::shared_ptr<System> system) : PluginImplBase(system)
{
    _parent->register_plugin(this);
}

CustomActionImpl::~CustomActionImpl()
{
    _parent->unregister_plugin(this);
}

void CustomActionImpl::init()
{
    using namespace std::placeholders;

    _parent->register_mavlink_command_handler(
        MAV_CMD_WAYPOINT_USER_1, // MAV_CMD_CUSTOM_ACTION,
        std::bind(&CustomActionImpl::process_custom_action_command, this, _1),
        this);
}

void CustomActionImpl::deinit()
{
    _parent->unregister_all_mavlink_command_handlers(this);
}

void CustomActionImpl::enable() {}

void CustomActionImpl::disable() {}

MavlinkCommandReceiver::Result
CustomActionImpl::process_custom_action_command(const MavlinkCommandReceiver::CommandLong& command)
{
    CustomAction::ActionToExecute action_to_exec;
    action_to_exec.id = command.params.param1;
    action_to_exec.timeout = command.params.param3;

    store_custom_action(action_to_exec);

    bool result = false;

    std::lock_guard<std::mutex> lock(_subscription_mutex);
    if (_custom_action_command_subscription) {
        auto callback = _custom_action_command_subscription;
        auto arg1 = custom_action();

        _parent->call_user_callback([callback, arg1]() { callback(arg1); });

        // Send first ACK marking the command as being in progress
        mavlink_message_t command_ack;
        mavlink_msg_command_ack_pack(
            _parent->get_own_system_id(),
            _parent->get_own_component_id(),
            &command_ack,
            MAV_CMD_WAYPOINT_USER_1,
            MAV_RESULT_IN_PROGRESS,
            0,
            action_to_exec.id, // Use the action ID in param4 to identify the action/process
            0,
            0);

        result = _parent->send_message(command_ack);
    }

    return result ? MavlinkCommandReceiver::Result::Success :
                    MavlinkCommandReceiver::Result::UnknownError;
}

void CustomActionImpl::store_custom_action(CustomAction::ActionToExecute action)
{
    std::lock_guard<std::mutex> lock(_custom_action_mutex);
    _custom_action = action;
}

CustomAction::Result CustomActionImpl::respond_custom_action(
    CustomAction::ActionToExecute action, CustomAction::Result result) const
{
    auto prom = std::promise<CustomAction::Result>();
    auto fut = prom.get_future();

    respond_custom_action_async(action, result, [&prom](CustomAction::Result action_result) {
        prom.set_value(action_result);
    });

    return fut.get();
}

void CustomActionImpl::respond_custom_action_async(
    CustomAction::ActionToExecute action,
    CustomAction::Result result,
    const CustomAction::ResultCallback& callback) const
{
    // Send ACKs based on the action status
    mavlink_message_t command_ack;
    mavlink_msg_command_ack_pack(
        _parent->get_own_system_id(),
        _parent->get_own_component_id(),
        &command_ack,
        MAV_CMD_WAYPOINT_USER_1,
        mavlink_command_result_from_custom_action_result(result),
        action.progress, // Set the command progress when applicable
        action.id, // Use the action ID in param4 to identify the action/process
        0,
        0);

    const CustomAction::Result action_result = _parent->send_message(command_ack) ?
                                                   CustomAction::Result::Success :
                                                   CustomAction::Result::Error;

    if (callback) {
        auto temp_callback = callback;
        _parent->call_user_callback(
            [temp_callback, action_result]() { temp_callback(action_result); });
    }
}

CustomAction::ActionToExecute CustomActionImpl::custom_action() const
{
    std::lock_guard<std::mutex> lock(_custom_action_mutex);
    return _custom_action;
}

void CustomActionImpl::custom_action_async(CustomAction::CustomActionCallback callback)
{
    std::lock_guard<std::mutex> lock(_subscription_mutex);
    _custom_action_command_subscription = callback;
}

CustomAction::Result
CustomActionImpl::set_custom_action(CustomAction::ActionToExecute& action) const
{
    auto prom = std::promise<CustomAction::Result>();
    auto fut = prom.get_future();

    set_custom_action_async(
        action, [&prom](CustomAction::Result result) { prom.set_value(result); });

    return fut.get();
}

void CustomActionImpl::set_custom_action_async(
    CustomAction::ActionToExecute& action, const CustomAction::ResultCallback& callback) const
{
    MavlinkCommandSender::CommandLong command{};

    // command.command = MAV_CMD_CUSTOM_ACTION;
    command.command = MAV_CMD_WAYPOINT_USER_1;
    command.params.param1 = action.id; // Action ID
    command.params.param2 = 0; // Action execution control (N/A)
    command.params.param3 = action.timeout; // Action timeout
    command.target_component_id = _parent->get_autopilot_id();

    _parent->send_command_async(
        command, [this, callback](MavlinkCommandSender::Result result, float) {
            command_result_callback(result, callback);
        });
}

std::pair<CustomAction::Result, CustomAction::ActionMetadata>
CustomActionImpl::custom_action_metadata(
    CustomAction::ActionToExecute& action, std::string& file) const
{
    auto prom = std::promise<std::pair<CustomAction::Result, CustomAction::ActionMetadata>>();
    auto fut = prom.get_future();

    custom_action_metadata_async(
        action, file, [&prom](CustomAction::Result result, CustomAction::ActionMetadata metadata) {
            prom.set_value(
                std::pair<CustomAction::Result, CustomAction::ActionMetadata>(result, metadata));
        });

    return fut.get();
}

void CustomActionImpl::custom_action_metadata_async(
    CustomAction::ActionToExecute& action,
    const std::string& file,
    const CustomAction::CustomActionMetadataCallback& callback) const
{
    UNUSED(action);

    CustomAction::ActionMetadata action_metadata{};
    CustomAction::Result parsing_result = CustomAction::Result::Unknown;

    auto result = std::pair<CustomAction::Result, CustomAction::ActionMetadata>(
        CustomAction::Result::Unknown, action_metadata);

    std::ifstream metadata_file(file);
    if (!metadata_file) {
        LogErr() << "Unable to open JSON file: " << file;
        parsing_result = CustomAction::Result::Error;
    }

    std::stringstream ss;
    ss << metadata_file.rdbuf();
    metadata_file.close();
    const auto raw_json = ss.str();

    Json::CharReaderBuilder builder;
    const std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
    Json::Value root;
    JSONCPP_STRING err;
    const bool ok =
        reader->parse(raw_json.c_str(), raw_json.c_str() + raw_json.length(), &root, &err);
    if (!ok) {
        LogErr() << "Parse error: " << err;
        parsing_result = CustomAction::Result::Error;
    }

    LogInfo() << "action " << action.id << ": " << root["action"];
    parsing_result = CustomAction::Result::Success;

    result.first = parsing_result;
    result.second = action_metadata;

    if (callback) {
        auto temp_callback = callback;
        _parent->call_user_callback(
            [temp_callback, result]() { temp_callback(result.first, result.second); });
    }
}

CustomAction::Result
CustomActionImpl::custom_action_result_from_command_result(MavlinkCommandSender::Result result)
{
    switch (result) {
        case MavlinkCommandSender::Result::Success:
            return CustomAction::Result::Success;
        case MavlinkCommandSender::Result::Timeout:
            return CustomAction::Result::Timeout;
        case MavlinkCommandSender::Result::NoSystem:
        case MavlinkCommandSender::Result::ConnectionError:
        case MavlinkCommandSender::Result::Busy:
        case MavlinkCommandSender::Result::CommandDenied:
        default:
            return CustomAction::Result::Error;
    }
}

MAV_RESULT
CustomActionImpl::mavlink_command_result_from_custom_action_result(CustomAction::Result result)
{
    switch (result) {
        case CustomAction::Result::Unknown:
        case CustomAction::Result::Error:
        case CustomAction::Result::Timeout:
            return MAV_RESULT_FAILED;
        case CustomAction::Result::Success:
            return MAV_RESULT_ACCEPTED;
        case CustomAction::Result::Unsupported:
            return MAV_RESULT_UNSUPPORTED;
        case CustomAction::Result::InProgress:
            return MAV_RESULT_IN_PROGRESS;
        default:
            return MAV_RESULT_FAILED;
    }
}

void CustomActionImpl::command_result_callback(
    MavlinkCommandSender::Result command_result, const CustomAction::ResultCallback& callback) const
{
    CustomAction::Result action_result = custom_action_result_from_command_result(command_result);

    if (callback) {
        auto temp_callback = callback;
        _parent->call_user_callback(
            [temp_callback, action_result]() { temp_callback(action_result); });
    }
}

} // namespace mavsdk
