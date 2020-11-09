#include "custom_action_impl.h"
#include "mavsdk_impl.h"

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
    _parent->register_mavlink_command_handler(
        MAV_CMD_WAYPOINT_USER_1,
        std::bind(&CustomActionImpl::process_custom_action_command, this, std::placeholders::_1),
        this);
}

void CustomActionImpl::deinit() {
    _parent->unregister_all_mavlink_command_handlers(this);
}

void CustomActionImpl::enable() {}

void CustomActionImpl::disable() {}

void CustomActionImpl::process_custom_action_command(
    const MavlinkCommandReceiver::CommandLong& command)
{
    CustomAction::ActionToExecute action_to_exec;
    action_to_exec.action = command.command;

    store_custom_action(action_to_exec);

    std::lock_guard<std::mutex> lock(_subscription_mutex);
    if (_custom_action_command_subscription) {
        auto callback = _custom_action_command_subscription;
        auto arg1 = CustomAction::Result::Success;
        auto arg2 = custom_action();
        _parent->call_user_callback([callback, arg1, arg2]() { callback(arg1, arg2); });
    }
}

void CustomActionImpl::store_custom_action(CustomAction::ActionToExecute action)
{
    std::lock_guard<std::mutex> lock(_custom_action_mutex);
    _custom_action = action;
}

CustomAction::ActionToExecute CustomActionImpl::custom_action() const
{
    std::lock_guard<std::mutex> lock(_custom_action_mutex);
    return _custom_action;
}

void CustomActionImpl::set_custom_action_async(const CustomAction::ResultCallback& callback) const
{
    MavlinkCommandSender::CommandLong command{};

    // command.command = MAV_CMD_CUSTOM_ACTION;
    command.command = MAV_CMD_WAYPOINT_USER_1;
    command.params.param1 = 0; // Action ID
    command.params.param2 = 0; // Action execution control
    command.params.param3 = 10; // Action timeout
    command.target_component_id = _parent->get_autopilot_id();

    _parent->send_command_async(
        command, [this, callback](MavlinkCommandSender::Result result, float) {
            command_result_callback(result, callback);
        });
}

CustomAction::Result CustomActionImpl::set_custom_action() const
{
    auto prom = std::promise<CustomAction::Result>();
    auto fut = prom.get_future();

    set_custom_action_async([&prom](CustomAction::Result result) { prom.set_value(result); });

    return fut.get();
}

void CustomActionImpl::custom_action_async(CustomAction::CustomActionCallback callback)
{
    std::lock_guard<std::mutex> lock(_subscription_mutex);
    _custom_action_command_subscription = callback;
}

// CustomAction::ActionToExecute
// CustomActionImpl::custom_action()
// {
//     // std::lock_guard<std::mutex> lock(_custom_action_command_mutex);
//     // return _custom_action_command;
//
//     auto prom = std::promise<CustomAction::ActionToExecute>();
//     auto fut = prom.get_future();
//
//     custom_action_async([&prom](CustomAction::Result result, CustomAction::ActionToExecute
//     action) { prom.set_value(action); });
//
//     return fut.get();
// }

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
