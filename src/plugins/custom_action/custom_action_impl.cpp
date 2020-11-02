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

void CustomActionImpl::init() {}

void CustomActionImpl::deinit() {}

void CustomActionImpl::enable() {}

void CustomActionImpl::disable() {}

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

// void CustomAction::subscribe_custom_action(CustomActionCallback callback)
// {
//     _impl->custom_action_async(callback);
// }
//
//
//
//
// CustomAction::ActionToExecute
// CustomAction::custom_action() const
// {
//     return _impl->custom_action();
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
