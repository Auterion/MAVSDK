#include "obstacle_avoidance_impl.h"

namespace mavsdk {

ObstacleAvoidanceImpl::ObstacleAvoidanceImpl(System& system) : PluginImplBase(system)
{
    _parent->register_plugin(this);
}

ObstacleAvoidanceImpl::ObstacleAvoidanceImpl(std::shared_ptr<System> system) :
    PluginImplBase(system)
{
    _parent->register_plugin(this);
}

ObstacleAvoidanceImpl::~ObstacleAvoidanceImpl()
{
    _parent->unregister_plugin(this);
}

void ObstacleAvoidanceImpl::init() {}

void ObstacleAvoidanceImpl::deinit() {}

void ObstacleAvoidanceImpl::enable() {}

void ObstacleAvoidanceImpl::disable() {}

void ObstacleAvoidanceImpl::start_async(const ObstacleAvoidance::ResultCallback callback)
{
    MavlinkCommandSender::CommandLong command{*_parent};

    command.command = MAV_CMD_COMPONENT_CONTROL;
    command.params.param1 = MAV_COMP_ID_OBSTACLE_AVOIDANCE;
    command.params.param2 = COMPONENT_CONTROL_START;
    command.target_component_id = MAV_COMP_ID_ONBOARD_COMPUTER;

    _parent->send_command_async(
        command, [this, callback](MavlinkCommandSender::Result result, float) {
            command_result_callback(result, callback);
        });
}

ObstacleAvoidance::Result ObstacleAvoidanceImpl::start()
{
    auto prom = std::promise<ObstacleAvoidance::Result>();
    auto fut = prom.get_future();

    start_async([&prom](ObstacleAvoidance::Result result) { prom.set_value(result); });

    return fut.get();
}

void ObstacleAvoidanceImpl::stop_async(const ObstacleAvoidance::ResultCallback callback)
{
    MavlinkCommandSender::CommandLong command{*_parent};

    command.command = MAV_CMD_COMPONENT_CONTROL;
    command.params.param1 = MAV_COMP_ID_OBSTACLE_AVOIDANCE;
    command.params.param2 = COMPONENT_CONTROL_STOP;
    command.target_component_id = MAV_COMP_ID_ONBOARD_COMPUTER;

    _parent->send_command_async(
        command, [this, callback](MavlinkCommandSender::Result result, float) {
            command_result_callback(result, callback);
        });
}

ObstacleAvoidance::Result ObstacleAvoidanceImpl::stop()
{
    auto prom = std::promise<ObstacleAvoidance::Result>();
    auto fut = prom.get_future();

    stop_async([&prom](ObstacleAvoidance::Result result) { prom.set_value(result); });

    return fut.get();
}

void ObstacleAvoidanceImpl::restart_async(const ObstacleAvoidance::ResultCallback callback)
{
    MavlinkCommandSender::CommandLong command{*_parent};

    command.command = MAV_CMD_COMPONENT_CONTROL;
    command.params.param1 = MAV_COMP_ID_OBSTACLE_AVOIDANCE;
    command.params.param2 = COMPONENT_CONTROL_RESTART;
    command.target_component_id = MAV_COMP_ID_ONBOARD_COMPUTER;

    _parent->send_command_async(
        command, [this, callback](MavlinkCommandSender::Result result, float) {
            command_result_callback(result, callback);
        });
}

ObstacleAvoidance::Result ObstacleAvoidanceImpl::restart()
{
    auto prom = std::promise<ObstacleAvoidance::Result>();
    auto fut = prom.get_future();

    restart_async([&prom](ObstacleAvoidance::Result result) { prom.set_value(result); });

    return fut.get();
}

void ObstacleAvoidanceImpl::state_enable_async(const ObstacleAvoidance::ResultCallback callback)
{
    MavlinkCommandSender::CommandLong command{*_parent};

    command.command = MAV_CMD_COMPONENT_CONTROL;
    command.params.param1 = MAV_COMP_ID_OBSTACLE_AVOIDANCE;
    command.params.param2 = COMPONENT_CONTROL_ENABLE;
    command.target_component_id = MAV_COMP_ID_ONBOARD_COMPUTER;

    _parent->send_command_async(
        command, [this, callback](MavlinkCommandSender::Result result, float) {
            command_result_callback(result, callback);
        });
}

ObstacleAvoidance::Result ObstacleAvoidanceImpl::state_enable()
{
    auto prom = std::promise<ObstacleAvoidance::Result>();
    auto fut = prom.get_future();

    state_enable_async([&prom](ObstacleAvoidance::Result result) { prom.set_value(result); });

    return fut.get();
}

void ObstacleAvoidanceImpl::state_disable_async(const ObstacleAvoidance::ResultCallback callback)
{
    MavlinkCommandSender::CommandLong command{*_parent};

    command.command = MAV_CMD_COMPONENT_CONTROL;
    command.params.param1 = MAV_COMP_ID_OBSTACLE_AVOIDANCE;
    command.params.param2 = COMPONENT_CONTROL_DISABLE;
    command.target_component_id = MAV_COMP_ID_ONBOARD_COMPUTER;

    _parent->send_command_async(
        command, [this, callback](MavlinkCommandSender::Result result, float) {
            command_result_callback(result, callback);
        });
}

ObstacleAvoidance::Result ObstacleAvoidanceImpl::state_disable()
{
    auto prom = std::promise<ObstacleAvoidance::Result>();
    auto fut = prom.get_future();

    state_disable_async([&prom](ObstacleAvoidance::Result result) { prom.set_value(result); });

    return fut.get();
}

ObstacleAvoidance::Result ObstacleAvoidanceImpl::obstacle_avoidance_result_from_command_result(
    MavlinkCommandSender::Result result)
{
    switch (result) {
        case MavlinkCommandSender::Result::Success:
            return ObstacleAvoidance::Result::Success;
        case MavlinkCommandSender::Result::NoSystem:
            return ObstacleAvoidance::Result::NoSystem;
        case MavlinkCommandSender::Result::ConnectionError:
            return ObstacleAvoidance::Result::ConnectionError;
        case MavlinkCommandSender::Result::Busy:
            return ObstacleAvoidance::Result::Busy;
        case MavlinkCommandSender::Result::CommandDenied:
            return ObstacleAvoidance::Result::CommandDenied;
        case MavlinkCommandSender::Result::Timeout:
            return ObstacleAvoidance::Result::Timeout;
        default:
            return ObstacleAvoidance::Result::Unknown;
    }
}

void ObstacleAvoidanceImpl::command_result_callback(
    MavlinkCommandSender::Result command_result,
    const ObstacleAvoidance::ResultCallback& callback) const
{
    ObstacleAvoidance::Result result =
        obstacle_avoidance_result_from_command_result(command_result);

    if (callback) {
        auto temp_callback = callback;
        _parent->call_user_callback([temp_callback, result]() { temp_callback(result); });
    }
}

} // namespace mavsdk
