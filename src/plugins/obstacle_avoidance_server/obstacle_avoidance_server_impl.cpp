#include "obstacle_avoidance_server_impl.h"

namespace mavsdk {

ObstacleAvoidanceServerImpl::ObstacleAvoidanceServerImpl(System& system) : PluginImplBase(system)
{
    _parent->register_plugin(this);
}

ObstacleAvoidanceServerImpl::ObstacleAvoidanceServerImpl(std::shared_ptr<System> system) :
    PluginImplBase(system)
{
    _parent->register_plugin(this);
}

ObstacleAvoidanceServerImpl::~ObstacleAvoidanceServerImpl()
{
    _parent->unregister_plugin(this);
}

void ObstacleAvoidanceServerImpl::init() {
    using namespace std::placeholders;

    _parent->register_mavlink_command_handler(
        MAV_CMD_COMPONENT_CONTROL,
        std::bind(&ObstacleAvoidanceServerImpl::process_component_control_command, this, _1),
        this);
}

void ObstacleAvoidanceServerImpl::deinit() {
    _parent->unregister_all_mavlink_command_handlers(this);
}

void ObstacleAvoidanceServerImpl::enable() {}

void ObstacleAvoidanceServerImpl::disable() {}

mavlink_message_t ObstacleAvoidanceServerImpl::process_component_control_command(
    const MavlinkCommandReceiver::CommandLong& command)
{
    mavlink_message_t command_ack;

    if (command.params.param1 == MAV_COMP_ID_OBSTACLE_AVOIDANCE) {
        ObstacleAvoidanceServer::ControlType control_type;
        control_type.control_type =
            static_cast<ObstacleAvoidanceServer::ControlType::Type>(command.params.param2);

        store_control(control_type);

        std::lock_guard<std::mutex> lock(_subscription_mutex);
        if (_component_control_command_subscription) {
            auto callback = _component_control_command_subscription;
            auto arg1 = control();

            _parent->call_user_callback([callback, arg1]() { callback(arg1); });

            // Send command accepted ACK
            mavlink_msg_command_ack_pack(
                _parent->get_own_system_id(),
                _parent->get_own_component_id(),
                &command_ack,
                MAV_CMD_COMPONENT_CONTROL,
                MAV_RESULT_ACCEPTED,
                0,
                0,
                0,
                0);
        }
    } else {
        // Send command denied ACK
        mavlink_msg_command_ack_pack(
            _parent->get_own_system_id(),
            _parent->get_own_component_id(),
            &command_ack,
            MAV_CMD_COMPONENT_CONTROL,
            MAV_RESULT_DENIED,
            0,
            0,
            0,
            0);
    }

    // The COMMAND_ACK is sent as a result to the callback so to be processed and
    // sent on the server side.
    return command_ack;
}

void ObstacleAvoidanceServerImpl::store_control(ObstacleAvoidanceServer::ControlType control)
{
    std::lock_guard<std::mutex> lock(_component_control_command_mutex);
    _control = control;
}

ObstacleAvoidanceServer::ControlType ObstacleAvoidanceServerImpl::control() const
{
    std::lock_guard<std::mutex> lock(_component_control_command_mutex);
    return _control;
}

void ObstacleAvoidanceServerImpl::control_async(ObstacleAvoidanceServer::ControlCallback callback)
{
    std::lock_guard<std::mutex> lock(_subscription_mutex);
    _component_control_command_subscription = callback;
}

} // namespace mavsdk
