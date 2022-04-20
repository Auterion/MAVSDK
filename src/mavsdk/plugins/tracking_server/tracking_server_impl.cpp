#include "tracking_server_impl.h"
#include <mutex>

namespace mavsdk {

TrackingServerImpl::TrackingServerImpl(System& system) : PluginImplBase(system)
{
    _parent->register_plugin(this);
}

TrackingServerImpl::TrackingServerImpl(std::shared_ptr<System> system) :
    PluginImplBase(std::move(system))
{
    _parent->register_plugin(this);
}

TrackingServerImpl::~TrackingServerImpl()
{
    _parent->unregister_plugin(this);
}

void TrackingServerImpl::init()
{
    _parent->register_mavlink_command_handler(
        MAV_CMD_CAMERA_TRACK_POINT,
        [this](const MavlinkCommandReceiver::CommandLong& command) {
            return process_track_point_command(command);
        },
        this);

    _parent->register_mavlink_command_handler(
        MAV_CMD_CAMERA_TRACK_RECTANGLE,
        [this](const MavlinkCommandReceiver::CommandLong& command) {
            return process_track_rectangle_command(command);
        },
        this);

    _parent->register_mavlink_command_handler(
        MAV_CMD_CAMERA_STOP_TRACKING,
        [this](const MavlinkCommandReceiver::CommandLong& command) {
            return process_track_off_command(command);
        },
        this);
}

void TrackingServerImpl::deinit()
{
    _parent->unregister_mavlink_command_handler(MAV_CMD_CAMERA_TRACK_POINT, this);
    _parent->unregister_mavlink_command_handler(MAV_CMD_CAMERA_TRACK_RECTANGLE, this);
    _parent->unregister_mavlink_command_handler(MAV_CMD_CAMERA_STOP_TRACKING, this);
}

void TrackingServerImpl::enable() {}

void TrackingServerImpl::disable() {}

void TrackingServerImpl::set_tracking_point_status(TrackingServer::TrackPoint tracked_point)
{
    mavlink_message_t message;
    mavlink_msg_camera_tracking_image_status_pack(
        _parent->get_own_system_id(),
        _parent->get_own_component_id(),
        &message,
        CAMERA_TRACKING_STATUS_FLAGS_ACTIVE,
        CAMERA_TRACKING_MODE_POINT,
        CAMERA_TRACKING_TARGET_DATA_IN_STATUS,
        tracked_point.point_x,
        tracked_point.point_y,
        tracked_point.radius,
        0.0f,
        0.0f,
        0.0f,
        0.0f);
    _parent->send_message(message);
}

void TrackingServerImpl::set_tracking_rectangle_status(
    TrackingServer::TrackRectangle tracked_rectangle)
{
    mavlink_message_t message;
    mavlink_msg_camera_tracking_image_status_pack(
        _parent->get_own_system_id(),
        _parent->get_own_component_id(),
        &message,
        CAMERA_TRACKING_STATUS_FLAGS_ACTIVE,
        CAMERA_TRACKING_MODE_RECTANGLE,
        CAMERA_TRACKING_TARGET_DATA_IN_STATUS,
        0.0f,
        0.0f,
        0.0f,
        tracked_rectangle.top_left_corner_x,
        tracked_rectangle.top_left_corner_y,
        tracked_rectangle.bottom_right_corner_x,
        tracked_rectangle.bottom_right_corner_y);
    _parent->send_message(message);
}

void TrackingServerImpl::set_tracking_off_status()
{
    mavlink_message_t message;
    mavlink_msg_camera_tracking_image_status_pack(
        _parent->get_own_system_id(),
        _parent->get_own_component_id(),
        &message,
        CAMERA_TRACKING_STATUS_FLAGS_IDLE,
        CAMERA_TRACKING_MODE_NONE,
        CAMERA_TRACKING_TARGET_DATA_NONE,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f);
    _parent->send_message(message);
}

void TrackingServerImpl::subscribe_tracking_point_command(
    TrackingServer::TrackingPointCommandCallback callback)
{
    std::lock_guard<std::mutex> lock(_mutex);
    _tracking_point_callback = callback;
}

void TrackingServerImpl::subscribe_tracking_rectangle_command(
    TrackingServer::TrackingRectangleCommandCallback callback)
{
    std::lock_guard<std::mutex> lock(_mutex);
    _tracking_rectangle_callback = callback;
}

void TrackingServerImpl::subscribe_tracking_off_command(
    TrackingServer::TrackingOffCommandCallback callback)
{
    std::lock_guard<std::mutex> lock(_mutex);
    _tracking_off_callback = callback;
}

TrackingServer::Result
TrackingServerImpl::respond_tracking_point_command(TrackingServer::CommandAnswer command_answer)
{
    std::lock_guard<std::mutex> lock(_mutex);

    mavlink_message_t message;
    mavlink_msg_command_ack_pack(
        _parent->get_own_system_id(),
        _parent->get_own_component_id(),
        &message,
        MAV_CMD_CAMERA_TRACK_POINT,
        mav_result_from_command_answer(command_answer),
        0,
        0,
        _tracking_point_command_sysid,
        _tracking_point_command_compid);

    return _parent->send_message(message) ? TrackingServer::Result::Success :
                                            TrackingServer::Result::ConnectionError;
}

TrackingServer::Result
TrackingServerImpl::respond_tracking_rectangle_command(TrackingServer::CommandAnswer command_answer)
{
    std::lock_guard<std::mutex> lock(_mutex);

    mavlink_message_t message;
    mavlink_msg_command_ack_pack(
        _parent->get_own_system_id(),
        _parent->get_own_component_id(),
        &message,
        MAV_CMD_CAMERA_TRACK_RECTANGLE,
        mav_result_from_command_answer(command_answer),
        0,
        0,
        _tracking_rectangle_command_sysid,
        _tracking_rectangle_command_compid);

    return _parent->send_message(message) ? TrackingServer::Result::Success :
                                            TrackingServer::Result::ConnectionError;
}

TrackingServer::Result
TrackingServerImpl::respond_tracking_off_command(TrackingServer::CommandAnswer command_answer)
{
    std::lock_guard<std::mutex> lock(_mutex);

    mavlink_message_t message;
    mavlink_msg_command_ack_pack(
        _parent->get_own_system_id(),
        _parent->get_own_component_id(),
        &message,
        MAV_CMD_CAMERA_STOP_TRACKING,
        mav_result_from_command_answer(command_answer),
        0,
        0,
        _tracking_off_command_sysid,
        _tracking_off_command_compid);

    return _parent->send_message(message) ? TrackingServer::Result::Success :
                                            TrackingServer::Result::ConnectionError;
}

std::optional<mavlink_message_t>
TrackingServerImpl::process_track_point_command(const MavlinkCommandReceiver::CommandLong& command)
{
    if (!is_command_sender_ok(command)) {
        LogWarn() << "Incoming track point command is for target sysid "
                  << int(command.target_system_id) << " instead of "
                  << int(_parent->get_own_system_id());
        return std::nullopt;
    }

    TrackingServer::TrackPoint track_point{
        command.params.param1, command.params.param2, command.params.param3};

    std::lock_guard<std::mutex> lock(_mutex);
    _tracking_point_command_sysid = command.origin_system_id;
    _tracking_point_command_compid = command.origin_component_id;

    auto temp_callback = _tracking_point_callback;

    _parent->call_user_callback([temp_callback, track_point]() { temp_callback(track_point); });

    // We don't send an ack but leave that to the user.
    return std::nullopt;
}

std::optional<mavlink_message_t> TrackingServerImpl::process_track_rectangle_command(
    const MavlinkCommandReceiver::CommandLong& command)
{
    if (!is_command_sender_ok(command)) {
        LogWarn() << "Incoming track rectangle command is for target sysid "
                  << int(command.target_system_id) << " instead of "
                  << int(_parent->get_own_system_id());
        return std::nullopt;
    }

    TrackingServer::TrackRectangle track_rectangle{
        command.params.param1, command.params.param2, command.params.param3, command.params.param4};

    std::lock_guard<std::mutex> lock(_mutex);
    _tracking_rectangle_command_sysid = command.origin_system_id;
    _tracking_rectangle_command_compid = command.origin_component_id;

    auto temp_callback = _tracking_rectangle_callback;

    _parent->call_user_callback(
        [temp_callback, track_rectangle]() { temp_callback(track_rectangle); });

    // We don't send an ack but leave that to the user.
    return std::nullopt;
}

std::optional<mavlink_message_t>
TrackingServerImpl::process_track_off_command(const MavlinkCommandReceiver::CommandLong& command)
{
    if (!is_command_sender_ok(command)) {
        LogWarn() << "Incoming track off command is for target sysid "
                  << int(command.target_system_id) << " instead of "
                  << int(_parent->get_own_system_id());
        return std::nullopt;
    }

    std::lock_guard<std::mutex> lock(_mutex);
    _tracking_off_command_sysid = command.origin_system_id;
    _tracking_off_command_compid = command.origin_component_id;

    auto temp_callback = _tracking_off_callback;

    _parent->call_user_callback([temp_callback]() { temp_callback(0); });

    // We don't send an ack but leave that to the user.
    return std::nullopt;
}

bool TrackingServerImpl::is_command_sender_ok(const MavlinkCommandReceiver::CommandLong& command)
{
    if (command.target_system_id != 0 && command.target_system_id != _parent->get_own_system_id()) {
        return false;
    } else {
        return true;
    }
}

MAV_RESULT
TrackingServerImpl::mav_result_from_command_answer(TrackingServer::CommandAnswer command_answer)
{
    switch (command_answer) {
        case TrackingServer::CommandAnswer::Accepted:
            return MAV_RESULT_ACCEPTED;
        case TrackingServer::CommandAnswer::TemporarilyRejected:
            return MAV_RESULT_TEMPORARILY_REJECTED;
        case TrackingServer::CommandAnswer::Denied:
            return MAV_RESULT_DENIED;
        case TrackingServer::CommandAnswer::Unsupported:
            return MAV_RESULT_UNSUPPORTED;
        case TrackingServer::CommandAnswer::Failed:
            return MAV_RESULT_FAILED;
    }

    LogErr() << "Unknown CommandAnswer";
    return MAV_RESULT_FAILED;
}

} // namespace mavsdk
