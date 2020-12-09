#include "mission_impl.h"
#include "system.h"
#include "global_include.h"
#include <algorithm>
#include <fstream> // for `std::ifstream`
#include <sstream> // for `std::stringstream`
#include <cmath>

namespace mavsdk {

using namespace std::placeholders; // for `_1`
using MissionItem = Mission::MissionItem;
using CameraAction = Mission::MissionItem::CameraAction;

MissionImpl::MissionImpl(System& system) : PluginImplBase(system)
{
    _parent->register_plugin(this);
}

MissionImpl::MissionImpl(std::shared_ptr<System> system) : PluginImplBase(system)
{
    _parent->register_plugin(this);
}

MissionImpl::MissionImpl(SystemImpl* system_impl) : PluginImplBase(system_impl)
{
    _parent->register_plugin(this);
}

MissionImpl::~MissionImpl()
{
    _parent->unregister_plugin(this);
}

void MissionImpl::init()
{
    _parent->register_mavlink_message_handler(
        MAVLINK_MSG_ID_MISSION_CURRENT,
        std::bind(&MissionImpl::process_mission_current, this, _1),
        this);

    _parent->register_mavlink_message_handler(
        MAVLINK_MSG_ID_MISSION_ITEM_REACHED,
        std::bind(&MissionImpl::process_mission_item_reached, this, _1),
        this);

    _parent->register_mavlink_message_handler(
        MAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION,
        [this](const mavlink_message_t& message) { process_gimbal_manager_information(message); },
        this);
}

void MissionImpl::enable()
{
    _parent->register_timeout_handler(
        [this]() { receive_protocol_timeout(); }, 1.0, &_gimbal_protocol_cookie);

    MavlinkCommandSender::CommandLong command{};
    command.command = MAV_CMD_REQUEST_MESSAGE;
    command.params.param1 = static_cast<float>(MAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION);
    command.target_component_id = 0; // any component
    _parent->send_command_async(command, nullptr);
}

void MissionImpl::disable()
{
    _gimbal_protocol = GimbalProtocol::Unknown;
}

void MissionImpl::deinit()
{
    _parent->unregister_timeout_handler(_gimbal_protocol_cookie);
    _parent->unregister_timeout_handler(_timeout_cookie);
    _parent->unregister_all_mavlink_message_handlers(this);
}

void MissionImpl::reset_mission_progress()
{
    std::lock_guard<std::recursive_mutex> lock(_mission_data.mutex);
    _mission_data.last_current_mavlink_mission_item = -1;
    _mission_data.last_reached_mavlink_mission_item = -1;
    _mission_data.last_current_reported_mission_item = -1;
    _mission_data.last_total_reported_mission_item = -1;
}

void MissionImpl::process_mission_current(const mavlink_message_t& message)
{
    mavlink_mission_current_t mission_current;
    mavlink_msg_mission_current_decode(&message, &mission_current);

    {
        std::lock_guard<std::recursive_mutex> lock(_mission_data.mutex);
        _mission_data.last_current_mavlink_mission_item = mission_current.seq;
    }

    report_progress();
}

void MissionImpl::process_mission_item_reached(const mavlink_message_t& message)
{
    mavlink_mission_item_reached_t mission_item_reached;
    mavlink_msg_mission_item_reached_decode(&message, &mission_item_reached);

    {
        std::lock_guard<std::recursive_mutex> lock(_mission_data.mutex);
        _mission_data.last_reached_mavlink_mission_item = mission_item_reached.seq;
    }

    report_progress();
}

void MissionImpl::process_gimbal_manager_information(const mavlink_message_t& message)
{
    UNUSED(message);
    LogDebug() << "Using gimbal protocol v2";
    _gimbal_protocol = GimbalProtocol::V2;
    _parent->unregister_timeout_handler(_gimbal_protocol_cookie);
}

void MissionImpl::wait_for_protocol()
{
    while (_gimbal_protocol == GimbalProtocol::Unknown) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void MissionImpl::wait_for_protocol_async(std::function<void()> callback)
{
    wait_for_protocol();
    callback();
}

void MissionImpl::receive_protocol_timeout()
{
    LogDebug() << "Falling back to gimbal protocol v1";
    _gimbal_protocol = GimbalProtocol::V1;
}

Mission::Result MissionImpl::upload_mission(const Mission::MissionPlan& mission_plan)
{
    auto prom = std::promise<Mission::Result>();
    auto fut = prom.get_future();

    upload_mission_async(mission_plan, [&prom](Mission::Result result) { prom.set_value(result); });
    return fut.get();
}

void MissionImpl::upload_mission_async(
    const Mission::MissionPlan& mission_plan, const Mission::ResultCallback& callback)
{
    if (_mission_data.last_upload.lock()) {
        _parent->call_user_callback([callback]() {
            if (callback) {
                callback(Mission::Result::Busy);
            }
        });
        return;
    }

    if (!_parent->does_support_mission_int()) {
        LogWarn() << "Mission int messages not supported";
        // report_mission_result(callback, Mission::Result::Error);
        return;
    }

    wait_for_protocol_async([callback, mission_plan, this]() {
        const auto int_items = convert_to_int_items(mission_plan.mission_items);

        _mission_data.last_upload = _parent->mission_transfer().upload_items_async(
            MAV_MISSION_TYPE_MISSION,
            int_items,
            [this, callback](MAVLinkMissionTransfer::Result result) {
                auto converted_result = convert_result(result);
                _parent->call_user_callback([callback, converted_result]() {
                    if (callback) {
                        callback(converted_result);
                    }
                });
            });
    });
}

Mission::Result MissionImpl::cancel_mission_upload()
{
    auto ptr = _mission_data.last_upload.lock();
    if (ptr) {
        ptr->cancel();
        return Mission::Result::Success;
    } else {
        return Mission::Result::Error;
    }
}

std::pair<Mission::Result, Mission::MissionPlan> MissionImpl::download_mission()
{
    auto prom = std::promise<std::pair<Mission::Result, Mission::MissionPlan>>();
    auto fut = prom.get_future();

    download_mission_async([&prom](Mission::Result result, Mission::MissionPlan mission_plan) {
        prom.set_value(std::make_pair<>(result, mission_plan));
    });
    return fut.get();
}

void MissionImpl::download_mission_async(const Mission::DownloadMissionCallback& callback)
{
    if (_mission_data.last_download.lock()) {
        _parent->call_user_callback([callback]() {
            if (callback) {
                Mission::MissionPlan mission_plan{};
                callback(Mission::Result::Busy, mission_plan);
            }
        });
        return;
    }

    _mission_data.last_download = _parent->mission_transfer().download_items_async(
        MAV_MISSION_TYPE_MISSION,
        [this, callback](
            MAVLinkMissionTransfer::Result result,
            std::vector<MAVLinkMissionTransfer::ItemInt> items) {
            auto result_and_items = convert_to_result_and_mission_items(result, items);
            _parent->call_user_callback([callback, result_and_items]() {
                callback(result_and_items.first, result_and_items.second);
            });
        });
}

Mission::Result MissionImpl::cancel_mission_download()
{
    auto ptr = _mission_data.last_download.lock();
    if (ptr) {
        ptr->cancel();
        return Mission::Result::Success;
    } else {
        return Mission::Result::Error;
    }
}

Mission::Result MissionImpl::set_return_to_launch_after_mission(bool enable_rtl)
{
    _enable_return_to_launch_after_mission = enable_rtl;
    return Mission::Result::Success;
}

std::pair<Mission::Result, bool> MissionImpl::get_return_to_launch_after_mission()
{
    return std::make_pair<>(Mission::Result::Success, _enable_return_to_launch_after_mission);
}

bool MissionImpl::has_valid_position(const MissionItem& item)
{
    return std::isfinite(item.latitude_deg) && std::isfinite(item.longitude_deg) &&
           std::isfinite(item.relative_altitude_m);
}

float MissionImpl::hold_time(const MissionItem& item)
{
    float hold_time_s;
    if (item.is_fly_through) {
        hold_time_s = 0.0f;
    } else {
        hold_time_s = 0.5f;
    }

    return hold_time_s;
}

float MissionImpl::acceptance_radius(const MissionItem& item)
{
    float acceptance_radius_m;
    if (item.is_fly_through) {
        // _acceptance_radius_m is 0, determine the radius using fly_through
        acceptance_radius_m = 3.0f;
    } else {
        // _acceptance_radius_m is 0, determine the radius using fly_through
        acceptance_radius_m = 1.0f;
    }

    return acceptance_radius_m;
}

std::vector<MAVLinkMissionTransfer::ItemInt>
MissionImpl::convert_to_int_items(const std::vector<MissionItem>& mission_items)
{
    std::vector<MAVLinkMissionTransfer::ItemInt> int_items;

    bool last_position_valid = false; // This flag is to protect us from using an invalid x/y.

    unsigned item_i = 0;
    _mission_data.mavlink_mission_item_to_mission_item_indices.clear();

    for (const auto& item : mission_items) {
        if (has_valid_position(item)) {
            // Current is the 0th waypoint
            const uint8_t current = ((int_items.size() == 0) ? 1 : 0);

            const int32_t x = int32_t(std::round(item.latitude_deg * 1e7));
            const int32_t y = int32_t(std::round(item.longitude_deg * 1e7));
            const float z = item.relative_altitude_m;

            MAVLinkMissionTransfer::ItemInt next_item{
                static_cast<uint16_t>(int_items.size()),
                static_cast<uint8_t>(MAV_FRAME_GLOBAL_RELATIVE_ALT_INT),
                static_cast<uint8_t>(MAV_CMD_NAV_WAYPOINT),
                current,
                1, // autocontinue
                hold_time(item),
                acceptance_radius(item),
                0.0f,
                NAN,
                x,
                y,
                z,
                MAV_MISSION_TYPE_MISSION};

            last_position_valid = true; // because we checked has_valid_position

            _mission_data.mavlink_mission_item_to_mission_item_indices.push_back(item_i);
            int_items.push_back(next_item);
        }

        if (std::isfinite(item.speed_m_s)) {
            // The speed has changed, we need to add a speed command.

            // Current is the 0th waypoint
            uint8_t current = ((int_items.size() == 0) ? 1 : 0);

            uint8_t autocontinue = 1;

            MAVLinkMissionTransfer::ItemInt next_item{
                static_cast<uint16_t>(int_items.size()),
                MAV_FRAME_MISSION,
                MAV_CMD_DO_CHANGE_SPEED,
                current,
                autocontinue,
                1.0f, // ground speed
                item.speed_m_s,
                -1.0f, // no throttle change
                0.0f, // absolute
                0,
                0,
                NAN,
                MAV_MISSION_TYPE_MISSION};

            _mission_data.mavlink_mission_item_to_mission_item_indices.push_back(item_i);
            int_items.push_back(next_item);
        }

        if (std::isfinite(item.gimbal_yaw_deg) || std::isfinite(item.gimbal_pitch_deg)) {
            const auto temp_gimbal_protocol = _gimbal_protocol.load();
            switch (temp_gimbal_protocol) {
                case GimbalProtocol::V1:
                    add_gimbal_items_v1(
                        int_items, item_i, item.gimbal_pitch_deg, item.gimbal_yaw_deg);
                    break;

                case GimbalProtocol::V2:
                    add_gimbal_items_v2(
                        int_items, item_i, item.gimbal_pitch_deg, item.gimbal_yaw_deg);
                    break;
                case GimbalProtocol::Unknown:
                    // This should not happen because we wait until we know the protocol version.
                    LogErr() << "Unknown gimbal protocol, skipping gimbal commands.";
                    break;
            }
        }

        // A loiter time of NAN is ignored but also a loiter time of 0 doesn't
        // make any sense and should be discarded.
        if (std::isfinite(item.loiter_time_s) && item.loiter_time_s > 0.0f) {
            if (!last_position_valid) {
                // In the case where we get a delay without a previous position, we will have to
                // ignore it.
                LogErr() << "Can't set camera action delay without previous position set.";

            } else {
                // Current is the 0th waypoint
                uint8_t current = ((int_items.size() == 0) ? 1 : 0);

                uint8_t autocontinue = 1;

                MAVLinkMissionTransfer::ItemInt next_item{
                    static_cast<uint16_t>(int_items.size()),
                    MAV_FRAME_MISSION,
                    MAV_CMD_NAV_DELAY,
                    current,
                    autocontinue,
                    item.loiter_time_s, // loiter time in seconds
                    -1, // no specified hour
                    -1, // no specified minute
                    -1, // no specified second
                    0,
                    0,
                    0,
                    MAV_MISSION_TYPE_MISSION};

                _mission_data.mavlink_mission_item_to_mission_item_indices.push_back(item_i);
                int_items.push_back(next_item);
            }

            if (item.is_fly_through) {
                LogWarn() << "Conflicting options set: fly_through=true and loiter_time>0.";
            }
        }

        if (item.camera_action != CameraAction::None) {
            // There is a camera action that we need to send.

            // Current is the 0th waypoint
            uint8_t current = ((int_items.size() == 0) ? 1 : 0);

            uint8_t autocontinue = 1;

            uint16_t command = 0;
            float param1 = NAN;
            float param2 = NAN;
            float param3 = NAN;
            switch (item.camera_action) {
                case CameraAction::TakePhoto:
                    command = MAV_CMD_IMAGE_START_CAPTURE;
                    param1 = 0.0f; // all camera IDs
                    param2 = 0.0f; // no duration, take only one picture
                    param3 = 1.0f; // only take one picture
                    break;
                case CameraAction::StartPhotoInterval:
                    command = MAV_CMD_IMAGE_START_CAPTURE;
                    param1 = 0.0f; // all camera IDs
                    param2 = item.camera_photo_interval_s;
                    param3 = 0.0f; // unlimited photos
                    break;
                case CameraAction::StopPhotoInterval:
                    command = MAV_CMD_IMAGE_STOP_CAPTURE;
                    param1 = 0.0f; // all camera IDs
                    break;
                case CameraAction::StartVideo:
                    command = MAV_CMD_VIDEO_START_CAPTURE;
                    param1 = 0.0f; // all camera IDs
                    break;
                case CameraAction::StopVideo:
                    command = MAV_CMD_VIDEO_STOP_CAPTURE;
                    param1 = 0.0f; // all camera IDs
                    break;
                default:
                    LogErr() << "Error: camera action not supported";
                    break;
            }

            MAVLinkMissionTransfer::ItemInt next_item{
                static_cast<uint16_t>(int_items.size()),
                MAV_FRAME_MISSION,
                command,
                current,
                autocontinue,
                param1,
                param2,
                param3,
                NAN,
                0,
                0,
                NAN,
                MAV_MISSION_TYPE_MISSION};

            _mission_data.mavlink_mission_item_to_mission_item_indices.push_back(item_i);
            int_items.push_back(next_item);
        }

        ++item_i;
    }

    // We need to decrement the item_i again because it was increased in the loop above
    // but the RTL item below still belongs to the last mission item.
    --item_i;

    if (_enable_return_to_launch_after_mission) {
        MAVLinkMissionTransfer::ItemInt next_item{
            static_cast<uint16_t>(int_items.size()),
            MAV_FRAME_MISSION,
            MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0, // current
            1, // autocontinue
            NAN, // loiter time in seconds
            NAN, // empty
            NAN, // radius around waypoint in meters ?
            NAN, // loiter at center of waypoint
            0,
            0,
            0,
            MAV_MISSION_TYPE_MISSION};

        _mission_data.mavlink_mission_item_to_mission_item_indices.push_back(item_i);
        int_items.push_back(next_item);
    }
    return int_items;
}

std::pair<Mission::Result, Mission::MissionPlan> MissionImpl::convert_to_result_and_mission_items(
    MAVLinkMissionTransfer::Result result,
    const std::vector<MAVLinkMissionTransfer::ItemInt>& int_items)
{
    std::pair<Mission::Result, Mission::MissionPlan> result_pair;

    result_pair.first = convert_result(result);
    if (result_pair.first != Mission::Result::Success) {
        return result_pair;
    }

    _mission_data.mavlink_mission_item_to_mission_item_indices.clear();

    Mission::DownloadMissionCallback callback;
    {
        _enable_return_to_launch_after_mission = false;

        MissionItem new_mission_item{};
        bool have_set_position = false;

        for (const auto& int_item : int_items) {
            LogDebug() << "Assembling Message: " << int(int_item.seq);

            if (int_item.command == MAV_CMD_NAV_WAYPOINT) {
                if (int_item.frame != MAV_FRAME_GLOBAL_RELATIVE_ALT_INT) {
                    LogErr() << "Waypoint frame not supported unsupported";
                    result_pair.first = Mission::Result::Unsupported;
                    break;
                }

                if (have_set_position) {
                    // When a new position comes in, create next mission item.
                    result_pair.second.mission_items.push_back(new_mission_item);
                    new_mission_item = {};
                    have_set_position = false;
                }

                new_mission_item.latitude_deg = double(int_item.x) * 1e-7;
                new_mission_item.longitude_deg = double(int_item.y) * 1e-7;
                new_mission_item.relative_altitude_m = int_item.z;

                new_mission_item.is_fly_through = !(int_item.param1 > 0);

                have_set_position = true;

            } else if (int_item.command == MAV_CMD_DO_MOUNT_CONTROL) {
                if (int(int_item.z) != MAV_MOUNT_MODE_MAVLINK_TARGETING) {
                    LogErr() << "Gimbal mount control mode unsupported";
                    result_pair.first = Mission::Result::Unsupported;
                    break;
                }

                new_mission_item.gimbal_pitch_deg = int_item.param1;
                new_mission_item.gimbal_yaw_deg = int_item.param3;

            } else if (int_item.command == MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW) {
                if (int_item.x !=
                    (GIMBAL_MANAGER_FLAGS_ROLL_LOCK | GIMBAL_MANAGER_FLAGS_PITCH_LOCK)) {
                    LogErr() << "Gimbal do pitchyaw flags unsupported";
                    result_pair.first = Mission::Result::Unsupported;
                    break;
                }

                new_mission_item.gimbal_pitch_deg = to_deg_from_rad(int_item.param1);
                new_mission_item.gimbal_yaw_deg = to_deg_from_rad(int_item.param2);

            } else if (int_item.command == MAV_CMD_DO_MOUNT_CONFIGURE) {
                if (int(int_item.param1) != MAV_MOUNT_MODE_MAVLINK_TARGETING) {
                    LogErr() << "Gimbal mount configure mode unsupported";
                    result_pair.first = Mission::Result::Unsupported;
                    break;
                }

                // FIXME: ultimately param4 doesn't count anymore and
                //        param7 holds the truth.
                if (int(int_item.param4) == 1 || int(int_item.z) == 2) {
                    _enable_absolute_gimbal_yaw_angle = true;
                } else {
                    _enable_absolute_gimbal_yaw_angle = false;
                }

            } else if (int_item.command == MAV_CMD_IMAGE_START_CAPTURE) {
                if (int_item.param2 > 0 && int(int_item.param3) == 0) {
                    new_mission_item.camera_action = CameraAction::StartPhotoInterval;
                    new_mission_item.camera_photo_interval_s = double(int_item.param2);
                } else if (int(int_item.param2) == 0 && int(int_item.param3) == 1) {
                    new_mission_item.camera_action = CameraAction::TakePhoto;
                } else {
                    LogErr() << "Mission item START_CAPTURE params unsupported.";
                    result_pair.first = Mission::Result::Unsupported;
                    break;
                }

            } else if (int_item.command == MAV_CMD_IMAGE_STOP_CAPTURE) {
                new_mission_item.camera_action = CameraAction::StopPhotoInterval;

            } else if (int_item.command == MAV_CMD_VIDEO_START_CAPTURE) {
                new_mission_item.camera_action = CameraAction::StartVideo;

            } else if (int_item.command == MAV_CMD_VIDEO_STOP_CAPTURE) {
                new_mission_item.camera_action = CameraAction::StopVideo;

            } else if (int_item.command == MAV_CMD_DO_CHANGE_SPEED) {
                if (int(int_item.param1) == 1 && int_item.param3 < 0 && int(int_item.param4) == 0) {
                    new_mission_item.speed_m_s = int_item.param2;
                } else {
                    LogErr() << "Mission item DO_CHANGE_SPEED params unsupported";
                    result_pair.first = Mission::Result::Unsupported;
                }

            } else if (int_item.command == MAV_CMD_NAV_LOITER_TIME) {
                // MAVSDK doesn't use LOITER_TIME anymore, but it is possible
                // a mission still uses it
                new_mission_item.loiter_time_s = int_item.param1;

            } else if (int_item.command == MAV_CMD_NAV_DELAY) {
                if (int_item.param1 != -1) {
                    // use delay in seconds directly
                    new_mission_item.loiter_time_s = int_item.param1;
                } else {
                    // TODO: we should support this by converting
                    // time of day data to delay in seconds
                    // leaving it out for now because a portable implementation
                    // is not trivial
                    LogErr() << "Mission item NAV_DELAY params unsupported";
                    result_pair.first = Mission::Result::Unsupported;
                }

            } else if (int_item.command == MAV_CMD_NAV_RETURN_TO_LAUNCH) {
                _enable_return_to_launch_after_mission = true;

            } else {
                LogErr() << "UNSUPPORTED mission item command (" << int_item.command << ")";
                result_pair.first = Mission::Result::Unsupported;
                break;
            }

            _mission_data.mavlink_mission_item_to_mission_item_indices.push_back(
                result_pair.second.mission_items.size());
        }

        // Don't forget to add last mission item.
        result_pair.second.mission_items.push_back(new_mission_item);
    }
    return result_pair;
}

Mission::Result MissionImpl::start_mission()
{
    auto prom = std::promise<Mission::Result>();
    auto fut = prom.get_future();

    start_mission_async([&prom](Mission::Result result) { prom.set_value(result); });
    return fut.get();
}

void MissionImpl::start_mission_async(const Mission::ResultCallback& callback)
{
    _parent->set_flight_mode_async(
        SystemImpl::FlightMode::Mission,
        [this, callback](MavlinkCommandSender::Result result, float) {
            report_flight_mode_change(callback, result);
        });
}

Mission::Result MissionImpl::pause_mission()
{
    auto prom = std::promise<Mission::Result>();
    auto fut = prom.get_future();

    pause_mission_async([&prom](Mission::Result result) { prom.set_value(result); });
    return fut.get();
}

void MissionImpl::pause_mission_async(const Mission::ResultCallback& callback)
{
    _parent->set_flight_mode_async(
        SystemImpl::FlightMode::Hold, [this, callback](MavlinkCommandSender::Result result, float) {
            report_flight_mode_change(callback, result);
        });
}

void MissionImpl::report_flight_mode_change(
    Mission::ResultCallback callback, MavlinkCommandSender::Result result)
{
    if (!callback) {
        return;
    }

    _parent->call_user_callback(
        [callback, result]() { callback(command_result_to_mission_result(result)); });
}

Mission::Result MissionImpl::command_result_to_mission_result(MavlinkCommandSender::Result result)
{
    switch (result) {
        case MavlinkCommandSender::Result::Success:
            return Mission::Result::Success;
        case MavlinkCommandSender::Result::NoSystem:
            return Mission::Result::Error; // FIXME
        case MavlinkCommandSender::Result::ConnectionError:
            return Mission::Result::Error; // FIXME
        case MavlinkCommandSender::Result::Busy:
            return Mission::Result::Busy;
        case MavlinkCommandSender::Result::CommandDenied:
            return Mission::Result::Error; // FIXME
        case MavlinkCommandSender::Result::Timeout:
            return Mission::Result::Timeout;
        case MavlinkCommandSender::Result::InProgress:
            return Mission::Result::Busy; // FIXME
        case MavlinkCommandSender::Result::UnknownError:
            return Mission::Result::Unknown;
        default:
            return Mission::Result::Unknown;
    }
}

Mission::Result MissionImpl::clear_mission()
{
    auto prom = std::promise<Mission::Result>();
    auto fut = prom.get_future();

    clear_mission_async([&prom](Mission::Result result) { prom.set_value(result); });
    return fut.get();
}

void MissionImpl::clear_mission_async(const Mission::ResultCallback& callback)
{
    _parent->mission_transfer().clear_items_async(
        MAV_MISSION_TYPE_MISSION, [this, callback](MAVLinkMissionTransfer::Result result) {
            auto converted_result = convert_result(result);
            _parent->call_user_callback([callback, converted_result]() {
                if (callback) {
                    callback(converted_result);
                }
            });
        });
}

Mission::Result MissionImpl::set_current_mission_item(int current)
{
    auto prom = std::promise<Mission::Result>();
    auto fut = prom.get_future();

    set_current_mission_item_async(
        current, [&prom](Mission::Result result) { prom.set_value(result); });
    return fut.get();
}

void MissionImpl::set_current_mission_item_async(
    int current, const Mission::ResultCallback& callback)
{
    int mavlink_index = -1;
    {
        std::lock_guard<std::recursive_mutex> lock(_mission_data.mutex);
        // We need to find the first mavlink item which maps to the current mission item.
        int i = 0;
        for (auto index : _mission_data.mavlink_mission_item_to_mission_item_indices) {
            if (index == current) {
                mavlink_index = i;
                break;
            }
            ++i;
        }
    }

    // If we don't have _mission_data cached from an upload or download,
    // we have to complain. The exception is current set to 0 because it
    // means to reset to the beginning.

    if (mavlink_index == -1 && current != 0) {
        _parent->call_user_callback([callback]() {
            if (callback) {
                // FIXME: come up with better error code.
                callback(Mission::Result::InvalidArgument);
                return;
            }
        });
    }

    _parent->mission_transfer().set_current_item_async(
        mavlink_index, [this, callback](MAVLinkMissionTransfer::Result result) {
            auto converted_result = convert_result(result);
            _parent->call_user_callback([callback, converted_result]() {
                if (callback) {
                    callback(converted_result);
                }
            });
        });
}

void MissionImpl::report_progress()
{
    const auto temp_callback = _mission_data.mission_progress_callback;
    if (temp_callback == nullptr) {
        return;
    }

    int current = current_mission_item();
    int total = total_mission_items();

    bool should_report = false;
    {
        std::lock_guard<std::recursive_mutex> lock(_mission_data.mutex);
        if (_mission_data.last_current_reported_mission_item != current) {
            _mission_data.last_current_reported_mission_item = current;
            should_report = true;
        }
        if (_mission_data.last_total_reported_mission_item != total) {
            _mission_data.last_total_reported_mission_item = total;
            should_report = true;
        }
    }

    if (should_report) {
        std::lock_guard<std::recursive_mutex> lock(_mission_data.mutex);
        _parent->call_user_callback([temp_callback, current, total]() {
            LogDebug() << "current: " << current << ", total: " << total;
            Mission::MissionProgress mission_progress;
            mission_progress.current = current;
            mission_progress.total = total;
            temp_callback(mission_progress);
        });
    }
}

std::pair<Mission::Result, bool> MissionImpl::is_mission_finished() const
{
    std::lock_guard<std::recursive_mutex> lock(_mission_data.mutex);

    if (_mission_data.last_current_mavlink_mission_item < 0) {
        return std::make_pair<Mission::Result, bool>(Mission::Result::Success, false);
    }

    if (_mission_data.last_reached_mavlink_mission_item < 0) {
        return std::make_pair<Mission::Result, bool>(Mission::Result::Success, false);
    }

    if (_mission_data.mavlink_mission_item_to_mission_item_indices.size() == 0) {
        return std::make_pair<Mission::Result, bool>(Mission::Result::Success, false);
    }

    // It is not straightforward to look at "current" because it jumps to 0
    // once the last item has been done. Therefore we have to lo decide using
    // "reached" here.
    // It seems that we never receive a reached when RTL is initiated after
    // a mission, and we need to account for that.
    const unsigned rtl_correction = _enable_return_to_launch_after_mission ? 2 : 1;

    return std::make_pair<Mission::Result, bool>(
        Mission::Result::Success,
        unsigned(_mission_data.last_reached_mavlink_mission_item + rtl_correction) ==
            _mission_data.mavlink_mission_item_to_mission_item_indices.size());
}

int MissionImpl::current_mission_item() const
{
    // If the mission is finished, let's return the total as the current
    // to signal this.
    if (is_mission_finished().second) {
        return total_mission_items();
    }

    std::lock_guard<std::recursive_mutex> lock(_mission_data.mutex);

    // We want to return the current mission item and not the underlying
    // mavlink mission item.
    if (_mission_data.last_current_mavlink_mission_item >=
            static_cast<int>(_mission_data.mavlink_mission_item_to_mission_item_indices.size()) ||
        _mission_data.last_current_mavlink_mission_item < 0) {
        return -1;
    }

    return _mission_data.mavlink_mission_item_to_mission_item_indices[static_cast<unsigned>(
        _mission_data.last_current_mavlink_mission_item)];
}

int MissionImpl::total_mission_items() const
{
    std::lock_guard<std::recursive_mutex> lock(_mission_data.mutex);
    if (_mission_data.mavlink_mission_item_to_mission_item_indices.size() == 0) {
        return 0;
    }
    return _mission_data.mavlink_mission_item_to_mission_item_indices.back() + 1;
}

Mission::MissionProgress MissionImpl::mission_progress()
{
    Mission::MissionProgress mission_progress;
    mission_progress.current = current_mission_item();
    mission_progress.total = total_mission_items();

    return mission_progress;
}

void MissionImpl::mission_progress_async(Mission::MissionProgressCallback callback)
{
    std::lock_guard<std::recursive_mutex> lock(_mission_data.mutex);
    _mission_data.mission_progress_callback = callback;
}

Mission::Result MissionImpl::convert_result(MAVLinkMissionTransfer::Result result)
{
    switch (result) {
        case MAVLinkMissionTransfer::Result::Success:
            return Mission::Result::Success;
        case MAVLinkMissionTransfer::Result::ConnectionError:
            return Mission::Result::Error; // FIXME
        case MAVLinkMissionTransfer::Result::Denied:
            return Mission::Result::Error; // FIXME
        case MAVLinkMissionTransfer::Result::TooManyMissionItems:
            return Mission::Result::TooManyMissionItems;
        case MAVLinkMissionTransfer::Result::Timeout:
            return Mission::Result::Timeout;
        case MAVLinkMissionTransfer::Result::Unsupported:
            return Mission::Result::Unsupported;
        case MAVLinkMissionTransfer::Result::UnsupportedFrame:
            return Mission::Result::Unsupported;
        case MAVLinkMissionTransfer::Result::NoMissionAvailable:
            return Mission::Result::NoMissionAvailable;
        case MAVLinkMissionTransfer::Result::Cancelled:
            return Mission::Result::TransferCancelled;
        case MAVLinkMissionTransfer::Result::MissionTypeNotConsistent:
            return Mission::Result::InvalidArgument; // FIXME
        case MAVLinkMissionTransfer::Result::InvalidSequence:
            return Mission::Result::InvalidArgument; // FIXME
        case MAVLinkMissionTransfer::Result::CurrentInvalid:
            return Mission::Result::InvalidArgument; // FIXME
        case MAVLinkMissionTransfer::Result::ProtocolError:
            return Mission::Result::Error; // FIXME
        case MAVLinkMissionTransfer::Result::InvalidParam:
            return Mission::Result::InvalidArgument; // FIXME
        default:
            return Mission::Result::Unknown;
    }
}

std::pair<Mission::Result, Mission::MissionPlan>
MissionImpl::import_qgroundcontrol_mission(const std::string& qgc_plan_file)
{
    Mission::MissionPlan mission_plan;
    auto result =
        std::pair<Mission::Result, Mission::MissionPlan>(Mission::Result::Unknown, mission_plan);

    std::ifstream file(qgc_plan_file);
    if (!file) {
        result.first = Mission::Result::FailedToOpenQgcPlan;
        return result;
    }

    std::stringstream ss;
    ss << file.rdbuf();
    file.close();
    const auto raw_json = ss.str();

    Json::CharReaderBuilder builder;
    const std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
    Json::Value root;
    JSONCPP_STRING err;
    const bool ok =
        reader->parse(raw_json.c_str(), raw_json.c_str() + raw_json.length(), &root, &err);
    if (!ok) {
        LogErr() << "Parse error: " << err;
        result.first = Mission::Result::FailedToParseQgcPlan;
        return result;
    }

    result.first = import_mission_items(result.second.mission_items, root);
    return result;
}

void MissionImpl::import_qgroundcontrol_mission_async(
    std::string qgc_plan_path, const Mission::ImportQgroundcontrolMissionCallback callback)
{
    auto fut = std::async([this, callback, qgc_plan_path]() {
        auto result = MissionImpl::import_qgroundcontrol_mission(qgc_plan_path);
        _parent->call_user_callback([&result, callback]() {
            if (callback) {
                callback(result.first, result.second);
            }
        });
    });

    UNUSED(fut);
}

// Build a mission item out of command, params and add them to the mission vector.
Mission::Result MissionImpl::build_mission_items(
    MAV_CMD command,
    std::vector<double> params,
    MissionItem& new_mission_item,
    std::vector<Mission::MissionItem>& all_mission_items)
{
    Mission::Result result = Mission::Result::Success;

    // Choosen "Do-While(0)" loop for the convenience of using `break` statement.
    do {
        if (command == MAV_CMD_NAV_WAYPOINT || command == MAV_CMD_NAV_TAKEOFF ||
            command == MAV_CMD_NAV_LAND) {
            if (has_valid_position(new_mission_item)) {
                if (command == MAV_CMD_NAV_TAKEOFF) {
                    LogWarn() << "Converted takeoff mission item to normal waypoint";
                } else if (command == MAV_CMD_NAV_LAND) {
                    LogWarn() << "Converted land mission item to normal waypoint";
                }
                all_mission_items.push_back(new_mission_item);
                new_mission_item = {};
            }

            if (command == MAV_CMD_NAV_WAYPOINT) {
                auto is_fly_through = !(int(params[0]) > 0);
                new_mission_item.is_fly_through = is_fly_through;
            }
            new_mission_item.loiter_time_s = params[0];
            auto lat = params[4], lon = params[5];
            new_mission_item.latitude_deg = lat;
            new_mission_item.longitude_deg = lon;

            auto rel_alt = float(params[6]);
            new_mission_item.relative_altitude_m = rel_alt;

        } else if (command == MAV_CMD_DO_MOUNT_CONTROL) {
            new_mission_item.gimbal_pitch_deg = float(params[0]);
            new_mission_item.gimbal_yaw_deg = float(params[2]);

        } else if (command == MAV_CMD_NAV_LOITER_TIME) {
            auto loiter_time_s = float(params[0]);
            new_mission_item.loiter_time_s = loiter_time_s;

        } else if (command == MAV_CMD_IMAGE_START_CAPTURE) {
            auto photo_interval = int(params[1]), photo_count = int(params[2]);

            if (photo_interval > 0 && photo_count == 0) {
                new_mission_item.camera_action = CameraAction::StartPhotoInterval;
                new_mission_item.camera_photo_interval_s = photo_interval;
            } else if (photo_interval == 0 && photo_count == 1) {
                new_mission_item.camera_action = CameraAction::TakePhoto;
            } else {
                LogErr() << "Mission item START_CAPTURE params unsupported.";
                result = Mission::Result::Unsupported;
                break;
            }

        } else if (command == MAV_CMD_IMAGE_STOP_CAPTURE) {
            new_mission_item.camera_action = CameraAction::StopPhotoInterval;

        } else if (command == MAV_CMD_VIDEO_START_CAPTURE) {
            new_mission_item.camera_action = CameraAction::StartVideo;

        } else if (command == MAV_CMD_VIDEO_STOP_CAPTURE) {
            new_mission_item.camera_action = CameraAction::StopVideo;

        } else if (command == MAV_CMD_DO_CHANGE_SPEED) {
            enum { AirSpeed = 0, GroundSpeed = 1 };
            auto speed_type = int(params[0]);
            auto speed_m_s = float(params[1]);
            auto throttle = params[2];
            auto is_absolute = (params[3] == 0);

            if (speed_type == int(GroundSpeed) && throttle < 0 && is_absolute) {
                new_mission_item.speed_m_s = speed_m_s;
            } else {
                LogErr() << command << "Mission item DO_CHANGE_SPEED params unsupported";
                result = Mission::Result::Unsupported;
                break;
            }
        } else {
            LogWarn() << "UNSUPPORTED mission item command (" << command << ")";
        }
    } while (false); // Executed once per method invokation.

    return result;
}

Mission::Result MissionImpl::import_simple_mission_item(
    std::vector<Mission::MissionItem>& all_mission_items,
    const Json::Value& json_mission_item,
    MissionItem& new_mission_item)
{
    // Parameters of Mission item & MAV command of it.
    MAV_CMD command = static_cast<MAV_CMD>(json_mission_item["command"].asInt());

    // Extract parameters of each mission item
    std::vector<double> params;
    for (auto& p : json_mission_item["params"]) {
        if (p.type() == Json::nullValue) {
            // QGC sets params as `null` if they should be unchanged.
            params.push_back(double(NAN));
        } else {
            params.push_back(p.asDouble());
        }
    }
    return build_mission_items(command, params, new_mission_item, all_mission_items);
}

Mission::Result MissionImpl::import_complex_mission_item(
    std::vector<Mission::MissionItem>& all_mission_items,
    const Json::Value& json_complex_mission_item,
    MissionItem& new_mission_item)
{
    if (json_complex_mission_item["TransectStyleComplexItem"].isNull()) {
        LogWarn() << "Unknown complex item type (" << json_complex_mission_item["complexItemType"]
                  << ")";
        return Mission::Result::UnsupportedMissionCmd;
    }

    // QGC supports more complex mission items than simple waypoints.
    // Surveys and coridor scans (NOT structure scans) are stored in a so called
    // "TransectStyleComplexItem" item inside the mission_items array. These ComplexItems also
    // contain an array ("Items") which contains waypoints. It is used by GQC to keep survey
    // parameters so one can edit it as a survey after importing. Structure scans are not supported
    // as thes do not contain simple mission items.
    Json::Value complex_item = json_complex_mission_item["TransectStyleComplexItem"];
    for (auto& json_mission_item : complex_item["Items"]) {
        Mission::Result result =
            import_simple_mission_item(all_mission_items, json_mission_item, new_mission_item);
        if (result != Mission::Result::Success) {
            return result;
        }
    }
    return Mission::Result::Success;
}

Mission::Result MissionImpl::import_mission_items(
    std::vector<Mission::MissionItem>& all_mission_items, const Json::Value& qgc_plan_json)
{
    const auto json_mission_items = qgc_plan_json["mission"];
    MissionItem new_mission_item{};

    // Iterate mission items and build Mavsdk mission items.
    for (auto& json_mission_item : json_mission_items["items"]) {
        Mission::Result result;

        // Check if mission item is complex (like a survey from qgc) or a simple item
        Json::Value type = json_mission_item["type"];

        if (!type.isNull() && type.asString() == "ComplexItem") {
            result =
                import_complex_mission_item(all_mission_items, json_mission_item, new_mission_item);
        } else {
            result =
                import_simple_mission_item(all_mission_items, json_mission_item, new_mission_item);
        }

        if (result != Mission::Result::Success) {
            break;
        }
    }
    // Don't forget to add the last mission which possibly didn't have position set.
    all_mission_items.push_back(new_mission_item);
    return Mission::Result::Success;
}

void MissionImpl::add_gimbal_items_v1(
    std::vector<MAVLinkMissionTransfer::ItemInt>& int_items,
    unsigned item_i,
    float pitch_deg,
    float yaw_deg)
{
    if (_enable_absolute_gimbal_yaw_angle) {
        // We need to configure the gimbal to use an absolute angle.

        // Current is the 0th waypoint
        uint8_t current = ((int_items.size() == 0) ? 1 : 0);

        uint8_t autocontinue = 1;

        MAVLinkMissionTransfer::ItemInt next_item{
            static_cast<uint16_t>(int_items.size()),
            MAV_FRAME_MISSION,
            MAV_CMD_DO_MOUNT_CONFIGURE,
            current,
            autocontinue,
            MAV_MOUNT_MODE_MAVLINK_TARGETING,
            0.0f, // stabilize roll
            0.0f, // stabilize pitch
            1.0f, // stabilize yaw, FIXME: for now we use this for an absolute yaw angle,
                  // because it works.
            0,
            0,
            2.0f, // eventually this is the correct flag to set absolute yaw angle.
            MAV_MISSION_TYPE_MISSION};

        _mission_data.mavlink_mission_item_to_mission_item_indices.push_back(item_i);
        int_items.push_back(next_item);
    }

    // The gimbal has changed, we need to add a gimbal command.

    // Current is the 0th waypoint
    uint8_t current = ((int_items.size() == 0) ? 1 : 0);

    uint8_t autocontinue = 1;

    MAVLinkMissionTransfer::ItemInt next_item{
        static_cast<uint16_t>(int_items.size()),
        MAV_FRAME_MISSION,
        MAV_CMD_DO_MOUNT_CONTROL,
        current,
        autocontinue,
        pitch_deg, // pitch
        0.0f, // roll (yes it is a weird order)
        yaw_deg, // yaw
        NAN,
        0,
        0,
        MAV_MOUNT_MODE_MAVLINK_TARGETING,
        MAV_MISSION_TYPE_MISSION};

    _mission_data.mavlink_mission_item_to_mission_item_indices.push_back(item_i);
    int_items.push_back(next_item);
}

void MissionImpl::add_gimbal_items_v2(
    std::vector<MAVLinkMissionTransfer::ItemInt>& int_items,
    unsigned item_i,
    float pitch_deg,
    float yaw_deg)
{
    uint8_t current = ((int_items.size() == 0) ? 1 : 0);

    uint8_t autocontinue = 1;

    // We don't set YAW_LOCK because we probably just want to face forward.
    uint32_t flags = GIMBAL_MANAGER_FLAGS_ROLL_LOCK | GIMBAL_MANAGER_FLAGS_PITCH_LOCK;

    MAVLinkMissionTransfer::ItemInt next_item{
        static_cast<uint16_t>(int_items.size()),
        MAV_FRAME_MISSION,
        MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
        current,
        autocontinue,
        to_rad_from_deg(pitch_deg), // pitch
        to_rad_from_deg(yaw_deg), // yaw
        NAN, // pitch rate
        NAN, // yaw rate
        static_cast<int32_t>(flags),
        0, // reserved
        0, // all devices
        MAV_MISSION_TYPE_MISSION};

    _mission_data.mavlink_mission_item_to_mission_item_indices.push_back(item_i);
    int_items.push_back(next_item);
}

} // namespace mavsdk
