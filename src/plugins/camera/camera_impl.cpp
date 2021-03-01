#include "camera_impl.h"
#include "camera_definition.h"
#include "system.h"
#include "global_include.h"
#include "http_loader.h"
#include "camera_definition_files.h"
#include <functional>
#include <cmath>
#include <sstream>

namespace mavsdk {

using namespace std::placeholders; // for `_1`

CameraImpl::CameraImpl(System& system) : PluginImplBase(system)
{
    _parent->register_plugin(this);
}

CameraImpl::CameraImpl(std::shared_ptr<System> system) : PluginImplBase(system)
{
    _parent->register_plugin(this);
}

CameraImpl::~CameraImpl()
{
    _parent->unregister_plugin(this);
}

void CameraImpl::init()
{
    _parent->register_mavlink_message_handler(
        MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS,
        std::bind(&CameraImpl::process_camera_capture_status, this, _1),
        this);

    _parent->register_mavlink_message_handler(
        MAVLINK_MSG_ID_STORAGE_INFORMATION,
        std::bind(&CameraImpl::process_storage_information, this, _1),
        this);

    _parent->register_mavlink_message_handler(
        MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED,
        std::bind(&CameraImpl::process_camera_image_captured, this, _1),
        this);

    _parent->register_mavlink_message_handler(
        MAVLINK_MSG_ID_CAMERA_SETTINGS,
        std::bind(&CameraImpl::process_camera_settings, this, _1),
        this);

    _parent->register_mavlink_message_handler(
        MAVLINK_MSG_ID_CAMERA_INFORMATION,
        std::bind(&CameraImpl::process_camera_information, this, _1),
        this);

    _parent->register_mavlink_message_handler(
        MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION,
        std::bind(&CameraImpl::process_video_information, this, _1),
        this);

    _parent->register_mavlink_message_handler(
        MAVLINK_MSG_ID_FLIGHT_INFORMATION,
        std::bind(&CameraImpl::process_flight_information, this, _1),
        this);

    _parent->add_call_every(
        std::bind(&CameraImpl::check_connection_status, this),
        0.5,
        &_check_connection_status_call_every_cookie);
}

void CameraImpl::deinit()
{
    _parent->remove_call_every(_check_connection_status_call_every_cookie);
    _parent->remove_call_every(_status.call_every_cookie);
    _parent->unregister_all_mavlink_message_handlers(this);
    _parent->cancel_all_param(this);

    {
        std::lock_guard<std::mutex> lock(_status.mutex);
        _status.subscription_callback = nullptr;
    }

    {
        std::lock_guard<std::mutex> lock(_mode.mutex);
        _mode.subscription_callback = nullptr;
    }

    {
        std::lock_guard<std::mutex> lock(_capture_info.mutex);
        _capture_info.callback = nullptr;
    }

    {
        std::lock_guard<std::mutex> lock(_video_stream_info.mutex);
        _video_stream_info.subscription_callback = nullptr;
    }

    {
        std::lock_guard<std::mutex> lock(_information.mutex);
        _information.subscription_callback = nullptr;
    }

    {
        std::lock_guard<std::mutex> lock(_subscribe_current_settings.mutex);
        _subscribe_current_settings.callback = nullptr;
    }

    {
        std::lock_guard<std::mutex> lock(_subscribe_possible_setting_options.mutex);
        _subscribe_possible_setting_options.callback = nullptr;
    }

    _camera_found = false;
}

void CameraImpl::check_connection_status()
{
    // FIXME: This is a workaround because we don't want to be tied to the
    // discovery of an autopilot which triggers enable() and disable() but
    // we are interested if a camera is connected or not.
    if (_parent->has_camera(_camera_id)) {
        if (!_camera_found) {
            _camera_found = true;
            manual_enable();
        }
    }
}

void CameraImpl::enable()
{
    // FIXME: We check for the connection status manually because
    // we're not interested in the connection state of the autopilot
    // but only the camera.
}

void CameraImpl::manual_enable()
{
    refresh_params();

    request_status();
    request_camera_information();
    request_flight_information();

    _parent->add_call_every(
        [this]() { request_camera_information(); }, 10.0, &_camera_information_call_every_cookie);
    _parent->add_call_every(
        [this]() { request_flight_information(); }, 10.0, &_flight_information_call_every_cookie);
}

void CameraImpl::disable()
{
    // FIXME: We check for the connection status manually because
    // we're not interested in the connection state of the autopilot
    // but only the camera.
}

void CameraImpl::manual_disable()
{
    invalidate_params();
    _parent->remove_call_every(_camera_information_call_every_cookie);
    _parent->remove_call_every(_flight_information_call_every_cookie);

    _camera_found = false;
}

Camera::Result CameraImpl::select_camera(const size_t id)
{
    static constexpr std::size_t MAX_SUPPORTED_ID = 5;

    if (id > MAX_SUPPORTED_ID) {
        return Camera::Result::WrongArgument;
    }

    // camera component IDs go from 100 to 105.
    _camera_id = id;

    // We should probably reload everything to make sure the
    // correct  camera is initialized.
    manual_disable();
    manual_enable();

    return Camera::Result::Success;
}

MavlinkCommandSender::CommandLong CameraImpl::make_command_request_flight_information()
{
    MavlinkCommandSender::CommandLong command_flight_information{};

    command_flight_information.command = MAV_CMD_REQUEST_FLIGHT_INFORMATION;
    command_flight_information.params.param1 = 1.0f; // Request it
    command_flight_information.target_component_id = MAV_COMP_ID_AUTOPILOT1;

    return command_flight_information;
}

MavlinkCommandSender::CommandLong CameraImpl::make_command_request_camera_info()
{
    MavlinkCommandSender::CommandLong command_camera_info{};

    command_camera_info.command = MAV_CMD_REQUEST_CAMERA_INFORMATION;
    command_camera_info.params.param1 = 1.0f; // Request it
    command_camera_info.target_component_id = _camera_id + MAV_COMP_ID_CAMERA;

    return command_camera_info;
}

MavlinkCommandSender::CommandLong
CameraImpl::make_command_take_photo(float interval_s, float no_of_photos)
{
    MavlinkCommandSender::CommandLong cmd_take_photo{};

    cmd_take_photo.command = MAV_CMD_IMAGE_START_CAPTURE;
    cmd_take_photo.params.param1 = 0.0f; // Reserved, set to 0
    cmd_take_photo.params.param2 = interval_s;
    cmd_take_photo.params.param3 = no_of_photos;
    cmd_take_photo.params.param4 = float(_capture.sequence++);
    cmd_take_photo.target_component_id = _camera_id + MAV_COMP_ID_CAMERA;

    return cmd_take_photo;
}

MavlinkCommandSender::CommandLong CameraImpl::make_command_stop_photo()
{
    MavlinkCommandSender::CommandLong cmd_stop_photo{};

    cmd_stop_photo.command = MAV_CMD_IMAGE_STOP_CAPTURE;
    cmd_stop_photo.target_component_id = _camera_id + MAV_COMP_ID_CAMERA;

    return cmd_stop_photo;
}

MavlinkCommandSender::CommandLong CameraImpl::make_command_start_video(float capture_status_rate_hz)
{
    MavlinkCommandSender::CommandLong cmd_start_video{};

    cmd_start_video.command = MAV_CMD_VIDEO_START_CAPTURE;
    cmd_start_video.params.param1 = 0.f; // Reserved, set to 0
    cmd_start_video.params.param2 = capture_status_rate_hz;
    cmd_start_video.target_component_id = _camera_id + MAV_COMP_ID_CAMERA;

    return cmd_start_video;
}

MavlinkCommandSender::CommandLong CameraImpl::make_command_stop_video()
{
    MavlinkCommandSender::CommandLong cmd_stop_video{};

    cmd_stop_video.command = MAV_CMD_VIDEO_STOP_CAPTURE;
    cmd_stop_video.params.param1 = 0.f; // Reserved, set to 0
    cmd_stop_video.target_component_id = _camera_id + MAV_COMP_ID_CAMERA;

    return cmd_stop_video;
}

MavlinkCommandSender::CommandLong CameraImpl::make_command_set_camera_mode(float mavlink_mode)
{
    MavlinkCommandSender::CommandLong cmd_set_camera_mode{};

    cmd_set_camera_mode.command = MAV_CMD_SET_CAMERA_MODE;
    cmd_set_camera_mode.params.param1 = 0.0f; // Reserved, set to 0
    cmd_set_camera_mode.params.param2 = mavlink_mode;
    cmd_set_camera_mode.target_component_id = _camera_id + MAV_COMP_ID_CAMERA;

    return cmd_set_camera_mode;
}

MavlinkCommandSender::CommandLong CameraImpl::make_command_request_camera_settings()
{
    MavlinkCommandSender::CommandLong cmd_req_camera_settings{};

    cmd_req_camera_settings.command = MAV_CMD_REQUEST_CAMERA_SETTINGS;
    cmd_req_camera_settings.params.param1 = 1.f; // Request it
    cmd_req_camera_settings.target_component_id = _camera_id + MAV_COMP_ID_CAMERA;

    return cmd_req_camera_settings;
}

MavlinkCommandSender::CommandLong CameraImpl::make_command_request_camera_capture_status()
{
    MavlinkCommandSender::CommandLong cmd_req_camera_cap_stat{};

    cmd_req_camera_cap_stat.command = MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS;
    cmd_req_camera_cap_stat.params.param1 = 1.0f; // Request it
    cmd_req_camera_cap_stat.target_component_id = _camera_id + MAV_COMP_ID_CAMERA;

    return cmd_req_camera_cap_stat;
}

MavlinkCommandSender::CommandLong
CameraImpl::make_command_request_camera_image_captured(const size_t photo_id)
{
    MavlinkCommandSender::CommandLong cmd_req_camera_image_captured{};

    cmd_req_camera_image_captured.command = MAV_CMD_REQUEST_MESSAGE;
    cmd_req_camera_image_captured.params.param1 =
        static_cast<float>(MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED);
    cmd_req_camera_image_captured.params.param2 = static_cast<float>(photo_id);
    cmd_req_camera_image_captured.target_component_id = _camera_id + MAV_COMP_ID_CAMERA;

    return cmd_req_camera_image_captured;
}

MavlinkCommandSender::CommandLong CameraImpl::make_command_request_storage_info()
{
    MavlinkCommandSender::CommandLong cmd_req_storage_info{};

    cmd_req_storage_info.command = MAV_CMD_REQUEST_STORAGE_INFORMATION;
    cmd_req_storage_info.params.param1 = 0.f; // Reserved, set to 0
    cmd_req_storage_info.params.param2 = 1.f; // Request it
    cmd_req_storage_info.target_component_id = _camera_id + MAV_COMP_ID_CAMERA;

    return cmd_req_storage_info;
}

MavlinkCommandSender::CommandLong CameraImpl::make_command_start_video_streaming()
{
    MavlinkCommandSender::CommandLong cmd_start_video_streaming{};

    cmd_start_video_streaming.command = MAV_CMD_VIDEO_START_STREAMING;
    cmd_start_video_streaming.target_component_id = _camera_id + MAV_COMP_ID_CAMERA;

    return cmd_start_video_streaming;
}

MavlinkCommandSender::CommandLong CameraImpl::make_command_stop_video_streaming()
{
    MavlinkCommandSender::CommandLong cmd_stop_video_streaming{};

    cmd_stop_video_streaming.command = MAV_CMD_VIDEO_STOP_STREAMING;
    cmd_stop_video_streaming.target_component_id = _camera_id + MAV_COMP_ID_CAMERA;

    return cmd_stop_video_streaming;
}

MavlinkCommandSender::CommandLong CameraImpl::make_command_request_video_stream_info()
{
    MavlinkCommandSender::CommandLong cmd_req_video_stream_info{};

    cmd_req_video_stream_info.command = MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION;
    cmd_req_video_stream_info.params.param2 = 1.0f;
    cmd_req_video_stream_info.target_component_id = _camera_id + MAV_COMP_ID_CAMERA;

    return cmd_req_video_stream_info;
}

Camera::Result CameraImpl::take_photo()
{
    // TODO: check whether we are in photo mode.

    std::lock_guard<std::mutex> lock(_capture.mutex);

    // Take 1 photo only with no interval
    auto cmd_take_photo = make_command_take_photo(0.f, 1.0f);

    return camera_result_from_command_result(_parent->send_command(cmd_take_photo));
}

Camera::Result CameraImpl::start_photo_interval(float interval_s)
{
    if (!interval_valid(interval_s)) {
        return Camera::Result::WrongArgument;
    }

    // TODO: check whether we are in photo mode.

    std::lock_guard<std::mutex> lock(_capture.mutex);

    auto cmd_take_photo_time_lapse = make_command_take_photo(interval_s, 0.f);

    return camera_result_from_command_result(_parent->send_command(cmd_take_photo_time_lapse));
}

Camera::Result CameraImpl::stop_photo_interval()
{
    auto cmd_stop_photo_interval = make_command_stop_photo();

    return camera_result_from_command_result(_parent->send_command(cmd_stop_photo_interval));
}

Camera::Result CameraImpl::start_video()
{
    // TODO: check whether video capture is already in progress.
    // TODO: check whether we are in video mode.

    // Capture status rate is not set
    auto cmd_start_video = make_command_start_video(0.f);

    return camera_result_from_command_result(_parent->send_command(cmd_start_video));
}

Camera::Result CameraImpl::stop_video()
{
    auto cmd_stop_video = make_command_stop_video();

    {
        std::lock_guard<std::mutex> lock(_video_stream_info.mutex);
        _video_stream_info.data.status = Camera::VideoStreamInfo::VideoStreamStatus::NotRunning;
    }

    return camera_result_from_command_result(_parent->send_command(cmd_stop_video));
}

void CameraImpl::take_photo_async(const Camera::ResultCallback& callback)
{
    // TODO: check whether we are in photo mode.

    std::lock_guard<std::mutex> lock(_capture.mutex);

    // Take 1 photo only with no interval
    auto cmd_take_photo = make_command_take_photo(0.f, 1.0f);

    _parent->send_command_async(
        cmd_take_photo, std::bind(&CameraImpl::receive_command_result, this, _1, callback));
}

void CameraImpl::start_photo_interval_async(
    float interval_s, const Camera::ResultCallback& callback)
{
    if (!interval_valid(interval_s)) {
        const auto temp_callback = callback;
        _parent->call_user_callback(
            [temp_callback]() { temp_callback(Camera::Result::WrongArgument); });
        return;
    }

    // TODO: check whether we are in photo mode.

    std::lock_guard<std::mutex> lock(_capture.mutex);

    auto cmd_take_photo_time_lapse = make_command_take_photo(interval_s, 0.f);

    _parent->send_command_async(
        cmd_take_photo_time_lapse,
        std::bind(&CameraImpl::receive_command_result, this, _1, callback));
}

void CameraImpl::stop_photo_interval_async(const Camera::ResultCallback& callback)
{
    auto cmd_stop_photo_interval = make_command_stop_photo();

    _parent->send_command_async(
        cmd_stop_photo_interval,
        std::bind(&CameraImpl::receive_command_result, this, _1, callback));
}

void CameraImpl::start_video_async(const Camera::ResultCallback& callback)
{
    // TODO: check whether video capture is already in progress.
    // TODO: check whether we are in video mode.

    // Capture status rate is not set
    auto cmd_start_video = make_command_start_video(0.f);

    _parent->send_command_async(
        cmd_start_video, std::bind(&CameraImpl::receive_command_result, this, _1, callback));
}

void CameraImpl::stop_video_async(const Camera::ResultCallback& callback)
{
    auto cmd_stop_video = make_command_stop_video();

    _parent->send_command_async(
        cmd_stop_video, std::bind(&CameraImpl::receive_command_result, this, _1, callback));
}

Camera::Information CameraImpl::information() const
{
    std::lock_guard<std::mutex> lock(_information.mutex);

    return _information.data;
}

void CameraImpl::subscribe_information(const Camera::InformationCallback& callback)
{
    std::lock_guard<std::mutex> lock(_information.mutex);
    _information.subscription_callback = callback;

    if (callback) {
        _parent->add_call_every([this]() { request_status(); }, 1.0, &_status.call_every_cookie);
    } else {
        _parent->remove_call_every(_status.call_every_cookie);
    }
}

Camera::Result CameraImpl::start_video_streaming()
{
    std::lock_guard<std::mutex> lock(_video_stream_info.mutex);

    if (_video_stream_info.available &&
        _video_stream_info.data.status == Camera::VideoStreamInfo::VideoStreamStatus::InProgress) {
        return Camera::Result::InProgress;
    }

    // TODO Check whether we're in video mode
    auto command = make_command_start_video_streaming();

    auto result = camera_result_from_command_result(_parent->send_command(command));
    // if (result == Camera::Result::Success) {
    // Cache video stream info; app may query immediately next.
    // TODO: check if we can/should do that.
    // auto info = get_video_stream_info();
    //}
    return result;
}

Camera::Result CameraImpl::stop_video_streaming()
{
    // TODO I think we need to maintain current state, whether we issued
    // video capture request or video streaming request, etc.We shouldn't
    // send stop video streaming if we've not started it!
    auto command = make_command_stop_video_streaming();

    auto result = camera_result_from_command_result(_parent->send_command(command));
    {
        std::lock_guard<std::mutex> lock(_video_stream_info.mutex);
        // TODO: check if we can/should do that.
        _video_stream_info.data.status = Camera::VideoStreamInfo::VideoStreamStatus::NotRunning;
    }
    return result;
}

void CameraImpl::request_video_stream_info()
{
    _parent->send_command_async(make_command_request_video_stream_info(), nullptr);
}

Camera::VideoStreamInfo CameraImpl::video_stream_info()
{
    std::lock_guard<std::mutex> lock(_video_stream_info.mutex);

    return _video_stream_info.data;
}

void CameraImpl::subscribe_video_stream_info(const Camera::VideoStreamInfoCallback callback)
{
    std::lock_guard<std::mutex> lock(_video_stream_info.mutex);

    _video_stream_info.subscription_callback = callback;

    if (callback) {
        _parent->add_call_every(
            [this]() { request_video_stream_info(); }, 1.0, &_video_stream_info.call_every_cookie);
    } else {
        _parent->remove_call_every(_video_stream_info.call_every_cookie);
    }
}

Camera::Result
CameraImpl::camera_result_from_command_result(const MavlinkCommandSender::Result command_result)
{
    switch (command_result) {
        case MavlinkCommandSender::Result::Success:
            return Camera::Result::Success;
        case MavlinkCommandSender::Result::NoSystem:
        // FALLTHROUGH
        case MavlinkCommandSender::Result::ConnectionError:
        // FALLTHROUGH
        case MavlinkCommandSender::Result::Busy:
            return Camera::Result::Error;
        case MavlinkCommandSender::Result::CommandDenied:
            return Camera::Result::Denied;
        case MavlinkCommandSender::Result::Timeout:
            return Camera::Result::Timeout;
        default:
            return Camera::Result::Unknown;
    }
}

Camera::Result CameraImpl::set_mode(const Camera::Mode mode)
{
    const float mavlink_mode = to_mavlink_camera_mode(mode);
    auto cmd_set_camera_mode = make_command_set_camera_mode(mavlink_mode);
    const auto command_result = _parent->send_command(cmd_set_camera_mode);
    const auto camera_result = camera_result_from_command_result(command_result);

    if (camera_result == Camera::Result::Success) {
        {
            std::lock_guard<std::mutex> lock(_mode.mutex);
            _mode.data = mode;
        }
        notify_mode();
        if (_camera_definition != nullptr) {
            save_camera_mode(mavlink_mode);
        }
    }

    return camera_result;
}

void CameraImpl::save_camera_mode(const float mavlink_camera_mode)
{
    if (!std::isfinite(mavlink_camera_mode)) {
        LogWarn() << "Can't save NAN as camera mode";
        return;
    }

    // If there is a camera definition (which is the case if we are
    // in this function, and if CAM_MODE is defined there, then
    // we reuse that type. Otherwise, we hardcode it to `uint32_t`.
    // Note that it could be that the camera definition defines options
    // different than {PHOTO, VIDEO}, in which case the mode received
    // from CAMERA_SETTINGS will be wrong. Not sure if it means that
    // it should be ignored in that case, but that may be tricky to
    // maintain (what if the MAVLink CAMERA_MODE enum evolves?), so
    // I am assuming here that in such a case, CAMERA_SETTINGS is
    // never sent by the camera.
    MAVLinkParameters::ParamValue value;
    if (_camera_definition->get_setting("CAM_MODE", value)) {
        if (value.is<uint8_t>()) {
            value.set<uint8_t>(static_cast<uint8_t>(mavlink_camera_mode));
        } else if (value.is<int8_t>()) {
            value.set<int8_t>(static_cast<int8_t>(mavlink_camera_mode));
        } else if (value.is<uint16_t>()) {
            value.set<uint16_t>(static_cast<uint16_t>(mavlink_camera_mode));
        } else if (value.is<int16_t>()) {
            value.set<int16_t>(static_cast<int16_t>(mavlink_camera_mode));
        } else if (value.is<uint32_t>()) {
            value.set<uint32_t>(static_cast<uint32_t>(mavlink_camera_mode));
        } else if (value.is<int32_t>()) {
            value.set<int32_t>(static_cast<int32_t>(mavlink_camera_mode));
        } else if (value.is<uint64_t>()) {
            value.set<uint64_t>(static_cast<uint64_t>(mavlink_camera_mode));
        } else if (value.is<int64_t>()) {
            value.set<int64_t>(static_cast<int64_t>(mavlink_camera_mode));
        } else if (value.is<float>()) {
            value.set<float>(static_cast<float>(mavlink_camera_mode));
        } else if (value.is<double>()) {
            value.set<double>(static_cast<double>(mavlink_camera_mode));
        }
    } else {
        value.set<uint32_t>(static_cast<uint32_t>(mavlink_camera_mode));
    }

    _camera_definition->set_setting("CAM_MODE", value);
    refresh_params();
}

float CameraImpl::to_mavlink_camera_mode(const Camera::Mode mode) const
{
    switch (mode) {
        case Camera::Mode::Photo:
            return CAMERA_MODE_IMAGE;
        case Camera::Mode::Video:
            return CAMERA_MODE_VIDEO;
        default:
        case Camera::Mode::Unknown:
            return NAN;
    }
}

void CameraImpl::set_mode_async(const Camera::Mode mode, const Camera::ResultCallback& callback)
{
    const auto mavlink_mode = to_mavlink_camera_mode(mode);
    auto cmd_set_camera_mode = make_command_set_camera_mode(mavlink_mode);

    _parent->send_command_async(
        cmd_set_camera_mode,
        [this, callback, mode](MavlinkCommandSender::Result result, float progress) {
            UNUSED(progress);
            receive_set_mode_command_result(result, callback, mode);
        });
}

Camera::Mode CameraImpl::mode()
{
    std::lock_guard<std::mutex> lock(_mode.mutex);
    return _mode.data;
}

void CameraImpl::subscribe_mode(const Camera::ModeCallback callback)
{
    {
        std::lock_guard<std::mutex> lock(_mode.mutex);
        _mode.subscription_callback = callback;
    }

    notify_mode();

    if (callback) {
        _parent->add_call_every(
            [this]() { request_camera_settings(); }, 1.0, &_mode.call_every_cookie);
    } else {
        _parent->remove_call_every(_mode.call_every_cookie);
    }
}

bool CameraImpl::interval_valid(float interval_s)
{
    // Reject everything faster than 1000 Hz, as well as negative inputs.
    if (interval_s < 0.001f) {
        LogWarn() << "Invalid interval input";
        return false;
    } else {
        return true;
    }
}

void CameraImpl::request_status()
{
    _parent->send_command_async(make_command_request_camera_capture_status(), nullptr);
    _parent->send_command_async(make_command_request_storage_info(), nullptr);
}

void CameraImpl::subscribe_status(const Camera::StatusCallback callback)
{
    std::lock_guard<std::mutex> lock(_status.mutex);

    _status.subscription_callback = callback;

    if (callback) {
        _parent->add_call_every([this]() { request_status(); }, 1.0, &_status.call_every_cookie);
    } else {
        _parent->remove_call_every(_status.call_every_cookie);
    }
}

Camera::Status CameraImpl::status()
{
    std::lock_guard<std::mutex> lock(_status.mutex);
    return _status.data;
}

void CameraImpl::subscribe_capture_info(Camera::CaptureInfoCallback callback)
{
    std::lock_guard<std::mutex> lock(_capture_info.mutex);
    _capture_info.callback = callback;
}

void CameraImpl::process_camera_capture_status(const mavlink_message_t& message)
{
    mavlink_camera_capture_status_t camera_capture_status;
    mavlink_msg_camera_capture_status_decode(&message, &camera_capture_status);

    {
        std::lock_guard<std::mutex> lock(_status.mutex);

        _status.data.video_on = (camera_capture_status.video_status == 1);
        _status.data.photo_interval_on =
            (camera_capture_status.image_status == 2 || camera_capture_status.image_status == 3);
        _status.received_camera_capture_status = true;
        _status.data.recording_time_s = float(camera_capture_status.recording_time_ms) / 1e3f;
        _status.image_count = camera_capture_status.image_count;

        if (_status.image_count_at_connection == -1) {
            _status.image_count_at_connection = camera_capture_status.image_count;
        }
    }

    {
        std::lock_guard<std::mutex> lock(_capture_info.mutex);

        if (_capture_info.last_advertised_image_index == -1) {
            _capture_info.last_advertised_image_index = camera_capture_status.image_count - 1;
        }
    }

    check_status();
}

void CameraImpl::process_storage_information(const mavlink_message_t& message)
{
    mavlink_storage_information_t storage_information;
    mavlink_msg_storage_information_decode(&message, &storage_information);

    {
        std::lock_guard<std::mutex> lock(_status.mutex);
        if (storage_information.status == 0) {
            _status.data.storage_status = Camera::Status::StorageStatus::NotAvailable;
        } else if (storage_information.status == 1) {
            _status.data.storage_status = Camera::Status::StorageStatus::Unformatted;
        } else if (storage_information.status == 2) {
            _status.data.storage_status = Camera::Status::StorageStatus::Formatted;
        }
        _status.data.available_storage_mib = storage_information.available_capacity;
        _status.data.used_storage_mib = storage_information.used_capacity;
        _status.data.total_storage_mib = storage_information.total_capacity;
        _status.received_storage_information = true;
    }

    check_status();
}

void CameraImpl::process_camera_image_captured(const mavlink_message_t& message)
{
    mavlink_camera_image_captured_t image_captured;
    mavlink_msg_camera_image_captured_decode(&message, &image_captured);

    {
        Camera::CaptureInfo capture_info = {};
        capture_info.position.latitude_deg = image_captured.lat / 1e7;
        capture_info.position.longitude_deg = image_captured.lon / 1e7;
        capture_info.position.absolute_altitude_m = image_captured.alt / 1e3f;
        capture_info.position.relative_altitude_m = image_captured.relative_alt / 1e3f;
        capture_info.time_utc_us = image_captured.time_utc;
        capture_info.attitude_quaternion.w = image_captured.q[0];
        capture_info.attitude_quaternion.x = image_captured.q[1];
        capture_info.attitude_quaternion.y = image_captured.q[2];
        capture_info.attitude_quaternion.z = image_captured.q[3];
        capture_info.attitude_euler_angle =
            to_euler_angle_from_quaternion(capture_info.attitude_quaternion);
        capture_info.file_url = std::string(image_captured.file_url);
        capture_info.is_success = (image_captured.capture_result == 1);
        capture_info.index = image_captured.image_index;

        if (capture_info.is_success) {
            _status.photo_list.insert(std::make_pair(image_captured.image_index, capture_info));
        }

        _captured_request_cv.notify_all();

        std::lock_guard<std::mutex> lock(_capture_info.mutex);
        if (_capture_info.last_advertised_image_index < capture_info.index &&
            _capture_info.callback) {
            _capture_info.last_advertised_image_index = capture_info.index;
            const auto temp_callback = _capture_info.callback;
            _parent->call_user_callback(
                [temp_callback, capture_info]() { temp_callback(capture_info); });
        }
    }
}

Camera::EulerAngle CameraImpl::to_euler_angle_from_quaternion(Camera::Quaternion quaternion)
{
    auto& q = quaternion;

    // FIXME: This is duplicated from telemetry/math_conversions.cpp.
    Camera::EulerAngle euler_angle;
    euler_angle.roll_deg = to_deg_from_rad(
        atan2f(2.0f * (q.w * q.x + q.y * q.z), 1.0f - 2.0f * (q.x * q.x + q.y * q.y)));

    euler_angle.pitch_deg = to_deg_from_rad(asinf(2.0f * (q.w * q.y - q.z * q.x)));
    euler_angle.yaw_deg = to_deg_from_rad(
        atan2f(2.0f * (q.w * q.z + q.x * q.y), 1.0f - 2.0f * (q.y * q.y + q.z * q.z)));
    return euler_angle;
}

void CameraImpl::process_camera_settings(const mavlink_message_t& message)
{
    mavlink_camera_settings_t camera_settings;
    mavlink_msg_camera_settings_decode(&message, &camera_settings);

    {
        std::lock_guard<std::mutex> lock(_mode.mutex);
        _mode.data = to_camera_mode(camera_settings.mode_id);
    }
    notify_mode();

    if (_camera_definition) {
        // This "parameter" needs to be manually set.
        save_camera_mode(camera_settings.mode_id);
    }
}

Camera::Mode CameraImpl::to_camera_mode(const uint8_t mavlink_camera_mode) const
{
    switch (mavlink_camera_mode) {
        case CAMERA_MODE_IMAGE:
            return Camera::Mode::Photo;
        case CAMERA_MODE_VIDEO:
            return Camera::Mode::Video;
        default:
            return Camera::Mode::Unknown;
    }
}

void CameraImpl::process_camera_information(const mavlink_message_t& message)
{
    mavlink_camera_information_t camera_information;
    mavlink_msg_camera_information_decode(&message, &camera_information);

    {
        std::lock_guard<std::mutex> lock(_information.mutex);
        _information.data.vendor_name = (char*)(camera_information.vendor_name);
        _information.data.model_name = (char*)(camera_information.model_name);

        if (_information.subscription_callback) {
            const auto temp_callback = _information.subscription_callback;
            const auto temp_information = _information.data;
            _parent->call_user_callback(
                [temp_callback, temp_information]() { temp_callback(temp_information); });
        }
    }

    if (!_camera_definition) {
        std::string content{};
        auto succeeded = fetch_camera_definition(camera_information, content);

        if (succeeded) {
            _camera_definition.reset(new CameraDefinition());
            _camera_definition->load_string(content);
            refresh_params();
            LogDebug() << "Successfully loaded camera definition";
        } else {
            LogDebug() << "Failed to fetch camera definition!";
        }
    }
}

bool CameraImpl::fetch_camera_definition(
    const mavlink_camera_information_t& camera_information, std::string& camera_definition_out)
{
    auto download_succeeded =
        download_definition_file(camera_information.cam_definition_uri, camera_definition_out);

    if (download_succeeded) {
        return true;
    }

    return load_stored_definition(camera_information, camera_definition_out);
}

bool CameraImpl::download_definition_file(
    const std::string& uri, std::string& camera_definition_out)
{
    HttpLoader http_loader;
    LogInfo() << "Downloading camera definition from: " << uri;
    if (!http_loader.download_text_sync(uri, camera_definition_out)) {
        LogErr() << "Failed to download camera definition.";
        return false;
    }

    return true;
}

bool CameraImpl::load_stored_definition(
    const mavlink_camera_information_t& camera_information, std::string& camera_definition_out)
{
    // TODO: we might also try to support the correct version of the xml files.
    if (strcmp((const char*)(camera_information.vendor_name), "Yuneec") == 0) {
        if (strcmp((const char*)(camera_information.model_name), "E90") == 0) {
            LogInfo() << "Using cached file for Yuneec E90.";
            camera_definition_out = e90xml;
            return true;
        } else if (strcmp((const char*)(camera_information.model_name), "E50") == 0) {
            LogInfo() << "Using cached file for Yuneec E50.";
            camera_definition_out = e50xml;
            return true;
        } else if (strcmp((const char*)(camera_information.model_name), "CGOET") == 0) {
            LogInfo() << "Using cached file for Yuneec ET.";
            camera_definition_out = cgoetxml;
            return true;
        } else if (strcmp((const char*)(camera_information.model_name), "E10T") == 0) {
            LogInfo() << "Using cached file for Yuneec E10T.";
            camera_definition_out = e10txml;
            return true;
        } else if (strcmp((const char*)(camera_information.model_name), "E30Z") == 0) {
            LogInfo() << "Using cached file for Yuneec E30Z.";
            camera_definition_out = e30zxml;
            return true;
        }
    }

    return false;
}

void CameraImpl::process_video_information(const mavlink_message_t& message)
{
    mavlink_video_stream_information_t received_video_info;
    mavlink_msg_video_stream_information_decode(&message, &received_video_info);

    {
        std::lock_guard<std::mutex> lock(_video_stream_info.mutex);
        // TODO: use stream_id and count
        _video_stream_info.data.status =
            (received_video_info.flags & VIDEO_STREAM_STATUS_FLAGS_RUNNING ?
                 Camera::VideoStreamInfo::VideoStreamStatus::InProgress :
                 Camera::VideoStreamInfo::VideoStreamStatus::NotRunning);
        auto& video_stream_info = _video_stream_info.data.settings;
        video_stream_info.frame_rate_hz = received_video_info.framerate;
        video_stream_info.horizontal_resolution_pix = received_video_info.resolution_h;
        video_stream_info.vertical_resolution_pix = received_video_info.resolution_v;
        video_stream_info.bit_rate_b_s = received_video_info.bitrate;
        video_stream_info.rotation_deg = received_video_info.rotation;
        video_stream_info.uri = received_video_info.uri;
        _video_stream_info.available = true;
    }

    notify_video_stream_info();
}

void CameraImpl::process_flight_information(const mavlink_message_t& message)
{
    mavlink_flight_information_t flight_information;
    mavlink_msg_flight_information_decode(&message, &flight_information);

    std::stringstream folder_name_stream;
    {
        std::lock_guard<std::mutex> information_lock(_information.mutex);

        // For Yuneec cameras, the folder names can be derived from the flight ID,
        // starting at 100 up to 999.
        if (_information.data.vendor_name == "Yuneec" && _information.data.model_name == "E90") {
            folder_name_stream << (101 + flight_information.flight_uuid % 899) << "E90HD";
        } else if (
            _information.data.vendor_name == "Yuneec" && _information.data.model_name == "E50") {
            folder_name_stream << (101 + flight_information.flight_uuid % 899) << "E50HD";
        } else if (
            _information.data.vendor_name == "Yuneec" && _information.data.model_name == "CGOET") {
            folder_name_stream << (101 + flight_information.flight_uuid % 899) << "CGOET";
        } else if (
            _information.data.vendor_name == "Yuneec" && _information.data.model_name == "E10T") {
            folder_name_stream << (101 + flight_information.flight_uuid % 899) << "E10T";
        } else {
            // Folder name unknown
        }
    }

    {
        std::lock_guard<std::mutex> lock(_status.mutex);
        _status.data.media_folder_name = folder_name_stream.str();
    }
}

void CameraImpl::notify_video_stream_info()
{
    std::lock_guard<std::mutex> lock(_video_stream_info.mutex);
    if (_video_stream_info.subscription_callback) {
        const auto temp_callback = _video_stream_info.subscription_callback;
        const auto temp_info = _video_stream_info.data;
        _parent->call_user_callback([temp_callback, temp_info]() { temp_callback(temp_info); });
    }
}

void CameraImpl::check_status()
{
    std::lock_guard<std::mutex> lock(_status.mutex);

    if (_status.received_camera_capture_status && _status.received_storage_information) {
        if (_status.subscription_callback) {
            const auto temp_callback = _status.subscription_callback;
            const auto temp_data = _status.data;
            _parent->call_user_callback([temp_callback, temp_data]() { temp_callback(temp_data); });
        }

        _status.received_camera_capture_status = false;
        _status.received_storage_information = false;
    }
}

void CameraImpl::receive_command_result(
    MavlinkCommandSender::Result command_result, const Camera::ResultCallback& callback)
{
    Camera::Result camera_result = camera_result_from_command_result(command_result);

    if (callback) {
        _parent->call_user_callback([callback, camera_result]() { callback(camera_result); });
    }
}

void CameraImpl::receive_set_mode_command_result(
    const MavlinkCommandSender::Result command_result,
    const Camera::ResultCallback callback,
    const Camera::Mode mode)
{
    Camera::Result camera_result = camera_result_from_command_result(command_result);

    if (callback) {
        const auto temp_callback = callback;
        _parent->call_user_callback(
            [temp_callback, camera_result]() { temp_callback(camera_result); });
    }

    if (command_result == MavlinkCommandSender::Result::Success && _camera_definition) {
        // This "parameter" needs to be manually set.
        {
            std::lock_guard<std::mutex> lock(_mode.mutex);
            _mode.data = mode;
        }

        const auto mavlink_mode = to_mavlink_camera_mode(mode);

        if (std::isnan(mavlink_mode)) {
            LogWarn() << "Unknown camera mode";
            return;
        }

        notify_mode();
        save_camera_mode(mavlink_mode);
    }
}

void CameraImpl::notify_mode()
{
    // Make a copy because it is passed to the thread pool
    const auto temp_callback = _mode.subscription_callback;

    if (temp_callback == nullptr) {
        return;
    }

    std::lock_guard<std::mutex> lock(_mode.mutex);
    auto mode = _mode.data;
    _parent->call_user_callback([mode, temp_callback]() { temp_callback(mode); });
}

bool CameraImpl::get_possible_setting_options(std::vector<std::string>& settings)
{
    settings.clear();

    if (!_camera_definition) {
        LogWarn() << "Error: no camera definition available yet";
        return false;
    }

    std::unordered_map<std::string, MAVLinkParameters::ParamValue> cd_settings{};
    _camera_definition->get_possible_settings(cd_settings);

    for (const auto& cd_setting : cd_settings) {
        if (cd_setting.first == "CAM_MODE") {
            // We ignore the mode param.
            continue;
        }

        settings.push_back(cd_setting.first);
    }

    return settings.size() > 0;
}

bool CameraImpl::get_possible_options(
    const std::string& setting_id, std::vector<Camera::Option>& options)
{
    options.clear();

    if (!_camera_definition) {
        LogWarn() << "Error: no camera definition available yet";
        return false;
    }

    std::vector<MAVLinkParameters::ParamValue> values;
    if (!_camera_definition->get_possible_options(setting_id, values)) {
        return false;
    }

    for (const auto& value : values) {
        std::stringstream ss{};
        ss << value;
        Camera::Option option{};
        option.option_id = ss.str();
        if (!is_setting_range(setting_id)) {
            get_option_str(setting_id, option.option_id, option.option_description);
        }
        options.push_back(option);
    }

    return options.size() > 0;
}

bool CameraImpl::is_setting_range(const std::string& setting_id)
{
    return _camera_definition->is_setting_range(setting_id);
}

Camera::Result CameraImpl::set_setting(Camera::Setting setting)
{
    auto prom = std::make_shared<std::promise<Camera::Result>>();
    auto ret = prom->get_future();

    set_setting_async(setting, [&prom](Camera::Result result) { prom->set_value(result); });

    return ret.get();
}

void CameraImpl::set_setting_async(Camera::Setting setting, const Camera::ResultCallback callback)
{
    set_option_async(setting.setting_id, setting.option, callback);
}

void CameraImpl::set_option_async(
    const std::string& setting_id,
    const Camera::Option& option,
    const Camera::ResultCallback& callback)
{
    if (!_camera_definition) {
        LogWarn() << "Error: no camera defnition available yet.";
        if (callback) {
            const auto temp_callback = callback;
            _parent->call_user_callback(
                [temp_callback]() { temp_callback(Camera::Result::Error); });
        }
        return;
    }

    // We get it first so that we have the type of the param value.
    MAVLinkParameters::ParamValue value;

    if (_camera_definition->is_setting_range(setting_id)) {
        // TODO: Get type from minimum.
        std::vector<MAVLinkParameters::ParamValue> all_values;
        if (!_camera_definition->get_all_options(setting_id, all_values)) {
            if (callback) {
                LogErr() << "Could not get all options to get type for range param.";
                const auto temp_callback = callback;
                _parent->call_user_callback(
                    [temp_callback]() { temp_callback(Camera::Result::Error); });
            }
            return;
        }

        if (all_values.size() == 0) {
            if (callback) {
                LogErr() << "Could not get any options to get type for range param.";
                const auto temp_callback = callback;
                _parent->call_user_callback(
                    [temp_callback]() { temp_callback(Camera::Result::Error); });
            }
            return;
        }
        value = all_values[0];
        // Now re-use that type.
        // FIXME: this is quite ugly, we should do better than that.
        if (!value.set_as_same_type(option.option_id)) {
            if (callback) {
                LogErr() << "Could not set option value to given type.";
                const auto temp_callback = callback;
                _parent->call_user_callback(
                    [temp_callback]() { temp_callback(Camera::Result::Error); });
            }
            return;
        }

    } else {
        if (!_camera_definition->get_option_value(setting_id, option.option_id, value)) {
            if (callback) {
                LogErr() << "Could not get option value.";
                const auto temp_callback = callback;
                _parent->call_user_callback(
                    [temp_callback]() { temp_callback(Camera::Result::Error); });
            }
            return;
        }

        std::vector<MAVLinkParameters::ParamValue> possible_values;
        _camera_definition->get_possible_options(setting_id, possible_values);
        bool allowed = false;
        for (const auto& possible_value : possible_values) {
            if (value == possible_value) {
                allowed = true;
            }
        }
        if (!allowed) {
            LogErr() << "Setting " << setting_id << "(" << option.option_id << ") not allowed";
            if (callback) {
                const auto temp_callback = callback;
                _parent->call_user_callback(
                    [temp_callback]() { temp_callback(Camera::Result::Error); });
            }
            return;
        }
    }

    _parent->set_param_async(
        setting_id,
        value,
        [this, callback, setting_id, value](MAVLinkParameters::Result result) {
            if (result == MAVLinkParameters::Result::Success) {
                if (!this->_camera_definition) {
                    if (callback) {
                        const auto temp_callback = callback;
                        _parent->call_user_callback(
                            [temp_callback]() { temp_callback(Camera::Result::Error); });
                    }
                    return;
                }

                if (!_camera_definition->set_setting(setting_id, value)) {
                    if (callback) {
                        const auto temp_callback = callback;
                        _parent->call_user_callback(
                            [temp_callback]() { temp_callback(Camera::Result::Error); });
                    }
                    return;
                }

                if (callback) {
                    const auto temp_callback = callback;
                    _parent->call_user_callback(
                        [temp_callback]() { temp_callback(Camera::Result::Success); });
                }

                // FIXME: We are already holding the lock when this lambda is run and need to
                //        schedule the refresh_params() for later.
                //        We (ab)use the thread pool for the user callbacks for this.
                _parent->call_user_callback([this]() { refresh_params(); });
            }
        },
        this,
        true);
}

void CameraImpl::get_setting_async(
    Camera::Setting setting, const Camera::GetSettingCallback callback)
{
    get_option_async(
        setting.setting_id,
        [this, setting, callback](Camera::Result result, const Camera::Option& option) {
            Camera::Setting new_setting{};
            new_setting.option = option;
            if (callback) {
                const auto temp_callback = callback;
                _parent->call_user_callback(
                    [temp_callback, result, new_setting]() { temp_callback(result, new_setting); });
            }
        });
}

std::pair<Camera::Result, Camera::Setting> CameraImpl::get_setting(Camera::Setting setting)
{
    auto prom = std::make_shared<std::promise<std::pair<Camera::Result, Camera::Setting>>>();
    auto ret = prom->get_future();

    get_setting_async(setting, [&prom](Camera::Result result, const Camera::Setting& new_setting) {
        prom->set_value(std::make_pair<>(result, new_setting));
    });

    return ret.get();
}

Camera::Result CameraImpl::get_option(const std::string& setting_id, Camera::Option& option)
{
    auto prom = std::make_shared<std::promise<Camera::Result>>();
    auto ret = prom->get_future();

    get_option_async(
        setting_id, [prom, &option](Camera::Result result, const Camera::Option& option_gotten) {
            prom->set_value(result);
            if (result == Camera::Result::Success) {
                option = option_gotten;
            }
        });

    auto status = ret.wait_for(std::chrono::seconds(1));

    if (status == std::future_status::ready) {
        return Camera::Result::Success;
    } else {
        return Camera::Result::Timeout;
    }
}

void CameraImpl::get_option_async(
    const std::string& setting_id,
    const std::function<void(Camera::Result, const Camera::Option&)>& callback)
{
    if (!_camera_definition) {
        LogWarn() << "Error: no camera defnition available yet.";
        if (callback) {
            Camera::Option empty_option{};
            const auto temp_callback = callback;
            _parent->call_user_callback([temp_callback, empty_option]() {
                temp_callback(Camera::Result::Error, empty_option);
            });
        }
        return;
    }

    MAVLinkParameters::ParamValue value;
    // We should have this cached and don't need to get the param.
    if (_camera_definition->get_setting(setting_id, value)) {
        if (callback) {
            Camera::Option new_option{};
            new_option.option_id = value.get_string();
            if (!is_setting_range(setting_id)) {
                get_option_str(setting_id, new_option.option_id, new_option.option_description);
            }
            const auto temp_callback = callback;
            _parent->call_user_callback([temp_callback, new_option]() {
                temp_callback(Camera::Result::Success, new_option);
            });
        }
    } else {
        // If this still happens, we request the param, but also complain.
        LogWarn() << "Setting '" << setting_id << "' not found.";
        if (callback) {
            Camera::Option no_option{};
            const auto temp_callback = callback;
            _parent->call_user_callback(
                [temp_callback, no_option]() { temp_callback(Camera::Result::Error, no_option); });
        }
    }
}

void CameraImpl::subscribe_current_settings(const Camera::CurrentSettingsCallback& callback)
{
    {
        std::lock_guard<std::mutex> lock(_subscribe_current_settings.mutex);
        _subscribe_current_settings.callback = callback;
    }
    notify_current_settings();
}

void CameraImpl::subscribe_possible_setting_options(
    const Camera::PossibleSettingOptionsCallback& callback)
{
    {
        std::lock_guard<std::mutex> lock(_subscribe_possible_setting_options.mutex);
        _subscribe_possible_setting_options.callback = callback;
    }
    notify_possible_setting_options();
}

void CameraImpl::notify_current_settings()
{
    std::lock_guard<std::mutex> lock(_subscribe_current_settings.mutex);

    if (!_subscribe_current_settings.callback) {
        return;
    }

    if (!_camera_definition) {
        LogErr() << "notify_current_settings has no camera definition";
        return;
    }

    std::vector<Camera::Setting> current_settings{};
    std::vector<std::string> possible_setting_options{};
    if (!get_possible_setting_options(possible_setting_options)) {
        LogErr() << "Could not get possible settings in current options subscription.";
        return;
    }

    for (auto& possible_setting : possible_setting_options) {
        // use the cache for this, presumably we updated it right before.
        MAVLinkParameters::ParamValue value;
        if (_camera_definition->get_setting(possible_setting, value)) {
            Camera::Setting setting{};
            setting.setting_id = possible_setting;
            setting.is_range = is_setting_range(possible_setting);
            get_setting_str(setting.setting_id, setting.setting_description);
            setting.option.option_id = value.get_string();
            if (!is_setting_range(possible_setting)) {
                get_option_str(
                    setting.setting_id,
                    setting.option.option_id,
                    setting.option.option_description);
            }
            current_settings.push_back(setting);
        }
    }

    const auto temp_callback = _subscribe_current_settings.callback;

    // We create a function object in order to move be able to move the settings into it.
    // FIXME: Use C++14 where this is not necessary anymore.
    _parent->call_user_callback(std::bind(
        [temp_callback](const std::vector<Camera::Setting>& settings) { temp_callback(settings); },
        std::move(current_settings)));
}

void CameraImpl::notify_possible_setting_options()
{
    std::lock_guard<std::mutex> lock(_subscribe_possible_setting_options.mutex);

    if (!_subscribe_possible_setting_options.callback) {
        return;
    }

    if (!_camera_definition) {
        LogErr() << "notify_possible_setting_options has no camera definition";
        return;
    }

    auto setting_options = possible_setting_options();
    if (setting_options.size() == 0) {
        return;
    }

    const auto temp_callback = _subscribe_possible_setting_options.callback;

    // We create a function object in order to move be able to move the settings into it.
    // FIXME: Use C++14 where this is not necessary anymore.
    _parent->call_user_callback(std::bind(
        [temp_callback](const std::vector<Camera::SettingOptions>& options) {
            temp_callback(options);
        },
        std::move(setting_options)));
}

std::vector<Camera::SettingOptions> CameraImpl::possible_setting_options()
{
    std::vector<Camera::SettingOptions> results{};

    std::vector<std::string> possible_settings{};
    if (!get_possible_setting_options(possible_settings)) {
        LogErr() << "Could not get possible settings.";
        return results;
    }

    for (auto& possible_setting : possible_settings) {
        Camera::SettingOptions setting_options{};
        setting_options.setting_id = possible_setting;
        setting_options.is_range = _camera_definition->is_setting_range(possible_setting);
        get_setting_str(setting_options.setting_id, setting_options.setting_description);
        get_possible_options(possible_setting, setting_options.options);
        results.push_back(setting_options);
    }

    return results;
}

void CameraImpl::refresh_params()
{
    if (!_camera_definition) {
        return;
    }

    std::vector<std::pair<std::string, MAVLinkParameters::ParamValue>> params;
    _camera_definition->get_unknown_params(params);
    if (params.size() == 0) {
        // We're assuming that we changed one option and this did not cause
        // any other possible settings to change. However, we still would
        // like to notify the current settings with this one change.
        notify_current_settings();
        return;
    }

    unsigned count = 0;
    for (const auto& param : params) {
        const std::string& param_name = param.first;
        const MAVLinkParameters::ParamValue& param_value_type = param.second;
        const bool is_last = (count + 1 == params.size());
        _parent->get_param_async(
            param_name,
            param_value_type,
            [param_name, is_last, this](
                MAVLinkParameters::Result result, MAVLinkParameters::ParamValue value) {
                if (result != MAVLinkParameters::Result::Success) {
                    return;
                }
                // We need to check again by the time this callback runs
                if (!this->_camera_definition) {
                    return;
                }

                if (!this->_camera_definition->set_setting(param_name, value)) {
                    return;
                }

                if (is_last) {
                    notify_current_settings();
                    notify_possible_setting_options();
                }
            },
            this,
            true);
        ++count;
    }
}

void CameraImpl::invalidate_params()
{
    if (!_camera_definition) {
        return;
    }

    _camera_definition->set_all_params_unknown();
}

bool CameraImpl::get_setting_str(const std::string& setting_id, std::string& description)
{
    if (!_camera_definition) {
        return false;
    }

    return _camera_definition->get_setting_str(setting_id, description);
}

bool CameraImpl::get_option_str(
    const std::string& setting_id, const std::string& option_id, std::string& description)
{
    if (!_camera_definition) {
        return false;
    }

    return _camera_definition->get_option_str(setting_id, option_id, description);
}

void CameraImpl::request_camera_settings()
{
    auto command_camera_settings = make_command_request_camera_settings();
    _parent->send_command_async(command_camera_settings, nullptr);
}

void CameraImpl::request_flight_information()
{
    auto command_flight_information = make_command_request_flight_information();
    _parent->send_command_async(command_flight_information, nullptr);
}

void CameraImpl::request_camera_information()
{
    auto command_camera_info = make_command_request_camera_info();
    _parent->send_command_async(command_camera_info, nullptr);
}

Camera::Result CameraImpl::format_storage()
{
    auto prom = std::make_shared<std::promise<Camera::Result>>();
    auto ret = prom->get_future();

    format_storage_async([prom](Camera::Result result) { prom->set_value(result); });

    return ret.get();
}

void CameraImpl::format_storage_async(Camera::ResultCallback callback)
{
    MavlinkCommandSender::CommandLong cmd_format{};

    cmd_format.command = MAV_CMD_STORAGE_FORMAT;
    cmd_format.params.param1 = 1.0f; // storage ID
    cmd_format.params.param2 = 1.0f; // format
    cmd_format.target_component_id = _camera_id + MAV_COMP_ID_CAMERA;

    _parent->send_command_async(
        cmd_format, std::bind(&CameraImpl::receive_command_result, this, _1, callback));
}

std::pair<Camera::Result, std::vector<Camera::CaptureInfo>>
CameraImpl::list_photos(Camera::PhotosRange photos_range)
{
    std::promise<std::pair<Camera::Result, std::vector<Camera::CaptureInfo>>> prom;
    auto ret = prom.get_future();

    list_photos_async(
        photos_range, [&prom](Camera::Result result, std::vector<Camera::CaptureInfo> photo_list) {
            prom.set_value(std::make_pair(result, photo_list));
        });

    return ret.get();
}

void CameraImpl::list_photos_async(
    Camera::PhotosRange photos_range, const Camera::ListPhotosCallback callback)
{
    if (!callback) {
        LogWarn() << "Trying to get a photo list with a null callback, ignoring...";
        return;
    }

    {
        std::lock_guard<std::mutex> lock(_status.mutex);

        if (_status.is_fetching_photos) {
            _parent->call_user_callback([callback]() {
                callback(Camera::Result::Busy, std::vector<Camera::CaptureInfo>{});
            });
            return;
        } else {
            _status.is_fetching_photos = true;
        }

        if (_status.image_count == -1) {
            LogErr() << "Cannot list photos: camera status has not been received yet!";
            _parent->call_user_callback([callback]() {
                callback(Camera::Result::Error, std::vector<Camera::CaptureInfo>{});
            });
            return;
        }
    }

    const int start_index = [this, photos_range]() {
        switch (photos_range) {
            case Camera::PhotosRange::SinceConnection:
                return _status.image_count_at_connection;
            case Camera::PhotosRange::All:
            // FALLTHROUGH
            default:
                return 0;
        }
    }();

    std::thread([this, start_index, callback]() {
        std::unique_lock<std::mutex> lock(_captured_request_mutex);

        for (int i = start_index; i < _status.image_count; i++) {
            // In case the vehicle sends capture info, but not those we are asking, we do not
            // want to loop infinitely. The safety_count is here to abort if this happens.
            auto safety_count = 0;
            const auto safety_count_boundary = 3;

            while (_status.photo_list.find(i) == _status.photo_list.end() &&
                   safety_count < safety_count_boundary) {
                safety_count++;

                auto request_try_number = 0;
                const auto request_try_limit =
                    3; // Timeout if the request times out that many times
                auto cv_status = std::cv_status::timeout;

                while (cv_status == std::cv_status::timeout) {
                    request_try_number++;
                    if (request_try_number >= request_try_limit) {
                        _parent->call_user_callback([callback]() {
                            callback(Camera::Result::Timeout, std::vector<Camera::CaptureInfo>{});
                        });
                        return;
                    }

                    _parent->send_command_async(
                        make_command_request_camera_image_captured(i), nullptr);
                    cv_status =
                        _captured_request_cv.wait_for(lock, std::chrono::duration<double>(1));
                }
            }

            if (safety_count == safety_count_boundary) {
                _parent->call_user_callback([callback]() {
                    callback(Camera::Result::Error, std::vector<Camera::CaptureInfo>{});
                });
                return;
            }
        }

        std::vector<Camera::CaptureInfo> photo_list;
        {
            std::lock_guard<std::mutex> status_lock(_status.mutex);

            for (auto capture_info : _status.photo_list) {
                if (capture_info.first >= start_index) {
                    photo_list.push_back(capture_info.second);
                }
            }

            _status.is_fetching_photos = false;

            const auto temp_callback = callback;
            _parent->call_user_callback([temp_callback, photo_list]() {
                temp_callback(Camera::Result::Success, photo_list);
            });
        }
    }).detach();
}

} // namespace mavsdk
