#pragma once

#include "global_include.h"
#include "mavlink_address.h"
#include "mavlink_include.h"
#include "mavlink_parameters.h"
#include "mavlink_commands.h"
#include "mavlink_message_handler.h"
#include "mavlink_mission_transfer.h"
#include "mavlink_statustext_handler.h"
#include "ping.h"
#include "timeout_handler.h"
#include "safe_queue.h"
#include "timesync.h"
#include "system.h"
#include <cstdint>
#include <functional>
#include <atomic>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <thread>
#include <mutex>
#include <future>

namespace mavsdk {

class MavsdkImpl;
class PluginImplBase;

// This class is the impl of System. This is to hide the private methods
// and functionality from the public library API.
class SystemImpl : public Sender {
public:
    enum class FlightMode {
        Unknown,
        Ready,
        Takeoff,
        Hold,
        Mission,
        ReturnToLaunch,
        Land,
        Offboard,
        FollowMe,
        Manual,
        Altctl,
        Posctl,
        Acro,
        Rattitude,
        Stabilized,
    };

    explicit SystemImpl(
        MavsdkImpl& parent, uint8_t system_id, uint8_t component_id, bool connected);
    ~SystemImpl();

    void enable_timesync();

    void subscribe_is_connected(System::IsConnectedCallback callback);

    void process_mavlink_message(mavlink_message_t& message);

    typedef std::function<void(const mavlink_message_t&)> mavlink_message_handler_t;

    void register_mavlink_message_handler(
        uint16_t msg_id, mavlink_message_handler_t callback, const void* cookie);

    void unregister_mavlink_message_handler(uint16_t msg_id, const void* cookie);
    void unregister_all_mavlink_message_handlers(const void* cookie);

    void register_timeout_handler(
        const std::function<void()>& callback, double duration_s, void** cookie);
    void refresh_timeout_handler(const void* cookie);
    void unregister_timeout_handler(const void* cookie);

    void add_call_every(std::function<void()> callback, float interval_s, void** cookie);
    void change_call_every(float interval_s, const void* cookie);
    void reset_call_every(const void* cookie);
    void remove_call_every(const void* cookie);

    bool send_message(mavlink_message_t& message) override;

    static FlightMode to_flight_mode_from_custom_mode(uint32_t custom_mode);

    using CommandResultCallback = MavlinkCommandSender::CommandResultCallback;

    MavlinkCommandSender::Result send_command(MavlinkCommandSender::CommandLong& command);
    MavlinkCommandSender::Result send_command(MavlinkCommandSender::CommandInt& command);

    void send_command_async(
        MavlinkCommandSender::CommandLong command, const CommandResultCallback callback);
    void send_command_async(
        MavlinkCommandSender::CommandInt command, const CommandResultCallback callback);

    MavlinkCommandSender::Result set_msg_rate(
        uint16_t message_id, double rate_hz, uint8_t component_id = MAV_COMP_ID_AUTOPILOT1);

    void set_msg_rate_async(
        uint16_t message_id,
        double rate_hz,
        CommandResultCallback callback,
        uint8_t component_id = MAV_COMP_ID_AUTOPILOT1);

    // Adds unique component ids
    void add_new_component(uint8_t component_id);
    size_t total_components() const;

    void register_component_discovered_callback(discover_callback_t callback);

    uint8_t get_autopilot_id() const;
    std::vector<uint8_t> get_camera_ids() const;
    uint8_t get_gimbal_id() const;

    bool is_standalone() const;
    bool has_autopilot() const;
    bool has_camera(int camera_id = -1) const;
    bool has_gimbal() const;

    uint64_t get_uuid() const;
    uint8_t get_system_id() const;

    void set_system_id(uint8_t system_id);

    uint8_t get_own_system_id() const;
    uint8_t get_own_component_id() const;
    uint8_t get_own_mav_type() const;

    bool does_support_mission_int() const { return _supports_mission_int; }

    bool is_armed() const { return _armed; }

    MAVLinkParameters::Result set_param_float(const std::string& name, float value);
    MAVLinkParameters::Result set_param_int(const std::string& name, int32_t value);
    MAVLinkParameters::Result set_param_ext_float(const std::string& name, float value);
    MAVLinkParameters::Result set_param_ext_int(const std::string& name, int32_t value);
    std::map<std::string, MAVLinkParameters::ParamValue> get_all_params();

    typedef std::function<void(MAVLinkParameters::Result result)> success_t;
    void set_param_float_async(
        const std::string& name, float value, success_t callback, const void* cookie);
    void set_param_int_async(
        const std::string& name, int32_t value, success_t callback, const void* cookie);
    void set_param_ext_float_async(
        const std::string& name, float value, success_t callback, const void* cookie);
    void set_param_ext_int_async(
        const std::string& name, int32_t value, success_t callback, const void* cookie);

    using SubscribeParamIntCallback = std::function<void(int)>;
    void subscribe_param_int(
        const std::string& name, SubscribeParamIntCallback callback, const void* cookie);
    using SubscribeParamFloatCallback = std::function<void(float)>;
    void subscribe_param_float(
        const std::string& name, SubscribeParamFloatCallback callback, const void* cookie);

    FlightMode get_flight_mode() const;

    MavlinkCommandSender::Result
    set_flight_mode(FlightMode mode, uint8_t component_id = MAV_COMP_ID_AUTOPILOT1);

    void set_flight_mode_async(
        FlightMode mode,
        CommandResultCallback callback,
        uint8_t component_id = MAV_COMP_ID_AUTOPILOT1);

    typedef std::function<void(MAVLinkParameters::Result result, float value)>
        get_param_float_callback_t;
    typedef std::function<void(MAVLinkParameters::Result result, int32_t value)>
        get_param_int_callback_t;

    std::pair<MAVLinkParameters::Result, float> get_param_float(const std::string& name);
    std::pair<MAVLinkParameters::Result, int> get_param_int(const std::string& name);
    std::pair<MAVLinkParameters::Result, float> get_param_ext_float(const std::string& name);
    std::pair<MAVLinkParameters::Result, int> get_param_ext_int(const std::string& name);

    // These methods can be used to cache a parameter when a system connects. For that
    // the callback can just be set to nullptr.
    void get_param_float_async(
        const std::string& name, get_param_float_callback_t callback, const void* cookie);
    void get_param_int_async(
        const std::string& name, get_param_int_callback_t callback, const void* cookie);
    void get_param_ext_float_async(
        const std::string& name, get_param_float_callback_t callback, const void* cookie);
    void get_param_ext_int_async(
        const std::string& name, get_param_int_callback_t callback, const void* cookie);

    typedef std::function<void(
        MAVLinkParameters::Result result, MAVLinkParameters::ParamValue value)>
        get_param_callback_t;

    void set_param_async(
        const std::string& name,
        MAVLinkParameters::ParamValue value,
        success_t callback,
        const void* cookie,
        bool extended = false);

    MAVLinkParameters::Result
    set_param(const std::string& name, MAVLinkParameters::ParamValue value, bool extended = false);

    void get_param_async(
        const std::string& name,
        MAVLinkParameters::ParamValue value_type,
        get_param_callback_t callback,
        const void* cookie,
        bool extended);

    void cancel_all_param(const void* cookie);

    void param_changed(const std::string& name);

    typedef std::function<void(const std::string& name)> param_changed_callback_t;
    void
    register_param_changed_handler(const param_changed_callback_t callback, const void* cookie);
    void unregister_param_changed_handler(const void* cookie);

    bool is_connected() const;

    Time& get_time() { return _time; };
    AutopilotTime& get_autopilot_time() { return _autopilot_time; };

    double get_ping_time_s() const { return _ping.last_ping_time_s(); }

    void register_plugin(PluginImplBase* plugin_impl);
    void unregister_plugin(PluginImplBase* plugin_impl);

    void call_user_callback_located(
        const std::string& filename, const int linenumber, const std::function<void()>& func);

    void send_autopilot_version_request();
    void send_flight_information_request();

    MAVLinkMissionTransfer& mission_transfer() { return _mission_transfer; };

    void intercept_incoming_messages(std::function<bool(mavlink_message_t&)> callback);
    void intercept_outgoing_messages(std::function<bool(mavlink_message_t&)> callback);

    // Non-copyable
    SystemImpl(const SystemImpl&) = delete;
    const SystemImpl& operator=(const SystemImpl&) = delete;

    void register_mavlink_command_handler(
        uint16_t cmd_id,
        MavlinkCommandReceiver::MavlinkCommandIntHandler callback,
        const void* cookie);
    void register_mavlink_command_handler(
        uint16_t cmd_id,
        MavlinkCommandReceiver::MavlinkCommandLongHandler callback,
        const void* cookie);
    void unregister_mavlink_command_handler(uint16_t cmd_id, const void* cookie);
    void unregister_all_mavlink_command_handlers(const void* cookie);

private:
    static bool is_autopilot(uint8_t comp_id);
    static bool is_camera(uint8_t comp_id);

    void request_autopilot_version();

    bool have_uuid() const { return _uuid != 0 && _uuid_initialized; }

    void process_heartbeat(const mavlink_message_t& message);
    void process_autopilot_version(const mavlink_message_t& message);
    void process_statustext(const mavlink_message_t& message);
    void heartbeats_timed_out();
    void set_connected();
    void set_disconnected();

    static std::string component_name(uint8_t component_id);
    static ComponentType component_type(uint8_t component_id);

    void system_thread();

    // We use std::pair instead of a std::optional.
    std::pair<MavlinkCommandSender::Result, MavlinkCommandSender::CommandLong>
    make_command_flight_mode(FlightMode mode, uint8_t component_id);

    MavlinkCommandSender::CommandLong
    make_command_msg_rate(uint16_t message_id, double rate_hz, uint8_t component_id);

    static void receive_float_param(
        MAVLinkParameters::Result result,
        MAVLinkParameters::ParamValue value,
        get_param_float_callback_t callback);
    static void receive_int_param(
        MAVLinkParameters::Result result,
        MAVLinkParameters::ParamValue value,
        get_param_int_callback_t callback);

    std::mutex _component_discovered_callback_mutex{};
    discover_callback_t _component_discovered_callback{nullptr};

    MAVLinkAddress _target_address{};

    Time _time{};
    AutopilotTime _autopilot_time{};

    // Needs to be before anything else because they can depend on it.
    MAVLinkMessageHandler _message_handler{};

    MavlinkStatustextHandler _statustext_handler{};

    uint64_t _uuid{0};

    int _uuid_retries = 0;
    std::atomic<bool> _uuid_initialized{false};

    bool _supports_mission_int{false};
    std::atomic<bool> _armed{false};
    std::atomic<bool> _hitl_enabled{false};
    bool _always_connected{false};

    MavsdkImpl& _parent;

    CommandResultCallback _command_result_callback{nullptr};

    std::thread* _system_thread{nullptr};
    std::atomic<bool> _should_exit{false};

    static constexpr double _HEARTBEAT_TIMEOUT_S = 3.0;

    std::mutex _connection_mutex{};
    std::atomic<bool> _connected{false};
    System::IsConnectedCallback _is_connected_callback{nullptr};
    void* _heartbeat_timeout_cookie = nullptr;

    std::atomic<bool> _autopilot_version_pending{false};
    void* _autopilot_version_timed_out_cookie = nullptr;

    static constexpr double _ping_interval_s = 5.0;

    MAVLinkParameters _params;
    MavlinkCommandSender _send_commands;
    MavlinkCommandReceiver _receive_commands;

    Timesync _timesync;
    Ping _ping;

    MAVLinkMissionTransfer _mission_transfer;

    std::mutex _plugin_impls_mutex{};
    std::vector<PluginImplBase*> _plugin_impls{};

    // We used set to maintain unique component ids
    std::unordered_set<uint8_t> _components{};

    std::mutex _param_changed_callbacks_mutex{};
    std::unordered_map<const void*, param_changed_callback_t> _param_changed_callbacks{};

    std::function<bool(mavlink_message_t&)> _incoming_messages_intercept_callback{nullptr};
    std::function<bool(mavlink_message_t&)> _outgoing_messages_intercept_callback{nullptr};

    std::atomic<FlightMode> _flight_mode{FlightMode::Unknown};
};

} // namespace mavsdk
