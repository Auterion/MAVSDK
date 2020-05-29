#include <memory>

#include "wifi_impl.h"
#include "mavsdk_impl.h"
#include "global_include.h"

namespace mavsdk {

using namespace std::placeholders; // for `_1`

WifiImpl::WifiImpl(System& system) : PluginImplBase(system)
{
    _parent->register_plugin(this);
}

WifiImpl::~WifiImpl()
{
    _parent->unregister_plugin(this);
}

void WifiImpl::init() {}

void WifiImpl::deinit()
{
    _parent->unregister_all_mavlink_message_handlers(this);
}

void WifiImpl::enable() {}
void WifiImpl::disable() {}

void WifiImpl::get_access_point_configuration_async(
    const Wifi::GetAccessPointConfigurationCallback callback) const
{
    void* message_cookie = nullptr; // TODO: Is this a way to get a random number?
    void* timeout_cookie = nullptr;
    // bool
    // enum with state (idle/waiting)

    _parent->register_mavlink_message_handler(
        MAVLINK_MSG_ID_WIFI_CONFIG_AP,
        [this, callback, &message_cookie, &timeout_cookie](
            const mavlink_message_t& mavlink_message) {
            //_parent->unregister_timeout_handler(&timeout_cookie);
            //_parent->unregister_mavlink_message_handler(
            // MAVLINK_MSG_ID_WIFI_CONFIG_AP, &message_cookie);

            LogErr() << "SPARTA - get received something";

            mavlink_wifi_config_ap_t mavlink_configuration;
            mavlink_msg_wifi_config_ap_decode(&mavlink_message, &mavlink_configuration);

            const auto result = extractResult(mavlink_configuration);
            const auto configuration = extractAccessPointConfiguration(mavlink_configuration);
            _parent->call_user_callback(
                [callback, result, configuration]() { callback(result, configuration); });
        },
        &message_cookie);

    MAVLinkCommands::CommandLong command_request_message;

    command_request_message.command = MAV_CMD_REQUEST_MESSAGE;
    command_request_message.params.param1 = MAVLINK_MSG_ID_WIFI_CONFIG_AP;
    command_request_message.target_component_id =
        MAV_COMP_ID_PATHPLANNER; // TODO this is hardcoded to the wifi module right now

    _parent->send_command_async(
        command_request_message, nullptr); // TODO: can the command be refused? -> treat that result
    _parent->register_timeout_handler(
        [this, callback, &message_cookie]() {
            //_parent->unregister_mavlink_message_handler(
            // MAVLINK_MSG_ID_WIFI_CONFIG_AP, &message_cookie);
            callback(Wifi::Result::Timeout, Wifi::AccessPointConfiguration{});
        },
        TIMEOUT_S,
        &timeout_cookie);
}

Wifi::AccessPointConfiguration WifiImpl::extractAccessPointConfiguration(
    const mavlink_wifi_config_ap_t& mavlink_configuration) const
{
    Wifi::AccessPointConfiguration configuration;
    configuration.ssid = std::string(mavlink_configuration.ssid);
    configuration.password = std::string(mavlink_configuration.password);

    switch (mavlink_configuration.mode) {
        case WIFI_CONFIG_AP_MODE_AP:
            configuration.mode = Wifi::Mode::AccessPoint;
            break;
        case WIFI_CONFIG_AP_MODE_STATION:
            configuration.mode = Wifi::Mode::Station;
            break;
        case WIFI_CONFIG_AP_MODE_UNDEFINED:
        default:
            configuration.mode = Wifi::Mode::Undefined;
            break;
    }

    return configuration;
}

Wifi::Result WifiImpl::extractResult(const mavlink_wifi_config_ap_t& mavlink_configuration) const
{
    switch (mavlink_configuration.response) {
        case WIFI_CONFIG_AP_RESPONSE_ACCEPTED:
            return Wifi::Result::Success;
        case WIFI_CONFIG_AP_RESPONSE_REJECTED:
            return Wifi::Result::Rejected;
        case WIFI_CONFIG_AP_RESPONSE_MODE_ERROR:
            return Wifi::Result::ModeError;
        case WIFI_CONFIG_AP_RESPONSE_SSID_ERROR:
            return Wifi::Result::SsidError;
        case WIFI_CONFIG_AP_RESPONSE_PASSWORD_ERROR:
            return Wifi::Result::PasswordError;
        case WIFI_CONFIG_AP_RESPONSE_UNDEFINED:
        default:
            return Wifi::Result::Unknown;
    }
}

std::pair<Wifi::Result, Wifi::AccessPointConfiguration>
WifiImpl::get_access_point_configuration() const
{
    auto prom = std::promise<std::pair<Wifi::Result, Wifi::AccessPointConfiguration>>();
    auto fut = prom.get_future();

    get_access_point_configuration_async(
        [&prom](Wifi::Result result, Wifi::AccessPointConfiguration configuration) {
            prom.set_value(std::make_pair(result, configuration));
        });

    return fut.get();
}

void WifiImpl::set_access_point_configuration_async(
    Wifi::AccessPointConfiguration configuration, const Wifi::ResultCallback callback) const
{
    void* message_cookie = nullptr;
    void* timeout_cookie = nullptr;

    _parent->register_mavlink_message_handler(
        MAVLINK_MSG_ID_WIFI_CONFIG_AP,
        [this, callback, &message_cookie, &timeout_cookie](
            const mavlink_message_t& mavlink_message) {
            //_parent->unregister_timeout_handler(&timeout_cookie);
            //_parent->unregister_mavlink_message_handler(
            // MAVLINK_MSG_ID_WIFI_CONFIG_AP, &message_cookie);

            mavlink_wifi_config_ap_t mavlink_configuration;
            mavlink_msg_wifi_config_ap_decode(&mavlink_message, &mavlink_configuration);

            LogErr() << "SPARTA - got message back";
            const auto result = extractResult(mavlink_configuration);
            _parent->call_user_callback([callback, result]() { callback(result); });
        },
        &message_cookie);

    int8_t mode;
    switch (configuration.mode) {
        case Wifi::Mode::AccessPoint:
            mode = WIFI_CONFIG_AP_MODE_AP;
            break;
        case Wifi::Mode::Station:
            mode = WIFI_CONFIG_AP_MODE_STATION;
            break;
        case Wifi::Mode::Undefined:
        default:
            mode = WIFI_CONFIG_AP_MODE_UNDEFINED;
            break;
    }

    int8_t irrelevant = 0;

    mavlink_message_t message{};
    mavlink_msg_wifi_config_ap_pack(
        _parent->get_own_system_id(),
        _parent->get_own_component_id(),
        &message,
        configuration.ssid.c_str(),
        configuration.password.c_str(),
        mode,
        irrelevant);

    if (!_parent->send_message(message)) {
        //_parent->unregister_mavlink_message_handler(MAVLINK_MSG_ID_WIFI_CONFIG_AP,
        //&message_cookie); _parent->call_user_callback([callback]() {
        // callback(Wifi::Result::Unknown);
        //}); // TODO: Is that really an unknown result?
    } else {
        _parent->register_timeout_handler(
            [this, callback, &message_cookie]() {
                //_parent->unregister_mavlink_message_handler(
                // MAVLINK_MSG_ID_WIFI_CONFIG_AP, &message_cookie);
                callback(Wifi::Result::Timeout);
            },
            TIMEOUT_S,
            &timeout_cookie);
    }
}

Wifi::Result
WifiImpl::set_access_point_configuration(Wifi::AccessPointConfiguration configuration) const
{
    auto prom = std::promise<Wifi::Result>();
    auto fut = prom.get_future();

    set_access_point_configuration_async(
        configuration, [&prom](Wifi::Result result) { prom.set_value(result); });

    return fut.get();
}

} // namespace mavsdk
