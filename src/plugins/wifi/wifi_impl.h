#pragma once

#include "mavlink_include.h"
#include "plugins/wifi/wifi.h"
#include "plugin_impl_base.h"

namespace mavsdk {

class WifiImpl : public PluginImplBase {
public:
    WifiImpl(System& system);
    ~WifiImpl();

    void init() override;
    void deinit() override;

    void enable() override;
    void disable() override;

    std::pair<Wifi::Result, Wifi::AccessPointConfiguration> get_access_point_configuration() const;
    Wifi::Result set_access_point_configuration(Wifi::AccessPointConfiguration configuration) const;

    void get_access_point_configuration_async(
        const Wifi::GetAccessPointConfigurationCallback callback) const;
    void set_access_point_configuration_async(
        Wifi::AccessPointConfiguration configuration, const Wifi::ResultCallback callback) const;

private:
    Wifi::AccessPointConfiguration
    extractAccessPointConfiguration(const mavlink_wifi_config_ap_t& mavlink_configuration) const;
    Wifi::Result extractResult(const mavlink_wifi_config_ap_t& mavlink_configuration) const;

    const double ARBITRARY_INT = 42;
    const double TIMEOUT_S = 5;
};

} // namespace mavsdk
