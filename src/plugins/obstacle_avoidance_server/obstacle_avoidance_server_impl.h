#pragma once

#include "plugins/obstacle_avoidance_server/obstacle_avoidance_server.h"
#include "plugin_impl_base.h"

namespace mavsdk {

class ObstacleAvoidanceServerImpl : public PluginImplBase {
public:
    explicit ObstacleAvoidanceServerImpl(System& system);
    explicit ObstacleAvoidanceServerImpl(std::shared_ptr<System> system);
    ~ObstacleAvoidanceServerImpl();

    void init() override;
    void deinit() override;

    void enable() override;
    void disable() override;

    mavlink_message_t
    process_component_control_command(const MavlinkCommandReceiver::CommandLong& command);

    ObstacleAvoidanceServer::Control control() const;

    void subscribe_control(ObstacleAvoidanceServer::ControlCallback callback);

private:
    void store_control(ObstacleAvoidanceServer::Control control);

    std::atomic<ObstacleAvoidanceServer::Control> _control{};

    std::mutex _subscription_mutex{};
    ObstacleAvoidanceServer::ControlCallback _component_control_command_subscription{nullptr};
};

} // namespace mavsdk
