#pragma once

#include "plugins/obstacle_avoidance/obstacle_avoidance.h"
#include "plugin_impl_base.h"

namespace mavsdk {

class ObstacleAvoidanceImpl : public PluginImplBase {
public:
    explicit ObstacleAvoidanceImpl(System& system);
    explicit ObstacleAvoidanceImpl(std::shared_ptr<System> system);
    ~ObstacleAvoidanceImpl();

    void init() override;
    void deinit() override;

    void enable() override;
    void disable() override;

    void start_async(const ObstacleAvoidance::ResultCallback callback);
    ObstacleAvoidance::Result start();

    void stop_async(const ObstacleAvoidance::ResultCallback callback);
    ObstacleAvoidance::Result stop();

    void restart_async(const ObstacleAvoidance::ResultCallback callback);
    ObstacleAvoidance::Result restart();

    void state_enable_async(const ObstacleAvoidance::ResultCallback callback);
    ObstacleAvoidance::Result state_enable();

    void state_disable_async(const ObstacleAvoidance::ResultCallback callback);
    ObstacleAvoidance::Result state_disable();

private:
    static ObstacleAvoidance::Result
    obstacle_avoidance_result_from_command_result(MavlinkCommandSender::Result result);

    void command_result_callback(
        MavlinkCommandSender::Result command_result,
        const ObstacleAvoidance::ResultCallback& callback) const;
};

} // namespace mavsdk
