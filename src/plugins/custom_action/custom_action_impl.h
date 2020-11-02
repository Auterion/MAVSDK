#pragma once

#include "plugins/custom_action/custom_action.h"
#include "plugin_impl_base.h"

namespace mavsdk {

class CustomActionImpl : public PluginImplBase {
public:
    explicit CustomActionImpl(System& system);
    explicit CustomActionImpl(std::shared_ptr<System> system);
    ~CustomActionImpl();

    void init() override;
    void deinit() override;

    void enable() override;
    void disable() override;

    CustomAction::Result set_custom_action() const;

    void set_custom_action_async(const CustomAction::ResultCallback& callback) const;

    // void subscribe_custom_action(CustomAction::CustomActionCallback callback);

    // ActionToExecute custom_action();

    void command_result_callback(
        MavlinkCommandSender::Result command_result,
        const CustomAction::ResultCallback& callback) const;

private:
    static CustomAction::Result
    custom_action_result_from_command_result(MavlinkCommandSender::Result result);
};

} // namespace mavsdk
