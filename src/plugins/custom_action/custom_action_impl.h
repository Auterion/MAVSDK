#pragma once

#include "plugins/custom_action/custom_action.h"
#include "plugin_impl_base.h"
#include <json/json.h>

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

    MavlinkCommandReceiver::Result
    process_custom_action_command(const MavlinkCommandReceiver::CommandLong& command);

    CustomAction::Result
    respond_custom_action(CustomAction::ActionToExecute action, CustomAction::Result result) const;

    void respond_custom_action_async(
        CustomAction::ActionToExecute action,
        CustomAction::Result result,
        const CustomAction::ResultCallback& callback) const;

    CustomAction::Result set_custom_action(CustomAction::ActionToExecute& action) const;

    void set_custom_action_async(
        CustomAction::ActionToExecute& action, const CustomAction::ResultCallback& callback) const;

    CustomAction::ActionToExecute custom_action() const;

    void custom_action_async(CustomAction::CustomActionCallback callback);

    std::pair<CustomAction::Result, CustomAction::ActionMetadata>
    custom_action_metadata(CustomAction::ActionToExecute& action, std::string& file) const;

    void custom_action_metadata_async(
        CustomAction::ActionToExecute& action,
        const std::string& file,
        const CustomAction::CustomActionMetadataCallback& callback) const;

    void command_result_callback(
        MavlinkCommandSender::Result command_result,
        const CustomAction::ResultCallback& callback) const;

private:
    static CustomAction::Result
    custom_action_result_from_command_result(MavlinkCommandSender::Result result);

    static MAV_RESULT mavlink_command_result_from_custom_action_result(CustomAction::Result result);

    void store_custom_action(CustomAction::ActionToExecute action);

    mutable std::mutex _custom_action_mutex{};
    CustomAction::ActionToExecute _custom_action{};

    std::mutex _subscription_mutex{};
    CustomAction::CustomActionCallback _custom_action_command_subscription{nullptr};
};

} // namespace mavsdk
