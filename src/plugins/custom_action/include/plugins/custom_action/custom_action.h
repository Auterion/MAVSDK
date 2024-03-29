// WARNING: THIS FILE IS AUTOGENERATED! As such, it should not be edited.
// Edits need to be made to the proto files
// (see https://github.com/mavlink/MAVSDK-Proto/blob/main/protos/custom_action/custom_action.proto)

#pragma once

#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "plugin_base.h"

namespace mavsdk {

class System;
class CustomActionImpl;

/**
 * @brief Allows to send, receive and process custom actions, which description and
 * configuration are defined in a JSON file.
 */
class CustomAction : public PluginBase {
public:
    /**
     * @brief Constructor. Creates the plugin for a specific System.
     *
     * The plugin is typically created as shown below:
     *
     *     ```cpp
     *     auto custom_action = CustomAction(system);
     *     ```
     *
     * @param system The specific system associated with this plugin.
     */
    explicit CustomAction(System& system); // deprecated

    /**
     * @brief Constructor. Creates the plugin for a specific System.
     *
     * The plugin is typically created as shown below:
     *
     *     ```cpp
     *     auto custom_action = CustomAction(system);
     *     ```
     *
     * @param system The specific system associated with this plugin.
     */
    explicit CustomAction(std::shared_ptr<System> system); // new

    /**
     * @brief Destructor (internal use only).
     */
    ~CustomAction();

    /**
     * @brief Used to identify action to be executed, its timeout / max execution time,
     * and, while being processed, its execution progress, which is used to send
     * MAVLink command ACKs with progress to the autopilot side.
     */
    struct ActionToExecute {
        uint32_t id{}; /**< @brief ID of the action */
        double timeout{}; /**< @brief Action timeout / max execution time */
        double progress{}; /**< @brief Action progress */
    };

    /**
     * @brief Equal operator to compare two `CustomAction::ActionToExecute` objects.
     *
     * @return `true` if items are equal.
     */
    friend bool
    operator==(const CustomAction::ActionToExecute& lhs, const CustomAction::ActionToExecute& rhs);

    /**
     * @brief Stream operator to print information about a `CustomAction::ActionToExecute`.
     *
     * @return A reference to the stream.
     */
    friend std::ostream&
    operator<<(std::ostream& str, CustomAction::ActionToExecute const& action_to_execute);

    /**
     * @brief General definition of a COMMAND_LONG or a COMMAND_INT MAVLink message to be
     * sent and executed during a custom action.
     */
    struct Command {
        /**
         * @brief Command type enumeration
         */
        enum class CommandType {
            Long, /**< @brief Command long. */
            Int, /**< @brief Command int. */
        };

        /**
         * @brief Stream operator to print information about a `CustomAction::CommandType`.
         *
         * @return A reference to the stream.
         */
        friend std::ostream&
        operator<<(std::ostream& str, CustomAction::Command::CommandType const& command_type);

        CommandType type{}; /**< @brief Type enum value. LONG or INT */
        uint32_t target_system_id{}; /**< @brief Target system ID */
        uint32_t
            target_component_id{}; /**< @brief Target component ID. Should match the MAV_COMP */
        uint32_t frame{}; /**< @brief The coordinate system of the COMMAND. Used in COMMAND_INT */
        uint32_t command{}; /**< @brief Command to send to target system and component. Should match
                               the MAV_CMD */
        double param1{}; /**< @brief Command parameter 1 */
        double param2{}; /**< @brief Command parameter 2 */
        double param3{}; /**< @brief Command parameter 3 */
        double param4{}; /**< @brief Command parameter 4 */
        double param5{}; /**< @brief Command parameter 5. In COMMAND_INT: local x position or
                            latitude. Casted to int32 before sending the command */
        double param6{}; /**< @brief Command parameter 6. In COMMAND_INT: local y position or
                            longitude. Casted to int32 before sending the command */
        double param7{}; /**< @brief Command parameter 7. In COMMAND_INT: z position: global:
                            altitude in meters (relative or absolute, depending on frame) */
        bool is_local{}; /**< @brief In COMMAND_INT: Set to true if x/y are local positions.
                            Otherwise, these are lat/lon */
    };

    /**
     * @brief Equal operator to compare two `CustomAction::Command` objects.
     *
     * @return `true` if items are equal.
     */
    friend bool operator==(const CustomAction::Command& lhs, const CustomAction::Command& rhs);

    /**
     * @brief Stream operator to print information about a `CustomAction::Command`.
     *
     * @return A reference to the stream.
     */
    friend std::ostream& operator<<(std::ostream& str, CustomAction::Command const& command);

    /**
     * @brief Used to define a parameter to be set by the MAVSDK available API.
     */
    struct Parameter {
        /**
         * @brief Parameter type enumeration.
         */
        enum class ParameterType {
            Int, /**< @brief MAV_PARAM_TYPE_ intenger types.. */
            Float, /**< @brief MAV_PARAM_TYPE_ floating point types.. */
        };

        /**
         * @brief Stream operator to print information about a `CustomAction::ParameterType`.
         *
         * @return A reference to the stream.
         */
        friend std::ostream&
        operator<<(std::ostream& str, CustomAction::Parameter::ParameterType const& parameter_type);

        ParameterType type{}; /**< @brief Type enum value. INT or FLOAT. */
        std::string name{}; /**< @brief Parameter name. */
        float value{}; /**< @brief Parameter value. Defaults to float, but can be truncated to an
                          int. */
    };

    /**
     * @brief Equal operator to compare two `CustomAction::Parameter` objects.
     *
     * @return `true` if items are equal.
     */
    friend bool operator==(const CustomAction::Parameter& lhs, const CustomAction::Parameter& rhs);

    /**
     * @brief Stream operator to print information about a `CustomAction::Parameter`.
     *
     * @return A reference to the stream.
     */
    friend std::ostream& operator<<(std::ostream& str, CustomAction::Parameter const& parameter);

    /**
     * @brief Defines totally or partially a custom action. Can be a MAVLink command or a
     * script (with full or relative path).
     */
    struct Stage {
        /**
         * @brief State transition condition enumeration.
         */
        enum class StateTransitionCondition {
            OnResultSuccess, /**< @brief Transitions to the next stage case the script/command is
                                successful.. */
            OnTimeout, /**< @brief Transitions to the next stage after a defined time.. */
            OnLandingComplete, /**< @brief Transitions to the next stage after the vehicle is
                                  landed.. */
            OnTakeoffComplete, /**< @brief Transitions to the next stage after the vehicle finishes
                                  takeoff.. */
            OnModeChange, /**< @brief Transitions to the next stage after the vehicle changes from
                             one user-specified fligght mode to another.. */
            OnCustomConditionTrue, /**< @brief Transitions to the next stage after a user-specified
                                      condition is true.. */
            OnCustomConditionFalse, /**< @brief Transitions to the next stage after a user-specified
                                       condition is false.. */
        };

        /**
         * @brief Stream operator to print information about a
         * `CustomAction::StateTransitionCondition`.
         *
         * @return A reference to the stream.
         */
        friend std::ostream& operator<<(
            std::ostream& str,
            CustomAction::Stage::StateTransitionCondition const& state_transition_condition);

        Command command{}; /**< @brief Command to run in the stage (if applicable). */
        std::string script{}; /**< @brief Script to run in that stage (if applicable). */
        Parameter parameter_set{}; /**< @brief Parameter to set in the stage (if applicable). */
        StateTransitionCondition
            state_transition_condition{}; /**< @brief State transition condition enum value. */
        double timeout{}; /**< @brief Time in seconds when the stage should stop. */
    };

    /**
     * @brief Equal operator to compare two `CustomAction::Stage` objects.
     *
     * @return `true` if items are equal.
     */
    friend bool operator==(const CustomAction::Stage& lhs, const CustomAction::Stage& rhs);

    /**
     * @brief Stream operator to print information about a `CustomAction::Stage`.
     *
     * @return A reference to the stream.
     */
    friend std::ostream& operator<<(std::ostream& str, CustomAction::Stage const& stage);

    /**
     * @brief Metadata that describes the custom action and defines its stages.
     */
    struct ActionMetadata {
        /**
         * @brief State transition condition enumeration.
         */
        enum class ActionCompleteCondition {
            OnLastStageComplete, /**< @brief Action is complete when the last stage is complete.. */
            OnTimeout, /**< @brief Action is complete when a defined time as passed.. */
            OnResultSuccess, /**< @brief Action is complete when the script/command is successful..
                              */
            OnCustomConditionTrue, /**< @brief Action is complete when user-specified condition is
                                      true.. */
            OnCustomConditionFalse, /**< @brief Action is complete whenr a user-specified condition
                                       is false.. */
        };

        /**
         * @brief Stream operator to print information about a
         * `CustomAction::ActionCompleteCondition`.
         *
         * @return A reference to the stream.
         */
        friend std::ostream& operator<<(
            std::ostream& str,
            CustomAction::ActionMetadata::ActionCompleteCondition const& action_complete_condition);

        uint32_t id{}; /**< @brief ID of the action */
        std::string action_name{}; /**< @brief Name of the action */
        std::string action_description{}; /**< @brief Description of the action */
        std::string global_script{}; /**< @brief Script to run for this specific action. Runs
                                        instead of the stages. */
        double global_timeout{}; /**< @brief Timeout for the action in seconds. If a global script
                                    is set, it is used as a timeout for the script. Otherwise, for a
                                    staged action, defines the global timeout for the action.
                                    independently of the state of the stage processing. */
        ActionCompleteCondition
            action_complete_condition{}; /**< @brief Action complete condition enum value */
        std::vector<Stage>
            stages{}; /**< @brief Timestamped ordered stages. Runs instead of the global script. */
    };

    /**
     * @brief Equal operator to compare two `CustomAction::ActionMetadata` objects.
     *
     * @return `true` if items are equal.
     */
    friend bool
    operator==(const CustomAction::ActionMetadata& lhs, const CustomAction::ActionMetadata& rhs);

    /**
     * @brief Stream operator to print information about a `CustomAction::ActionMetadata`.
     *
     * @return A reference to the stream.
     */
    friend std::ostream&
    operator<<(std::ostream& str, CustomAction::ActionMetadata const& action_metadata);

    /**
     * @brief Possible results returned for action requests.
     */
    enum class Result {
        Unknown, /**< @brief Unknown result. */
        Success, /**< @brief Command was accepted. */
        Error, /**< @brief Error occurred sending the command. */
        Timeout, /**< @brief Command timed out. */
        Unsupported, /**< @brief Functionality not supported. */
        InProgress, /**< @brief Command in progress. */
    };

    /**
     * @brief Stream operator to print information about a `CustomAction::Result`.
     *
     * @return A reference to the stream.
     */
    friend std::ostream& operator<<(std::ostream& str, CustomAction::Result const& result);

    /**
     * @brief Callback type for asynchronous CustomAction calls.
     */
    using ResultCallback = std::function<void(Result)>;

    /**
     * @brief Send custom action command to the system.
     *
     * This function is non-blocking. See 'set_custom_action' for the blocking counterpart.
     */
    void set_custom_action_async(ActionToExecute action_to_execute, const ResultCallback callback);

    /**
     * @brief Send custom action command to the system.
     *
     * This function is blocking. See 'set_custom_action_async' for the non-blocking counterpart.
     *
     * @return Result of request.
     */
    Result set_custom_action(ActionToExecute action_to_execute) const;

    /**
     * @brief Callback type for subscribe_custom_action.
     */

    using CustomActionCallback = std::function<void(ActionToExecute)>;

    /**
     * @brief Receive and process custom action command.
     */
    void subscribe_custom_action(CustomActionCallback callback);

    /**
     * @brief Poll for 'ActionToExecute' (blocking).
     *
     * @return One ActionToExecute update.
     */
    ActionToExecute custom_action() const;

    /**
     * @brief Callback type for subscribe_custom_action_cancellation.
     */

    using CustomActionCancellationCallback = std::function<void(bool)>;

    /**
     * @brief Receive and process custom action command cancellation.
     */
    void subscribe_custom_action_cancellation(CustomActionCancellationCallback callback);

    /**
     * @brief Poll for 'bool' (blocking).
     *
     * @return One bool update.
     */
    bool custom_action_cancellation() const;

    /**
     * @brief Respond to the custom action command with progress.
     *
     * This function is non-blocking. See 'respond_custom_action' for the blocking counterpart.
     */
    void respond_custom_action_async(
        ActionToExecute action_to_execute,
        Result custom_action_result,
        const ResultCallback callback);

    /**
     * @brief Respond to the custom action command with progress.
     *
     * This function is blocking. See 'respond_custom_action_async' for the non-blocking
     * counterpart.
     *
     * @return Result of request.
     */
    Result
    respond_custom_action(ActionToExecute action_to_execute, Result custom_action_result) const;

    /**
     * @brief Callback type for custom_action_metadata_async.
     */
    using CustomActionMetadataCallback = std::function<void(Result, ActionMetadata)>;

    /**
     * @brief Request custom action metadata.
     *
     * This function is non-blocking. See 'custom_action_metadata' for the blocking counterpart.
     */
    void custom_action_metadata_async(
        ActionToExecute action_to_execute,
        std::string file_path,
        const CustomActionMetadataCallback callback);

    /**
     * @brief Request custom action metadata.
     *
     * This function is blocking. See 'custom_action_metadata_async' for the non-blocking
     * counterpart.
     *
     * @return Result of request.
     */
    std::pair<Result, CustomAction::ActionMetadata>
    custom_action_metadata(ActionToExecute action_to_execute, std::string file_path) const;

    /**
     * @brief Execute custom action stage.
     *
     * This function is non-blocking. See 'execute_custom_action_stage' for the blocking
     * counterpart.
     */
    void execute_custom_action_stage_async(Stage stage, const ResultCallback callback);

    /**
     * @brief Execute custom action stage.
     *
     * This function is blocking. See 'execute_custom_action_stage_async' for the non-blocking
     * counterpart.
     *
     * @return Result of request.
     */
    Result execute_custom_action_stage(Stage stage) const;

    /**
     * @brief Execute custom action global script.
     *
     * This function is non-blocking. See 'execute_custom_action_global_script' for the blocking
     * counterpart.
     */
    void execute_custom_action_global_script_async(
        std::string global_script, const ResultCallback callback);

    /**
     * @brief Execute custom action global script.
     *
     * This function is blocking. See 'execute_custom_action_global_script_async' for the
     * non-blocking counterpart.
     *
     * @return Result of request.
     */
    Result execute_custom_action_global_script(std::string global_script) const;

    /**
     * @brief Copy constructor.
     */
    CustomAction(const CustomAction& other);

    /**
     * @brief Equality operator (object is not copyable).
     */
    const CustomAction& operator=(const CustomAction&) = delete;

private:
    /** @private Underlying implementation, set at instantiation */
    std::unique_ptr<CustomActionImpl> _impl;
};

} // namespace mavsdk