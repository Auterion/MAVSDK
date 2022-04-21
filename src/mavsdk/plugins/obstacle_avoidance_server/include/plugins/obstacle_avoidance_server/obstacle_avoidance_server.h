// WARNING: THIS FILE IS AUTOGENERATED! As such, it should not be edited.
// Edits need to be made to the proto files
// (see
// https://github.com/mavlink/MAVSDK-Proto/blob/main/protos/obstacle_avoidance_server/obstacle_avoidance_server.proto)

#pragma once

#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "mavsdk/plugin_base.h"

namespace mavsdk {

class System;
class ObstacleAvoidanceServerImpl;

/**
 * @brief Companion computer/server side API to manage and control obstacle avoidance
 * services.
 *
 * Currently a single obstacle avoidance service instance is supported which
 * needs to have component ID MAV_COMP_ID_OBSTACLE_AVOIDANCE.
 *
 * Note also that application/service specific configurations should live in
 * the application layer, as they are not defined at the MAVLink level.
 */
class ObstacleAvoidanceServer : public PluginBase {
public:
    /**
     * @brief Constructor. Creates the plugin for a specific System.
     *
     * The plugin is typically created as shown below:
     *
     *     ```cpp
     *     auto obstacle_avoidance_server = ObstacleAvoidanceServer(system);
     *     ```
     *
     * @param system The specific system associated with this plugin.
     */
    explicit ObstacleAvoidanceServer(System& system); // deprecated

    /**
     * @brief Constructor. Creates the plugin for a specific System.
     *
     * The plugin is typically created as shown below:
     *
     *     ```cpp
     *     auto obstacle_avoidance_server = ObstacleAvoidanceServer(system);
     *     ```
     *
     * @param system The specific system associated with this plugin.
     */
    explicit ObstacleAvoidanceServer(std::shared_ptr<System> system); // new

    /**
     * @brief Destructor (internal use only).
     */
    ~ObstacleAvoidanceServer();

    /**
     * @brief Control type.
     */
    struct Control {
        /**
         * @brief Possible obstacle avoidance service control commands, according to
         * MAVLink COMPONENT_CONTROL enum.
         */
        enum class ControlType {
            Unknown, /**< @brief Unknown control command.. */
            Start, /**< @brief Start/turn-on obstacle avoidance service.. */
            Stop, /**< @brief Stop/turn-off obstacle avoidance service.. */
            Restart, /**< @brief Restart/reboot obstacle avoidance service.. */
            Enable, /**< @brief Enable obstacle avoidance service. Used to switch the service from
                       an idle state to an active state.. */
            Disable, /**< @brief Disable obstacle avoidance service. Used to switch the service from
                        an active state to an idle state.. */
        };

        /**
         * @brief Stream operator to print information about a
         * `ObstacleAvoidanceServer::ControlType`.
         *
         * @return A reference to the stream.
         */
        friend std::ostream& operator<<(
            std::ostream& str, ObstacleAvoidanceServer::Control::ControlType const& control_type);

        ControlType control_type{}; /**< @brief Control type enum value. */
    };

    /**
     * @brief Equal operator to compare two `ObstacleAvoidanceServer::Control` objects.
     *
     * @return `true` if items are equal.
     */
    friend bool operator==(
        const ObstacleAvoidanceServer::Control& lhs, const ObstacleAvoidanceServer::Control& rhs);

    /**
     * @brief Stream operator to print information about a `ObstacleAvoidanceServer::Control`.
     *
     * @return A reference to the stream.
     */
    friend std::ostream&
    operator<<(std::ostream& str, ObstacleAvoidanceServer::Control const& control);

    /**
     * @brief Callback type for subscribe_control.
     */

    using ControlCallback = std::function<void(Control)>;

    /**
     * @brief Receive and process obstacle avoidance service control commands.
     */
    void subscribe_control(ControlCallback callback);

    /**
     * @brief Poll for 'Control' (blocking).
     *
     * @return One Control update.
     */
    Control control() const;

    /**
     * @brief Copy constructor.
     */
    ObstacleAvoidanceServer(const ObstacleAvoidanceServer& other);

    /**
     * @brief Equality operator (object is not copyable).
     */
    const ObstacleAvoidanceServer& operator=(const ObstacleAvoidanceServer&) = delete;

private:
    /** @private Underlying implementation, set at instantiation */
    std::unique_ptr<ObstacleAvoidanceServerImpl> _impl;
};

} // namespace mavsdk