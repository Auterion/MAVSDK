// WARNING: THIS FILE IS AUTOGENERATED! As such, it should not be edited.
// Edits need to be made to the proto files
// (see
// https://github.com/mavlink/MAVSDK-Proto/blob/main/protos/obstacle_avoidance/obstacle_avoidance.proto)

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

#include "handle.h"

namespace mavsdk {

class System;
class ObstacleAvoidanceImpl;

/**
 * @brief Plugin to manage and control obstacle avoidance services from the ground.
 *
 * Currently a single obstacle avoidance service instance is supported which
 * needs to have component ID MAV_COMP_ID_OBSTACLE_AVOIDANCE.
 *
 * Note also that application/service specific configurations should live in
 * the server application layer, as they are not defined at the MAVLink level.
 */
class ObstacleAvoidance : public PluginBase {
public:
    /**
     * @brief Constructor. Creates the plugin for a specific System.
     *
     * The plugin is typically created as shown below:
     *
     *     ```cpp
     *     auto obstacle_avoidance = ObstacleAvoidance(system);
     *     ```
     *
     * @param system The specific system associated with this plugin.
     */
    explicit ObstacleAvoidance(System& system); // deprecated

    /**
     * @brief Constructor. Creates the plugin for a specific System.
     *
     * The plugin is typically created as shown below:
     *
     *     ```cpp
     *     auto obstacle_avoidance = ObstacleAvoidance(system);
     *     ```
     *
     * @param system The specific system associated with this plugin.
     */
    explicit ObstacleAvoidance(std::shared_ptr<System> system); // new

    /**
     * @brief Destructor (internal use only).
     */
    ~ObstacleAvoidance() override;

    /**
     * @brief Possible results returned for obstacle avoidance control requests.
     */
    enum class Result {
        Unknown, /**< @brief Unknown result. */
        Success, /**< @brief Request was successful. */
        NoSystem, /**< @brief No system is connected. */
        ConnectionError, /**< @brief Connection error. */
        Busy, /**< @brief System is busy. */
        CommandDenied, /**< @brief Command refused by system. */
        Timeout, /**< @brief Request timed out. */
    };

    /**
     * @brief Stream operator to print information about a `ObstacleAvoidance::Result`.
     *
     * @return A reference to the stream.
     */
    friend std::ostream& operator<<(std::ostream& str, ObstacleAvoidance::Result const& result);

    /**
     * @brief Callback type for asynchronous ObstacleAvoidance calls.
     */
    using ResultCallback = std::function<void(Result)>;

    /**
     * @brief Start obstacle avoidance service.
     *
     * This function is non-blocking. See 'start' for the blocking counterpart.
     */
    void start_async(const ResultCallback callback);

    /**
     * @brief Start obstacle avoidance service.
     *
     * This function is blocking. See 'start_async' for the non-blocking counterpart.
     *
     * @return Result of request.
     */
    Result start() const;

    /**
     * @brief Stop obstacle avoidance service.
     *
     * This function is non-blocking. See 'stop' for the blocking counterpart.
     */
    void stop_async(const ResultCallback callback);

    /**
     * @brief Stop obstacle avoidance service.
     *
     * This function is blocking. See 'stop_async' for the non-blocking counterpart.
     *
     * @return Result of request.
     */
    Result stop() const;

    /**
     * @brief Restart obstacle avoidance service.
     *
     * This function is non-blocking. See 'restart' for the blocking counterpart.
     */
    void restart_async(const ResultCallback callback);

    /**
     * @brief Restart obstacle avoidance service.
     *
     * This function is blocking. See 'restart_async' for the non-blocking counterpart.
     *
     * @return Result of request.
     */
    Result restart() const;

    /**
     * @brief Enable obstacle avoidance service (switch from idle to active state).
     *
     * This function is non-blocking. See 'state_enable' for the blocking counterpart.
     */
    void state_enable_async(const ResultCallback callback);

    /**
     * @brief Enable obstacle avoidance service (switch from idle to active state).
     *
     * This function is blocking. See 'state_enable_async' for the non-blocking counterpart.
     *
     * @return Result of request.
     */
    Result state_enable() const;

    /**
     * @brief Disable obstacle avoidance service (switch from active to idle state).
     *
     * This function is non-blocking. See 'state_disable' for the blocking counterpart.
     */
    void state_disable_async(const ResultCallback callback);

    /**
     * @brief Disable obstacle avoidance service (switch from active to idle state).
     *
     * This function is blocking. See 'state_disable_async' for the non-blocking counterpart.
     *
     * @return Result of request.
     */
    Result state_disable() const;

    /**
     * @brief Copy constructor.
     */
    ObstacleAvoidance(const ObstacleAvoidance& other);

    /**
     * @brief Equality operator (object is not copyable).
     */
    const ObstacleAvoidance& operator=(const ObstacleAvoidance&) = delete;

private:
    /** @private Underlying implementation, set at instantiation */
    std::unique_ptr<ObstacleAvoidanceImpl> _impl;
};

} // namespace mavsdk