// WARNING: THIS FILE IS AUTOGENERATED! As such, it should not be edited.
// Edits need to be made to the proto files
// (see
// https://github.com/mavlink/MAVSDK-Proto/blob/main/protos/component_information_server/component_information_server.proto)

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
class ComponentInformationServerImpl;

/**
 * @brief Provide component information such as parameters.
 */
class ComponentInformationServer : public PluginBase {
public:
    /**
     * @brief Constructor. Creates the plugin for a specific System.
     *
     * The plugin is typically created as shown below:
     *
     *     ```cpp
     *     auto component_information_server = ComponentInformationServer(system);
     *     ```
     *
     * @param system The specific system associated with this plugin.
     */
    explicit ComponentInformationServer(System& system); // deprecated

    /**
     * @brief Constructor. Creates the plugin for a specific System.
     *
     * The plugin is typically created as shown below:
     *
     *     ```cpp
     *     auto component_information_server = ComponentInformationServer(system);
     *     ```
     *
     * @param system The specific system associated with this plugin.
     */
    explicit ComponentInformationServer(std::shared_ptr<System> system); // new

    /**
     * @brief Destructor (internal use only).
     */
    ~ComponentInformationServer();

    /**
     * @brief
     */
    struct FloatParam {
        std::string name{}; /**< @brief Name (max 16 chars) */
        std::string short_description{}; /**< @brief Short description */
        std::string long_description{}; /**< @brief Long description */
        std::string unit{}; /**< @brief Unit */
        int32_t decimal_places{}; /**< @brief Decimal places for user to show */
        float start_value{}; /**< @brief Current/starting value */
        float default_value{}; /**< @brief Default value */
        float min_value{}; /**< @brief Minimum value */
        float max_value{}; /**< @brief Maximum value */
    };

    /**
     * @brief Equal operator to compare two `ComponentInformationServer::FloatParam` objects.
     *
     * @return `true` if items are equal.
     */
    friend bool operator==(
        const ComponentInformationServer::FloatParam& lhs,
        const ComponentInformationServer::FloatParam& rhs);

    /**
     * @brief Stream operator to print information about a `ComponentInformationServer::FloatParam`.
     *
     * @return A reference to the stream.
     */
    friend std::ostream&
    operator<<(std::ostream& str, ComponentInformationServer::FloatParam const& float_param);

    /**
     * @brief
     */
    struct FloatParamUpdate {
        std::string name{}; /**< @brief Name of param that changed */
        float value{}; /**< @brief New value of param */
    };

    /**
     * @brief Equal operator to compare two `ComponentInformationServer::FloatParamUpdate` objects.
     *
     * @return `true` if items are equal.
     */
    friend bool operator==(
        const ComponentInformationServer::FloatParamUpdate& lhs,
        const ComponentInformationServer::FloatParamUpdate& rhs);

    /**
     * @brief Stream operator to print information about a
     * `ComponentInformationServer::FloatParamUpdate`.
     *
     * @return A reference to the stream.
     */
    friend std::ostream& operator<<(
        std::ostream& str, ComponentInformationServer::FloatParamUpdate const& float_param_update);

    /**
     * @brief Possible results returned for param requests.
     */
    enum class Result {
        Unknown, /**< @brief Unknown result. */
        Success, /**< @brief Request succeeded. */
        DuplicateParam, /**< @brief Duplicate param. */
        InvalidParamStartValue, /**< @brief Invalid start param value. */
        InvalidParamDefaultValue, /**< @brief Invalid default param value. */
        InvalidParamName, /**< @brief Invalid param name. */
        NoSystem, /**< @brief No system is connected. */
    };

    /**
     * @brief Stream operator to print information about a `ComponentInformationServer::Result`.
     *
     * @return A reference to the stream.
     */
    friend std::ostream&
    operator<<(std::ostream& str, ComponentInformationServer::Result const& result);

    /**
     * @brief Callback type for asynchronous ComponentInformationServer calls.
     */
    using ResultCallback = std::function<void(Result)>;

    /**
     * @brief Provide a param of type float.
     *
     * This function is blocking.
     *
     * @return Result of request.
     */
    Result provide_float_param(FloatParam param) const;

    /**
     * @brief Callback type for subscribe_float_param.
     */

    using FloatParamCallback = std::function<void(FloatParamUpdate)>;

    /**
     * @brief Subscribe to float param updates.
     */
    void subscribe_float_param(FloatParamCallback callback);

    /**
     * @brief Copy constructor.
     */
    ComponentInformationServer(const ComponentInformationServer& other);

    /**
     * @brief Equality operator (object is not copyable).
     */
    const ComponentInformationServer& operator=(const ComponentInformationServer&) = delete;

private:
    /** @private Underlying implementation, set at instantiation */
    std::unique_ptr<ComponentInformationServerImpl> _impl;
};

} // namespace mavsdk