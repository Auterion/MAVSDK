// WARNING: THIS FILE IS AUTOGENERATED! As such, it should not be edited.
// Edits need to be made to the proto files
// (see
// https://github.com/mavlink/MAVSDK-Proto/blob/master/protos/obstacle_avoidance/obstacle_avoidance.proto)

#include "obstacle_avoidance/obstacle_avoidance.grpc.pb.h"
#include "plugins/obstacle_avoidance/obstacle_avoidance.h"

#include "mavsdk.h"
#include "lazy_plugin.h"
#include "log.h"
#include <atomic>
#include <cmath>
#include <future>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

namespace mavsdk {
namespace mavsdk_server {

template<
    typename ObstacleAvoidance = ObstacleAvoidance,
    typename LazyPlugin = LazyPlugin<ObstacleAvoidance>>
class ObstacleAvoidanceServiceImpl final
    : public rpc::obstacle_avoidance::ObstacleAvoidanceService::Service {
public:
    ObstacleAvoidanceServiceImpl(LazyPlugin& lazy_plugin) : _lazy_plugin(lazy_plugin) {}

    template<typename ResponseType>
    void
    fillResponseWithResult(ResponseType* response, mavsdk::ObstacleAvoidance::Result& result) const
    {
        auto rpc_result = translateToRpcResult(result);

        auto* rpc_obstacle_avoidance_result =
            new rpc::obstacle_avoidance::ObstacleAvoidanceResult();
        rpc_obstacle_avoidance_result->set_result(rpc_result);
        std::stringstream ss;
        ss << result;
        rpc_obstacle_avoidance_result->set_result_str(ss.str());

        response->set_allocated_obstacle_avoidance_result(rpc_obstacle_avoidance_result);
    }

    static rpc::obstacle_avoidance::ObstacleAvoidanceResult::Result
    translateToRpcResult(const mavsdk::ObstacleAvoidance::Result& result)
    {
        switch (result) {
            default:
                LogErr() << "Unknown result enum value: " << static_cast<int>(result);
            // FALLTHROUGH
            case mavsdk::ObstacleAvoidance::Result::Unknown:
                return rpc::obstacle_avoidance::ObstacleAvoidanceResult_Result_RESULT_UNKNOWN;
            case mavsdk::ObstacleAvoidance::Result::Success:
                return rpc::obstacle_avoidance::ObstacleAvoidanceResult_Result_RESULT_SUCCESS;
            case mavsdk::ObstacleAvoidance::Result::NoSystem:
                return rpc::obstacle_avoidance::ObstacleAvoidanceResult_Result_RESULT_NO_SYSTEM;
            case mavsdk::ObstacleAvoidance::Result::ConnectionError:
                return rpc::obstacle_avoidance::
                    ObstacleAvoidanceResult_Result_RESULT_CONNECTION_ERROR;
            case mavsdk::ObstacleAvoidance::Result::Busy:
                return rpc::obstacle_avoidance::ObstacleAvoidanceResult_Result_RESULT_BUSY;
            case mavsdk::ObstacleAvoidance::Result::CommandDenied:
                return rpc::obstacle_avoidance::
                    ObstacleAvoidanceResult_Result_RESULT_COMMAND_DENIED;
            case mavsdk::ObstacleAvoidance::Result::Timeout:
                return rpc::obstacle_avoidance::ObstacleAvoidanceResult_Result_RESULT_TIMEOUT;
        }
    }

    static mavsdk::ObstacleAvoidance::Result
    translateFromRpcResult(const rpc::obstacle_avoidance::ObstacleAvoidanceResult::Result result)
    {
        switch (result) {
            default:
                LogErr() << "Unknown result enum value: " << static_cast<int>(result);
            // FALLTHROUGH
            case rpc::obstacle_avoidance::ObstacleAvoidanceResult_Result_RESULT_UNKNOWN:
                return mavsdk::ObstacleAvoidance::Result::Unknown;
            case rpc::obstacle_avoidance::ObstacleAvoidanceResult_Result_RESULT_SUCCESS:
                return mavsdk::ObstacleAvoidance::Result::Success;
            case rpc::obstacle_avoidance::ObstacleAvoidanceResult_Result_RESULT_NO_SYSTEM:
                return mavsdk::ObstacleAvoidance::Result::NoSystem;
            case rpc::obstacle_avoidance::ObstacleAvoidanceResult_Result_RESULT_CONNECTION_ERROR:
                return mavsdk::ObstacleAvoidance::Result::ConnectionError;
            case rpc::obstacle_avoidance::ObstacleAvoidanceResult_Result_RESULT_BUSY:
                return mavsdk::ObstacleAvoidance::Result::Busy;
            case rpc::obstacle_avoidance::ObstacleAvoidanceResult_Result_RESULT_COMMAND_DENIED:
                return mavsdk::ObstacleAvoidance::Result::CommandDenied;
            case rpc::obstacle_avoidance::ObstacleAvoidanceResult_Result_RESULT_TIMEOUT:
                return mavsdk::ObstacleAvoidance::Result::Timeout;
        }
    }

    grpc::Status Start(
        grpc::ServerContext* /* context */,
        const rpc::obstacle_avoidance::StartRequest* /* request */,
        rpc::obstacle_avoidance::StartResponse* response) override
    {
        if (_lazy_plugin.maybe_plugin() == nullptr) {
            if (response != nullptr) {
                auto result = mavsdk::ObstacleAvoidance::Result::NoSystem;
                fillResponseWithResult(response, result);
            }

            return grpc::Status::OK;
        }

        auto result = _lazy_plugin.maybe_plugin()->start();

        if (response != nullptr) {
            fillResponseWithResult(response, result);
        }

        return grpc::Status::OK;
    }

    grpc::Status Stop(
        grpc::ServerContext* /* context */,
        const rpc::obstacle_avoidance::StopRequest* /* request */,
        rpc::obstacle_avoidance::StopResponse* response) override
    {
        if (_lazy_plugin.maybe_plugin() == nullptr) {
            if (response != nullptr) {
                auto result = mavsdk::ObstacleAvoidance::Result::NoSystem;
                fillResponseWithResult(response, result);
            }

            return grpc::Status::OK;
        }

        auto result = _lazy_plugin.maybe_plugin()->stop();

        if (response != nullptr) {
            fillResponseWithResult(response, result);
        }

        return grpc::Status::OK;
    }

    grpc::Status Restart(
        grpc::ServerContext* /* context */,
        const rpc::obstacle_avoidance::RestartRequest* /* request */,
        rpc::obstacle_avoidance::RestartResponse* response) override
    {
        if (_lazy_plugin.maybe_plugin() == nullptr) {
            if (response != nullptr) {
                auto result = mavsdk::ObstacleAvoidance::Result::NoSystem;
                fillResponseWithResult(response, result);
            }

            return grpc::Status::OK;
        }

        auto result = _lazy_plugin.maybe_plugin()->restart();

        if (response != nullptr) {
            fillResponseWithResult(response, result);
        }

        return grpc::Status::OK;
    }

    grpc::Status StateEnable(
        grpc::ServerContext* /* context */,
        const rpc::obstacle_avoidance::StateEnableRequest* /* request */,
        rpc::obstacle_avoidance::StateEnableResponse* response) override
    {
        if (_lazy_plugin.maybe_plugin() == nullptr) {
            if (response != nullptr) {
                auto result = mavsdk::ObstacleAvoidance::Result::NoSystem;
                fillResponseWithResult(response, result);
            }

            return grpc::Status::OK;
        }

        auto result = _lazy_plugin.maybe_plugin()->state_enable();

        if (response != nullptr) {
            fillResponseWithResult(response, result);
        }

        return grpc::Status::OK;
    }

    grpc::Status StateDisable(
        grpc::ServerContext* /* context */,
        const rpc::obstacle_avoidance::StateDisableRequest* /* request */,
        rpc::obstacle_avoidance::StateDisableResponse* response) override
    {
        if (_lazy_plugin.maybe_plugin() == nullptr) {
            if (response != nullptr) {
                auto result = mavsdk::ObstacleAvoidance::Result::NoSystem;
                fillResponseWithResult(response, result);
            }

            return grpc::Status::OK;
        }

        auto result = _lazy_plugin.maybe_plugin()->state_disable();

        if (response != nullptr) {
            fillResponseWithResult(response, result);
        }

        return grpc::Status::OK;
    }

    void stop()
    {
        _stopped.store(true);
        for (auto& prom : _stream_stop_promises) {
            if (auto handle = prom.lock()) {
                handle->set_value();
            }
        }
    }

private:
    void register_stream_stop_promise(std::weak_ptr<std::promise<void>> prom)
    {
        // If we have already stopped, set promise immediately and don't add it to list.
        if (_stopped.load()) {
            if (auto handle = prom.lock()) {
                handle->set_value();
            }
        } else {
            _stream_stop_promises.push_back(prom);
        }
    }

    void unregister_stream_stop_promise(std::shared_ptr<std::promise<void>> prom)
    {
        for (auto it = _stream_stop_promises.begin(); it != _stream_stop_promises.end();
             /* ++it */) {
            if (it->lock() == prom) {
                it = _stream_stop_promises.erase(it);
            } else {
                ++it;
            }
        }
    }

    LazyPlugin& _lazy_plugin;
    std::atomic<bool> _stopped{false};
    std::vector<std::weak_ptr<std::promise<void>>> _stream_stop_promises{};
};

} // namespace mavsdk_server
} // namespace mavsdk