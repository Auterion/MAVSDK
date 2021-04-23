// WARNING: THIS FILE IS AUTOGENERATED! As such, it should not be edited.
// Edits need to be made to the proto files
// (see
// https://github.com/mavlink/MAVSDK-Proto/blob/master/protos/obstacle_avoidance_server/obstacle_avoidance_server.proto)

#include "obstacle_avoidance_server/obstacle_avoidance_server.grpc.pb.h"
#include "plugins/obstacle_avoidance_server/obstacle_avoidance_server.h"

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

template<typename ObstacleAvoidanceServer = ObstacleAvoidanceServer>
class ObstacleAvoidanceServerServiceImpl final
    : public rpc::obstacle_avoidance_server::ObstacleAvoidanceServerService::Service {
public:
    ObstacleAvoidanceServerServiceImpl(ObstacleAvoidanceServer& obstacle_avoidance_server) :
        _obstacle_avoidance_server(obstacle_avoidance_server)
    {}

    static rpc::obstacle_avoidance_server::Control::ControlType translateToRpcControlType(
        const mavsdk::ObstacleAvoidanceServer::Control::ControlType& control_type)
    {
        switch (control_type) {
            default:
                LogErr() << "Unknown control_type enum value: " << static_cast<int>(control_type);
            // FALLTHROUGH
            case mavsdk::ObstacleAvoidanceServer::Control::ControlType::Unknown:
                return rpc::obstacle_avoidance_server::Control_ControlType_CONTROL_TYPE_UNKNOWN;
            case mavsdk::ObstacleAvoidanceServer::Control::ControlType::Start:
                return rpc::obstacle_avoidance_server::Control_ControlType_CONTROL_TYPE_START;
            case mavsdk::ObstacleAvoidanceServer::Control::ControlType::Stop:
                return rpc::obstacle_avoidance_server::Control_ControlType_CONTROL_TYPE_STOP;
            case mavsdk::ObstacleAvoidanceServer::Control::ControlType::Restart:
                return rpc::obstacle_avoidance_server::Control_ControlType_CONTROL_TYPE_RESTART;
            case mavsdk::ObstacleAvoidanceServer::Control::ControlType::Enable:
                return rpc::obstacle_avoidance_server::Control_ControlType_CONTROL_TYPE_ENABLE;
            case mavsdk::ObstacleAvoidanceServer::Control::ControlType::Disable:
                return rpc::obstacle_avoidance_server::Control_ControlType_CONTROL_TYPE_DISABLE;
        }
    }

    static mavsdk::ObstacleAvoidanceServer::Control::ControlType translateFromRpcControlType(
        const rpc::obstacle_avoidance_server::Control::ControlType control_type)
    {
        switch (control_type) {
            default:
                LogErr() << "Unknown control_type enum value: " << static_cast<int>(control_type);
            // FALLTHROUGH
            case rpc::obstacle_avoidance_server::Control_ControlType_CONTROL_TYPE_UNKNOWN:
                return mavsdk::ObstacleAvoidanceServer::Control::ControlType::Unknown;
            case rpc::obstacle_avoidance_server::Control_ControlType_CONTROL_TYPE_START:
                return mavsdk::ObstacleAvoidanceServer::Control::ControlType::Start;
            case rpc::obstacle_avoidance_server::Control_ControlType_CONTROL_TYPE_STOP:
                return mavsdk::ObstacleAvoidanceServer::Control::ControlType::Stop;
            case rpc::obstacle_avoidance_server::Control_ControlType_CONTROL_TYPE_RESTART:
                return mavsdk::ObstacleAvoidanceServer::Control::ControlType::Restart;
            case rpc::obstacle_avoidance_server::Control_ControlType_CONTROL_TYPE_ENABLE:
                return mavsdk::ObstacleAvoidanceServer::Control::ControlType::Enable;
            case rpc::obstacle_avoidance_server::Control_ControlType_CONTROL_TYPE_DISABLE:
                return mavsdk::ObstacleAvoidanceServer::Control::ControlType::Disable;
        }
    }

    static std::unique_ptr<rpc::obstacle_avoidance_server::Control>
    translateToRpcControl(const mavsdk::ObstacleAvoidanceServer::Control& control)
    {
        auto rpc_obj = std::make_unique<rpc::obstacle_avoidance_server::Control>();

        rpc_obj->set_control_type(translateToRpcControlType(control.control_type));

        return rpc_obj;
    }

    static mavsdk::ObstacleAvoidanceServer::Control
    translateFromRpcControl(const rpc::obstacle_avoidance_server::Control& control)
    {
        mavsdk::ObstacleAvoidanceServer::Control obj;

        obj.control_type = translateFromRpcControlType(control.control_type());

        return obj;
    }

    grpc::Status SubscribeControl(
        grpc::ServerContext* /* context */,
        const mavsdk::rpc::obstacle_avoidance_server::SubscribeControlRequest* /* request */,
        grpc::ServerWriter<rpc::obstacle_avoidance_server::ControlResponse>* writer) override
    {
        auto stream_closed_promise = std::make_shared<std::promise<void>>();
        auto stream_closed_future = stream_closed_promise->get_future();
        register_stream_stop_promise(stream_closed_promise);

        auto is_finished = std::make_shared<bool>(false);
        auto subscribe_mutex = std::make_shared<std::mutex>();

        _obstacle_avoidance_server.subscribe_control(
            [this, &writer, &stream_closed_promise, is_finished, subscribe_mutex](
                const mavsdk::ObstacleAvoidanceServer::Control control) {
                rpc::obstacle_avoidance_server::ControlResponse rpc_response;

                rpc_response.set_allocated_control(translateToRpcControl(control).release());

                std::unique_lock<std::mutex> lock(*subscribe_mutex);
                if (!*is_finished && !writer->Write(rpc_response)) {
                    _obstacle_avoidance_server.subscribe_control(nullptr);

                    *is_finished = true;
                    unregister_stream_stop_promise(stream_closed_promise);
                    stream_closed_promise->set_value();
                }
            });

        stream_closed_future.wait();
        std::unique_lock<std::mutex> lock(*subscribe_mutex);
        *is_finished = true;

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

    ObstacleAvoidanceServer& _obstacle_avoidance_server;
    std::atomic<bool> _stopped{false};
    std::vector<std::weak_ptr<std::promise<void>>> _stream_stop_promises{};
};

} // namespace mavsdk_server
} // namespace mavsdk