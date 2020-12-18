// WARNING: THIS FILE IS AUTOGENERATED! As such, it should not be edited.
// Edits need to be made to the proto files
// (see
// https://github.com/mavlink/MAVSDK-Proto/blob/master/protos/custom_action/custom_action.proto)

#include "custom_action/custom_action.grpc.pb.h"
#include "plugins/custom_action/custom_action.h"

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

template<typename CustomAction = CustomAction>
class CustomActionServiceImpl final : public rpc::custom_action::CustomActionService::Service {
public:
    CustomActionServiceImpl(CustomAction& custom_action) : _custom_action(custom_action) {}

    template<typename ResponseType>
    void fillResponseWithResult(ResponseType* response, mavsdk::CustomAction::Result& result) const
    {
        auto rpc_result = translateToRpcResult(result);

        auto* rpc_custom_action_result = new rpc::custom_action::CustomActionResult();
        rpc_custom_action_result->set_result(rpc_result);
        std::stringstream ss;
        ss << result;
        rpc_custom_action_result->set_result_str(ss.str());

        response->set_allocated_custom_action_result(rpc_custom_action_result);
    }

    static std::unique_ptr<rpc::custom_action::ActionToExecute>
    translateToRpcActionToExecute(const mavsdk::CustomAction::ActionToExecute& action_to_execute)
    {
        auto rpc_obj = std::make_unique<rpc::custom_action::ActionToExecute>();

        rpc_obj->set_id(action_to_execute.id);

        rpc_obj->set_timeout(action_to_execute.timeout);

        rpc_obj->set_progress(action_to_execute.progress);

        return rpc_obj;
    }

    static mavsdk::CustomAction::ActionToExecute
    translateFromRpcActionToExecute(const rpc::custom_action::ActionToExecute& action_to_execute)
    {
        mavsdk::CustomAction::ActionToExecute obj;

        obj.id = action_to_execute.id();

        obj.timeout = action_to_execute.timeout();

        obj.progress = action_to_execute.progress();

        return obj;
    }

    static rpc::custom_action::Command::Type
    translateToRpcType(const mavsdk::CustomAction::Command::Type& type)
    {
        switch (type) {
            default:
                LogErr() << "Unknown type enum value: " << static_cast<int>(type);
            // FALLTHROUGH
            case mavsdk::CustomAction::Command::Type::Long:
                return rpc::custom_action::Command_Type_TYPE_LONG;
            case mavsdk::CustomAction::Command::Type::Int:
                return rpc::custom_action::Command_Type_TYPE_INT;
        }
    }

    static mavsdk::CustomAction::Command::Type
    translateFromRpcType(const rpc::custom_action::Command::Type type)
    {
        switch (type) {
            default:
                LogErr() << "Unknown type enum value: " << static_cast<int>(type);
            // FALLTHROUGH
            case rpc::custom_action::Command_Type_TYPE_LONG:
                return mavsdk::CustomAction::Command::Type::Long;
            case rpc::custom_action::Command_Type_TYPE_INT:
                return mavsdk::CustomAction::Command::Type::Int;
        }
    }

    static std::unique_ptr<rpc::custom_action::Command>
    translateToRpcCommand(const mavsdk::CustomAction::Command& command)
    {
        auto rpc_obj = std::make_unique<rpc::custom_action::Command>();

        rpc_obj->set_type(translateToRpcType(command.type));

        rpc_obj->set_target_system_id(command.target_system_id);

        rpc_obj->set_target_component_id(command.target_component_id);

        rpc_obj->set_frame(command.frame);

        rpc_obj->set_command(command.command);

        rpc_obj->set_param1(command.param1);

        rpc_obj->set_param2(command.param2);

        rpc_obj->set_param3(command.param3);

        rpc_obj->set_param4(command.param4);

        rpc_obj->set_param5(command.param5);

        rpc_obj->set_param6(command.param6);

        rpc_obj->set_param7(command.param7);

        rpc_obj->set_is_local(command.is_local);

        return rpc_obj;
    }

    static mavsdk::CustomAction::Command
    translateFromRpcCommand(const rpc::custom_action::Command& command)
    {
        mavsdk::CustomAction::Command obj;

        obj.type = translateFromRpcType(command.type());

        obj.target_system_id = command.target_system_id();

        obj.target_component_id = command.target_component_id();

        obj.frame = command.frame();

        obj.command = command.command();

        obj.param1 = command.param1();

        obj.param2 = command.param2();

        obj.param3 = command.param3();

        obj.param4 = command.param4();

        obj.param5 = command.param5();

        obj.param6 = command.param6();

        obj.param7 = command.param7();

        obj.is_local = command.is_local();

        return obj;
    }

    static std::unique_ptr<rpc::custom_action::Stage>
    translateToRpcStage(const mavsdk::CustomAction::Stage& stage)
    {
        auto rpc_obj = std::make_unique<rpc::custom_action::Stage>();

        rpc_obj->set_allocated_command(translateToRpcCommand(stage.command).release());

        rpc_obj->set_run_script(stage.run_script);

        rpc_obj->set_timestamp_start(stage.timestamp_start);

        rpc_obj->set_timestamp_stop(stage.timestamp_stop);

        return rpc_obj;
    }

    static mavsdk::CustomAction::Stage translateFromRpcStage(const rpc::custom_action::Stage& stage)
    {
        mavsdk::CustomAction::Stage obj;

        obj.command = translateFromRpcCommand(stage.command());

        obj.run_script = stage.run_script();

        obj.timestamp_start = stage.timestamp_start();

        obj.timestamp_stop = stage.timestamp_stop();

        return obj;
    }

    static std::unique_ptr<rpc::custom_action::ActionMetadata>
    translateToRpcActionMetadata(const mavsdk::CustomAction::ActionMetadata& action_metadata)
    {
        auto rpc_obj = std::make_unique<rpc::custom_action::ActionMetadata>();

        rpc_obj->set_id(action_metadata.id);

        rpc_obj->set_name(action_metadata.name);

        rpc_obj->set_description(action_metadata.description);

        rpc_obj->set_run_general_script(action_metadata.run_general_script);

        for (const auto& elem : action_metadata.stages) {
            auto* ptr = rpc_obj->add_stages();
            ptr->CopyFrom(*translateToRpcStage(elem).release());
        }

        return rpc_obj;
    }

    static mavsdk::CustomAction::ActionMetadata
    translateFromRpcActionMetadata(const rpc::custom_action::ActionMetadata& action_metadata)
    {
        mavsdk::CustomAction::ActionMetadata obj;

        obj.id = action_metadata.id();

        obj.name = action_metadata.name();

        obj.description = action_metadata.description();

        obj.run_general_script = action_metadata.run_general_script();

        for (const auto& elem : action_metadata.stages()) {
            obj.stages.push_back(
                translateFromRpcStage(static_cast<mavsdk::rpc::custom_action::Stage>(elem)));
        }

        return obj;
    }

    static rpc::custom_action::CustomActionResult::Result
    translateToRpcResult(const mavsdk::CustomAction::Result& result)
    {
        switch (result) {
            default:
                LogErr() << "Unknown result enum value: " << static_cast<int>(result);
            // FALLTHROUGH
            case mavsdk::CustomAction::Result::Unknown:
                return rpc::custom_action::CustomActionResult_Result_RESULT_UNKNOWN;
            case mavsdk::CustomAction::Result::Success:
                return rpc::custom_action::CustomActionResult_Result_RESULT_SUCCESS;
            case mavsdk::CustomAction::Result::Error:
                return rpc::custom_action::CustomActionResult_Result_RESULT_ERROR;
            case mavsdk::CustomAction::Result::Timeout:
                return rpc::custom_action::CustomActionResult_Result_RESULT_TIMEOUT;
            case mavsdk::CustomAction::Result::Unsupported:
                return rpc::custom_action::CustomActionResult_Result_RESULT_UNSUPPORTED;
            case mavsdk::CustomAction::Result::InProgress:
                return rpc::custom_action::CustomActionResult_Result_RESULT_IN_PROGRESS;
        }
    }

    static mavsdk::CustomAction::Result
    translateFromRpcResult(const rpc::custom_action::CustomActionResult::Result result)
    {
        switch (result) {
            default:
                LogErr() << "Unknown result enum value: " << static_cast<int>(result);
            // FALLTHROUGH
            case rpc::custom_action::CustomActionResult_Result_RESULT_UNKNOWN:
                return mavsdk::CustomAction::Result::Unknown;
            case rpc::custom_action::CustomActionResult_Result_RESULT_SUCCESS:
                return mavsdk::CustomAction::Result::Success;
            case rpc::custom_action::CustomActionResult_Result_RESULT_ERROR:
                return mavsdk::CustomAction::Result::Error;
            case rpc::custom_action::CustomActionResult_Result_RESULT_TIMEOUT:
                return mavsdk::CustomAction::Result::Timeout;
            case rpc::custom_action::CustomActionResult_Result_RESULT_UNSUPPORTED:
                return mavsdk::CustomAction::Result::Unsupported;
            case rpc::custom_action::CustomActionResult_Result_RESULT_IN_PROGRESS:
                return mavsdk::CustomAction::Result::InProgress;
        }
    }

    grpc::Status SetCustomAction(
        grpc::ServerContext* /* context */,
        const rpc::custom_action::SetCustomActionRequest* request,
        rpc::custom_action::SetCustomActionResponse* response) override
    {
        if (request == nullptr) {
            LogWarn() << "SetCustomAction sent with a null request! Ignoring...";
            return grpc::Status::OK;
        }

        auto result =
            _custom_action.set_custom_action(translateFromRpcActionToExecute(request->action()));

        if (response != nullptr) {
            fillResponseWithResult(response, result);
        }

        return grpc::Status::OK;
    }

    grpc::Status SubscribeCustomAction(
        grpc::ServerContext* /* context */,
        const mavsdk::rpc::custom_action::SubscribeCustomActionRequest* /* request */,
        grpc::ServerWriter<rpc::custom_action::CustomActionResponse>* writer) override
    {
        auto stream_closed_promise = std::make_shared<std::promise<void>>();
        auto stream_closed_future = stream_closed_promise->get_future();
        register_stream_stop_promise(stream_closed_promise);

        auto is_finished = std::make_shared<bool>(false);

        std::mutex subscribe_mutex{};

        _custom_action.subscribe_custom_action(
            [this, &writer, &stream_closed_promise, is_finished, &subscribe_mutex](
                const mavsdk::CustomAction::ActionToExecute custom_action) {
                rpc::custom_action::CustomActionResponse rpc_response;

                rpc_response.set_allocated_action(
                    translateToRpcActionToExecute(custom_action).release());

                std::unique_lock<std::mutex> lock(subscribe_mutex);
                if (!*is_finished && !writer->Write(rpc_response)) {
                    _custom_action.subscribe_custom_action(nullptr);

                    *is_finished = true;
                    unregister_stream_stop_promise(stream_closed_promise);
                    lock.unlock();
                    stream_closed_promise->set_value();
                }
            });

        stream_closed_future.wait();
        return grpc::Status::OK;
    }

    grpc::Status SubscribeCustomActionCancellation(
        grpc::ServerContext* /* context */,
        const mavsdk::rpc::custom_action::SubscribeCustomActionCancellationRequest* /* request */,
        grpc::ServerWriter<rpc::custom_action::CustomActionCancellationResponse>* writer) override
    {
        auto stream_closed_promise = std::make_shared<std::promise<void>>();
        auto stream_closed_future = stream_closed_promise->get_future();
        register_stream_stop_promise(stream_closed_promise);

        auto is_finished = std::make_shared<bool>(false);

        std::mutex subscribe_mutex{};

        _custom_action.subscribe_custom_action_cancellation(
            [this, &writer, &stream_closed_promise, is_finished, &subscribe_mutex](
                const bool custom_action_cancellation) {
                rpc::custom_action::CustomActionCancellationResponse rpc_response;

                rpc_response.set_cancel(custom_action_cancellation);

                std::unique_lock<std::mutex> lock(subscribe_mutex);
                if (!*is_finished && !writer->Write(rpc_response)) {
                    _custom_action.subscribe_custom_action_cancellation(nullptr);

                    *is_finished = true;
                    unregister_stream_stop_promise(stream_closed_promise);
                    lock.unlock();
                    stream_closed_promise->set_value();
                }
            });

        stream_closed_future.wait();
        return grpc::Status::OK;
    }

    grpc::Status RespondCustomAction(
        grpc::ServerContext* /* context */,
        const rpc::custom_action::RespondCustomActionRequest* request,
        rpc::custom_action::RespondCustomActionResponse* response) override
    {
        if (request == nullptr) {
            LogWarn() << "RespondCustomAction sent with a null request! Ignoring...";
            return grpc::Status::OK;
        }

        auto result = _custom_action.respond_custom_action(
            translateFromRpcActionToExecute(request->action()),
            translateFromRpcCustomActionResult(request->result()));

        if (response != nullptr) {
            fillResponseWithResult(response, result);
        }

        return grpc::Status::OK;
    }

    grpc::Status CustomActionMetadata(
        grpc::ServerContext* /* context */,
        const rpc::custom_action::CustomActionMetadataRequest* request,
        rpc::custom_action::CustomActionMetadataResponse* response) override
    {
        if (request == nullptr) {
            LogWarn() << "CustomActionMetadata sent with a null request! Ignoring...";
            return grpc::Status::OK;
        }

        auto result = _custom_action.custom_action_metadata(
            translateFromRpcAction(request->action()), request->file_path());

        if (response != nullptr) {
            fillResponseWithResult(response, result.first);

            response->set_allocated_action_config(
                translateToRpcActionMetadata(result.second).release());
        }

        return grpc::Status::OK;
    }

    grpc::Status ExecuteCustomActionStage(
        grpc::ServerContext* /* context */,
        const rpc::custom_action::ExecuteCustomActionStageRequest* request,
        rpc::custom_action::ExecuteCustomActionStageResponse* response) override
    {
        if (request == nullptr) {
            LogWarn() << "ExecuteCustomActionStage sent with a null request! Ignoring...";
            return grpc::Status::OK;
        }

        auto result =
            _custom_action.execute_custom_action_stage(translateFromRpcStage(request->stage()));

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

    CustomAction& _custom_action;
    std::atomic<bool> _stopped{false};
    std::vector<std::weak_ptr<std::promise<void>>> _stream_stop_promises{};
};

} // namespace mavsdk_server
} // namespace mavsdk