// Generated by the gRPC C++ plugin.
// If you make any local change, they will be lost.
// source: param_server/param_server.proto

#include "param_server/param_server.pb.h"
#include "param_server/param_server.grpc.pb.h"

#include <functional>
#include <grpcpp/impl/codegen/async_stream.h>
#include <grpcpp/impl/codegen/async_unary_call.h>
#include <grpcpp/impl/codegen/channel_interface.h>
#include <grpcpp/impl/codegen/client_unary_call.h>
#include <grpcpp/impl/codegen/client_callback.h>
#include <grpcpp/impl/codegen/message_allocator.h>
#include <grpcpp/impl/codegen/method_handler.h>
#include <grpcpp/impl/codegen/rpc_service_method.h>
#include <grpcpp/impl/codegen/server_callback.h>
#include <grpcpp/impl/codegen/server_callback_handlers.h>
#include <grpcpp/impl/codegen/server_context.h>
#include <grpcpp/impl/codegen/service_type.h>
#include <grpcpp/impl/codegen/sync_stream.h>
namespace mavsdk {
namespace rpc {
namespace param_server {

static const char* ParamServerService_method_names[] = {
  "/mavsdk.rpc.param_server.ParamServerService/RetrieveParamInt",
  "/mavsdk.rpc.param_server.ParamServerService/ProvideParamInt",
  "/mavsdk.rpc.param_server.ParamServerService/RetrieveParamFloat",
  "/mavsdk.rpc.param_server.ParamServerService/ProvideParamFloat",
  "/mavsdk.rpc.param_server.ParamServerService/RetrieveAllParams",
};

std::unique_ptr< ParamServerService::Stub> ParamServerService::NewStub(const std::shared_ptr< ::grpc::ChannelInterface>& channel, const ::grpc::StubOptions& options) {
  (void)options;
  std::unique_ptr< ParamServerService::Stub> stub(new ParamServerService::Stub(channel, options));
  return stub;
}

ParamServerService::Stub::Stub(const std::shared_ptr< ::grpc::ChannelInterface>& channel, const ::grpc::StubOptions& options)
  : channel_(channel), rpcmethod_RetrieveParamInt_(ParamServerService_method_names[0], options.suffix_for_stats(),::grpc::internal::RpcMethod::NORMAL_RPC, channel)
  , rpcmethod_ProvideParamInt_(ParamServerService_method_names[1], options.suffix_for_stats(),::grpc::internal::RpcMethod::NORMAL_RPC, channel)
  , rpcmethod_RetrieveParamFloat_(ParamServerService_method_names[2], options.suffix_for_stats(),::grpc::internal::RpcMethod::NORMAL_RPC, channel)
  , rpcmethod_ProvideParamFloat_(ParamServerService_method_names[3], options.suffix_for_stats(),::grpc::internal::RpcMethod::NORMAL_RPC, channel)
  , rpcmethod_RetrieveAllParams_(ParamServerService_method_names[4], options.suffix_for_stats(),::grpc::internal::RpcMethod::NORMAL_RPC, channel)
  {}

::grpc::Status ParamServerService::Stub::RetrieveParamInt(::grpc::ClientContext* context, const ::mavsdk::rpc::param_server::RetrieveParamIntRequest& request, ::mavsdk::rpc::param_server::RetrieveParamIntResponse* response) {
  return ::grpc::internal::BlockingUnaryCall< ::mavsdk::rpc::param_server::RetrieveParamIntRequest, ::mavsdk::rpc::param_server::RetrieveParamIntResponse, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), rpcmethod_RetrieveParamInt_, context, request, response);
}

void ParamServerService::Stub::async::RetrieveParamInt(::grpc::ClientContext* context, const ::mavsdk::rpc::param_server::RetrieveParamIntRequest* request, ::mavsdk::rpc::param_server::RetrieveParamIntResponse* response, std::function<void(::grpc::Status)> f) {
  ::grpc::internal::CallbackUnaryCall< ::mavsdk::rpc::param_server::RetrieveParamIntRequest, ::mavsdk::rpc::param_server::RetrieveParamIntResponse, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_RetrieveParamInt_, context, request, response, std::move(f));
}

void ParamServerService::Stub::async::RetrieveParamInt(::grpc::ClientContext* context, const ::mavsdk::rpc::param_server::RetrieveParamIntRequest* request, ::mavsdk::rpc::param_server::RetrieveParamIntResponse* response, ::grpc::ClientUnaryReactor* reactor) {
  ::grpc::internal::ClientCallbackUnaryFactory::Create< ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_RetrieveParamInt_, context, request, response, reactor);
}

::grpc::ClientAsyncResponseReader< ::mavsdk::rpc::param_server::RetrieveParamIntResponse>* ParamServerService::Stub::PrepareAsyncRetrieveParamIntRaw(::grpc::ClientContext* context, const ::mavsdk::rpc::param_server::RetrieveParamIntRequest& request, ::grpc::CompletionQueue* cq) {
  return ::grpc::internal::ClientAsyncResponseReaderHelper::Create< ::mavsdk::rpc::param_server::RetrieveParamIntResponse, ::mavsdk::rpc::param_server::RetrieveParamIntRequest, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), cq, rpcmethod_RetrieveParamInt_, context, request);
}

::grpc::ClientAsyncResponseReader< ::mavsdk::rpc::param_server::RetrieveParamIntResponse>* ParamServerService::Stub::AsyncRetrieveParamIntRaw(::grpc::ClientContext* context, const ::mavsdk::rpc::param_server::RetrieveParamIntRequest& request, ::grpc::CompletionQueue* cq) {
  auto* result =
    this->PrepareAsyncRetrieveParamIntRaw(context, request, cq);
  result->StartCall();
  return result;
}

::grpc::Status ParamServerService::Stub::ProvideParamInt(::grpc::ClientContext* context, const ::mavsdk::rpc::param_server::ProvideParamIntRequest& request, ::mavsdk::rpc::param_server::ProvideParamIntResponse* response) {
  return ::grpc::internal::BlockingUnaryCall< ::mavsdk::rpc::param_server::ProvideParamIntRequest, ::mavsdk::rpc::param_server::ProvideParamIntResponse, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), rpcmethod_ProvideParamInt_, context, request, response);
}

void ParamServerService::Stub::async::ProvideParamInt(::grpc::ClientContext* context, const ::mavsdk::rpc::param_server::ProvideParamIntRequest* request, ::mavsdk::rpc::param_server::ProvideParamIntResponse* response, std::function<void(::grpc::Status)> f) {
  ::grpc::internal::CallbackUnaryCall< ::mavsdk::rpc::param_server::ProvideParamIntRequest, ::mavsdk::rpc::param_server::ProvideParamIntResponse, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_ProvideParamInt_, context, request, response, std::move(f));
}

void ParamServerService::Stub::async::ProvideParamInt(::grpc::ClientContext* context, const ::mavsdk::rpc::param_server::ProvideParamIntRequest* request, ::mavsdk::rpc::param_server::ProvideParamIntResponse* response, ::grpc::ClientUnaryReactor* reactor) {
  ::grpc::internal::ClientCallbackUnaryFactory::Create< ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_ProvideParamInt_, context, request, response, reactor);
}

::grpc::ClientAsyncResponseReader< ::mavsdk::rpc::param_server::ProvideParamIntResponse>* ParamServerService::Stub::PrepareAsyncProvideParamIntRaw(::grpc::ClientContext* context, const ::mavsdk::rpc::param_server::ProvideParamIntRequest& request, ::grpc::CompletionQueue* cq) {
  return ::grpc::internal::ClientAsyncResponseReaderHelper::Create< ::mavsdk::rpc::param_server::ProvideParamIntResponse, ::mavsdk::rpc::param_server::ProvideParamIntRequest, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), cq, rpcmethod_ProvideParamInt_, context, request);
}

::grpc::ClientAsyncResponseReader< ::mavsdk::rpc::param_server::ProvideParamIntResponse>* ParamServerService::Stub::AsyncProvideParamIntRaw(::grpc::ClientContext* context, const ::mavsdk::rpc::param_server::ProvideParamIntRequest& request, ::grpc::CompletionQueue* cq) {
  auto* result =
    this->PrepareAsyncProvideParamIntRaw(context, request, cq);
  result->StartCall();
  return result;
}

::grpc::Status ParamServerService::Stub::RetrieveParamFloat(::grpc::ClientContext* context, const ::mavsdk::rpc::param_server::RetrieveParamFloatRequest& request, ::mavsdk::rpc::param_server::RetrieveParamFloatResponse* response) {
  return ::grpc::internal::BlockingUnaryCall< ::mavsdk::rpc::param_server::RetrieveParamFloatRequest, ::mavsdk::rpc::param_server::RetrieveParamFloatResponse, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), rpcmethod_RetrieveParamFloat_, context, request, response);
}

void ParamServerService::Stub::async::RetrieveParamFloat(::grpc::ClientContext* context, const ::mavsdk::rpc::param_server::RetrieveParamFloatRequest* request, ::mavsdk::rpc::param_server::RetrieveParamFloatResponse* response, std::function<void(::grpc::Status)> f) {
  ::grpc::internal::CallbackUnaryCall< ::mavsdk::rpc::param_server::RetrieveParamFloatRequest, ::mavsdk::rpc::param_server::RetrieveParamFloatResponse, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_RetrieveParamFloat_, context, request, response, std::move(f));
}

void ParamServerService::Stub::async::RetrieveParamFloat(::grpc::ClientContext* context, const ::mavsdk::rpc::param_server::RetrieveParamFloatRequest* request, ::mavsdk::rpc::param_server::RetrieveParamFloatResponse* response, ::grpc::ClientUnaryReactor* reactor) {
  ::grpc::internal::ClientCallbackUnaryFactory::Create< ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_RetrieveParamFloat_, context, request, response, reactor);
}

::grpc::ClientAsyncResponseReader< ::mavsdk::rpc::param_server::RetrieveParamFloatResponse>* ParamServerService::Stub::PrepareAsyncRetrieveParamFloatRaw(::grpc::ClientContext* context, const ::mavsdk::rpc::param_server::RetrieveParamFloatRequest& request, ::grpc::CompletionQueue* cq) {
  return ::grpc::internal::ClientAsyncResponseReaderHelper::Create< ::mavsdk::rpc::param_server::RetrieveParamFloatResponse, ::mavsdk::rpc::param_server::RetrieveParamFloatRequest, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), cq, rpcmethod_RetrieveParamFloat_, context, request);
}

::grpc::ClientAsyncResponseReader< ::mavsdk::rpc::param_server::RetrieveParamFloatResponse>* ParamServerService::Stub::AsyncRetrieveParamFloatRaw(::grpc::ClientContext* context, const ::mavsdk::rpc::param_server::RetrieveParamFloatRequest& request, ::grpc::CompletionQueue* cq) {
  auto* result =
    this->PrepareAsyncRetrieveParamFloatRaw(context, request, cq);
  result->StartCall();
  return result;
}

::grpc::Status ParamServerService::Stub::ProvideParamFloat(::grpc::ClientContext* context, const ::mavsdk::rpc::param_server::ProvideParamFloatRequest& request, ::mavsdk::rpc::param_server::ProvideParamFloatResponse* response) {
  return ::grpc::internal::BlockingUnaryCall< ::mavsdk::rpc::param_server::ProvideParamFloatRequest, ::mavsdk::rpc::param_server::ProvideParamFloatResponse, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), rpcmethod_ProvideParamFloat_, context, request, response);
}

void ParamServerService::Stub::async::ProvideParamFloat(::grpc::ClientContext* context, const ::mavsdk::rpc::param_server::ProvideParamFloatRequest* request, ::mavsdk::rpc::param_server::ProvideParamFloatResponse* response, std::function<void(::grpc::Status)> f) {
  ::grpc::internal::CallbackUnaryCall< ::mavsdk::rpc::param_server::ProvideParamFloatRequest, ::mavsdk::rpc::param_server::ProvideParamFloatResponse, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_ProvideParamFloat_, context, request, response, std::move(f));
}

void ParamServerService::Stub::async::ProvideParamFloat(::grpc::ClientContext* context, const ::mavsdk::rpc::param_server::ProvideParamFloatRequest* request, ::mavsdk::rpc::param_server::ProvideParamFloatResponse* response, ::grpc::ClientUnaryReactor* reactor) {
  ::grpc::internal::ClientCallbackUnaryFactory::Create< ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_ProvideParamFloat_, context, request, response, reactor);
}

::grpc::ClientAsyncResponseReader< ::mavsdk::rpc::param_server::ProvideParamFloatResponse>* ParamServerService::Stub::PrepareAsyncProvideParamFloatRaw(::grpc::ClientContext* context, const ::mavsdk::rpc::param_server::ProvideParamFloatRequest& request, ::grpc::CompletionQueue* cq) {
  return ::grpc::internal::ClientAsyncResponseReaderHelper::Create< ::mavsdk::rpc::param_server::ProvideParamFloatResponse, ::mavsdk::rpc::param_server::ProvideParamFloatRequest, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), cq, rpcmethod_ProvideParamFloat_, context, request);
}

::grpc::ClientAsyncResponseReader< ::mavsdk::rpc::param_server::ProvideParamFloatResponse>* ParamServerService::Stub::AsyncProvideParamFloatRaw(::grpc::ClientContext* context, const ::mavsdk::rpc::param_server::ProvideParamFloatRequest& request, ::grpc::CompletionQueue* cq) {
  auto* result =
    this->PrepareAsyncProvideParamFloatRaw(context, request, cq);
  result->StartCall();
  return result;
}

::grpc::Status ParamServerService::Stub::RetrieveAllParams(::grpc::ClientContext* context, const ::mavsdk::rpc::param_server::RetrieveAllParamsRequest& request, ::mavsdk::rpc::param_server::RetrieveAllParamsResponse* response) {
  return ::grpc::internal::BlockingUnaryCall< ::mavsdk::rpc::param_server::RetrieveAllParamsRequest, ::mavsdk::rpc::param_server::RetrieveAllParamsResponse, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), rpcmethod_RetrieveAllParams_, context, request, response);
}

void ParamServerService::Stub::async::RetrieveAllParams(::grpc::ClientContext* context, const ::mavsdk::rpc::param_server::RetrieveAllParamsRequest* request, ::mavsdk::rpc::param_server::RetrieveAllParamsResponse* response, std::function<void(::grpc::Status)> f) {
  ::grpc::internal::CallbackUnaryCall< ::mavsdk::rpc::param_server::RetrieveAllParamsRequest, ::mavsdk::rpc::param_server::RetrieveAllParamsResponse, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_RetrieveAllParams_, context, request, response, std::move(f));
}

void ParamServerService::Stub::async::RetrieveAllParams(::grpc::ClientContext* context, const ::mavsdk::rpc::param_server::RetrieveAllParamsRequest* request, ::mavsdk::rpc::param_server::RetrieveAllParamsResponse* response, ::grpc::ClientUnaryReactor* reactor) {
  ::grpc::internal::ClientCallbackUnaryFactory::Create< ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_RetrieveAllParams_, context, request, response, reactor);
}

::grpc::ClientAsyncResponseReader< ::mavsdk::rpc::param_server::RetrieveAllParamsResponse>* ParamServerService::Stub::PrepareAsyncRetrieveAllParamsRaw(::grpc::ClientContext* context, const ::mavsdk::rpc::param_server::RetrieveAllParamsRequest& request, ::grpc::CompletionQueue* cq) {
  return ::grpc::internal::ClientAsyncResponseReaderHelper::Create< ::mavsdk::rpc::param_server::RetrieveAllParamsResponse, ::mavsdk::rpc::param_server::RetrieveAllParamsRequest, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), cq, rpcmethod_RetrieveAllParams_, context, request);
}

::grpc::ClientAsyncResponseReader< ::mavsdk::rpc::param_server::RetrieveAllParamsResponse>* ParamServerService::Stub::AsyncRetrieveAllParamsRaw(::grpc::ClientContext* context, const ::mavsdk::rpc::param_server::RetrieveAllParamsRequest& request, ::grpc::CompletionQueue* cq) {
  auto* result =
    this->PrepareAsyncRetrieveAllParamsRaw(context, request, cq);
  result->StartCall();
  return result;
}

ParamServerService::Service::Service() {
  AddMethod(new ::grpc::internal::RpcServiceMethod(
      ParamServerService_method_names[0],
      ::grpc::internal::RpcMethod::NORMAL_RPC,
      new ::grpc::internal::RpcMethodHandler< ParamServerService::Service, ::mavsdk::rpc::param_server::RetrieveParamIntRequest, ::mavsdk::rpc::param_server::RetrieveParamIntResponse, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(
          [](ParamServerService::Service* service,
             ::grpc::ServerContext* ctx,
             const ::mavsdk::rpc::param_server::RetrieveParamIntRequest* req,
             ::mavsdk::rpc::param_server::RetrieveParamIntResponse* resp) {
               return service->RetrieveParamInt(ctx, req, resp);
             }, this)));
  AddMethod(new ::grpc::internal::RpcServiceMethod(
      ParamServerService_method_names[1],
      ::grpc::internal::RpcMethod::NORMAL_RPC,
      new ::grpc::internal::RpcMethodHandler< ParamServerService::Service, ::mavsdk::rpc::param_server::ProvideParamIntRequest, ::mavsdk::rpc::param_server::ProvideParamIntResponse, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(
          [](ParamServerService::Service* service,
             ::grpc::ServerContext* ctx,
             const ::mavsdk::rpc::param_server::ProvideParamIntRequest* req,
             ::mavsdk::rpc::param_server::ProvideParamIntResponse* resp) {
               return service->ProvideParamInt(ctx, req, resp);
             }, this)));
  AddMethod(new ::grpc::internal::RpcServiceMethod(
      ParamServerService_method_names[2],
      ::grpc::internal::RpcMethod::NORMAL_RPC,
      new ::grpc::internal::RpcMethodHandler< ParamServerService::Service, ::mavsdk::rpc::param_server::RetrieveParamFloatRequest, ::mavsdk::rpc::param_server::RetrieveParamFloatResponse, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(
          [](ParamServerService::Service* service,
             ::grpc::ServerContext* ctx,
             const ::mavsdk::rpc::param_server::RetrieveParamFloatRequest* req,
             ::mavsdk::rpc::param_server::RetrieveParamFloatResponse* resp) {
               return service->RetrieveParamFloat(ctx, req, resp);
             }, this)));
  AddMethod(new ::grpc::internal::RpcServiceMethod(
      ParamServerService_method_names[3],
      ::grpc::internal::RpcMethod::NORMAL_RPC,
      new ::grpc::internal::RpcMethodHandler< ParamServerService::Service, ::mavsdk::rpc::param_server::ProvideParamFloatRequest, ::mavsdk::rpc::param_server::ProvideParamFloatResponse, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(
          [](ParamServerService::Service* service,
             ::grpc::ServerContext* ctx,
             const ::mavsdk::rpc::param_server::ProvideParamFloatRequest* req,
             ::mavsdk::rpc::param_server::ProvideParamFloatResponse* resp) {
               return service->ProvideParamFloat(ctx, req, resp);
             }, this)));
  AddMethod(new ::grpc::internal::RpcServiceMethod(
      ParamServerService_method_names[4],
      ::grpc::internal::RpcMethod::NORMAL_RPC,
      new ::grpc::internal::RpcMethodHandler< ParamServerService::Service, ::mavsdk::rpc::param_server::RetrieveAllParamsRequest, ::mavsdk::rpc::param_server::RetrieveAllParamsResponse, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(
          [](ParamServerService::Service* service,
             ::grpc::ServerContext* ctx,
             const ::mavsdk::rpc::param_server::RetrieveAllParamsRequest* req,
             ::mavsdk::rpc::param_server::RetrieveAllParamsResponse* resp) {
               return service->RetrieveAllParams(ctx, req, resp);
             }, this)));
}

ParamServerService::Service::~Service() {
}

::grpc::Status ParamServerService::Service::RetrieveParamInt(::grpc::ServerContext* context, const ::mavsdk::rpc::param_server::RetrieveParamIntRequest* request, ::mavsdk::rpc::param_server::RetrieveParamIntResponse* response) {
  (void) context;
  (void) request;
  (void) response;
  return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
}

::grpc::Status ParamServerService::Service::ProvideParamInt(::grpc::ServerContext* context, const ::mavsdk::rpc::param_server::ProvideParamIntRequest* request, ::mavsdk::rpc::param_server::ProvideParamIntResponse* response) {
  (void) context;
  (void) request;
  (void) response;
  return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
}

::grpc::Status ParamServerService::Service::RetrieveParamFloat(::grpc::ServerContext* context, const ::mavsdk::rpc::param_server::RetrieveParamFloatRequest* request, ::mavsdk::rpc::param_server::RetrieveParamFloatResponse* response) {
  (void) context;
  (void) request;
  (void) response;
  return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
}

::grpc::Status ParamServerService::Service::ProvideParamFloat(::grpc::ServerContext* context, const ::mavsdk::rpc::param_server::ProvideParamFloatRequest* request, ::mavsdk::rpc::param_server::ProvideParamFloatResponse* response) {
  (void) context;
  (void) request;
  (void) response;
  return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
}

::grpc::Status ParamServerService::Service::RetrieveAllParams(::grpc::ServerContext* context, const ::mavsdk::rpc::param_server::RetrieveAllParamsRequest* request, ::mavsdk::rpc::param_server::RetrieveAllParamsResponse* response) {
  (void) context;
  (void) request;
  (void) response;
  return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
}


}  // namespace mavsdk
}  // namespace rpc
}  // namespace param_server

