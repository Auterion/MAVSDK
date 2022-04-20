// WARNING: THIS FILE IS AUTOGENERATED! As such, it should not be edited.
// Edits need to be made to the proto files
// (see https://github.com/mavlink/MAVSDK-Proto/blob/master/protos/mission/mission.proto)

#include "mission/mission.grpc.pb.h"
#include "plugins/mission/mission.h"

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

template<typename Mission = Mission, typename LazyPlugin = LazyPlugin<Mission>>
class MissionServiceImpl final : public rpc::mission::MissionService::Service {
public:
    MissionServiceImpl(LazyPlugin& lazy_plugin) : _lazy_plugin(lazy_plugin) {}


    template<typename ResponseType>
    void fillResponseWithResult(ResponseType* response, mavsdk::Mission::Result& result) const
    {
        auto rpc_result = translateToRpcResult(result);

        auto* rpc_mission_result = new rpc::mission::MissionResult();
        rpc_mission_result->set_result(rpc_result);
        std::stringstream ss;
        ss << result;
        rpc_mission_result->set_result_str(ss.str());

        response->set_allocated_mission_result(rpc_mission_result);
    }



    static rpc::mission::MissionItem::CameraAction translateToRpcCameraAction(const mavsdk::Mission::MissionItem::CameraAction& camera_action)
    {
        switch (camera_action) {
            default:
                LogErr() << "Unknown camera_action enum value: " << static_cast<int>(camera_action);
            // FALLTHROUGH
            case mavsdk::Mission::MissionItem::CameraAction::None:
                return rpc::mission::MissionItem_CameraAction_CAMERA_ACTION_NONE;
            case mavsdk::Mission::MissionItem::CameraAction::TakePhoto:
                return rpc::mission::MissionItem_CameraAction_CAMERA_ACTION_TAKE_PHOTO;
            case mavsdk::Mission::MissionItem::CameraAction::StartPhotoInterval:
                return rpc::mission::MissionItem_CameraAction_CAMERA_ACTION_START_PHOTO_INTERVAL;
            case mavsdk::Mission::MissionItem::CameraAction::StopPhotoInterval:
                return rpc::mission::MissionItem_CameraAction_CAMERA_ACTION_STOP_PHOTO_INTERVAL;
            case mavsdk::Mission::MissionItem::CameraAction::StartVideo:
                return rpc::mission::MissionItem_CameraAction_CAMERA_ACTION_START_VIDEO;
            case mavsdk::Mission::MissionItem::CameraAction::StopVideo:
                return rpc::mission::MissionItem_CameraAction_CAMERA_ACTION_STOP_VIDEO;
            case mavsdk::Mission::MissionItem::CameraAction::StartPhotoDistance:
                return rpc::mission::MissionItem_CameraAction_CAMERA_ACTION_START_PHOTO_DISTANCE;
            case mavsdk::Mission::MissionItem::CameraAction::StopPhotoDistance:
                return rpc::mission::MissionItem_CameraAction_CAMERA_ACTION_STOP_PHOTO_DISTANCE;
        }
    }

    static mavsdk::Mission::MissionItem::CameraAction translateFromRpcCameraAction(const rpc::mission::MissionItem::CameraAction camera_action)
    {
        switch (camera_action) {
            default:
                LogErr() << "Unknown camera_action enum value: " << static_cast<int>(camera_action);
            // FALLTHROUGH
            case rpc::mission::MissionItem_CameraAction_CAMERA_ACTION_NONE:
                return mavsdk::Mission::MissionItem::CameraAction::None;
            case rpc::mission::MissionItem_CameraAction_CAMERA_ACTION_TAKE_PHOTO:
                return mavsdk::Mission::MissionItem::CameraAction::TakePhoto;
            case rpc::mission::MissionItem_CameraAction_CAMERA_ACTION_START_PHOTO_INTERVAL:
                return mavsdk::Mission::MissionItem::CameraAction::StartPhotoInterval;
            case rpc::mission::MissionItem_CameraAction_CAMERA_ACTION_STOP_PHOTO_INTERVAL:
                return mavsdk::Mission::MissionItem::CameraAction::StopPhotoInterval;
            case rpc::mission::MissionItem_CameraAction_CAMERA_ACTION_START_VIDEO:
                return mavsdk::Mission::MissionItem::CameraAction::StartVideo;
            case rpc::mission::MissionItem_CameraAction_CAMERA_ACTION_STOP_VIDEO:
                return mavsdk::Mission::MissionItem::CameraAction::StopVideo;
            case rpc::mission::MissionItem_CameraAction_CAMERA_ACTION_START_PHOTO_DISTANCE:
                return mavsdk::Mission::MissionItem::CameraAction::StartPhotoDistance;
            case rpc::mission::MissionItem_CameraAction_CAMERA_ACTION_STOP_PHOTO_DISTANCE:
                return mavsdk::Mission::MissionItem::CameraAction::StopPhotoDistance;
        }
    }


    static std::unique_ptr<rpc::mission::MissionItem> translateToRpcMissionItem(const mavsdk::Mission::MissionItem &mission_item)
    {
        auto rpc_obj = std::make_unique<rpc::mission::MissionItem>();


            
        rpc_obj->set_latitude_deg(mission_item.latitude_deg);
            
        
            
        rpc_obj->set_longitude_deg(mission_item.longitude_deg);
            
        
            
        rpc_obj->set_relative_altitude_m(mission_item.relative_altitude_m);
            
        
            
        rpc_obj->set_speed_m_s(mission_item.speed_m_s);
            
        
            
        rpc_obj->set_is_fly_through(mission_item.is_fly_through);
            
        
            
        rpc_obj->set_gimbal_pitch_deg(mission_item.gimbal_pitch_deg);
            
        
            
        rpc_obj->set_gimbal_yaw_deg(mission_item.gimbal_yaw_deg);
            
        
            
                
        rpc_obj->set_camera_action(translateToRpcCameraAction(mission_item.camera_action));
                
            
        
            
        rpc_obj->set_loiter_time_s(mission_item.loiter_time_s);
            
        
            
        rpc_obj->set_camera_photo_interval_s(mission_item.camera_photo_interval_s);
            
        
            
        rpc_obj->set_acceptance_radius_m(mission_item.acceptance_radius_m);
            
        
            
        rpc_obj->set_yaw_deg(mission_item.yaw_deg);
            
        
            
        rpc_obj->set_camera_photo_distance_m(mission_item.camera_photo_distance_m);
            
        

        return rpc_obj;
    }

    static mavsdk::Mission::MissionItem translateFromRpcMissionItem(const rpc::mission::MissionItem& mission_item)
    {
        mavsdk::Mission::MissionItem obj;


            
        obj.latitude_deg = mission_item.latitude_deg();
            
        
            
        obj.longitude_deg = mission_item.longitude_deg();
            
        
            
        obj.relative_altitude_m = mission_item.relative_altitude_m();
            
        
            
        obj.speed_m_s = mission_item.speed_m_s();
            
        
            
        obj.is_fly_through = mission_item.is_fly_through();
            
        
            
        obj.gimbal_pitch_deg = mission_item.gimbal_pitch_deg();
            
        
            
        obj.gimbal_yaw_deg = mission_item.gimbal_yaw_deg();
            
        
            
        obj.camera_action = translateFromRpcCameraAction(mission_item.camera_action());
            
        
            
        obj.loiter_time_s = mission_item.loiter_time_s();
            
        
            
        obj.camera_photo_interval_s = mission_item.camera_photo_interval_s();
            
        
            
        obj.acceptance_radius_m = mission_item.acceptance_radius_m();
            
        
            
        obj.yaw_deg = mission_item.yaw_deg();
            
        
            
        obj.camera_photo_distance_m = mission_item.camera_photo_distance_m();
            
        
        return obj;
    }





    static std::unique_ptr<rpc::mission::MissionPlan> translateToRpcMissionPlan(const mavsdk::Mission::MissionPlan &mission_plan)
    {
        auto rpc_obj = std::make_unique<rpc::mission::MissionPlan>();


            
                
        for (const auto& elem : mission_plan.mission_items) {
            auto* ptr = rpc_obj->add_mission_items();
            ptr->CopyFrom(*translateToRpcMissionItem(elem).release());
        }
                
            
        

        return rpc_obj;
    }

    static mavsdk::Mission::MissionPlan translateFromRpcMissionPlan(const rpc::mission::MissionPlan& mission_plan)
    {
        mavsdk::Mission::MissionPlan obj;


            
                for (const auto& elem : mission_plan.mission_items()) {
                    obj.mission_items.push_back(translateFromRpcMissionItem(static_cast<mavsdk::rpc::mission::MissionItem>(elem)));
                }
            
        
        return obj;
    }





    static std::unique_ptr<rpc::mission::MissionProgress> translateToRpcMissionProgress(const mavsdk::Mission::MissionProgress &mission_progress)
    {
        auto rpc_obj = std::make_unique<rpc::mission::MissionProgress>();


            
        rpc_obj->set_current(mission_progress.current);
            
        
            
        rpc_obj->set_total(mission_progress.total);
            
        

        return rpc_obj;
    }

    static mavsdk::Mission::MissionProgress translateFromRpcMissionProgress(const rpc::mission::MissionProgress& mission_progress)
    {
        mavsdk::Mission::MissionProgress obj;


            
        obj.current = mission_progress.current();
            
        
            
        obj.total = mission_progress.total();
            
        
        return obj;
    }




    static rpc::mission::MissionResult::Result translateToRpcResult(const mavsdk::Mission::Result& result)
    {
        switch (result) {
            default:
                LogErr() << "Unknown result enum value: " << static_cast<int>(result);
            // FALLTHROUGH
            case mavsdk::Mission::Result::Unknown:
                return rpc::mission::MissionResult_Result_RESULT_UNKNOWN;
            case mavsdk::Mission::Result::Success:
                return rpc::mission::MissionResult_Result_RESULT_SUCCESS;
            case mavsdk::Mission::Result::Error:
                return rpc::mission::MissionResult_Result_RESULT_ERROR;
            case mavsdk::Mission::Result::TooManyMissionItems:
                return rpc::mission::MissionResult_Result_RESULT_TOO_MANY_MISSION_ITEMS;
            case mavsdk::Mission::Result::Busy:
                return rpc::mission::MissionResult_Result_RESULT_BUSY;
            case mavsdk::Mission::Result::Timeout:
                return rpc::mission::MissionResult_Result_RESULT_TIMEOUT;
            case mavsdk::Mission::Result::InvalidArgument:
                return rpc::mission::MissionResult_Result_RESULT_INVALID_ARGUMENT;
            case mavsdk::Mission::Result::Unsupported:
                return rpc::mission::MissionResult_Result_RESULT_UNSUPPORTED;
            case mavsdk::Mission::Result::NoMissionAvailable:
                return rpc::mission::MissionResult_Result_RESULT_NO_MISSION_AVAILABLE;
            case mavsdk::Mission::Result::UnsupportedMissionCmd:
                return rpc::mission::MissionResult_Result_RESULT_UNSUPPORTED_MISSION_CMD;
            case mavsdk::Mission::Result::TransferCancelled:
                return rpc::mission::MissionResult_Result_RESULT_TRANSFER_CANCELLED;
            case mavsdk::Mission::Result::NoSystem:
                return rpc::mission::MissionResult_Result_RESULT_NO_SYSTEM;
            case mavsdk::Mission::Result::Next:
                return rpc::mission::MissionResult_Result_RESULT_NEXT;
        }
    }

    static mavsdk::Mission::Result translateFromRpcResult(const rpc::mission::MissionResult::Result result)
    {
        switch (result) {
            default:
                LogErr() << "Unknown result enum value: " << static_cast<int>(result);
            // FALLTHROUGH
            case rpc::mission::MissionResult_Result_RESULT_UNKNOWN:
                return mavsdk::Mission::Result::Unknown;
            case rpc::mission::MissionResult_Result_RESULT_SUCCESS:
                return mavsdk::Mission::Result::Success;
            case rpc::mission::MissionResult_Result_RESULT_ERROR:
                return mavsdk::Mission::Result::Error;
            case rpc::mission::MissionResult_Result_RESULT_TOO_MANY_MISSION_ITEMS:
                return mavsdk::Mission::Result::TooManyMissionItems;
            case rpc::mission::MissionResult_Result_RESULT_BUSY:
                return mavsdk::Mission::Result::Busy;
            case rpc::mission::MissionResult_Result_RESULT_TIMEOUT:
                return mavsdk::Mission::Result::Timeout;
            case rpc::mission::MissionResult_Result_RESULT_INVALID_ARGUMENT:
                return mavsdk::Mission::Result::InvalidArgument;
            case rpc::mission::MissionResult_Result_RESULT_UNSUPPORTED:
                return mavsdk::Mission::Result::Unsupported;
            case rpc::mission::MissionResult_Result_RESULT_NO_MISSION_AVAILABLE:
                return mavsdk::Mission::Result::NoMissionAvailable;
            case rpc::mission::MissionResult_Result_RESULT_UNSUPPORTED_MISSION_CMD:
                return mavsdk::Mission::Result::UnsupportedMissionCmd;
            case rpc::mission::MissionResult_Result_RESULT_TRANSFER_CANCELLED:
                return mavsdk::Mission::Result::TransferCancelled;
            case rpc::mission::MissionResult_Result_RESULT_NO_SYSTEM:
                return mavsdk::Mission::Result::NoSystem;
            case rpc::mission::MissionResult_Result_RESULT_NEXT:
                return mavsdk::Mission::Result::Next;
        }
    }






    static std::unique_ptr<rpc::mission::ProgressData> translateToRpcProgressData(const mavsdk::Mission::ProgressData &progress_data)
    {
        auto rpc_obj = std::make_unique<rpc::mission::ProgressData>();


            
        rpc_obj->set_progress(progress_data.progress);
            
        

        return rpc_obj;
    }

    static mavsdk::Mission::ProgressData translateFromRpcProgressData(const rpc::mission::ProgressData& progress_data)
    {
        mavsdk::Mission::ProgressData obj;


            
        obj.progress = progress_data.progress();
            
        
        return obj;
    }





    static std::unique_ptr<rpc::mission::ProgressDataOrMission> translateToRpcProgressDataOrMission(const mavsdk::Mission::ProgressDataOrMission &progress_data_or_mission)
    {
        auto rpc_obj = std::make_unique<rpc::mission::ProgressDataOrMission>();


            
        rpc_obj->set_has_progress(progress_data_or_mission.has_progress);
            
        
            
        rpc_obj->set_progress(progress_data_or_mission.progress);
            
        
            
        rpc_obj->set_has_mission(progress_data_or_mission.has_mission);
            
        
            
                
        rpc_obj->set_allocated_mission_plan(translateToRpcMissionPlan(progress_data_or_mission.mission_plan).release());
                
            
        

        return rpc_obj;
    }

    static mavsdk::Mission::ProgressDataOrMission translateFromRpcProgressDataOrMission(const rpc::mission::ProgressDataOrMission& progress_data_or_mission)
    {
        mavsdk::Mission::ProgressDataOrMission obj;


            
        obj.has_progress = progress_data_or_mission.has_progress();
            
        
            
        obj.progress = progress_data_or_mission.progress();
            
        
            
        obj.has_mission = progress_data_or_mission.has_mission();
            
        
            
        obj.mission_plan = translateFromRpcMissionPlan(progress_data_or_mission.mission_plan());
            
        
        return obj;
    }



    grpc::Status UploadMission(
        grpc::ServerContext* /* context */,
        const rpc::mission::UploadMissionRequest* request,
        rpc::mission::UploadMissionResponse* response) override
    {
        if (_lazy_plugin.maybe_plugin() == nullptr) {
            
            if (response != nullptr) {
                auto result = mavsdk::Mission::Result::NoSystem;
                fillResponseWithResult(response, result);
            }
            
            return grpc::Status::OK;
        }

        if (request == nullptr) {
            LogWarn() << "UploadMission sent with a null request! Ignoring...";
            return grpc::Status::OK;
        }
            
        
        auto result = _lazy_plugin.maybe_plugin()->upload_mission(translateFromRpcMissionPlan(request->mission_plan()));
        

        
        if (response != nullptr) {
            fillResponseWithResult(response, result);
        }
        

        return grpc::Status::OK;
    }

    grpc::Status SubscribeUploadMissionWithProgress(grpc::ServerContext* /* context */, const mavsdk::rpc::mission::SubscribeUploadMissionWithProgressRequest* request, grpc::ServerWriter<rpc::mission::UploadMissionWithProgressResponse>* writer) override
    {
        if (_lazy_plugin.maybe_plugin() == nullptr) {
            
                rpc::mission::UploadMissionWithProgressResponse rpc_response;
                auto result = mavsdk::Mission::Result::NoSystem;
                fillResponseWithResult(&rpc_response, result);
                writer->Write(rpc_response);
            
            return grpc::Status::OK;
        }

        auto stream_closed_promise = std::make_shared<std::promise<void>>();
        auto stream_closed_future = stream_closed_promise->get_future();
        register_stream_stop_promise(stream_closed_promise);

        auto is_finished = std::make_shared<bool>(false);
        auto subscribe_mutex = std::make_shared<std::mutex>();

        _lazy_plugin.maybe_plugin()->upload_mission_with_progress_async(translateFromRpcMissionPlan(request->mission_plan()), 
            [this, &writer, &stream_closed_promise, is_finished, subscribe_mutex](mavsdk::Mission::Result result,const mavsdk::Mission::ProgressData upload_mission_with_progress) {

            rpc::mission::UploadMissionWithProgressResponse rpc_response;
        
            rpc_response.set_allocated_progress_data(translateToRpcProgressData(upload_mission_with_progress).release());
        

        
            auto rpc_result = translateToRpcResult(result);
            auto* rpc_mission_result = new rpc::mission::MissionResult();
            rpc_mission_result->set_result(rpc_result);
            std::stringstream ss;
            ss << result;
            rpc_mission_result->set_result_str(ss.str());
            rpc_response.set_allocated_mission_result(rpc_mission_result);
        

            std::unique_lock<std::mutex> lock(*subscribe_mutex);
            if (!*is_finished && !writer->Write(rpc_response)) {
                
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

    grpc::Status CancelMissionUpload(
        grpc::ServerContext* /* context */,
        const rpc::mission::CancelMissionUploadRequest* /* request */,
        rpc::mission::CancelMissionUploadResponse* response) override
    {
        if (_lazy_plugin.maybe_plugin() == nullptr) {
            
            if (response != nullptr) {
                auto result = mavsdk::Mission::Result::NoSystem;
                fillResponseWithResult(response, result);
            }
            
            return grpc::Status::OK;
        }

        
        auto result = _lazy_plugin.maybe_plugin()->cancel_mission_upload();
        

        
        if (response != nullptr) {
            fillResponseWithResult(response, result);
        }
        

        return grpc::Status::OK;
    }

    grpc::Status DownloadMission(
        grpc::ServerContext* /* context */,
        const rpc::mission::DownloadMissionRequest* /* request */,
        rpc::mission::DownloadMissionResponse* response) override
    {
        if (_lazy_plugin.maybe_plugin() == nullptr) {
            
            if (response != nullptr) {
                auto result = mavsdk::Mission::Result::NoSystem;
                fillResponseWithResult(response, result);
            }
            
            return grpc::Status::OK;
        }

        

        auto result = _lazy_plugin.maybe_plugin()->download_mission();

        if (response != nullptr) {
            fillResponseWithResult(response, result.first);
            
            response->set_allocated_mission_plan(translateToRpcMissionPlan(result.second).release());
            
        }


        return grpc::Status::OK;
    }

    grpc::Status SubscribeDownloadMissionWithProgress(grpc::ServerContext* /* context */, const mavsdk::rpc::mission::SubscribeDownloadMissionWithProgressRequest* /* request */, grpc::ServerWriter<rpc::mission::DownloadMissionWithProgressResponse>* writer) override
    {
        if (_lazy_plugin.maybe_plugin() == nullptr) {
            
                rpc::mission::DownloadMissionWithProgressResponse rpc_response;
                auto result = mavsdk::Mission::Result::NoSystem;
                fillResponseWithResult(&rpc_response, result);
                writer->Write(rpc_response);
            
            return grpc::Status::OK;
        }

        auto stream_closed_promise = std::make_shared<std::promise<void>>();
        auto stream_closed_future = stream_closed_promise->get_future();
        register_stream_stop_promise(stream_closed_promise);

        auto is_finished = std::make_shared<bool>(false);
        auto subscribe_mutex = std::make_shared<std::mutex>();

        _lazy_plugin.maybe_plugin()->download_mission_with_progress_async(
            [this, &writer, &stream_closed_promise, is_finished, subscribe_mutex](mavsdk::Mission::Result result,const mavsdk::Mission::ProgressDataOrMission download_mission_with_progress) {

            rpc::mission::DownloadMissionWithProgressResponse rpc_response;
        
            rpc_response.set_allocated_progress_data(translateToRpcProgressDataOrMission(download_mission_with_progress).release());
        

        
            auto rpc_result = translateToRpcResult(result);
            auto* rpc_mission_result = new rpc::mission::MissionResult();
            rpc_mission_result->set_result(rpc_result);
            std::stringstream ss;
            ss << result;
            rpc_mission_result->set_result_str(ss.str());
            rpc_response.set_allocated_mission_result(rpc_mission_result);
        

            std::unique_lock<std::mutex> lock(*subscribe_mutex);
            if (!*is_finished && !writer->Write(rpc_response)) {
                
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

    grpc::Status CancelMissionDownload(
        grpc::ServerContext* /* context */,
        const rpc::mission::CancelMissionDownloadRequest* /* request */,
        rpc::mission::CancelMissionDownloadResponse* response) override
    {
        if (_lazy_plugin.maybe_plugin() == nullptr) {
            
            if (response != nullptr) {
                auto result = mavsdk::Mission::Result::NoSystem;
                fillResponseWithResult(response, result);
            }
            
            return grpc::Status::OK;
        }

        
        auto result = _lazy_plugin.maybe_plugin()->cancel_mission_download();
        

        
        if (response != nullptr) {
            fillResponseWithResult(response, result);
        }
        

        return grpc::Status::OK;
    }

    grpc::Status StartMission(
        grpc::ServerContext* /* context */,
        const rpc::mission::StartMissionRequest* /* request */,
        rpc::mission::StartMissionResponse* response) override
    {
        if (_lazy_plugin.maybe_plugin() == nullptr) {
            
            if (response != nullptr) {
                auto result = mavsdk::Mission::Result::NoSystem;
                fillResponseWithResult(response, result);
            }
            
            return grpc::Status::OK;
        }

        
        auto result = _lazy_plugin.maybe_plugin()->start_mission();
        

        
        if (response != nullptr) {
            fillResponseWithResult(response, result);
        }
        

        return grpc::Status::OK;
    }

    grpc::Status PauseMission(
        grpc::ServerContext* /* context */,
        const rpc::mission::PauseMissionRequest* /* request */,
        rpc::mission::PauseMissionResponse* response) override
    {
        if (_lazy_plugin.maybe_plugin() == nullptr) {
            
            if (response != nullptr) {
                auto result = mavsdk::Mission::Result::NoSystem;
                fillResponseWithResult(response, result);
            }
            
            return grpc::Status::OK;
        }

        
        auto result = _lazy_plugin.maybe_plugin()->pause_mission();
        

        
        if (response != nullptr) {
            fillResponseWithResult(response, result);
        }
        

        return grpc::Status::OK;
    }

    grpc::Status ClearMission(
        grpc::ServerContext* /* context */,
        const rpc::mission::ClearMissionRequest* /* request */,
        rpc::mission::ClearMissionResponse* response) override
    {
        if (_lazy_plugin.maybe_plugin() == nullptr) {
            
            if (response != nullptr) {
                auto result = mavsdk::Mission::Result::NoSystem;
                fillResponseWithResult(response, result);
            }
            
            return grpc::Status::OK;
        }

        
        auto result = _lazy_plugin.maybe_plugin()->clear_mission();
        

        
        if (response != nullptr) {
            fillResponseWithResult(response, result);
        }
        

        return grpc::Status::OK;
    }

    grpc::Status SetCurrentMissionItem(
        grpc::ServerContext* /* context */,
        const rpc::mission::SetCurrentMissionItemRequest* request,
        rpc::mission::SetCurrentMissionItemResponse* response) override
    {
        if (_lazy_plugin.maybe_plugin() == nullptr) {
            
            if (response != nullptr) {
                auto result = mavsdk::Mission::Result::NoSystem;
                fillResponseWithResult(response, result);
            }
            
            return grpc::Status::OK;
        }

        if (request == nullptr) {
            LogWarn() << "SetCurrentMissionItem sent with a null request! Ignoring...";
            return grpc::Status::OK;
        }
            
        
        auto result = _lazy_plugin.maybe_plugin()->set_current_mission_item(request->index());
        

        
        if (response != nullptr) {
            fillResponseWithResult(response, result);
        }
        

        return grpc::Status::OK;
    }

    grpc::Status IsMissionFinished(
        grpc::ServerContext* /* context */,
        const rpc::mission::IsMissionFinishedRequest* /* request */,
        rpc::mission::IsMissionFinishedResponse* response) override
    {
        if (_lazy_plugin.maybe_plugin() == nullptr) {
            
            if (response != nullptr) {
                auto result = mavsdk::Mission::Result::NoSystem;
                fillResponseWithResult(response, result);
            }
            
            return grpc::Status::OK;
        }

        

        auto result = _lazy_plugin.maybe_plugin()->is_mission_finished();

        if (response != nullptr) {
            fillResponseWithResult(response, result.first);
            
            response->set_is_finished(result.second);
            
        }


        return grpc::Status::OK;
    }

    grpc::Status SubscribeMissionProgress(grpc::ServerContext* /* context */, const mavsdk::rpc::mission::SubscribeMissionProgressRequest* /* request */, grpc::ServerWriter<rpc::mission::MissionProgressResponse>* writer) override
    {
        if (_lazy_plugin.maybe_plugin() == nullptr) {
            
            return grpc::Status::OK;
        }

        auto stream_closed_promise = std::make_shared<std::promise<void>>();
        auto stream_closed_future = stream_closed_promise->get_future();
        register_stream_stop_promise(stream_closed_promise);

        auto is_finished = std::make_shared<bool>(false);
        auto subscribe_mutex = std::make_shared<std::mutex>();

        _lazy_plugin.maybe_plugin()->subscribe_mission_progress(
            [this, &writer, &stream_closed_promise, is_finished, subscribe_mutex](const mavsdk::Mission::MissionProgress mission_progress) {

            rpc::mission::MissionProgressResponse rpc_response;
        
            rpc_response.set_allocated_mission_progress(translateToRpcMissionProgress(mission_progress).release());
        

        

            std::unique_lock<std::mutex> lock(*subscribe_mutex);
            if (!*is_finished && !writer->Write(rpc_response)) {
                
                _lazy_plugin.maybe_plugin()->subscribe_mission_progress(nullptr);
                
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

    grpc::Status GetReturnToLaunchAfterMission(
        grpc::ServerContext* /* context */,
        const rpc::mission::GetReturnToLaunchAfterMissionRequest* /* request */,
        rpc::mission::GetReturnToLaunchAfterMissionResponse* response) override
    {
        if (_lazy_plugin.maybe_plugin() == nullptr) {
            
            if (response != nullptr) {
                auto result = mavsdk::Mission::Result::NoSystem;
                fillResponseWithResult(response, result);
            }
            
            return grpc::Status::OK;
        }

        

        auto result = _lazy_plugin.maybe_plugin()->get_return_to_launch_after_mission();

        if (response != nullptr) {
            fillResponseWithResult(response, result.first);
            
            response->set_enable(result.second);
            
        }


        return grpc::Status::OK;
    }

    grpc::Status SetReturnToLaunchAfterMission(
        grpc::ServerContext* /* context */,
        const rpc::mission::SetReturnToLaunchAfterMissionRequest* request,
        rpc::mission::SetReturnToLaunchAfterMissionResponse* response) override
    {
        if (_lazy_plugin.maybe_plugin() == nullptr) {
            
            if (response != nullptr) {
                auto result = mavsdk::Mission::Result::NoSystem;
                fillResponseWithResult(response, result);
            }
            
            return grpc::Status::OK;
        }

        if (request == nullptr) {
            LogWarn() << "SetReturnToLaunchAfterMission sent with a null request! Ignoring...";
            return grpc::Status::OK;
        }
            
        
        auto result = _lazy_plugin.maybe_plugin()->set_return_to_launch_after_mission(request->enable());
        

        
        if (response != nullptr) {
            fillResponseWithResult(response, result);
        }
        

        return grpc::Status::OK;
    }


    void stop() {
        _stopped.store(true);
        for (auto& prom : _stream_stop_promises) {
            if (auto handle = prom.lock()) {
                handle->set_value();
            }
        }
    }

private:
    void register_stream_stop_promise(std::weak_ptr<std::promise<void>> prom) {
        // If we have already stopped, set promise immediately and don't add it to list.
        if (_stopped.load()) {
            if (auto handle = prom.lock()) {
                handle->set_value();
            }
        } else {
            _stream_stop_promises.push_back(prom);
        }
    }

    void unregister_stream_stop_promise(std::shared_ptr<std::promise<void>> prom) {
        for (auto it = _stream_stop_promises.begin(); it != _stream_stop_promises.end(); /* ++it */) {
            if (it->lock() == prom) {
                it = _stream_stop_promises.erase(it);
            } else {
                ++it;
            }
        }
    }

    LazyPlugin& _lazy_plugin;
    std::atomic<bool> _stopped{false};
    std::vector<std::weak_ptr<std::promise<void>>> _stream_stop_promises {};
};

} // namespace mavsdk_server
} // namespace mavsdk