#include <future>

#include <iostream>
#include <map>
#include <thread>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/camera/camera.h>
#include <mavsdk/plugins/camera_server/camera_server.h>

using namespace mavsdk;

using std::chrono::duration_cast;
using std::chrono::seconds;
using std::chrono::milliseconds;
using std::chrono::system_clock;
using std::this_thread::sleep_for;

int main(int argc, char** argv)
{
    mavsdk::Mavsdk mavsdk;
    mavsdk::Mavsdk::Configuration configuration(mavsdk::Mavsdk::Configuration::UsageType::Camera);
    mavsdk.set_configuration(configuration);

    // 14030 is the default camera port for PX4 SITL
    // FIXME: the camera should probably initiate the connection and not just listen
    auto result = mavsdk.add_any_connection("udp://:14030");
    if (result != mavsdk::ConnectionResult::Success) {
        std::cerr << "Could not establish connection: " << result << std::endl;
        return 1;
    }
    std::cout << "Created camera server connection" << std::endl;

    auto camera_server = mavsdk::CameraServer{
        mavsdk.server_component_by_type(mavsdk::Mavsdk::ServerComponentType::Camera)};

    // First add all subscriptions. This defines the camera capabilities.

    camera_server.subscribe_take_photo([&camera_server](int32_t index) {
        camera_server.set_in_progress(true);

        std::cout << "taking a picture (" << +index << ")..." << std::endl;

        // TODO : actually capture image here
        // simulating with delay
        sleep_for(milliseconds(500));

        // TODO: populate with telemetry data
        auto position = CameraServer::Position{};
        auto attitude = CameraServer::Quaternion{};

        auto timestamp =
            duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
        auto success = true;

        camera_server.set_in_progress(false);

        camera_server.respond_take_photo(
            mavsdk::CameraServer::TakePhotoFeedback::Ok,
            mavsdk::CameraServer::CaptureInfo{
                .position = position,
                .attitude_quaternion = attitude,
                .time_utc_us = static_cast<uint64_t>(timestamp),
                .is_success = success,
                .index = index,
                .file_url = {},
            });
    });

    // Then set the initial state of everything.

    // TODO: this state is not guaranteed, e.g. a new system appears
    // while a capture is in progress
    camera_server.set_in_progress(false);

    // Finally call set_information() to "activate" the camera plugin.

    auto ret = camera_server.set_information({
        .vendor_name = "MAVSDK",
        .model_name = "Example Camera Server",
        .firmware_version = "1.0.0",
        .focal_length_mm = 3.0,
        .horizontal_sensor_size_mm = 3.68,
        .vertical_sensor_size_mm = 2.76,
        .horizontal_resolution_px = 3280,
        .vertical_resolution_px = 2464,
        .lens_id = 0,
        .definition_file_version = 0, // TODO: add this
        .definition_file_uri = "", // TODO: implement this using MAVLink FTP
    });

    if (ret != CameraServer::Result::Success) {
        std::cerr << "Failed to set camera info, exiting" << std::endl;
        return 2;
    }

    while (true) {
        // Wait forever?
        sleep_for(seconds(1));
    }

    return 0;
}
