//
// Example to demonstrate how to switch to photo mode and take a picture.
//

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/camera/camera.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <future>
#include <memory>
#include <thread>

using namespace mavsdk;
using std::chrono::seconds;
using std::this_thread::sleep_for;

void usage(std::string bin_name)
{
    std::cerr << "Usage : " << bin_name << " <connection_url>\n"
              << "Connection URL format should be :\n"
              << " For TCP : tcp://[server_host][:server_port]\n"
              << " For UDP : udp://[bind_host][:bind_port]\n"
              << " For Serial : serial:///path/to/serial/dev[:baudrate]\n"
              << "For example, to connect to the simulator use URL: udp://:14540\n";
}

int main(int argc, char** argv)
{
    if (argc != 2) {
        usage(argv[0]);
        return 1;
    }

    Mavsdk mavsdk;
    ConnectionResult connection_result = mavsdk.add_any_connection(argv[1]);

    if (connection_result != ConnectionResult::Success) {
        std::cerr << "Connection failed: " << connection_result << '\n';
        return 1;
    }

    std::cout << "Waiting to discover system...\n";

	std::shared_ptr<mavsdk::System> system;
	bool found_camera_system = false;
	while (!found_camera_system) {
			//Iterate through detected systems
			for (auto sys : mavsdk.systems()) {
					std::cout << "Found system with MAVLink system ID: " << static_cast<int>(sys->get_system_id())
										<< ", connected: " << (sys->is_connected() ? "yes" : "no")
										<< ", has camera: " << (sys->has_camera() ? "yes" : "no") << '\n';

					if (sys->has_camera()) {
							system = sys;
							found_camera_system = true;
							break;
					}
			}
			sleep_for(seconds(1));
	}

	auto camera = Camera{system};

	camera.subscribe_capture_info([](Camera::CaptureInfo capture_info) {
		if (capture_info.is_success) {
				std::cout << "Capture Info: " << capture_info.index << ": " << capture_info.file_url
				<< " " << capture_info.time_utc_us << std::endl;
		} else {
				std::cout << "Failed capture info" << std::endl;
		}
	
	});

	while (true) {
		auto key = getchar();

		if (key == 'd') {
			std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
			Camera::Result res = camera.format_storage();
			std::cout << "Res:" << res << std::endl;
			std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
		} else if (key == 'l') {
			auto photos = camera.list_photos(Camera::PhotosRange::All);
			if (photos.first == Camera::Result::Success) {
				std::cout << "List photos All" << std::endl;
				for (auto p : photos.second) {
					std::cout << p.index << ": " << p.file_url << " " << p.time_utc_us << std::endl;
				}
				std::cout << std::endl;
			}
		}			
	}

    return 0;
}