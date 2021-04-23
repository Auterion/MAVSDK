// WARNING: THIS FILE IS AUTOGENERATED! As such, it should not be edited.
// Edits need to be made to the proto files
// (see https://github.com/mavlink/MAVSDK-Proto/blob/master/protos/calibration/calibration.proto)

#include <iomanip>

#include "calibration_impl.h"
#include "plugins/calibration/calibration.h"

namespace mavsdk {

using ProgressData = Calibration::ProgressData;

Calibration::Calibration(System& system) :
    PluginBase(),
    _impl{std::make_unique<CalibrationImpl>(system)}
{}

Calibration::Calibration(std::shared_ptr<System> system) :
    PluginBase(),
    _impl{std::make_unique<CalibrationImpl>(system)}
{}

Calibration::~Calibration() {}

void Calibration::calibrate_gyro_async(CalibrateGyroCallback callback)
{
    _impl->calibrate_gyro_async(callback);
}

void Calibration::calibrate_accelerometer_async(CalibrateAccelerometerCallback callback)
{
    _impl->calibrate_accelerometer_async(callback);
}

void Calibration::calibrate_magnetometer_async(CalibrateMagnetometerCallback callback)
{
    _impl->calibrate_magnetometer_async(callback);
}

void Calibration::calibrate_level_horizon_async(CalibrateLevelHorizonCallback callback)
{
    _impl->calibrate_level_horizon_async(callback);
}

void Calibration::calibrate_gimbal_accelerometer_async(
    CalibrateGimbalAccelerometerCallback callback)
{
    _impl->calibrate_gimbal_accelerometer_async(callback);
}

Calibration::Result Calibration::cancel() const
{
    return _impl->cancel();
}

std::ostream& operator<<(std::ostream& str, Calibration::Result const& result)
{
    switch (result) {
        case Calibration::Result::Unknown:
            return str << "Unknown";
        case Calibration::Result::Success:
            return str << "Success";
        case Calibration::Result::Next:
            return str << "Next";
        case Calibration::Result::Failed:
            return str << "Failed";
        case Calibration::Result::NoSystem:
            return str << "No System";
        case Calibration::Result::ConnectionError:
            return str << "Connection Error";
        case Calibration::Result::Busy:
            return str << "Busy";
        case Calibration::Result::CommandDenied:
            return str << "Command Denied";
        case Calibration::Result::Timeout:
            return str << "Timeout";
        case Calibration::Result::Cancelled:
            return str << "Cancelled";
        case Calibration::Result::FailedArmed:
            return str << "Failed Armed";
        default:
            return str << "Unknown";
    }
}

bool operator==(const Calibration::ProgressData& lhs, const Calibration::ProgressData& rhs)
{
    return (rhs.has_progress == lhs.has_progress) &&
           ((std::isnan(rhs.progress) && std::isnan(lhs.progress)) ||
            rhs.progress == lhs.progress) &&
           (rhs.has_status_text == lhs.has_status_text) && (rhs.status_text == lhs.status_text);
}

std::ostream& operator<<(std::ostream& str, Calibration::ProgressData const& progress_data)
{
    str << std::setprecision(15);
    str << "progress_data:" << '\n' << "{\n";
    str << "    has_progress: " << progress_data.has_progress << '\n';
    str << "    progress: " << progress_data.progress << '\n';
    str << "    has_status_text: " << progress_data.has_status_text << '\n';
    str << "    status_text: " << progress_data.status_text << '\n';
    str << '}';
    return str;
}

} // namespace mavsdk