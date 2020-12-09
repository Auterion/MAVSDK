#pragma once

#include "plugins/calibration/calibration.h"
#include "calibration_statustext_parser.h"
#include "mavlink_include.h"
#include "plugin_impl_base.h"
#include "system.h"

namespace mavsdk {

class CalibrationImpl : public PluginImplBase {
public:
    explicit CalibrationImpl(System& system);
    explicit CalibrationImpl(std::shared_ptr<System> system);
    explicit CalibrationImpl(SystemImpl* system_impl);
    ~CalibrationImpl();

    void init() override;
    void deinit() override;

    void enable() override;
    void disable() override;

    void cancel() const;

    void calibrate_gyro_async(const Calibration::CalibrateGyroCallback& callback);
    void calibrate_accelerometer_async(const Calibration::CalibrateAccelerometerCallback& callback);
    void calibrate_magnetometer_async(const Calibration::CalibrateMagnetometerCallback& callback);
    void calibrate_level_horizon_async(const Calibration::CalibrateAccelerometerCallback& callback);
    void calibrate_gimbal_accelerometer_async(
        const Calibration::CalibrateGimbalAccelerometerCallback& callback);

private:
    typedef std::function<void(const Calibration::Result result, const Calibration::ProgressData)>
        CalibrationCallback;

    void call_callback(
        const CalibrationCallback& callback,
        const Calibration::Result& result,
        const Calibration::ProgressData progress_data);
    void process_statustext(const mavlink_message_t& message);

    void command_result_callback(MavlinkCommandSender::Result command_result, float progress);

    static Calibration::Result
    calibration_result_from_command_result(MavlinkCommandSender::Result result);

    void report_started();
    void report_done();
    void report_warning(const std::string& warning);
    void report_failed(const std::string& failed);
    void report_cancelled();
    void report_progress(float progress);
    void report_instruction(const std::string& instruction);

    CalibrationStatustextParser _parser{};

    mutable std::mutex _calibration_mutex{};

    bool _is_gyro_ok = false;
    bool _is_accelerometer_ok = false;
    bool _is_magnetometer_ok = false;

    std::atomic<bool> _is_gyro_running = {false};
    std::atomic<bool> _is_accelerometer_running = {false};
    std::atomic<bool> _is_magnetometer_running = {false};

    enum class State {
        None,
        GyroCalibration,
        AccelerometerCalibration,
        MagnetometerCalibration,
        LevelHorizonCalibration,
        GimbalAccelerometerCalibration
    } _state{State::None};

    CalibrationCallback _calibration_callback{nullptr};
};

} // namespace mavsdk
