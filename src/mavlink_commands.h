#pragma once

#include "mavlink_include.h"
#include "locked_queue.h"
#include <cstdint>
#include <string>
#include <functional>

namespace dronelink {

class DeviceImpl;

class MavlinkCommands
{
public:
    explicit MavlinkCommands(DeviceImpl *parent);
    ~MavlinkCommands();

    enum class Result {
        SUCCESS = 0,
        NO_DEVICE,
        CONNECTION_ERROR,
        BUSY,
        COMMAND_DENIED,
        TIMEOUT
    };

    typedef std::function<void(Result)> command_result_callback_t;

    struct Params {
        float v[7];
    };

    Result send_command(uint16_t command,
                        const Params params,
                        uint8_t target_system_id,
                        uint8_t target_component_id);

    void queue_command_async(uint16_t command,
                             const Params params,
                             uint8_t target_system_id,
                             uint8_t target_component_id,
                             command_result_callback_t callback);

    void do_work();

    // Non-copyable
    MavlinkCommands(const MavlinkCommands &) = delete;
    const MavlinkCommands &operator=(const MavlinkCommands &) = delete;

private:

    struct Work {
        enum class State {
            NONE,
            WAITING,
            TEMPORARILY_REJECTED,
            IN_PROGRESS,
            DONE,
            FAILED
        } state = State::NONE;

        int retries_to_do = 3;
        double timeout_s = 0.5;
        uint16_t mavlink_command = 0;
        mavlink_message_t mavlink_message {};
        command_result_callback_t callback {};
    };

    void receive_command_ack(mavlink_message_t message);
    void receive_timeout();

    DeviceImpl *_parent;
    LockedQueue<Work> _work_queue {};
};

} // namespace dronelink
