#pragma once

namespace ardupilot {

// Enumeration representing the available modes for the Arudpilot rover autopilot.
enum class RoverMode {
    Manual = 0,
    Acro = 1,
    Steering = 3,
    Hold = 4,
    Loiter = 5,
    Follow = 6,
    Simple = 7,
    Auto = 10,
    RTL = 11,
    Smart_RTL = 12,
    Guided = 15,
    Initializing = 16,
    Unknown = 100
};

// Enumeration representing the available modes for the Arudpilot copter autopilot.
enum class CopterMode {
    Stabilize = 0,
    Acro = 1,
    Alt_Hold = 2,
    Auto = 3,
    Guided = 4,
    Loiter = 5,
    RTL = 6,
    Circle = 7,
    Land = 9,
    Drift = 11,
    Sport = 13,
    Flip = 14,
    Auto_Tune = 15,
    POS_HOLD = 16,
    Break = 17,
    Throw = 18,
    Avoid_ADBS = 19,
    Guided_No_GPS = 20,
    Smart_RTL = 21,
    Flow_Hold = 22,
    Follow = 23,
    Zigzag = 24,
    System_ID = 25,
    Auto_Rotate = 26,
    Auto_RTL = 27,
    Turtle = 28,
    Unknown = 100
};
} // namespace ardupilot