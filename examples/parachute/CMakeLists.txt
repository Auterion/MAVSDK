cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(parachute)

add_executable(parachute
    parachute.cpp
)

find_package(MAVSDK REQUIRED)

target_link_libraries(parachute
    MAVSDK::mavsdk
    MAVSDK::mavsdk_mavlink_passthrough
)

if(NOT MSVC)
    add_compile_options(parachute PRIVATE -Wall -Wextra)
else()
    add_compile_options(parachute PRIVATE -WX -W2)
endif()
