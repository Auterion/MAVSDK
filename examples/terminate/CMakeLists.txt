cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(terminate)

add_executable(terminate
    terminate.cpp
)

find_package(MAVSDK REQUIRED)

target_link_libraries(terminate
    MAVSDK::mavsdk
    MAVSDK::mavsdk_action
)

if(NOT MSVC)
    add_compile_options(terminate PRIVATE -Wall -Wextra)
else()
    add_compile_options(terminate PRIVATE -WX -W2)
endif()
