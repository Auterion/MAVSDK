cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(ftp_server)

add_executable(ftp_server
    ftp_server.cpp
)

find_package(MAVSDK REQUIRED)

target_link_libraries(ftp_server
    MAVSDK::mavsdk_ftp
    MAVSDK::mavsdk
)

if(NOT MSVC)
    add_compile_options(ftp_server PRIVATE -Wall -Wextra)
else()
    add_compile_options(ftp_server PRIVATE -WX -W2)
endif()
