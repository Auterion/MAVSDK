cmake_minimum_required(VERSION 3.13)

project(external-mavlink)
include(ExternalProject)

# Without this GitHub actions picks the wrong python3 executable.
set(Python3_FIND_REGISTRY "NEVER")
find_package(Python3 COMPONENTS Interpreter REQUIRED)

if (NOT MAVLINK_DIALECT)
    set(MAVLINK_DIALECT ras_a)
endif()

ExternalProject_add(
    mavlink
    GIT_REPOSITORY https://github.com/Dronecode/air-iop-definitions
    GIT_TAG 67e4ebc74144246f292b8229798d7e4f7360ecd8 # RAS-A IOP v1.2
    PREFIX mavlink
    CONFIGURE_COMMAND Python3::Interpreter
        -m pymavlink.tools.mavgen
        --lang=C
        --wire-protocol=2.0
        --output=${CMAKE_INSTALL_PREFIX}/include/mavlink/v2.0/
        message_definitions/v1.0/${MAVLINK_DIALECT}.xml
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
    BUILD_IN_SOURCE TRUE
)
