#!/usr/bin/env sh

set -e

if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <path/to/Firmware>"
    exit 1
fi

if [ "${PX4_VERSION}" ]; then
    echo "PX4 Autopilot Version Specified: " ${PX4_VERSION}
    PX4_FIRMWARE_DIR=$1
elif [ "${APM_VERSION}" ]; then
    echo "Ardupilot Autopilot Version Specified: " ${APM_VERSION}
    APM_FIRMWARE_DIR=$1
else
    echo "No Autopilot Version specified. Exiting."
    exit 1
fi

NPROCS=$(nproc --all)
AUTERION_PX4_VERSIONS="auterion-develop"

cmake -DCMAKE_BUILD_TYPE=Debug -DASAN=ON -DBUILD_MAVSDK_SERVER=OFF -DBUILD_SHARED_LIBS=ON -j $NPROCS -Bbuild/debug -H.;
cmake --build build/debug -- -j $NPROCS;

# Filter the tests to run according to availability in upstream
if [ -n "`echo $AUTERION_PX4_VERSIONS | xargs -n1 echo | grep -e \"^$PX4_VERSION$\"`" ] ; then
  PX4_SIM_SPEED_FACTOR=10 AUTOSTART_SITL=1 PX4_FIRMWARE_DIR=$PX4_FIRMWARE_DIR HEADLESS=1 build/default/src/integration_tests/integration_tests_runner --gtest_filter="SitlTest.*:-SitlTest.AP*"
else
  PX4_SIM_SPEED_FACTOR=10 AUTOSTART_SITL=1 PX4_FIRMWARE_DIR=$PX4_FIRMWARE_DIR HEADLESS=1 build/default/src/integration_tests/integration_tests_runner --gtest_filter="SitlTest.*:-SitlTest.CustomAction*"
fi
