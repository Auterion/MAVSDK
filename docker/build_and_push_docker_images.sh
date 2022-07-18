#!/usr/bin/env bash

set -e

DOCKER_CMD=docker

if ! command -v $DOCKER_CMD &> /dev/null
then
    echo "docker not found, trying podman instead"
    DOCKER_CMD=podman
fi

$DOCKER_CMD build -f Dockerfile-Fedora-34 -t mavsdk/mavsdk-fedora-34 .
$DOCKER_CMD build -f Dockerfile-Fedora-35 -t mavsdk/mavsdk-fedora-35 .
$DOCKER_CMD build -f Dockerfile-Fedora-36 -t mavsdk/mavsdk-fedora-36 .
$DOCKER_CMD build -f Dockerfile-Ubuntu-20.04 -t mavsdk/mavsdk-ubuntu-20.04 .
$DOCKER_CMD build -f Dockerfile-Ubuntu-20.04-PX4-SITL-v1.11 -t mavsdk/mavsdk-ubuntu-20.04-px4-sitl-v1.11 .
$DOCKER_CMD build -f Dockerfile-Ubuntu-20.04-PX4-SITL-v1.12 -t mavsdk/mavsdk-ubuntu-20.04-px4-sitl-v1.12 .
$DOCKER_CMD build -f Dockerfile-Ubuntu-20.04-PX4-SITL-v1.13 -t mavsdk/mavsdk-ubuntu-20.04-px4-sitl-v1.13 .
$DOCKER_CMD build -f Dockerfile-Ubuntu-20.04-APM-SITL-Copter-4.1.2 -t mavsdk/mavsdk-ubuntu-20.04-apm-sitl-copter-4.1.2 .
$DOCKER_CMD build -f Dockerfile-Ubuntu-20.04-APM-SITL-Rover-4.1.2 -t mavsdk/mavsdk-ubuntu-20.04-apm-sitl-rover-4.1.2 .
$DOCKER_CMD build -f Dockerfile.dockcross-linux-armv6-custom -t mavsdk/mavsdk-dockcross-linux-armv6-custom .
$DOCKER_CMD build -f Dockerfile.dockcross-linux-armv7-custom -t mavsdk/mavsdk-dockcross-linux-armv7-custom .
$DOCKER_CMD build -f Dockerfile.dockcross-linux-arm64-custom -t mavsdk/mavsdk-dockcross-linux-arm64-custom .

# APX4 releases
$DOCKER_CMD build --build-arg SSH_PRIVATE_KEY="$(cat ~/.ssh/id_rsa)" -f Dockerfile-Ubuntu-20.04-APX4-SITL-v2.1 -t auterion/mavsdk-ubuntu-20.04-apx4-sitl-v2.1 .
$DOCKER_CMD build --build-arg SSH_PRIVATE_KEY="$(cat ~/.ssh/id_rsa)" -f Dockerfile-Ubuntu-20.04-APX4-SITL-develop -t auterion/mavsdk-ubuntu-20.04-apx4-sitl-develop .

$DOCKER_CMD push mavsdk/mavsdk-fedora-34:latest
$DOCKER_CMD push mavsdk/mavsdk-fedora-35:latest
$DOCKER_CMD push mavsdk/mavsdk-fedora-36:latest
$DOCKER_CMD push mavsdk/mavsdk-ubuntu-20.04:latest
$DOCKER_CMD push mavsdk/mavsdk-ubuntu-20.04-px4-sitl-v1.11:latest
$DOCKER_CMD push mavsdk/mavsdk-ubuntu-20.04-px4-sitl-v1.12:latest
$DOCKER_CMD push mavsdk/mavsdk-ubuntu-20.04-px4-sitl-v1.13:latest
$DOCKER_CMD push mavsdk/mavsdk-ubuntu-20.04-apm-sitl-copter-4.1.2:latest
$DOCKER_CMD push mavsdk/mavsdk-ubuntu-20.04-apm-sitl-rover-4.1.2:latest
$DOCKER_CMD push mavsdk/mavsdk-dockcross-linux-armv6-custom:latest
$DOCKER_CMD push mavsdk/mavsdk-dockcross-linux-armv7-custom:latest
$DOCKER_CMD push mavsdk/mavsdk-dockcross-linux-arm64-custom:latest

$DOCKER_CMD push auterion/mavsdk-ubuntu-20.04-apx4-sitl-v2.1:latest
$DOCKER_CMD push auterion/mavsdk-ubuntu-20.04-apx4-sitl-develop:latest

