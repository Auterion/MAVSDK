#!/usr/bin/env bash

set -e

docker build -f Dockerfile-Fedora-34 -t mavsdk/mavsdk-fedora-34 .
docker build -f Dockerfile-Fedora-35 -t mavsdk/mavsdk-fedora-35 .
docker build -f Dockerfile-Ubuntu-20.04 -t mavsdk/mavsdk-ubuntu-20.04 .
docker build -f Dockerfile-Ubuntu-20.04-PX4-SITL-v1.11 -t mavsdk/mavsdk-ubuntu-20.04-px4-sitl-v1.11 .
docker build -f Dockerfile-Ubuntu-20.04-PX4-SITL-v1.12 -t mavsdk/mavsdk-ubuntu-20.04-px4-sitl-v1.12 .
docker build -f Dockerfile.dockcross-linux-armv6-custom -t mavsdk/mavsdk-dockcross-linux-armv6-custom .
docker build -f Dockerfile.dockcross-linux-armv7-custom -t mavsdk/mavsdk-dockcross-linux-armv7-custom .
docker build -f Dockerfile.dockcross-linux-arm64-custom -t mavsdk/mavsdk-dockcross-linux-arm64-custom .
docker build -f Dockerfile-Ubuntu-20.04-APM-SITL-Copter-4.1.2 -t mavsdk/mavsdk-ubuntu-20.04-apm-sitl-copter-4.1.2 .
docker build -f Dockerfile-Ubuntu-20.04-APM-SITL-Rover-4.1.2 -t mavsdk/mavsdk-ubuntu-20.04-apm-sitl-rover-4.1.2 .

docker push mavsdk/mavsdk-fedora-34:latest
docker push mavsdk/mavsdk-fedora-35:latest
docker push mavsdk/mavsdk-ubuntu-20.04:latest
docker push mavsdk/mavsdk-ubuntu-20.04-px4-sitl-v1.11:latest
docker push mavsdk/mavsdk-ubuntu-20.04-px4-sitl-v1.12:latest
docker push mavsdk/mavsdk-ubuntu-20.04-apm-sitl-copter-4.1.2
docker push mavsdk/mavsdk-ubuntu-20.04-apm-sitl-rover-4.1.2
