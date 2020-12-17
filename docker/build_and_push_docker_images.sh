#!/usr/bin/env bash

set -e

docker build -f Dockerfile-Fedora-32 -t mavsdk/mavsdk-fedora-32 .
docker build -f Dockerfile-Fedora-33 -t mavsdk/mavsdk-fedora-33 .
docker build -f Dockerfile-Ubuntu-18.04 -t mavsdk/mavsdk-ubuntu-18.04 .
docker build -f Dockerfile-Ubuntu-20.04 -t mavsdk/mavsdk-ubuntu-20.04 .
docker build -f Dockerfile-Ubuntu-18.04-PX4-SITL-v1.9 -t mavsdk/mavsdk-ubuntu-18.04-px4-sitl-v1.9 .
docker build -f Dockerfile-Ubuntu-18.04-PX4-SITL-v1.10 -t mavsdk/mavsdk-ubuntu-18.04-px4-sitl-v1.10 .
docker build -f Dockerfile-Ubuntu-20.04-PX4-SITL-v1.11 -t mavsdk/mavsdk-ubuntu-20.04-px4-sitl-v1.11 .

# APX4 releases
docker build --build-arg SSH_PRIVATE_KEY="$(cat ~/.ssh/id_rsa)" -f Dockerfile-Ubuntu-20.04-APX4-SITL-v2.1 -t auterion/mavsdk-ubuntu-20.04-apx4-sitl-v2.1 .
docker build --build-arg SSH_PRIVATE_KEY="$(cat ~/.ssh/id_rsa)" -f Dockerfile-Ubuntu-20.04-APX4-SITL-develop -t auterion/mavsdk-ubuntu-20.04-apx4-sitl-develop .

docker push mavsdk/mavsdk-fedora-32:latest
docker push mavsdk/mavsdk-fedora-33:latest
docker push mavsdk/mavsdk-ubuntu-18.04:latest
docker push mavsdk/mavsdk-ubuntu-20.04:latest
docker push mavsdk/mavsdk-ubuntu-18.04-px4-sitl-v1.9:latest
docker push mavsdk/mavsdk-ubuntu-18.04-px4-sitl-v1.10:latest
docker push mavsdk/mavsdk-ubuntu-20.04-px4-sitl-v1.11:latest

# APX4 releases
docker push auterion/mavsdk-ubuntu-20.04-apx4-sitl-v2.1:latest
docker push auterion/mavsdk-ubuntu-20.04-apx4-sitl-develop:latest
