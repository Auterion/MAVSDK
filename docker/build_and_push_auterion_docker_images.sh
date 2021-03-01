#!/usr/bin/env bash
# Specific to APX4 releases

set -e

docker build --build-arg SSH_PRIVATE_KEY="$(cat ~/.ssh/id_rsa)" -f Dockerfile-Ubuntu-20.04-APX4-SITL-v2.2 -t auterion/mavsdk-ubuntu-20.04-apx4-sitl-v2.2 .
docker build --build-arg SSH_PRIVATE_KEY="$(cat ~/.ssh/id_rsa)" -f Dockerfile-Ubuntu-20.04-APX4-SITL-v2.1 -t auterion/mavsdk-ubuntu-20.04-apx4-sitl-v2.1 .
docker build --build-arg SSH_PRIVATE_KEY="$(cat ~/.ssh/id_rsa)" -f Dockerfile-Ubuntu-20.04-APX4-SITL-develop -t auterion/mavsdk-ubuntu-20.04-apx4-sitl-develop .

docker push auterion/mavsdk-ubuntu-20.04-apx4-sitl-v2.2:latest
docker push auterion/mavsdk-ubuntu-20.04-apx4-sitl-v2.1:latest
docker push auterion/mavsdk-ubuntu-20.04-apx4-sitl-develop:latest
