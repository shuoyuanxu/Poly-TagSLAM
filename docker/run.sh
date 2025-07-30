#!/bin/bash

PROJECT_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"

xhost +local:docker

# Use THIS folder as context
docker build -t aprilslam_docker -f "$PROJECT_ROOT/docker/Dockerfile" "$PROJECT_ROOT"

docker run -it --rm \
  --net=host \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$PROJECT_ROOT:/catkin_ws/src/aprilslamcpp" \
  --name aprilslam_container \
  aprilslam_docker
