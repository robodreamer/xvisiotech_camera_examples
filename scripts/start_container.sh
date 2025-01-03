#!/bin/bash

docker build \
  --file .devcontainer/Dockerfile \
  --build-arg USER_UID=$(id -u) \
  --build-arg USER_GID=$(id -g) \
  -t xvisio-cam-dev .
docker run --privileged -it --rm \
		--mount type=bind,source=$(pwd),target=/workspaces/xvisio-cam -w /workspaces/xvisio-cam \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    xvisio-cam-dev \
    /bin/bash