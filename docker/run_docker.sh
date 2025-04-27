#!/bin/bash

xhost +local:docker || true
ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"


if [[ $1 = "--nvidia" ]] || [[ $1 = "-n" ]]
  then
    docker run --gpus all \
                -it --rm \
                --name robot \
                -e DISPLAY \
                -e QT_X11_NO_MITSHM=1 \
                -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
                -v $XAUTHORITY:$XAUTHORITY \
                -e XAUTHORITY \
                --net=host \
                --privileged \
                mobile_robot

else
    echo "[!] If you wanna use nvidia gpu, please use script with -n or --nvidia argument"
    docker run  -it --rm \
                --name robot \
                -e DISPLAY \
                -e QT_X11_NO_MITSHM=1 \
                -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
                -v $XAUTHORITY:$XAUTHORITY \
                -e XAUTHORITY \
                --net=host \
                --privileged \
                mobile_robot
fi

!/bin/bash

