#!/bin/bash

usage(){
    echo "Usage: $0 [-r <humble|iron|rolling>]"
    exit 1
}


XSOCK=/tmp/.X11-unix
XAUTH=$HOME/.Xauthority

xhost +local:docker
docker run \
    -it --rm \
    -v ${XSOCK}:${XSOCK} \
    -v ${XAUTH}:${XAUTH} \
    -e DISPLAY=${DISPLAY} \
    -e XAUTHORITY=${XAUTH} \
    --env=QT_X11_NO_MITSHM=1 \
    --privileged \
    --net=host \
    --gpus all \
    --name="sjtu_drone" \
    josedallatorre/sjtu_drone:latest 

xhost -local:docker
