#!/bin/bash

xhost +local:root

sudo docker run -it --rm \
    --network host \
    --name asti_container \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    asti2024
