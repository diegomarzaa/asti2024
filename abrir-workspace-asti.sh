#!/bin/bash

xhost +local:root   # Dar permisos para que se puedan visualizar ventanas como el simulador de Gazebo

# -it: Para que puedas usar el contenedor directamente desde la terminal    # --rm: Para no dejar contenedores abiertos que ocupen memoria
# Montamos la carpeta `asti2024_ws` en tu máquina al contenedor, para que los cambios que hagas en el código dentro del contenedor se guarden en tu máquina

sudo docker run -it --rm \
    --name asti_container \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$(pwd)/asti2024_ws:/root/asti2024_ws" \
    asti2024