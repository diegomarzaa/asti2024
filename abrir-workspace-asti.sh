#!/bin/bash

xhost +local:root   # Dar permisos para que se puedan visualizar ventanas como el simulador de Gazebo

sudo docker run -it --rm \      # -it: Para que puedas usar el contenedor directamente desde la terminal
                                # --rm: Para no dejar contenedores abiertos que ocupen memoria
    --name asti_container \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$(pwd)/retos_ws:/root/retos_ws" \   # Montamos la carpeta `retos_ws` en tu máquina al contenedor, para que los cambios que hagas en el código dentro del contenedor se guarden en tu máquina
    asti2024