#!/bin/bash

xhost +local:root   # Dar permisos para que se puedan visualizar ventanas como el simulador de Gazebo

# -it: Para que puedas usar el contenedor directamente desde la terminal    # --rm: Para no dejar contenedores abiertos que ocupen memoria
# Montamos la carpeta `asti2024_ws` en tu máquina al contenedor, para que los cambios que hagas en el código dentro del contenedor se guarden en tu máquina
# --network host: Para que el contenedor pueda acceder a la red de tu máquina

sudo docker run -it --rm \
    --network host \
    --name asti_container \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$(pwd)/asti2024_ws:/root/asti2024_ws" \
    asti2024

# Agregar --privileged si da errores de permisos
# Agregar --gpus all si se quiere usar la GPU para simulaciones
# Agregar --device=/dev/ttyUSB0:/dev/ttyUSB0 si se quiere usar un puerto USB para testear motores directamente