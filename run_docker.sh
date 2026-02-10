#!/bin/bash

# Nazwa obrazu
IMAGE_NAME="ros2_gui"

# 1. Zezwól Dockerowi na dostęp do X11 (ekranu)
xhost +local:docker > /dev/null

# 2. Uruchom kontener
docker run --gpus all -it \
    --name ros2_humble_container \
    --rm \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --network host \
    $IMAGE_NAME

# 3. Po zakończeniu (wyjściu z kontenera) cofnij uprawnienia xhost
xhost -local:docker > /dev/null
