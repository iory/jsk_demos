#!/bin/bash

xhost +local:root
docker stop "train-object-detection-from-images"
docker rm "train-object-detection-from-images"
docker run --rm \
       --gpus all \
       --shm-size=1g \
       --name "train-object-detection-from-images" \
       --env="DISPLAY" \
       --env="QT_X11_NO_MITSHM=1" \
       --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
       --volume="$(pwd):/workspace:rw" \
       -it train-object-detection-from-images /bin/bash
xhost +local:docker
