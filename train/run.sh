#!/bin/bash

function message() {
    local color=$1; shift;
    local message=$@

    # Output command being executed
    echo -e "\e[${color}m${message}\e[0m"
}

DATASET_DIR=$(realpath $1); shift 1;
DATASET_NAME=$(basename $DATASET_DIR)
if [ -z "${DATASET_DIR}" ]; then
    echo "[ERROR]: DATASET_DIR should be set."
    exit 1
fi
echo "Target Dir: ${DATASET_DIR}"

if [ -t 1 ]; then
    TTY_OPT='-ti'
else
    TTY_OPT=''
fi


docker run --rm \
       -u "$(id -u $USER):$(id -g $USER)" \
       --userns=host \
       --gpus all \
       --shm-size=1g \
       --name $USER-train-pytorch-object-detection \
       --env="DISPLAY" \
       --env="QT_X11_NO_MITSHM=1" \
       --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
       --volume="$HOME/.project_t:/home/user/.project_t:rw" \
       --volume="${DATASET_DIR}:/workspace/target_data:rw" \
       ${TTY_OPT} train-object-detection-from-images 'python -- generate_data.py --from-images-dir /workspace/target_data --compress-annotation-data'

message 32 "Done generating model file for pytorch object detection"
message 32 " - ${DATASET_DIR}/generated_data/yolov7-seg-coco/weights/best.pt"
message 32 " - ${DATASET_DIR}/generated_data/from_images_dir.yaml"
