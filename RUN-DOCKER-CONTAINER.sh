#!/bin/bash

IMAGE_NAME=roomba_hack
CONTAINER_NAME="roomba_hack"
echo "$0: IMAGE=${IMAGE_NAME}"
echo "$0: CONTAINER=${CONTAINER_NAME}"

ROOMBA_IP="192.168.10.70"

EXISTING_CONTAINER_ID=`docker ps -aq -f name=${CONTAINER_NAME}`
if [ ! -z "${EXISTING_CONTAINER_ID}" ]; then
    docker exec -it ${CONTAINER_NAME} bash
else
    if [ "$(uname -m)" == "aarch64" ]; then
        docker run -it --rm \
            --privileged \
            --runtime nvidia \
            --net host \
            --volume ${PWD}/catkin_ws/:/root/roomba_hack/catkin_ws/ \
            --volume /dev/:/dev/ \
            --name ${CONTAINER_NAME} \
            ${IMAGE_NAME}:jetson \
            bash -c "sed -i 's/TMP_IP/${ROOMBA_IP}/' ~/scripts/initialize-bash-shell.sh; bash"
    else
        docker run -it --rm \
            --privileged \
            --gpus all \
            --env DISPLAY=${DISPLAY} \
            --net host \
            --volume ${PWD}/catkin_ws/:/root/roomba_hack/catkin_ws/ \
            --volume /dev/:/dev/ \
            --volume /tmp/.X11-unix:/tmp/.X11-unix \
            --name ${CONTAINER_NAME} \
            ${IMAGE_NAME}:latest \
            bash -c "sed -i 's/TMP_IP/${ROOMBA_IP}/' ~/scripts/initialize-bash-shell.sh;
            bash"
    fi
fi
