#!/bin/bash

IMAGE_NAME=roomba_hack
CONTAINER_NAME="roomba_hack"
echo "$0: IMAGE=${IMAGE_NAME}"
echo "$0: CONTAINER=${CONTAINER_NAME}"

if [ ! -z $1 ]; then
    ROOMBA_IP=$1
else
    ROOMBA_IP=`hostname -I | cut -d' ' -f1`
fi

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
            --volume /tmp/argus_socket:/tmp/argus_socket \
            --name ${CONTAINER_NAME} \
            ${IMAGE_NAME}:jetson \
            bash -c "sed -i 's/TMP_IP/localhost/' ~/scripts/initialize-bash-shell.sh;
                     sed -i 's/noetic/melodic/' ~/.bashrc;
                     sed -i 's/noetic/melodic/' ~/scripts/initialize-bash-shell.sh;
                     source /root/.bashrc;
                     roslaunch roomba_bringup bringup.launch"
    else
        xhost +
        docker run -it --rm \
            --privileged \
            --gpus all \
            --env DISPLAY=${DISPLAY} \
            --net host \
            --volume ${PWD}/catkin_ws/:/root/roomba_hack/catkin_ws/ \
            --volume /dev/:/dev/ \
            --volume /tmp/.X11-unix:/tmp/.X11-unix \
            --name ${CONTAINER_NAME} \
            ${IMAGE_NAME} \
            bash -c "sed -i 's/TMP_IP/${ROOMBA_IP}/' ~/scripts/initialize-bash-shell.sh;
                     bash"
    fi
fi
