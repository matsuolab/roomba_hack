#!/bin/bash

IMAGE_NAME=roomba_hack
CONTAINER_NAME="roomba_hack"
echo "$0: IMAGE=${IMAGE_NAME}"
echo "$0: CONTAINER=${CONTAINER_NAME}"

if [ "$(uname -m)" == "aarch64" ]; then
    docker run -it --rm \
        --privileged \
        --runtime nvidia \
        --env ROS_MASTER_URI=${ros_master_uri} \
        --env ROS_IP=${ros_ip} \
        --net host \
        --volume ${PWD}/catkin_ws/:/root/roomba_hack/catkin_ws/ \
        --volume /dev/:/dev/ \
        --name ${CONTAINER_NAME} \
        ${IMAGE_NAME}:jetson \
        bash
else
    xhost +
    docker run -it --rm \
        --privileged \
        --gpus all \
        --env DISPLAY=${DISPLAY} \
        --env ROS_MASTER_URI=${ros_master_uri} \
        --env ROS_IP=${ros_ip} \
        --net host \
        --volume ${PWD}/catkin_ws/:/root/roomba_hack/catkin_ws/ \
        --volume /dev/:/dev/ \
        --volume /tmp/.X11-unix:/tmp/.X11-unix \
        --name ${CONTAINER_NAME} \
        ${IMAGE_NAME} \
        bash
fi