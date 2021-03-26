#!/bin/bash

IMAGE_NAME=roomba_hack
CONTAINER_NAME="roomba_hack"
echo "$0: IMAGE=${IMAGE_NAME}"
echo "$0: CONTAINER=${CONTAINER_NAME}"

docker run -it --rm \
    --privileged \
    --env ROS_MASTER_URI=${ros_master_uri} \
    --env ROS_IP=${ros_ip} \
    --net host \
    --volume ${PWD}/catkin_ws/:/root/roomba_hack/catkin_ws/ \
    --volume /dev/:/dev/ \
    --name ${CONTAINER_NAME} \
    ${IMAGE_NAME} \
    bash
