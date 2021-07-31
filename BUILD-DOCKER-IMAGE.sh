#!/bin/bash

IMAGE_NAME=roomba_hack
TAG_NAME=latest
DOCKERFILE_NAME=Dockerfile

dpkg -s nvidia-container-runtime > /dev/null 2>&1
if [ ! $? -eq 0 ];then
    DOCKERFILE_NAME=Dockerfile.cpu
    TAG_NAME=cpu
elif [ "$(uname -m)" == "aarch64" ]; then
    echo "Build docker image for jetson"
    DOCKERFILE_NAME=Dockerfile.jetson
    TAG_NAME=jetson
fi

docker build . -f docker/${DOCKERFILE_NAME} -t ${IMAGE_NAME}:${TAG_NAME}
