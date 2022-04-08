#!/bin/bash

IMAGE_NAME=roomba_hack
TAG_NAME=latest
BASE_IMAGE=nvidia/cudagl:11.1.1-devel-ubuntu20.04
DOCKERFILE_NAME=Dockerfile

dpkg -s nvidia-container-runtime > /dev/null 2>&1
if [ ! $? -eq 0 ];then
    BASE_IMAGE=20.04
elif [ "$(uname -m)" == "aarch64" ]; then
    echo "Build docker image for jetson"
    DOCKERFILE_NAME=Dockerfile.jetson
    TAG_NAME=jetson
    BASE_IMAGE=nvcr.io/nvidia/l4t-base:r32.6.1
fi

docker build . -f docker/${DOCKERFILE_NAME} -t ${IMAGE_NAME}:${TAG_NAME} --build-arg BASE_IMAGE=${BASE_IMAGE}
