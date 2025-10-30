#!/bin/bash

IMAGE_NAME="dev"
IMAGE_TAG="koopa-kingdom"

ROOT="$(dirname ${BASH_SOURCE[0]})"

# Check if the image exists
if docker image inspect "${IMAGE_NAME}:${IMAGE_TAG}" > /dev/null 2>&1; then
    echo "Image ${IMAGE_NAME}:${IMAGE_TAG} exists. Removing it..."
    docker rmi "${IMAGE_NAME}:${IMAGE_TAG}"
fi


docker build -t "${IMAGE_NAME}:${IMAGE_TAG}" "$HOME/AutoNav/env/docker/dockerfiles" 



