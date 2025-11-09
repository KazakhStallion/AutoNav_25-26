#!/bin/bash
set -e

BASE_REF="autonav:koopa-kingdom"
IMAGE_REF="dev:koopa-kingdom"

# Ensure base exists
if ! docker image inspect "$BASE_REF" >/dev/null 2>&1; then
  echo "Base $BASE_REF not found. Build it first with Dockerfile.base."
  exit 1
fi

# Rebuild only the dev layer
if docker image inspect "$IMAGE_REF" >/dev/null 2>&1; then
  echo "Image $IMAGE_REF exists. Removing it..."
  docker rmi "$IMAGE_REF"
fi

# Build the dev image
docker build -t "$IMAGE_REF" \
  -f "$HOME/AutoNav_25-26/env/docker/dockerfiles/Dockerfile" \
  "$HOME/AutoNav_25-26"
