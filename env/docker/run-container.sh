#!/bin/bash

set -e # makes script exit on command failure

# ===== PARAMETERS =====
IMAGE_TAG="dev:koopa-kingdom"
CONTAINER_NAME="koopa-kingdom"
HOST_WORKDIR="$HOME/AutoNav_25-26"
CONTAINER_WORKDIR="/autonav"
ENTRYPOINT="/usr/local/bin/scripts/entrypoint.sh"
SCRIPT_DIR="$(dirname ${BASH_SOURCE[0]})"

# ===== DETECT PLATFORM =====
PLATFORM=$(uname -m)

# ===== USERNAME =====
USERNAME="${USERNAME:-admin}"

DOCKER_ARGS=()

# ===== ENVIRONMENT VARIABLES =====
DOCKER_ARGS+=("-e ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}")
DOCKER_ARGS+=("-e USER=${USERNAME}")
DOCKER_ARGS+=("-e USERNAME=${USERNAME}")
DOCKER_ARGS+=("-e HOST_USER_UID=$(id -u)")
DOCKER_ARGS+=("-e HOST_USER_GID=$(id -g)")
DOCKER_ARGS+=("-e WORKDIR=${CONTAINER_WORKDIR}")

# ===== D-BUS & BLUETOOTH =====
DOCKER_ARGS+=("-v /var/run/dbus:/var/run/dbus")
DOCKER_ARGS+=("-v /sys/class/bluetooth:/sys/class/bluetooth")

# ===== DISPLAY FORWARDING =====
DOCKER_ARGS+=("-v /tmp/.X11-unix:/tmp/.X11-unix")
DOCKER_ARGS+=("-v $HOME/.Xauthority:/home/${USERNAME}/.Xauthority:rw")
DOCKER_ARGS+=("-e DISPLAY")

# ===== SSH AGENT =====
if [[ -n $SSH_AUTH_SOCK ]]; then
    DOCKER_ARGS+=("-v $SSH_AUTH_SOCK:/ssh-agent")
    DOCKER_ARGS+=("-e SSH_AUTH_SOCK=/ssh-agent")
fi

# ===== JETSON SPECIFIC =====
if [[ $PLATFORM == "aarch64" ]]; then
    echo "Detected Jetson platform (aarch64)"
    # Jetson-specific mounts and devices
    DOCKER_ARGS+=("-v /usr/bin/tegrastats:/usr/bin/tegrastats")
    DOCKER_ARGS+=("-v /tmp/:/tmp/")
    DOCKER_ARGS+=("--pid=host")
    
    # jtop support
    if [[ $(getent group jtop) ]]; then
        DOCKER_ARGS+=("-v /run/jtop.sock:/run/jtop.sock:ro")
    fi
fi

# ===== MOUNTS & WORKING DIRECTORY =====
DOCKER_ARGS+=("-v ${HOST_WORKDIR}:${CONTAINER_WORKDIR}")
DOCKER_ARGS+=("-v /etc/localtime:/etc/localtime:ro")
DOCKER_ARGS+=("--workdir ${CONTAINER_WORKDIR}/isaac_ros-dev")
DOCKER_ARGS+=("-v $SCRIPT_DIR/entrypoint_additions:/usr/local/bin/scripts/entrypoint_additions")
DOCKER_ARGS+=("-v $SCRIPT_DIR/entrypoint.sh:/usr/local/bin/scripts/entrypoint.sh")

# ===== PERSISTENT BUILD VOLUMES =====
# These volumes persist the build artifacts across container runs
DOCKER_ARGS+=("-v ${CONTAINER_NAME}-build:${CONTAINER_WORKDIR}/isaac_ros-dev/build")
DOCKER_ARGS+=("-v ${CONTAINER_NAME}-install:${CONTAINER_WORKDIR}/isaac_ros-dev/install")
DOCKER_ARGS+=("-v ${CONTAINER_NAME}-log:${CONTAINER_WORKDIR}/isaac_ros-dev/log")

# ZED settings/resources
if [[ -d "$HOME/zed/settings" ]]; then
    DOCKER_ARGS+=("-v $HOME/zed/settings:/usr/local/zed/settings")
fi
if [[ -d "$HOME/zed/resources" ]]; then
    DOCKER_ARGS+=("-v $HOME/zed/resources:/usr/local/zed/resources")
fi

DOCKER_ARGS+=("--entrypoint $ENTRYPOINT")

# GPU env
DOCKER_ARGS+=("-e NVIDIA_VISIBLE_DEVICES=all")
DOCKER_ARGS+=("-e NVIDIA_DRIVER_CAPABILITIES=all")

# Ensure container user can open /dev/nvhost* and friends
VID_GID=$(getent group video  | cut -d: -f3)
REN_GID=$(getent group render | cut -d: -f3)
INPUT_GID=$(getent group input | cut -d: -f3)
if [[ -n "$VID_GID" ]]; then DOCKER_ARGS+=("--group-add $VID_GID"); fi
if [[ -n "$REN_GID" ]]; then DOCKER_ARGS+=("--group-add $REN_GID"); fi
if [[ -n "$INPUT_GID" ]]; then DOCKER_ARGS+=("--group-add $INPUT_GID"); fi

# ===== RE-USE EXISTING CONTAINER =====
if [ "$(docker ps -a --quiet --filter status=running --filter name=^/${CONTAINER_NAME}$)" ]; then
    echo "Container $CONTAINER_NAME is already running. Attaching..."
    docker exec -i -t -u ${USERNAME} --workdir "${CONTAINER_WORKDIR}/isaac_ros-dev" $CONTAINER_NAME /bin/bash "$@"
    exit 0
fi

# Check if container exists but is stopped
if [ "$(docker ps -a --quiet --filter status=exited --filter name=^/${CONTAINER_NAME}$)" ]; then
    echo "Container $CONTAINER_NAME exists but is stopped. Starting and attaching..."
    docker start $CONTAINER_NAME
    docker exec -i -t -u ${USERNAME} --workdir "${CONTAINER_WORKDIR}/isaac_ros-dev" $CONTAINER_NAME /bin/bash "$@"
    exit 0
fi

# ===== CREATE NEW CONTAINER =====
echo "Starting new container: $CONTAINER_NAME"
echo "Mounting: ${HOST_WORKDIR} → ${CONTAINER_WORKDIR}"

docker run -it \
    --runtime nvidia \
    --gpus all \
    --privileged \
    --network host \
    --ipc host \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    --device /dev/bus/usb:/dev/bus/usb \
    ${DOCKER_ARGS[@]} \
    --name "$CONTAINER_NAME" \
    $IMAGE_TAG \
    /bin/bash