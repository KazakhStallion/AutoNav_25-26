#!/bin/bash
set -e  # exit on error

# params
IMAGE_TAG="dev:koopa-kingdom"
CONTAINER_NAME="koopa-kingdom"

# HOST paths
WORKDIR="$HOME/AutoNav_25-26"
SCRIPT_DIR="$(dirname ${BASH_SOURCE[0]})"

# NVIDIA entrypoint inside container
ENTRYPOINT="/usr/local/bin/scripts/entrypoint.sh"

# The username NVIDIA's entrypoint should create/map to our host user
USERNAME="${USER}"

# start freshhhh
docker rm -f "$CONTAINER_NAME" >/dev/null 2>&1 || true

DOCKER_ARGS=()

# ---------- env expected by NVIDIA entrypoint ----------
DOCKER_ARGS+=("-e USERNAME=${USERNAME}")
DOCKER_ARGS+=("-e HOST_USER_UID=$(id -u)")
DOCKER_ARGS+=("-e HOST_USER_GID=$(id -g)")

# misc env
DOCKER_ARGS+=("-e ROS_DOMAIN_ID")
DOCKER_ARGS+=("-e WORKDIR=$WORKDIR")

# GUI / DBus / Bluetooth
DOCKER_ARGS+=("-v /tmp/.X11-unix:/tmp/.X11-unix")
DOCKER_ARGS+=("-e DISPLAY")
# mount host Xauthority into the user home the entrypoint will create
DOCKER_ARGS+=("-v $HOME/.Xauthority:/home/${USERNAME}/.Xauthority:rw")
DOCKER_ARGS+=("-v /var/run/dbus:/var/run/dbus")
DOCKER_ARGS+=("-v /sys/class/bluetooth:/sys/class/bluetooth")

# SSH agent (optional)
if [[ -n "$SSH_AUTH_SOCK" ]]; then
  DOCKER_ARGS+=("-v $SSH_AUTH_SOCK:/ssh-agent")
  DOCKER_ARGS+=("-e SSH_AUTH_SOCK=/ssh-agent")
fi

# Jetson / NVIDIA runtime & device libs
DOCKER_ARGS+=("--runtime" "nvidia")
DOCKER_ARGS+=("-e" "NVIDIA_VISIBLE_DEVICES=all")
DOCKER_ARGS+=("-e" "NVIDIA_DRIVER_CAPABILITIES=all")

# Only add Jetson-specific mounts when running on aarch64
if [[ "$(uname -m)" == "aarch64" ]]; then
  DOCKER_ARGS+=("-v /usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra")
  DOCKER_ARGS+=("-v /usr/src/jetson_multimedia_api:/usr/src/jetson_multimedia_api")
  [ -d /usr/share/vpi3 ] && DOCKER_ARGS+=("-v /usr/share/vpi3:/usr/share/vpi3")
  DOCKER_ARGS+=("--pid=host")
  DOCKER_ARGS+=("-v /dev:/dev")
  if [ -S /run/jtop.sock ]; then
      DOCKER_ARGS+=("-v /run/jtop.sock:/run/jtop.sock:ro")
  fi
fi

# project mounts & entrypoint
DOCKER_ARGS+=("-v $WORKDIR:/autonav")
DOCKER_ARGS+=("-v /etc/localtime:/etc/localtime:ro")            # time sync
DOCKER_ARGS+=("-v $SCRIPT_DIR/entrypoint_additions:/usr/local/bin/scripts/entrypoint_additions")  # entrypoint additions
DOCKER_ARGS+=("-v $SCRIPT_DIR/entrypoint.sh:$ENTRYPOINT")       # NVIDIA entrypoint
DOCKER_ARGS+=("--entrypoint" "$ENTRYPOINT")

# Workdir inside container is your workspace
DOCKER_ARGS+=("--workdir" "/autonav/isaac_ros-dev")

# fast path: reuse running container
if docker ps --format '{{.Names}}' | grep -qx "$CONTAINER_NAME"; then
  # attach as the mapped user
  docker exec -it -u "$USERNAME" -w "/autonav/isaac_ros-dev" "$CONTAINER_NAME" /bin/bash
  exit 0
fi

# After the NVIDIA entrypoint creates $USERNAME and gosu's, this runs:
LAUNCH='bash -lc "
  source /opt/ros/humble/setup.bash;
  cd /autonav/isaac_ros-dev;
  if [ ! -d install ]; then
    echo \"[container] First build of overlay...\" ;
    colcon build --symlink-install ;
  fi ;
  source install/setup.bash ;
  exec bash
"'

# ---------- run ----------
docker run -it --rm \
  --privileged \
  --network host \
  --ipc=host \
  --name "$CONTAINER_NAME" \
  "${DOCKER_ARGS[@]}" \
  "$IMAGE_TAG" \
  /bin/bash -c "$LAUNCH"