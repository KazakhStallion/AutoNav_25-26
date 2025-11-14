FROM autonav:koopa-kingdom

USER root
RUN apt-get update && apt-get install -y --no-install-recommends \
    gosu \
    bluez \
    bluez-tools \
    bluez-utils \
    bluetoothctl \
    dbus \
    evtest \
    joystick \
    udev \
    nano \
    iproute2 \
 && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir pyserial

RUN rm -rf /autonav/isaac_ros-dev/install \
           /autonav/isaac_ros-dev/build \
           /autonav/isaac_ros-dev/log

# Create workspace directory
RUN mkdir -p /workspace

WORKDIR /workspace