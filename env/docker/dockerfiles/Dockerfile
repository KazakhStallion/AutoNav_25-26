FROM autonav:koopa-kingdom

USER root
RUN apt-get update && apt-get install -y --no-install-recommends \
    gosu \
    bluez \
    bluez-tools \
    dbus \
    evtest \
    joystick \
    udev \
    nano \
    iproute2 \
 && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir pyserial

# Update rosdep cache
RUN rosdep update

# Create workspace directory
RUN mkdir -p /workspace

WORKDIR /workspace