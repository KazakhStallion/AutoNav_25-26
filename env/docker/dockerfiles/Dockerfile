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
 && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir pyserial