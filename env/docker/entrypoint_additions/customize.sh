#!/bin/bash

BASHRC="/home/admin/.bashrc"

add_once() { grep -qxF "$1" "$BASHRC" || echo "$1" >> "$BASHRC"; }

add_once "PS1='${debian_chroot:+($debian_chroot)}\[\033[01;31m\]bowser\[\033[01;33m\]@\[\033[01;32m\]koopa-kingdom\[\033[00m\]:\[\033[01;33m\]\w\[\033[00m\]\$ '"
add_once "[ -f /opt/ros/humble/setup.bash] && source /opt/ros/humble/setup.bash"
add_once "[ -f /autonav/isaac_ros-dev/install/setup.bash ] && source /autonav/isaac_ros-dev/install/setup.bash"