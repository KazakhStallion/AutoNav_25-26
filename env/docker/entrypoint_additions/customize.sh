#!/bin/bash

BASHRC="/home/admin/.bashrc"

echo "PS1='${debian_chroot:+($debian_chroot)}\[\033[01;31m\]bowser\[\033[01;33m\]@\[\033[01;32m\]koopa-kingdom\[\033[00m\]:\[\033[01;33m\]\w\[\033[00m\]\$ '" >> $BASHRC
echo "source /opt/ros/humble/setup.bash" >> $BASHRC
echo "source /autonav/isaac_ros-dev/install/setup.bash" >> $BASHRC

