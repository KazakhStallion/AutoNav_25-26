#!/bin/bash
set -e

JETPACK_MAJOR=$1
JETPACK_MINOR=$2
L4T_MAJOR=$3
L4T_MINOR=$4
ZED_SDK_MAJOR=$5
ZED_SDK_MINOR=$6

ttk="===>"

echo "Europe/Paris" > /etc/timezone
echo "# R${L4T_MAJOR} (release), REVISION: ${L4T_MINOR}" > /etc/nv_tegra_release
    
# Install ZED SDK
echo "${ttk} Installing ZED SDK v${ZED_SDK_MAJOR}.${ZED_SDK_MINOR} for Jetpack ${JETPACK_MAJOR}.${JETPACK_MINOR} (L4T v${L4T_MAJOR}.${L4T_MINOR})"

# Download and install ZED SDK for Jetson
wget -q --no-check-certificate -O /tmp/ZED_SDK_Linux_JP.run \
    https://download.stereolabs.com/zedsdk/${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}/l4t${L4T_MAJOR}.${L4T_MINOR}/jetsons && \
chmod +x /tmp/ZED_SDK_Linux_JP.run && \
/tmp/ZED_SDK_Linux_JP.run -- silent skip_cuda && \
rm -rf /usr/local/zed/resources/* && \
rm -f /tmp/ZED_SDK_Linux_JP.run

echo "${ttk} ZED SDK installed successfully"