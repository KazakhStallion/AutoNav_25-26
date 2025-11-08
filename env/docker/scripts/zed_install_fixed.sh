#!/bin/bash
set -euo pipefail

ZED_SDK_MAJOR=${1:-5}
ZED_SDK_MINOR=${2:-1}

# Detect L4T from base (don’t modify it)
if [[ -r /etc/nv_tegra_release ]]; then
  # Example: "# R36 (release), REVISION: 4.4, ..."
  L4T_MAJOR=$(sed -n 's/.*R\([0-9]\+\).*/\1/p' /etc/nv_tegra_release)
  L4T_MINOR=$(sed -n 's/.*REVISION: \([0-9.]\+\).*/\1/p' /etc/nv_tegra_release)
else
  echo "ERROR: /etc/nv_tegra_release not found; are you on an l4t-jetpack base?" >&2
  exit 1
fi

# Normalize minor to first two components (e.g., 4.4 from 4.4)
L4T_MINOR_BARE=${L4T_MINOR%% *}
L4T_MINOR_TRIM=${L4T_MINOR_BARE%%.*}.${L4T_MINOR_BARE#*.}

echo "===> Detected L4T: ${L4T_MAJOR}.${L4T_MINOR_TRIM} ; installing ZED SDK ${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}"

# Build the ZED SDK URL: Stereolabs uses l4t<MAJOR>.<MINOR> directory for Jetsons
ZED_URL="https://download.stereolabs.com/zedsdk/${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}/l4t${L4T_MAJOR}.${L4T_MINOR_TRIM}/jetsons"

# Download & install (Jetson uses system CUDA; keep skip_cuda)
wget -q --no-check-certificate -O /tmp/ZED_SDK_Linux_JP.run "${ZED_URL}"
chmod +x /tmp/ZED_SDK_Linux_JP.run
/tmp/ZED_SDK_Linux_JP.run -- silent skip_cuda
rm -rf /usr/local/zed/resources/* || true
rm -f /tmp/ZED_SDK_Linux_JP.run

echo "===> ZED SDK installed."