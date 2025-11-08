#!/bin/bash
set -euo pipefail

ZED_SDK_MAJOR=${1:-5}
ZED_SDK_MINOR=${2:-1}

# Detect L4T in the container (e.g., 36.4.0). If unavailable, fail.
[[ -r /etc/nv_tegra_release ]] || { echo "No /etc/nv_tegra_release"; exit 1; }

# Extract "36" and "4" from something like "R36 ... REVISION: 4.4"
L4T_MAJOR=$(sed -n 's/.*R\([0-9]\+\).*/\1/p' /etc/nv_tegra_release)
REV=$(sed -n 's/.*REVISION:[[:space:]]*\([0-9.]\+\).*/\1/p' /etc/nv_tegra_release)
L4T_MINOR_BASE=${REV%%.*}   # "4" from "4.0" or "4.4" etc.

echo "===> Detected L4T: ${L4T_MAJOR}.${REV} (using base ${L4T_MAJOR}.${L4T_MINOR_BASE})"

# Stereolabs URLs are l4t<MAJOR>.<MINOR> (no patch)
ZED_URL="https://download.stereolabs.com/zedsdk/${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}/l4t${L4T_MAJOR}.${L4T_MINOR_BASE}/jetsons"

wget -q --no-check-certificate -O /tmp/ZED_SDK_Linux_JP.run "${ZED_URL}"
chmod +x /tmp/ZED_SDK_Linux_JP.run
/tmp/ZED_SDK_Linux_JP.run -- silent skip_cuda
rm -rf /usr/local/zed/resources/* || true
rm -f /tmp/ZED_SDK_Linux_JP.run

echo "===> ZED SDK ${ZED_SDK_MAJOR}.${ZED_SDK_MINOR} installed for L4T ${L4T_MAJOR}.${L4T_MINOR_BASE}"
