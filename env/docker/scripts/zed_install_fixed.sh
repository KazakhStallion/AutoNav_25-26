#!/bin/bash
set -euo pipefail
# Usage: zed_install.sh <ZED_SDK_MAJOR> <ZED_SDK_MINOR> <L4T_BASE>
# Example: zed_install.sh 5 1 36.4   # (JP 6.1.x → L4T 36.4)
ZED_SDK_MAJOR=${1:?Missing ZED_SDK_MAJOR}
ZED_SDK_MINOR=${2:?Missing ZED_SDK_MINOR}
L4T_BASE=${3:?Missing L4T_BASE (e.g., 36.4)}

ZED_URL="https://download.stereolabs.com/zedsdk/${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}/l4t${L4T_BASE}/jetsons"
echo "===> Installing ZED SDK ${ZED_SDK_MAJOR}.${ZED_SDK_MINOR} for L4T ${L4T_BASE}"
echo "===> Fetching ${ZED_URL}"

wget -q --no-check-certificate -O /tmp/ZED_SDK_Linux_JP.run "${ZED_URL}"
chmod +x /tmp/ZED_SDK_Linux_JP.run
/tmp/ZED_SDK_Linux_JP.run -- silent skip_cuda
rm -rf /usr/local/zed/resources/* || true
rm -f /tmp/ZED_SDK_Linux_JP.run

echo "===> Done."