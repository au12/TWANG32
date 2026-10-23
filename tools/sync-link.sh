#!/usr/bin/env bash
# Copies the shared link/IMU headers from the main firmware into the controller sketch.
# (Arduino sketches can't include files from outside their own folder.)
set -euo pipefail
cd "$(dirname "$0")/.."
for f in twang_link.h twang_imu.h; do
  cp TWANG32/src/link/$f controller/TWANGController/$f
done
echo "controller headers synced"
