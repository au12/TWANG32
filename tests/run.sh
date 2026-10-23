#!/usr/bin/env bash
# Builds the portable core natively and runs the test suite.
# Usage: tests/run.sh [output-dir-for-wav-files]
set -euo pipefail
cd "$(dirname "$0")/.."
OUT="${1:-build/test-audio}"
mkdir -p build "$OUT"
CORE=TWANG32/src/core
g++ -std=c++17 -O1 -g -Wall -Wextra -Wno-unused-parameter \
  -fsanitize=address,undefined -fno-omit-frame-pointer \
  tests/test_main.cpp $CORE/tw_json.cpp $CORE/tw_level.cpp $CORE/tw_game.cpp $CORE/tw_gen.cpp \
  $CORE/tw_synth.cpp $CORE/tw_music.cpp $CORE/tw_sfx.cpp \
  -o build/twang_tests
./build/twang_tests "$OUT"

# wireless link protocol + input processing
g++ -std=c++17 -O1 -g -Wall -Wextra -Wno-unused-parameter -fsanitize=address,undefined \
  tests/test_link.cpp $CORE/tw_json.cpp $CORE/tw_level.cpp $CORE/tw_game.cpp $CORE/tw_gen.cpp \
  -o build/test_link
./build/test_link

# the controller sketch carries copies of the shared headers: they must match
for f in twang_link.h twang_imu.h; do
  if ! cmp -s TWANG32/src/link/$f controller/TWANGController/$f; then
    echo "controller/TWANGController/$f differs from TWANG32/src/link/$f: run tools/sync-link.sh"
    exit 1
  fi
done
