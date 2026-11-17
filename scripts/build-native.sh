#!/usr/bin/env sh
set -eu
ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
cmake -S "$ROOT" -B "$ROOT/build/native" -DCMAKE_BUILD_TYPE=Release
cmake --build "$ROOT/build/native"

