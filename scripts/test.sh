#!/usr/bin/env sh
set -eu
ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
ctest --test-dir "$ROOT/build/native" --output-on-failure

