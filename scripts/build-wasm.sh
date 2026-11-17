#!/usr/bin/env sh
set -eu
ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
mkdir -p "$ROOT/web/src/wasm/generated"
emcc "$ROOT"/engine/src/*.c "$ROOT/engine/wasm/wasm_exports.c" -I"$ROOT/engine/include" \
  -std=c11 -O2 -Wall -Wextra -Wpedantic -sMODULARIZE=1 -sEXPORT_ES6=1 -sENVIRONMENT=web \
  -sALLOW_MEMORY_GROWTH=1 -sFILESYSTEM=0 -sEXPORTED_RUNTIME_METHODS='["HEAPF64","UTF8ToString"]' \
  -sEXPORTED_FUNCTIONS='["_malloc","_free","_wasm_sim_create_default","_wasm_sim_destroy","_wasm_sim_last_error","_wasm_sim_reset","_wasm_sim_set_armed","_wasm_sim_set_mode","_wasm_sim_set_input","_wasm_sim_step","_wasm_sim_apply_basic_config","_wasm_sim_snapshot_length","_wasm_sim_get_snapshot","_wasm_sim_history_count","_wasm_sim_clear_history","_wasm_sim_history_row_length","_wasm_sim_get_history_row"]' \
  -o "$ROOT/web/src/wasm/generated/drone_sim.js"
