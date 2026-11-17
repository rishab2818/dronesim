$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$out = Join-Path $root 'web/src/wasm/generated'
New-Item -ItemType Directory -Force $out | Out-Null
$sources = Get-ChildItem (Join-Path $root 'engine/src/*.c') | ForEach-Object FullName
& emcc @sources (Join-Path $root 'engine/wasm/wasm_exports.c') `
  -I (Join-Path $root 'engine/include') -std=c11 -O2 -Wall -Wextra -Wpedantic `
  -s MODULARIZE=1 -s EXPORT_ES6=1 -s ENVIRONMENT=web `
  -s ALLOW_MEMORY_GROWTH=1 -s FILESYSTEM=0 `
  -s 'EXPORTED_RUNTIME_METHODS=["HEAPF64","UTF8ToString"]' `
  -s 'EXPORTED_FUNCTIONS=["_malloc","_free","_wasm_sim_create_default","_wasm_sim_destroy","_wasm_sim_last_error","_wasm_sim_reset","_wasm_sim_set_armed","_wasm_sim_set_mode","_wasm_sim_set_input","_wasm_sim_step","_wasm_sim_apply_basic_config","_wasm_sim_snapshot_length","_wasm_sim_get_snapshot","_wasm_sim_history_count","_wasm_sim_clear_history","_wasm_sim_history_row_length","_wasm_sim_get_history_row"]' `
  -o (Join-Path $out 'drone_sim.js')

