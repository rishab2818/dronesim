# C ↔ WebAssembly interface

`engine/wasm/wasm_exports.c` is marshaling only. Emscripten exports an opaque `SimHandle*` and small wrapper functions. React never sees C structs and never performs physics.

Snapshots are copied into a caller-allocated array of 44 IEEE-754 doubles. Indices 0–39 currently contain: time, step, position (3), velocity (3), quaternion wxyz (4), body rates (3), acceleration (3), world force (3), body moment (3), RPM (4), thrust (4), normalized motor targets (4), collective, status, mode, and armed. Indices 40–43 are reserved and zero. `wasm_sim_snapshot_length()` is an ABI check.

History rows contain 27 doubles: time, position (3), quaternion (4), velocity (3), body rates (3), RPM (4), thrust (4), motor commands (4), status. This array ABI deliberately avoids assumptions about compiler padding/alignment. All allocation and `HEAPF64` access is isolated in `web/src/wasm/wasmClient.ts`.

Emscripten emits `drone_sim.js` and `drone_sim.wasm` into `web/src/wasm/generated`. Vite processes the ES module and copies its `.wasm` dependency into the production asset graph. Opening `index.html` via `file://` will not work.

