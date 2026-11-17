# Drone Physics Lab

A deterministic, educational 6-DOF quadcopter simulator. Portable C11 owns motor dynamics, mixing/control, forces, moments, rigid-body integration, ground contact, and flight recording. React, Three.js, and Recharts only capture input and present C-produced state.

## Quick start

Prerequisites are CMake 3.20+, a C11 compiler, Node/npm, and an activated Emscripten SDK. See [complete Windows and Unix instructions](docs/BUILD_AND_RUN.md).

```powershell
./scripts/build-native.ps1
./scripts/test.ps1
./scripts/build-wasm.ps1
cd web
npm install
npm test
npm run dev
```

Open the URL Vite prints. The cockpit starts in beginner-assisted mode: WASD requests smoothly acceleration-limited horizontal speed, Q/E requests limited yaw rate, and collective around hover requests a limited climb/descent rate. Releasing the controls requests a smooth stop. Training drag is enabled. At roughly 38.6% the default 1 kg vehicle balances weight after spool-up. T/G or Up/Down adjusts persistent collective without triggering Windows Sticky Keys; fine ±1%, coarse ±5%, Hover, and vertical-brake controls are also provided. Camera follow is enabled by default. Manual open-loop physics and drag-free motion remain available as advanced options.

## What is implemented

- Milestone 0: CMake/native/React scaffolds, units and coordinate conventions.
- Milestone 1: vector, matrix, quaternion, validated full inertia, exact motor lag, rotor wrench, 6-DOF Newton–Euler dynamics, fixed semi-implicit integration, simple ground/crash model, native CLI.
- Milestone 2: stable double-array Wasm ABI, isolated TypeScript memory adapter, fixed 500 Hz accumulator with overload indication.
- Milestone 3: C-quaternion-driven Three.js X-quad, labeled motors/rotors, orbit camera, grid, truth telemetry, configuration apply/reset.
- Milestone 4: manual mixer, robust keyboard state, throttle, on-screen controls, bounded C trail/history, charts, CSV export, presets and direct motor override.
- Milestone 5: native and frontend tests plus reverse-engineering documentation.
- Milestone 6: ideal beginner velocity assistance with dynamic acceleration/rate limits using perfect internal truth; no position/altitude hold and no simulated sensors.

## Verification record

On 2026-11-16, the local GCC build completed with `-std=c11 -Wall -Wextra -Wpedantic`; 40/40 native assertions passed. The deterministic repeat test is byte-equal on that build. Vitest passed 4/4 input tests. Emscripten 7.0.0 produced the Wasm module, and `tsc -b && vite build` succeeded with the Wasm asset in the production graph. Tolerances range from `1e−12` for pure math identities to `1e−6 rad/s` for motor convergence; scenario comparisons should use physical tolerances across different platforms.

The native CLI prints a physics-generated lift, pitch, and descent CSV trace. Emscripten output is generated, never hand-edited, under `web/src/wasm/generated`.

## Assumptions

Rotor coefficients are illustrative; throttle maps linearly to target shaft speed; thrust maps quadratically to speed; the body is rigid; rotor gyroscopic effects are omitted; optional drag is isotropic; the ground is a simplified flat constraint. See [physics derivations](docs/PHYSICS.md) and [limitations](docs/LIMITATIONS.md).

## Start reading here

1. [`engine/include/sim_types.h`](engine/include/sim_types.h) — state, units, configuration, and public vocabulary.
2. [`engine/src/sim_api.c`](engine/src/sim_api.c) — exact order of one physics step.
3. [`engine/src/controls.c`](engine/src/controls.c) — keyboard intent to four motor targets.
4. [`engine/src/motors.c`](engine/src/motors.c) — RPM lag, thrust, and moments.
5. [`engine/src/dynamics.c`](engine/src/dynamics.c) — Newton–Euler equations before integration.

Then follow [one W key through every layer](docs/TRACE_ONE_FRAME.md) and use the [12-lesson course](docs/LEARNING_PATH.md).

