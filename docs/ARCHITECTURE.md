# Architecture

```text
keyboard / buttons
        ↓
PressedControls + PilotCommand (TypeScript, no dynamics)
        ↓ one compact command per fixed tick
sim_set_input → controls/controller → four bounded motor targets
        ↓
motors_advance → omega; motors_compute_wrench → thrust + r×F + reaction torque
        ↓
dynamics_calculate → world force + body angular acceleration
        ↓
integrator_semi_implicit_euler → velocity, position, body rate, quaternion
        ↓
ground_resolve → contact / takeoff / crash
        ↓
Recorder + SimSnapshot → Wasm double arrays
        ↓
Zustand → Three.js pose/trail + truth telemetry + Recharts
```

## One 2 ms step

`sim_step` in `engine/src/sim_api.c` is the orchestration point. It (1) mixes the current input, (2) advances the exact first-order motor solution, (3) sums the rotor wrench, (4) calls the 6-DOF dynamics, (5) integrates velocities before pose, (6) resolves ground contact, (7) increments the integer step counter, and (8) records at 20 Hz. No allocation occurs in this path.

The native CLI links the `drone_engine` static library and calls the public `sim_api.h`. The browser compiles those exact sources plus `wasm_exports.c`. `wasmClient.ts` owns pointers and copies; `simulationLoop.ts` owns the render-time accumulator; components only present state.

## Module map

| Module | Responsibility |
|---|---|
| `vector3`, `matrix3`, `quaternion` | drone-independent numerical primitives |
| `drone_config` | defaults, units, validation, hover calculation |
| `controls`, `controller` | direct mixer and optional truth-feedback assistance |
| `motors` | actuator lag, thrust, arm and reaction moments |
| `dynamics` | Newton–Euler force and acceleration equations |
| `integrator` | fixed-step state advancement |
| `ground` | flat-plane constraint and crash classification |
| `recorder` | bounded 20 Hz ground-truth history |
| `sim_api` | ownership, sequencing, errors, public interface |

