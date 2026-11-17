# Trace one `W` input through the simulator

Assume the vehicle is armed at collective 0.43, level, with all motors near the same speed.

1. `inputMapper.ts` stores `KeyW` as `pitchForward`. It ignores key repeat and editable fields.
2. `App.tsx` reads the pressed set during the fixed tick and calls `WasmClient.setInput({pitch:1,...})`.
3. `wasm_sim_set_input` constructs `PilotInput`; `sim_set_input` validates and stores it.
4. `sim_step` calls `controls_mix_manual`. With pitch authority `0.16`, the targets are `[0.27,0.59,0.59,0.27]`: front M1/M4 slow and rear M2/M3 speed up.
5. `motors_advance` applies the 80 ms exact exponential lag. RPM does not jump.
6. `motors_compute_wrench` calculates each `kTω²`. Rear thrust is greater. For each rotor it adds `r×F`; the net moment has negative body X, meaning nose-forward for this frame convention.
7. `dynamics_calculate` applies `I⁻¹(M−ω×Iω)` to get body angular acceleration. It also rotates total body thrust by the current C quaternion before adding gravity.
8. `integrator_semi_implicit_euler` updates body rate, then `quat_integrate_body_rate` changes `q_body_to_world`. As the drone tilts, later thrust vectors acquire a +Y world component and the path bends forward. No position is written by the input code.
9. `ground_resolve` applies the simple plane constraint if needed; `Recorder` takes a 20 Hz truth sample.
10. `wasm_sim_get_snapshot` copies named quantities into 44 doubles. `wasmClient.ts` is the only code reading Wasm memory.
11. `DroneScene.tsx` changes basis from Z-up physics to Y-up Three.js and applies the C quaternion. The camera pixels change because the mesh pose changed; visual propeller spin never feeds back.

Useful breakpoints: `controls_mix_manual`, `motors_advance`, `motors_compute_wrench`, `dynamics_calculate`, `integrator_semi_implicit_euler`, `quat_integrate_body_rate`, `wasm_sim_get_snapshot`, and `WasmClient.snapshot`.

