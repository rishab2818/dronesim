# Twelve-lesson learning path

Work in order. Each experiment is intentionally small enough to undo with Git or by restoring the named constant.

## 1. Frames, units, and vectors

**Idea.** A vector is three signed numbers whose meaning depends on a named frame. Read `docs/COORDINATE_FRAMES.md`, then `vector3.h` and `vector3.c` (`vec3_add`, `vec3_cross`). **Prerequisite.** Multiplication and the Pythagorean theorem; vector length is `sqrt(x²+y²+z²)`. **Example.** `(3,4,0) m` has length `5 m`; `(1,0,0)×(0,1,0)=(0,0,1)`. **Experiment.** Reverse the operands in the cross-product test; expect `−Z`.

Check: (1) Does `(1,2,3)` alone say world or body? **No; the variable/frame name must.** (2) Unit of velocity? **m/s.** (3) Why right-handed? **It fixes rotation and cross-product signs consistently.**

## 2. Mass, weight, and Newton's law

**Idea.** Force changes velocity; mass resists that change. Read `dynamics.c` (`dynamics_calculate`) and `drone_config.c` defaults. **Prerequisite.** Division and signed axes. **Example.** A 1 kg body weighs `9.80665 N` downward; with no thrust, `a=F/m=−9.80665 m/s²`. **Experiment.** Change gravity to `1.62`; free-fall acceleration should match the Moon-like value.

Check: (1) Are kg and N interchangeable? **No.** (2) Why is weight negative Z? **Up is +Z.** (3) If mass doubles at fixed force? **Acceleration halves.**

## 3. RPM, angular speed, and quadratic thrust

**Idea.** Propeller thrust grows with the square of shaft speed. Read `motors.c` (`motors_advance`, `motors_compute_wrench`). **Prerequisite.** Squares, exponentials as “smooth decay,” and `rad/s = RPM·2π/60`. **Example.** 5000 RPM is 523.6 rad/s and makes 4.112 N with default `kT`. **Experiment.** Compare settled thrust at 0.25 and 0.50 command; expect about four times more.

Check: (1) Double ω gives how much thrust? **Four times.** (2) Is RPM used internally? **No, rad/s.** (3) What does motor `τ` do? **Sets response lag, not final speed.**

## 4. Hover and thrust-to-weight

**Idea.** Hover is force balance, not a special state. Read `drone_hover_omega` and the CLI collective setup. **Prerequisite.** Square roots and equilibrium. **Example.** `sqrt(1·9.80665/(4·1.5e−5))=404.28 rad/s=3861 RPM`. **Experiment.** Increase mass to 1.4 kg and calculate the new hover command before flying.

Check: (1) Is hover 50% by definition? **No.** (2) Why four in the formula? **Four equal rotors.** (3) Can the vehicle hover above maximum ω? **No; configuration lacks thrust margin.**

## 5. Discrete integration

**Idea.** The computer advances continuous-looking motion in finite steps. Read `integrator.c` and `simulationLoop.ts`. **Prerequisite.** Rate times time gives change. **Example.** Starting at rest with `a=2 m/s²`, one 0.002 s step gives `v=0.004 m/s` and semi-implicit position change `0.000008 m`. **Experiment.** Run a scenario at 1 ms and 2 ms and compare final state.

Check: (1) Why fixed `dt`? **Determinism and controlled error.** (2) Why update velocity first? **Semi-implicit Euler is more stable.** (3) Does render FPS change physics `dt`? **No.**

## 6. Moment arms and torque

**Idea.** Off-centre thrust rotates the body through `M=r×F`. Read `motors_compute_wrench` and rotor positions in `drone_config_default`. **Prerequisite.** Cross products from lesson 1. **Example.** `r=(0.159,0.159,0) m`, `F=(0,0,4) N` gives `M=(0.636,−0.636,0) N·m`. **Experiment.** Double arm positions in a test config; moment doubles at the same thrust.

Check: (1) Force through COM gives arm moment? **Zero.** (2) Moment unit? **N·m.** (3) Why do equal four-rotor arm moments cancel? **Symmetric opposite positions.**

## 7. Roll, pitch, yaw, rate, and inertia

**Idea.** Angular acceleration depends on moment and the full inertia tensor. Read `matrix3.c`, `dynamics_calculate`, and `CONTROL_MIXER.md`. **Prerequisite.** Matrices as linear maps; inverse solves `Ix=b`. **Example.** `0.018 N·m / 0.018 kg·m² = 1 rad/s²` when cross coupling is zero. **Experiment.** Double Ixx and repeat a pitch pulse; expect about half the initial X acceleration.

Check: (1) Pitch axis here? **Body X.** (2) Roll axis here? **Body Y.** (3) Why validate positive-definite inertia? **Negative rotational energy is nonphysical and inversion may fail.**

## 8. Quaternions

**Idea.** A normalized four-number quaternion represents orientation without Euler singularities. Read `quaternion.c`, especially `quat_rotate` and `quat_integrate_body_rate`. **Prerequisite.** Sine/cosine and the idea of multiplying rotations. **Example.** `q=(cos45°,0,0,sin45°)` rotates +X to +Y by 90° about +Z. **Experiment.** Remove normalization in a temporary branch and run many steps; measure norm drift.

Check: (1) Quaternion storage order? **w,x,y,z.** (2) Is Euler angle integrated? **No.** (3) Why `q⊗ω` order? **ω is expressed in the body frame and q maps body to world.**

## 9. Four-motor mixing

**Idea.** Pilot intent becomes differential motor targets, never direct motion. Read `controls.c` (`controls_mix_manual`) and `test_mixer`. **Prerequisite.** Adding positive/negative contributions and clamping. **Example.** At 0.43 collective and full forward pitch, targets are 0.27/0.59/0.59/0.27. **Experiment.** Hold W for 0.25 s then release; observe continued attitude/rate in manual mode.

Check: (1) Which pair speeds for forward pitch? **Rear M2/M3.** (2) Why alternating yaw signs? **Opposite rotor reactions cancel when equal.** (3) Does key release level the drone? **No.**

## 10. Numerical error and validation

**Idea.** A passing simulation is an argument supported by invariants and tolerances. Read `test_suite.c`, `drone_config_validate`, and `ground.c`. **Prerequisite.** Absolute error `|actual−expected|`. **Example.** Quaternion norm is tested within `1e−12`; cross-platform scenario states need looser numeric tolerances. **Experiment.** Make inertia asymmetric and confirm validation rejects it.

Check: (1) Why not exact equality for computed floats? **Rounding.** (2) What can be exact on one build? **Repeated deterministic command sequences often are.** (3) Why test signs separately? **A stable simulation can still rotate the wrong way.**

## 11. Crossing into WebAssembly

**Idea.** The same C code runs in-browser; only a small adapter copies data. Read `wasm_exports.c`, `wasmClient.ts`, and `WASM_INTERFACE.md`. **Prerequisite.** Memory as numbered bytes; a double occupies 8 bytes. **Example.** A 44-double snapshot needs 352 bytes; position starts at index 2. **Experiment.** Add a reserved snapshot value in C and read it only in `wasmClient.ts`.

Check: (1) May a component call `_malloc`? **No.** (2) Why arrays instead of a struct cast? **C padding differs by ABI/compiler.** (3) Does TypeScript compute forces? **No.**

## 12. Trace, compare, and introduce control

**Idea.** A flight trace lets you compare hypotheses; assistance is explicit feedback, not a sensor simulation. Read `TRACE_ONE_FRAME.md`, `recorder.c`, and `controller.c`. **Prerequisite.** Feedback compares desired and current rates. **Example.** Desired roll rate 1.8 rad/s minus measured truth 0.3 gives error 1.5; proportional gain 0.55 contributes 0.825 before limiting/mixing. **Experiment.** Fly the same pulse in manual and assisted modes and export both histories.

Check: (1) Recorder rate? **20 Hz by default.** (2) Is assisted state a gyro reading? **No, perfect ground truth.** (3) Does assistance hold position/altitude? **No.**

