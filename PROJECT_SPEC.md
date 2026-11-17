# Drone Physics Simulator — Project Specification

**Document:** `PROJECT_SPEC.md`  
**Status:** Build specification for Codex  
**Version:** 1.0  
**Primary goal:** Build an understandable, modular, physics-based quadcopter simulator where a user pilots a specified virtual drone with a keyboard, sees its 3D motion and flight path, and can study/reverse-engineer every calculation.  
**Core philosophy:** **C does the simulation; web technologies only handle rendering and user interaction.** No external sensors, no GPS, no flight-controller hardware, no networking, and no external physics engine in v1.

---

## 1. Instructions to Codex

You are the implementation engineer. **Build a working project, not just a design or a mockup.** Follow this specification, write compilable code, run the tests and build, and document what you implemented and any deviations. Do not ask for clarification for ordinary design decisions: choose the simplest physically defensible implementation and record assumptions. Do not silently replace the C physics with JavaScript or a gaming physics engine.

Priorities, in order:

1. **Correctness and transparency of physics.** Explicit frames, units, sign conventions, assumptions, and tests.
2. **Modularity:** small, separately testable C modules with clear header interfaces.
3. **Actually controllable drone:** keyboard commands must alter motor states, forces, torques, and trajectory; no hard-coded animation.
4. **Useful but basic GUI:** interactive 3D view, settings, telemetry, plots, and flight-path trace.
5. **Education:** comments explain non-obvious equations, and dedicated learning notes explain how to reproduce and reverse-engineer results.

Deliver source, scripts, documentation, tests, and a working locally served browser application. Prefer complete core functionality over flashy styling. Build incrementally and validate after each milestone; never leave fake controls or placeholder telemetry presented as functional.

### Non-negotiable constraints

- Physics, rigid-body motion, motor dynamics, controller/mixing logic, numerical integration, and flight data generation **must be implemented in portable C11**.
- Compile the same C core natively for command-line tests and to WebAssembly for the browser. Use Emscripten to create the WASM build.
- React/TypeScript must **not** calculate flight dynamics; it may format values, control playback, capture input, display charts, and draw the scene.
- **No sensors in v1.** Do not add simulated accelerometer, gyro, magnetometer, barometer, GPS, camera, or sensor noise. Internal simulation ground-truth state is allowed for telemetry and optional ideal assisted control, and must be labelled as ground truth—not a sensor reading.
- Do not use Rapier, Cannon, Ammo, Bullet, Unity, Unreal, Gazebo, PX4, ArduPilot, or an external physics simulation engine.
- No backend or database required. No network access at runtime after dependencies are installed.
- Runs locally on a laptop with a recent desktop browser and keyboard.
- Determinism: same initial state, configuration, fixed time step, and input-command sequence should reproduce the same results on the same build/platform.

---

## 2. What the user should experience

The user opens the local web application and sees:

- A central **3D viewport** containing a quadcopter, visible propellers, ground grid, origin/axes, and a camera that can orbit, pan, and zoom.
- A **flight trail** tracing the drone's actual computed center-of-mass position in 3D.
- A left or right **Drone Configuration** panel (mass, geometry, inertia, motors, propeller coefficients, environment parameters) with editable, validated values and a Reset/Apply flow.
- A **Flight Controls** panel with readable keyboard mappings, input-mode selector, throttle, Arm/Disarm, Start/Pause/Resume/Reset, and a Clear Trail button.
- A compact **Telemetry** panel with X/Y/Z, altitude, velocity, speed, Euler angles for human display, angular rates, motor RPM, motor thrust, net force, simulation time, and flight status. Label telemetry **simulation truth**.
- Simple **time-series charts**: altitude vs. time, speed vs. time, motor RPM vs. time; optionally attitude vs. time. Chart samples must originate from C.
- A small **event log** for arm, takeoff, ground contact, crash, pause, reset, and invalid configuration.
- A **demo scenario** selector that loads repeatable input sequences and produces reproducible flight paths.

Desktop-first UI; avoid a complex design system. Ensure keyboard controls do not trigger while typing into fields and do not scroll the page.

---

## 3. Recommended stack

| Layer | Technology | Role |
|---|---|---|
| Physics, motors, integration, simulation API | **C11** | All simulation math and state transitions |
| Native builds/tests | **CMake + CTest** | Build standalone C library/executable and execute tests |
| Browser integration | **Emscripten -> WebAssembly** | Compile and call C engine from TypeScript |
| Application UI | **React + TypeScript + Vite** | Main GUI and event handling |
| 3D scene | **Three.js + React Three Fiber + Drei** | Visualize physical state, camera, axes, and flight trail |
| Styling | **Tailwind CSS** | Simple responsive panels |
| UI state | **Zustand** | GUI configuration, playback, and telemetry snapshots |
| Charts | **Recharts** | Render C-produced time-series data |
| C tests | **CTest with small C assertion-based test harness** | Mathematical invariants and deterministic scenarios |
| Frontend tests | **Vitest + React Testing Library** | Keyboard and GUI behavior |
| Optional E2E tests | **Playwright** | Browser smoke test for load/controls |

Avoid adding libraries unless they solve a demonstrable problem. Pin compatible versions in lockfiles.

---

## 4. Coordinate frames, conventions, and units

**Document and enforce these before implementing equations.**

- World frame: right-handed ENU-like Cartesian coordinate system: **+X East/right in initial scene, +Y North/forward, +Z up**. Ground plane `z = 0`.
- Body frame: right-handed: **+X right, +Y forward, +Z up**, aligned with world frame at initial attitude.
- Translation and velocity in world frame: meters, meters/second.
- Angular velocity in body frame: radians/second.
- Orientation state: **unit quaternion** `q_body_to_world`, normalized after each integration step. No Euler-angle integration. Display roll/pitch/yaw by a documented convention.
- Mass: kilograms; moment of inertia: kg·m²; force: newtons; torque: N·m; time: seconds; motor speed: rad/s internally (RPM only for GUI); angles: radians internally (degrees only for GUI).
- Gravity: world vector `(0, 0, -g)`, default `g = 9.80665 m/s²`.
- Motors numbered consistently and visibly in both docs and GUI. Define their body-frame locations in configuration rather than assuming their positions inside the physics routines.
- Rotor spinning directions `s_i = +1/-1` must be declared for every rotor and match the implemented reaction-torque sign convention.
- No arbitrary conversions or magic numbers scattered throughout modules.

**Important:** State explicitly whether a rotation in the chosen frame corresponds to nose-up, banking right, or clockwise yaw; validate it with unit tests. Do not assume conventional drone 'pitch' signs without testing. Use a labeled coordinate-frame diagram in the docs.

---

## 5. Default drone configuration

Choose these as initial defaults **only**, not as claims of real measured hardware performance:

| Property | Default |
|---|---|
| Airframe | Four-rotor quadcopter, X configuration |
| Total mass | `1.0 kg` |
| Motor-to-motor diagonal | `0.45 m` |
| Rotor positions | `(±a, ±a, 0)`, `a = diagonal / (2*sqrt(2))` |
| Propeller diameter | `0.254 m` (10 inches) |
| Rotor thrust law | `T_i = kT * omega_i²` |
| Reaction yaw torque law | `tau_reaction_i = s_i * kQ * omega_i²` about body +Z, with documented `s_i` convention |
| Motor response | First-order angular speed lag, configurable time constant |
| Arm/initial state | On ground, motors off, level, zero translational and angular velocity |
| Wind | Zero |
| Aerodynamic drag | Disabled by default in ideal mode |
| Ground | Flat, rigid plane at `z=0` |
| Gravity | `9.80665 m/s²` |
| Physics fixed time step | `0.002 s` (500 Hz) initially; configurable with validation |
| Visual rendering | Browser frame rate, independent of fixed physics step |

**Calibrate defaults carefully:** Pick `kT`, `kQ`, motor max RPM, motor time constant, and inertia such that the nominal quadcopter can hover with margin below max throttle, show visible yaw authority, and behave plausibly. Mark them as illustrative coefficients. Provide one documented worked example converting a chosen RPM to thrust; verify hover RPM numerically from `4*kT*omega_hover² = m*g`. Do not invent a motor's real datasheet numbers.

### Configuration rules

- Use a named `DroneConfig` containing mass, inertia tensor, rotor count fixed at 4 for v1, per-rotor positions/directions, thrust/torque coefficients, rotor max speed, motor lag, optional body drag coefficients, and ground-contact parameters.
- Use a named `EnvironmentConfig` containing gravity, air density (metadata/future aerodynamic work), wind (zero in ideal mode), and drag flags.
- Support saving/loading a JSON config through GUI (TS handles file I/O; C validates and applies the decoded config). Define a versioned, documented schema.
- Validate finite numbers, positive mass/inertia, physically valid motor positions, nonzero propulsion coefficients, throttle bounds, and physically usable inertia tensor (symmetric, positive-definite). Errors must be shown clearly in GUI.
- Apply config only when stopped/reset to prevent mid-flight parameter jumps, unless a change is intentionally supported and documented.

---

## 6. Physics model in C

Implement a **6-DOF rigid-body** drone model. Use these equations, with derivations and sign conventions documented in `docs/PHYSICS.md`.

### 6.1 Motor dynamics

Each motor has a commanded normalized throttle `u_i` in `[0,1]`. The target shaft speed is `omega_target_i = u_i * omega_max` (linear mapping is an explicit simplified assumption). For a positive time constant `tau_motor`, use:

`d(omega_i)/dt = (omega_target_i - omega_i) / tau_motor`

Use stable integration or the exact discrete update `omega_next = omega_target + (omega_current - omega_target)*exp(-dt/tau_motor)`; handle zero time constant explicitly. A more sophisticated ESC/battery model is out of scope.

### 6.2 Rotor forces and moments

- Rotor thrust vector in body frame: `F_i_body = (0, 0, kT * omega_i²)`.
- Arm moment: `M_arm_i = r_i × F_i_body`.
- Rotor reaction moment about body Z: `M_yaw_i = (0,0,s_i*kQ*omega_i²)`.
- Sum thrust and moments across all 4 rotors; apply body-to-world rotation to the thrust force.
- Opposite motor spin pairs must cancel net yaw torque when all speeds are equal.

### 6.3 Translation

`F_world = R(q) * ΣF_i_body + (0,0,-m*g) + F_drag_world`

`a_world = F_world / m`

`dv_world/dt = a_world`; `dp_world/dt = v_world`.

### 6.4 Rotation

Use the rigid-body Euler equation in the **body frame**:

`I * domega_body/dt = M_body - omega_body × (I * omega_body)`

Compute `omega_dot` using a robust 3x3 matrix solve or inverse validated at config load (not an unchecked diagonal-only shortcut). Propagate `q_body_to_world` with an angular-velocity quaternion derivative consistent with the chosen multiplication convention. Renormalize. Describe any neglected rotor gyroscopic effects.

### 6.5 Integration

- Provide a well-defined numerical integrator module (`integrator.c`).
- Start with a clear **fixed-step semi-implicit Euler** implementation for translation plus a coherent quaternion update; document limitations.
- Optionally add RK4 later behind an integration-mode setting, but do not delay a working v1 for RK4.
- Never scale the physics step with fluctuating browser frame time. The GUI uses a bounded accumulator to request fixed-size C steps. Prevent spiral-of-death with a documented max steps per rendered frame and clear slow-simulation indication rather than silently changing `dt`.
- Avoid allocation during each physics step.
- Expose exact simulation time as step count × fixed `dt` rather than accumulating display time errors.

### 6.6 Ground interaction and crashes

- Model a flat ground at `z=0`, with a documented simple collision approximation using the vehicle's effective ground clearance/radius. Do **not** claim it is a full collision solver.
- Never let the center of mass visually sink through the ground. Basic touchdown may zero downward velocity; define damping and friction simplifications.
- Model takeoff from rest: drone remains grounded until upward thrust is sufficient to leave contact.
- Detect a crash on excessive impact speed and/or attitude using configurable thresholds; mark state **Crashed**, cut motor commands, and require reset.
- Do not introduce physically impossible automatic mid-air stabilization.

### 6.7 Fidelity and limitations

State what is *not* modeled: blade-element aerodynamic effects, rotor wash, propeller flex, complex airframe drag, battery voltage, ESC commutation, turbulent wind, detailed landing gear, rotor gyroscopic coupling (unless implemented), fluid-solid interactions, and structural deformation. Ideal environment does **not** mean ignoring conservation laws or inertia.

---

## 7. Keyboard flight controls

**Goal:** Fly using a familiar WASD scheme. The keyboard affects **motor commands**, never directly teleports or changes world position.

### 7.1 Flight modes

Provide two clearly named selectable modes:

**Mode A — Manual/Direct (required and default initially):**

- Pilot keys request **collective motor thrust** and **roll/pitch/yaw differential motor authority** through a deterministic mixer written in C.
- There is **no automatic self-leveling or position hold**. Releasing WASD returns the differential command to zero but does **not** instantly remove angular velocity or attitude. Drone can keep rotating or drift. This is intentional physics, not a bug.
- Important naming: 'direct' here means **open-loop motor mixing from keyboard**. Add an optional advanced panel allowing direct per-motor throttle overrides (`M1`–`M4`) for experiments.

**Mode B — Ideal Assisted (secondary feature, after Mode A works):**

- Map key commands to desired roll/pitch or angular rates and implement a **C controller** (simple PID/PD where appropriate) that converts desired orientation/rates into motor commands.
- May consume the simulator's **perfect internal ground-truth state**, not simulated sensors; clearly label this as an idealized, unrealistically omniscient controller.
- It may self-level and stabilize attitude; **do not** automatically add GPS, position hold, or altitude hold. No position hold unless implemented as a separately labeled future feature.
- Make mode switching reset controller integrators and avoid discontinuous unsafe throttle jumps.

### 7.2 Initial key map

| Input | Action |
|---|---|
| `W` / `S` | Pitch forward / backward request |
| `A` / `D` | Roll left / right request |
| `Q` / `E` | Yaw left / right request |
| `Shift` (held) | Increase collective throttle at configurable rate |
| `Ctrl` (held) | Decrease collective throttle at configurable rate |
| `Space` | Pause/resume simulation (when focus is not in a text input) |
| `R` | Reset flight after a confirmation if flight data would be lost |
| GUI controls | Arm, Disarm, Reset, Clear Trail, flight mode, motor override |

Notes: Throttle is a **persistent setpoint** adjusted while Shift/Ctrl are held, clamped `[0,1]`; display percentage and actual rotor speed. For accessible laptops, add GUI throttle slider and clickable on-screen control buttons. Keyboard **WASD pitch/roll is not equivalent to horizontal translation**; any horizontal acceleration must follow tilt and thrust in the C dynamics. Provide diagrams/tooltips so this is understandable. Determine and verify actual signs by an acceptance test; positive keyboard behavior should match what a viewer perceives when camera is in the standard rear-follow view.

### 7.3 Input safety/robustness

- Key presses/releases stored as a compact command struct and sent to C on the simulation tick; key repeat must not create escalating one-off commands.
- On browser blur, `visibilitychange`, and input-mode change, clear pressed-key state so commands cannot get stuck. Preserve collective throttle unless disarmed/reset.
- Do not capture keys while a textbox/select/slider is focused or while shortcuts would interfere with accessibility.
- On disarm, command zero motor speed, continue physically simulating the fall until touchdown/crash unless the app is reset. Document this decision.
- Provide an **Arm** requirement before motor commands take effect. Start simulation and arming are separate concepts.
- Use an explicit mixer that converts collective plus roll/pitch/yaw requests into four bounded rotor targets with sensible priority and saturation handling. Document the mapping/matrix. Ensure balanced neutral commands produce equal motor targets.

---

## 8. API boundary between C and the GUI

Create a pure C API in `include/sim_api.h` with no DOM, Three.js, Emscripten-only, or React types. Export the required functions via a small WASM adapter and wrap them in TypeScript.

Suggested interface (adjust exact names as needed, document final ABI):

```c
typedef struct SimHandle SimHandle;

typedef enum {
    SIM_OK = 0,
    SIM_ERR_INVALID_ARGUMENT,
    SIM_ERR_INVALID_CONFIGURATION,
    SIM_ERR_INVALID_STATE
} SimResult;

SimHandle *sim_create(const DroneConfig *drone, const EnvironmentConfig *env);
void sim_destroy(SimHandle *sim);
SimResult sim_reset(SimHandle *sim);
SimResult sim_set_armed(SimHandle *sim, int armed);
SimResult sim_set_mode(SimHandle *sim, FlightMode mode);
SimResult sim_set_input(SimHandle *sim, const PilotInput *input);
SimResult sim_step(SimHandle *sim, double fixed_dt_seconds);
SimResult sim_get_state(const SimHandle *sim, SimSnapshot *out);
```

- Place concrete structs and enums in public headers, with `uint32_t` API/config versions when structs cross WASM boundaries.
- Keep internal `SimHandle` opaque to consumers.
- Prefer one packed/stable versioned snapshot copied through Emscripten HEAP views using a documented layout or named getters; **never assume JS and C struct memory layouts without checking padding/alignment**. Add an ABI-layout test or export struct sizes/offsets.
- For browser integration, hide pointer allocations/copies in a single `wasmClient.ts` adapter. UI components must not call raw `_malloc`, `_free`, or `ccall` directly.
- Provide error code + readable error message retrieval; do not rely on throwing uncaught exceptions from C.
- Provide a way to fetch bounded flight history samples/trail points without expensive per-point JS↔WASM calls.
- Native CLI must call exactly the same core API.
- Ensure the Vite dev server serves `.wasm` and Emscripten glue correctly; document `locateFile`/asset-loading setup. Do not use `file://` to launch the app.

---

## 9. Simulation recording, tracing and replay

- Track the **ground-truth center-of-mass** position `(x,y,z)` at a configurable sampling interval (e.g. 20 Hz), not every render frame.
- Maintain a bounded ring buffer or bounded sample array (e.g. last 60,000 trail samples); no unbounded memory growth.
- Store enough per-sample data for charts: simulation time, position, orientation, linear/angular velocity, motor RPM, motor thrust, command values, and current flight status.
- Draw actual flight trail in world coordinates; clear it independently from simulation reset.
- Support **CSV export** of flight history with headers and units, initiated by GUI.
- Support **JSON export** of scenario: versioned config + seed (if needed) + fixed step + timestamped input commands. A browser-loadable replay imports those commands and re-simulates them deterministically. Separate scenario from rendered history.
- Include presets: `Hover balance`, `Too little thrust`, `Vertical climb`, `Pitch forward`, `Yaw spin`, `Motor imbalance`, `Cut motors`, and `Crash test`. Each must be a reproducible sequence of motor/pilot commands applied through C. Do not cheat by setting positions directly.
- Offer following camera (optional) and free orbit camera; ensure the trail stays visible after the drone leaves its starting location.

---

## 10. GUI implementation details

Recommended desktop layout:

```
┌──────────────────────────────────────────────────────────────────┐
│ Drone Physics Simulator | Mode | Run/Pause | Arm | Reset | Export│
├─────────────────┬────────────────────────────┬───────────────────┤
│ Drone Config    │                            │ Telemetry         │
│ mass, inertia   │      3D FLIGHT VIEW        │ altitude / speed  │
│ motors / props  │   drone + axes + trail     │ attitude / RPM    │
│ environment     │                            │ force / torque    │
│ Apply / Reset   │                            │ status / time     │
├─────────────────┴────────────────────────────┴───────────────────┤
│ Keyboard: WASD pitch/roll | Q/E yaw | Shift/Ctrl throttle        │
│ Throttle slider | Per-motor override (advanced) | Demo presets   │
├──────────────────────────────────────────────────────────────────┤
│ Charts: altitude vs time | speed vs time | M1–M4 RPM vs time     │
└──────────────────────────────────────────────────────────────────┘
```

- 3D drone body orientation must come **only from the C quaternion**. Show motor numbering and spinning directions in a toggleable overlay.
- Render propellers spinning visually in proportion to reported RPM (visual-only animation); do not feed the graphics animation back into physics.
- Ground grid, world axes, horizontal reference, altitude readout, optional thrust-vector arrows, and optional force/torque vectors.
- Visually distinguish armed/disarmed, flying/grounded/crashed and manual/assisted.
- Limit charts/GUI updates to a sensible cadence (e.g. 10–20 Hz); render the scene smoothly independently of fixed physics steps.
- Prefer user-readable explanations/tooltips over decorative dashboards.
- Provide a simple `Help / How It Works` panel including control keys and the difference between hover, tilt, and translation.

---

## 11. Proposed modular repository structure

Codex may refine naming, but **preserve these boundaries**:

```text
drone-simulator/
├── PROJECT_SPEC.md
├── README.md
├── CMakeLists.txt
├── .gitignore
├── .github/workflows/ci.yml              # optional, if straightforward
├── engine/
│   ├── include/
│   │   ├── sim_api.h
│   │   ├── sim_types.h                    # shared structs, enums, units
│   │   ├── vector3.h
│   │   ├── matrix3.h
│   │   ├── quaternion.h
│   │   ├── drone_config.h
│   │   ├── motors.h
│   │   ├── dynamics.h
│   │   ├── integrator.h
│   │   ├── ground.h
│   │   ├── controls.h
│   │   ├── controller.h                   # assisted mode
│   │   └── recorder.h
│   ├── src/
│   │   ├── sim_api.c                      # orchestration + public API
│   │   ├── vector3.c
│   │   ├── matrix3.c
│   │   ├── quaternion.c
│   │   ├── drone_config.c
│   │   ├── motors.c
│   │   ├── dynamics.c
│   │   ├── integrator.c
│   │   ├── ground.c
│   │   ├── controls.c
│   │   ├── controller.c
│   │   └── recorder.c
│   ├── wasm/
│   │   └── wasm_exports.c               # minimal WASM ABI adapter
│   ├── cli/
│   │   └── main.c                       # headless simulation runner
│   └── tests/
│       ├── test_math.c
│       ├── test_motors.c
│       ├── test_mixer.c
│       ├── test_dynamics.c
│       ├── test_ground.c
│       ├── test_integrator.c
│       ├── test_replay.c
│       └── test_suite.c
├── web/
│   ├── package.json
│   ├── package-lock.json
│   ├── vite.config.ts
│   ├── tsconfig.json
│   ├── index.html
│   ├── public/wasm/                     # generated files; don't hand-edit
│   └── src/
│       ├── main.tsx
│       ├── App.tsx
│       ├── wasm/wasmClient.ts
│       ├── simulation/simulationLoop.ts
│       ├── simulation/inputMapper.ts
│       ├── simulation/configSchema.ts
│       ├── store/useSimulationStore.ts
│       ├── components/DroneScene.tsx
│       ├── components/FlightTrail.tsx
│       ├── components/ConfigurationPanel.tsx
│       ├── components/FlightControls.tsx
│       ├── components/TelemetryPanel.tsx
│       ├── components/ChartsPanel.tsx
│       ├── components/EventLog.tsx
│       └── styles.css
├── scripts/
│   ├── build-native.sh
│   ├── build-wasm.sh
│   ├── test.sh
│   └── run-dev.sh
├── scenarios/
│   ├── hover.json
│   ├── climb.json
│   ├── pitch.json
│   ├── yaw.json
│   └── crash.json
└── docs/
    ├── ARCHITECTURE.md
    ├── PHYSICS.md
    ├── COORDINATE_FRAMES.md
    ├── CONTROL_MIXER.md
    ├── WASM_INTERFACE.md
    ├── BUILD_AND_RUN.md
    ├── LIMITATIONS.md
    ├── LEARNING_PATH.md
    └── EXPERIMENTS.md
```

If Windows users need the project, provide cross-platform alternatives (PowerShell commands or CMake/npm scripts) instead of relying solely on bash. Build output belongs under ignored build directories.

### Module responsibilities (strict)

- `vector3`, `matrix3`, `quaternion`: self-contained numerical primitives; no drone knowledge.
- `drone_config`: defaults, checks, derived geometry and validated coefficients.
- `motors`: setpoints, motor lag, rotor thrust and reaction torque.
- `dynamics`: forces, body moments and accelerations; pure calculations where possible.
- `integrator`: fixed-time state advancement and quaternion normalization.
- `ground`: simple contact and crash logic.
- `controls`: key-intent to normalized pilot-input and motor mixer; no React dependencies.
- `controller`: optional explicitly ideal state-feedback assist; no sensor simulation.
- `recorder`: bounded samples, input event logging, and replay data.
- `sim_api`: owns state and calls modules in a documented fixed sequence.
- `wasm_exports`: marshaling only; no domain physics.
- `wasmClient.ts`: the only frontend module aware of raw WASM pointers and memory layout.
- `simulationLoop.ts`: accumulator/timing and snapshot delivery; no forces/equations.
- React components: configuration/input/presentation, not physics.

---

## 12. Verification and acceptance tests

**Implement automated tests and publish results in `README.md`.** Include numeric tolerances and explain those tolerances.

### Math

- Vector dot/cross product tests; right-handed cross product sign.
- Quaternion identity leaves vectors unchanged; 90° known rotation yields correct axis; normalized quaternion stays close to unit length after extended simulation.
- Inertia tensor validation rejects invalid tensors.

### Motors and mixing

- All motors off -> zero thrust/rotor moment.
- With identical motor RPM, thrust and torque have correct units and `T ∝ RPM²`.
- Identical 4-motor commands produce zero net roll/pitch/yaw moment in balanced default layout.
- Differential commands produce the **expected signed** roll/pitch/yaw moments.
- Mixer saturation never produces values outside `[0,1]`; motors converge to commanded RPM as expected.

### Motion

- Zero thrust in free flight -> downward acceleration `-g` (with no drag/contact).
- In level free flight, total thrust `m*g` -> near-zero vertical acceleration.
- Level drone with thrust greater than weight -> upward acceleration close to `(T - m*g)/m`.
- A tilted drone's thrust has a horizontal component and leads to horizontal acceleration.
- A nonzero moment causes expected angular acceleration; repeated integration changes attitude.
- In zero-force/zero-moment free flight, momentum behavior matches model expectations.
- Drone stays on ground with insufficient upward thrust and takes off when thrust exceeds weight after spool-up.
- Crash thresholds and reset behavior work as specified.

### System and UI

- Deterministic headless scenario repeat gives equivalent numerical state within tolerance.
- CSV output columns and units correct; saved/reloaded scenario reconstructs flight within tolerance.
- WASM results match native results within an explicitly documented tolerance for a fixed scenario.
- `WASD`, `Q/E`, `Shift/Ctrl` update C pilot commands; keys released on blur; typing in inputs does not control flight.
- Pause stops simulation-time progression; reset restores initial values; clear trail clears trail only.
- The drone model's position and quaternion match C snapshots (no graphics-generated motion).
- App handles failed WASM loads and invalid configs with readable errors.

### Minimum demonstrable outcomes

1. Start on the ground, arm, increase throttle: motors spool up, the drone rises when thrust exceeds weight.
2. At approximate hover thrust, in a level ideal state, altitude stays nearly steady for a bounded test interval.
3. Apply `W`: front/back motor differential induces pitch, which ultimately changes direction of motion; the trail bends correctly.
4. Apply `A` or `D`: roll and lateral acceleration appear via physics.
5. Apply `Q` or `E`: drone yaws in the expected direction.
6. Release controls in **Manual/Direct**: it does not magically stop angular momentum or auto-level.
7. Reset and replay an identical scenario: flight trace reproduces within tolerance.
8. Export a CSV with actual C-sampled positions, velocities, angles and motor speeds.
9. Change total mass, reset, repeat identical motor commands: different acceleration and trajectory.
10. Run native C tests and build WebAssembly successfully using documented commands.

---

## 13. Performance and maintainability

- Use a fixed physics rate nominally 500 Hz and render on requestAnimationFrame, with snapshot interpolation for smooth visual motion if useful. Interpolation must be presentation-only and must not alter physics state.
- Keep event-loop work bounded; consider a Web Worker later only if profiling proves necessary. Do not complicate v1 without evidence.
- Avoid per-step heap allocation; use bounded recording buffers and batch history transfers to JS.
- Build C with warnings enabled (`-Wall -Wextra -Wpedantic` as applicable); use debug assertions/optional sanitizer builds natively.
- Pure, testable math; no hidden globals for physics state; support more than one independent `SimHandle`.
- Source comments should explain **why**, equations, units and non-obvious edge cases, not narrate every line.
- Document public APIs in headers. Keep functions small and single-purpose; avoid one huge `main.c`, `physics.c`, `App.tsx`, or giant mixed JS/C module.
- Respect C memory ownership; document allocations, lifetimes and freeing in native and WASM builds.
- Avoid overengineering ECS/game frameworks or adding dependencies unrelated to v1.

---

## 14. Required documentation for learning and reverse engineering

This project is also intended as a **self-teaching tool for someone learning physics, mathematics, C, and drone mechanics from the basics**. Write docs that are technically accurate but beginner-friendly.

### `docs/LEARNING_PATH.md`

Create a 12-lesson progression, each with:

1. What concept this lesson teaches, in plain language.
2. Prerequisite math/physics explained briefly (do not assume calculus fluency).
3. Exact C files/functions to read.
4. A worked numeric example, including units.
5. One simple code change/experiment and expected result.
6. 3–5 questions with answers to check understanding.

Suggested lessons:

1. Coordinate systems, meters, seconds, vectors.
2. Mass, weight, force and Newton's laws.
3. Propeller thrust and why increasing RPM changes lift quadratically.
4. Hover and thrust-to-weight ratio.
5. Acceleration, velocity, position and discrete time steps.
6. Center of mass, moment arms, torque and cross products.
7. Roll, pitch, yaw, angular velocity and inertia.
8. Quaternions and why Euler-angle integration is problematic.
9. Four-motor mixing and how WASD translates into rotor commands.
10. Numerical error, simulation-rate independence and validation.
11. Reading C state through the WebAssembly boundary into the 3D UI.
12. Tracing a flight, comparing experiments and introducing control without sensors.

### `docs/PHYSICS.md`

For each core equation, give:

- A plain-language interpretation.
- Symbol definitions and units.
- What C function calculates it.
- A worked example with numbers.
- What breaks if a sign/unit is wrong.
- Simplifications and assumptions.

### `docs/ARCHITECTURE.md`

Include a data-flow diagram: keyboard -> input state -> C pilot command -> mixer -> motor state -> forces/moments -> integrator -> physical state -> snapshot -> 3D view/charts. Describe what happens during **one 2 ms physics step** in call order, with links to filenames and functions. Contrast native executable vs browser/WASM builds.

### `docs/EXPERIMENTS.md`

Provide at least 10 experiments with setup, expected outcome and explanation:

- Find hover RPM from mass and `kT`.
- Increase mass while holding RPM.
- Compare 20%, 50%, 80% throttle and nonlinear thrust.
- Create pitch with two motor pairs.
- Cause yaw via contra-rotating propeller torque.
- Compare two motor time constants.
- Shift center of mass / alter inertia if model supports it.
- Turn drag on/off.
- Cut motors mid-flight.
- Compare a 2 ms and 1 ms integration step.
- Repeat scenario and verify deterministic path.
- Deliberately flip a motor direction in a separate test configuration to observe consequences.

### Source-tracing guide

Add `docs/TRACE_ONE_FRAME.md` or a dedicated section showing a concrete trace from `W` keydown to raw pilot command -> chosen four rotor targets -> RPM after one step -> Newtons -> N·m -> body acceleration -> updated quaternion and position -> WASM snapshot -> new pixel/3D placement. Name exact functions and structs so a learner can set breakpoints and follow execution.

---

## 15. Build and running instructions

Provide commands that actually work for:

1. Installing standard prerequisites (Node.js LTS, npm, CMake, a C11 compiler, Emscripten SDK).
2. Building the native engine and headless CLI.
3. Running native tests/CTest.
4. Building the WASM artifact and copying/generated output into the frontend's expected public location.
5. Installing frontend dependencies.
6. Starting Vite and opening the browser localhost URL.
7. Building a production-ready static site.
8. Running frontend tests and, if implemented, E2E smoke tests.

Document both Linux/macOS shell and Windows PowerShell alternatives where practical. Make `npm` scripts and CMake targets the source of truth for repeated workflows. README must explain the minimum install, quick start and expected user-visible behavior. No unexplained environment variables.

---

## 16. Implementation plan — build in vertical milestones

**Milestone 0 — Scaffold and conventions**

- Repository, CMake, Vite/React, dependencies, lint/build scripts, coordinate frame and model assumptions.
- Deliver: minimal GUI loads and native C test executable runs.

**Milestone 1 — C physics kernel**

- Math primitives, config validation, motor forces, 6-DOF dynamics, integration, ground handling.
- Deliver: native CLI runs a scripted lift/hover/tip scenario; physics tests pass.

**Milestone 2 — Browser WASM bridge**

- Versioned ABI, C snapshots, TS bridge, fixed-step simulation loop.
- Deliver: browser displays live truth-state numbers matching native trajectory.

**Milestone 3 — 3D flight and basic GUI**

- Render real state, drone model/propellers, ground grid, orbit camera, configuration, arm/run/pause/reset.
- Deliver: visibly rising/falling drone with no animation cheats.

**Milestone 4 — WASD piloting and trace**

- Open-loop mixer, keyboard handling, throttle, 3D trail, presets, basic data plots.
- Deliver: keyboard-controlled physics-driven trajectory and valid replay.

**Milestone 5 — Learning and quality**

- Full docs, experiments, exports, validation, browser tests, usability and bug fixes.
- Deliver: one-command documented build workflow, trustworthy examples and no known failing tests.

**Milestone 6 — Optional ideal assisted mode**

- Implement only after milestones 1–5 are working. Controlled angular-rate/attitude assistance using internal truth state, clearly separate from no-sensor manual mode.
- Deliver: assisted behavior demonstrably different from manual mode, with tests and documentation.

Codex should complete as many milestones as the session allows, **prioritizing a working minimal vertical slice**, then report precisely which features/tests are complete and what remains. Do not assert features work unless executed/verified.

---

## 17. Definition of done

The project is complete when:

- All actual physics is authored in C, shared between native and WASM builds.
- User can open local browser UI, see a 3D drone, arm, set throttle, fly with WASD/QE, and trace the trajectory.
- Changes to mass, inertia, motor coefficients or gravity cause computed motion changes, not just changed labels.
- Keyboard input affects motor commands; computed forces and torques drive drone position/orientation; Three.js renders it faithfully.
- No simulated sensors are required, and no hidden stabilization exists in Manual/Direct mode.
- Data export, reset, scenario playback, telemetry and charts work.
- Ground interaction works sufficiently for takeoff/landing/crash experiments, with limitations disclosed.
- Automated tests cover mathematical sign conventions and key flight behavior; both C native and web builds succeed.
- A curious learner can follow **one keyboard event through every C calculation to the 3D movement** and reproduce at least one worked example by hand.

---

## 18. Non-goals and future enhancements

Do **not** scope-creep the first version with GPS, IMU, sensor fusion, PX4 integration, hardware-in-the-loop, wireless radio simulation, networking/multiplayer, complex CFD, terrain maps, drone swarms, battery chemistry, or real flight hardware control.

Potential later versions: PID tuning UI, motor torque curves from real data, aerodynamic drag model, ground effect, gusts, air density/altitude effects, actuator faults, realistic wind, parameter estimation, and comparison with measured flights. These are future ideas—not v1 deliverables.

---

## 19. Final report Codex must provide

At the end of implementation, report:

1. What was implemented, by milestone.
2. Commands to run native tests, build WASM, and start the GUI.
3. Actual automated tests run and their results.
4. Where the main physics loop lives and how the C/WASM boundary works.
5. Which assumptions and approximations were made.
6. Any incomplete features or known issues, without claiming them complete.
7. **Where to start reading**: first five files a beginner should inspect to understand the drone from keyboard command to resulting movement.

**Reminder:** The objective is a real, modular engineering learning simulator, not a drone-shaped animation.
