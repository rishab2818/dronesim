# Physics model

All calculations use SI units. The default values are illustrative coefficients chosen for a useful educational vehicle, not measurements from a named motor or propeller.

## Motor response

`motors_advance` computes `ωtarget = u ωmax`, then uses the exact constant-command solution

`ωnext = ωtarget + (ωcurrent − ωtarget) exp(−dt/τ)`.

Here `u` is unitless `[0,1]`, `ω` is rad/s, `dt` and `τ` are seconds. At 50% command with a 1047.2 rad/s maximum, the target is 523.6 rad/s (5000 RPM). With `τ=0.08 s`, starting at zero and stepping 2 ms gives `12.93 rad/s`. A missing minus sign makes the motor diverge. The model assumes a linear command-to-speed target and ignores battery/ESC electrical behavior. Zero `τ` changes speed immediately.

## Rotor thrust and moment

`motors_compute_wrench` applies `Ti = kT ωi²` along body +Z, `Mi,arm = ri × Fi`, and `Mi,yaw = (0,0,si kQ ωi²)`. `kT` has units N/(rad/s)² and `kQ` has N·m/(rad/s)².

Worked example: 5000 RPM is `523.6 rad/s`. With `kT=1.5×10⁻⁵`, one rotor makes `1.5×10⁻⁵ × 523.6² = 4.112 N`; four make 16.45 N. Reversing `r × F` flips roll/pitch. Giving all rotors the same spin sign creates false yaw at hover. Rotor gyroscopic moments are neglected.

## Hover

Balance weight and thrust: `4 kT ωhover² = mg`, so

`ωhover = sqrt(mg / (4 kT))`.

For `m=1 kg`, `g=9.80665 m/s²`, and the default `kT`, `ωhover=404.28 rad/s = 3860.7 RPM`, or about 38.6% of the 10,000 RPM limit. The spare thrust is deliberate. Mixing RPM and rad/s would make thrust wrong by `(60/2π)² ≈ 91.2`.

## Translation

`dynamics_calculate` evaluates

`Fworld = R(q) Fbody + (0,0,−mg) + Fdrag`, and `a = Fworld/m`.

With level thrust 12 N, the default vehicle has `az=(12−9.80665)/1 = 2.19335 m/s²`. At a 20° pitch, part of thrust becomes horizontal and the vertical component falls. `integrator_semi_implicit_euler` then uses `vnext=v+a dt` and `pnext=p+vnext dt`. A force-frame mistake makes a tilted drone accelerate along the wrong axis. Drag, when enabled, is the simplified `−c |v−wind| (v−wind)`.

## Rotation

In body coordinates, `dynamics_calculate` solves

`I ωdot = M − ω × (Iω)`.

It inverts the validated full 3×3 symmetric positive-definite inertia tensor, not just its diagonal. For zero initial rate, a 0.018 N·m X moment and `Ixx=0.018 kg·m²` give `1 rad/s²`. The cross term matters once the body rotates on multiple axes. A singular or non-positive inertia is rejected before simulation.

## Quaternion attitude

`quat_integrate_body_rate` uses `qdot = ½ q ⊗ (0,ωbody)` because `q` maps body vectors into world coordinates. It renormalizes after each step. At `ωz=1 rad/s`, a 2 ms step changes yaw by roughly 0.002 rad. Multiplying in the opposite order applies the rate in the wrong frame; skipping normalization slowly corrupts vector lengths. Euler angles are derived only for human display.

## Ground and crashes

`ground_resolve` keeps the centre of mass at or above `ground_clearance_m`. It holds the vehicle down while upward thrust does not exceed weight, allows liftoff above that threshold, zeros downward speed at ordinary contact, damps horizontal motion and body rates, and flags a crash above the configured impact speed or tilt. This approximation does not calculate contact points, impulses, bounce, or landing-gear geometry.

## Time and determinism

Simulation time is integer step count times the fixed `dt`; it is not accumulated from browser frames. The browser may execute at most 50 physics steps per render frame and reports when it cannot keep real time. Fixed configuration, fixed time step, and identical command sequence reproduce the same state on the same build/platform.

