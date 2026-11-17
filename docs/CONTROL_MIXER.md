# Manual mixer and ideal assistance

`controls_mix_manual` is open-loop. It clamps every result to `[0,1]` and uses:

```text
M1 = collective - roll - pitchForward + yaw
M2 = collective - roll + pitchForward - yaw
M3 = collective + roll + pitchForward + yaw
M4 = collective + roll - pitchForward - yaw
```

The implementation scales roll/pitch authority by 0.16 and yaw by 0.10 before this matrix. Equal neutral commands yield equal thrust. A right-roll request speeds the left pair, whose `r × F` moment points along body +Y. A forward-pitch request speeds the rear pair, producing a negative body-X moment. Yaw uses the alternating rotor reaction moments.

Clamping is the v1 saturation policy. It is simple and deterministic, but at high collective a moment request can lose authority. There is no automatic leveling in Manual/Direct mode.

`controller_mix_assisted` is a separate beginner velocity controller. WASD requests body-relative horizontal speed, limited to 2.5 m/s. Collective displacement around the calculated hover point requests up to 2.0 m/s climb or 1.5 m/s descent, while Q/E requests up to 0.8 rad/s yaw. Small control displacement requests a proportionally small rate.

The requested horizontal, vertical, and yaw rates pass through acceleration limiters before feedback is applied. Thus pressing or releasing a key cannot instantly change a target. Horizontal velocity error becomes a requested attitude capped at 15 degrees; vertical velocity error becomes collective around the true configuration-dependent hover command. Releasing the controls requests zero velocity, so the drone brakes smoothly and stops wherever braking ends. It still has no position or altitude hold.

The controller reads exact internal truth rather than simulated sensors. `ControllerState` stores its rate targets, collective slew state, and integrators; reset or a mode change clears all of them. Manual/Direct mode bypasses this entire path.

