# Experiments

Each experiment begins with Reset so configuration changes are legal and results are comparable.

1. **Find hover RPM.** Use `sqrt(mg/(4kT))`; defaults give about 3861 RPM. Arm and set 38.6%. After spool-up, vertical acceleration should approach zero.
2. **Increase mass.** Change 1 kg to 1.4 kg but retain the same collective. Acceleration falls because weight grows and the same force is divided by more mass.
3. **Compare 20/50/80%.** Record settled thrust. Because `T∝ω²`, ratios are roughly `0.04:0.25:0.64`, not `0.2:0.5:0.8`.
4. **Pitch pair.** Hover, hold W briefly, then release. Rear motors rise, the body pitches forward, and horizontal acceleration appears only after tilt.
5. **Yaw reaction.** Hold Q or E near hover. Alternating spin signs make a Z moment while balanced arm moments largely cancel.
6. **Motor lag.** Compare `τ=0.02 s` and `0.20 s`. The latter reaches takeoff later and softens differential response.
7. **Change inertia.** Double Ixx. The same W-induced X moment produces roughly half the pitch angular acceleration.
8. **Toggle drag.** Launch the same pitch case with drag disabled/enabled. Drag reduces speed; it does not create leveling or position hold.
9. **Cut motors.** Climb, disarm, and keep running. RPM decays through motor lag and the vehicle falls under gravity until contact/crash.
10. **Step convergence.** Run the CLI logic with 2 ms and 1 ms fixed steps. States should approach one another, but will not be bit-identical because numerical integration changes.
11. **Repeatability.** Reset and repeat the same preset. Export both CSV files; corresponding rows should match on the same build.
12. **Wrong rotor direction.** In a separate test configuration, change one spin sign. Equal motor speed now creates a yaw moment, showing why contra-rotation matters.

