# Fidelity and limitations

This is an educational rigid-body model, not a flight certification tool. It models motor lag, quadratic thrust, reaction torque, the full 3×3 inertia equation, gravity, optional isotropic quadratic body drag, and a simple flat-ground constraint.

It does not model blade-element aerodynamics, inflow, rotor wash, ground effect, propeller flex, rotor gyroscopic coupling, battery voltage/sag, ESC commutation, motor electrical dynamics, turbulence, realistic landing gear, structural deformation, or fluid-solid interaction. Air density is metadata for future models. Ground contact clamps the centre of mass at the configured clearance, damps horizontal motion/rates, and uses speed/tilt crash thresholds; it is not a contact-manifold or impulse solver.

Semi-implicit Euler introduces time-step error. Comparing 2 ms and 1 ms runs is a useful convergence check. Native and Wasm floating-point results may differ in their last few bits; scenario comparisons should use tolerances, not byte equality across platforms.

