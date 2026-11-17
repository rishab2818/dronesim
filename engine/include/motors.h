#ifndef DRONESIM_MOTORS_H
#define DRONESIM_MOTORS_H
#include "sim_types.h"

typedef struct { double omega_rad_s[SIM_ROTOR_COUNT]; } MotorState;
void motors_reset(MotorState *motors);
void motors_advance(MotorState *motors, const double command[SIM_ROTOR_COUNT],
                    const DroneConfig *config, double dt);
void motors_compute_wrench(const MotorState *motors, const DroneConfig *config,
                           Vec3 *force_body_n, Vec3 *moment_body_nm,
                           double thrust_n[SIM_ROTOR_COUNT]);

#endif

