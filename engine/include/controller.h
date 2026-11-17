#ifndef DRONESIM_CONTROLLER_H
#define DRONESIM_CONTROLLER_H
#include "sim_types.h"

/** State for the beginner-mode command shaper and feedback controller. */
typedef struct {
    double roll_integral, pitch_integral, yaw_integral;
    double lateral_speed_target_m_s;
    double forward_speed_target_m_s;
    double vertical_speed_target_m_s;
    double yaw_rate_target_rad_s;
    double collective_limited;
} ControllerState;
void controller_reset(ControllerState *controller);
void controller_mix_assisted(ControllerState *controller, const PilotInput *input,
                             const RigidBodyState *body, const DroneConfig *drone,
                             const EnvironmentConfig *environment, double dt,
                             double out[SIM_ROTOR_COUNT]);

#endif

