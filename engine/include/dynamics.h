#ifndef DRONESIM_DYNAMICS_H
#define DRONESIM_DYNAMICS_H
#include "sim_types.h"

typedef struct {
    Vec3 force_world_n;
    Vec3 moment_body_nm;
    Vec3 acceleration_world_m_s2;
    Vec3 angular_acceleration_body_rad_s2;
} DynamicsOutput;

SimResult dynamics_calculate(const DroneConfig *drone, const EnvironmentConfig *env,
                             const RigidBodyState *body, Vec3 rotor_force_body_n,
                             Vec3 rotor_moment_body_nm, DynamicsOutput *out);

#endif

