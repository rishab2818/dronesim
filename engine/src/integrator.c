#include "integrator.h"

void integrator_semi_implicit_euler(RigidBodyState *b,const DynamicsOutput *d,double dt){
    /* Update velocities first; positions then use the new velocities. This is
       more energy-stable than explicit Euler while remaining easy to trace. */
    b->velocity_world_m_s=vec3_add(b->velocity_world_m_s,vec3_scale(d->acceleration_world_m_s2,dt));
    b->position_world_m=vec3_add(b->position_world_m,vec3_scale(b->velocity_world_m_s,dt));
    b->omega_body_rad_s=vec3_add(b->omega_body_rad_s,vec3_scale(d->angular_acceleration_body_rad_s2,dt));
    b->q_body_to_world=quat_integrate_body_rate(b->q_body_to_world,b->omega_body_rad_s,dt);
}

