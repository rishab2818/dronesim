#include "dynamics.h"
#include <math.h>

SimResult dynamics_calculate(const DroneConfig *d,const EnvironmentConfig *e,
 const RigidBodyState *b,Vec3 thrust_body,Vec3 moment,DynamicsOutput *o){
    Mat3 inverse;Vec3 relative,drag=vec3(0,0,0),angular_momentum,gyroscopic;
    if(!d||!e||!b||!o||!mat3_inverse(d->inertia_kg_m2,&inverse))return SIM_ERR_INVALID_ARGUMENT;
    o->force_world_n=quat_rotate(b->q_body_to_world,thrust_body);
    o->force_world_n.z-=d->mass_kg*e->gravity_m_s2;
    if(e->drag_enabled){
        relative=vec3_sub(b->velocity_world_m_s,e->wind_m_s);
        /* Simple isotropic quadratic drag: F=-c*|v|*v. */
        drag=vec3_scale(relative,-d->body_drag_coefficient*vec3_length(relative));
        o->force_world_n=vec3_add(o->force_world_n,drag);
    }
    o->moment_body_nm=moment;
    o->acceleration_world_m_s2=vec3_scale(o->force_world_n,1.0/d->mass_kg);
    angular_momentum=mat3_mul_vec(d->inertia_kg_m2,b->omega_body_rad_s);
    gyroscopic=vec3_cross(b->omega_body_rad_s,angular_momentum);
    o->angular_acceleration_body_rad_s2=mat3_mul_vec(inverse,vec3_sub(moment,gyroscopic));
    return SIM_OK;
}

