#include "ground.h"
#include <math.h>

FlightStatus ground_resolve(RigidBodyState *b,const DroneConfig *c,FlightStatus previous,
 double upward_force,double weight,int *event){
    Vec3 up_world=quat_rotate(b->q_body_to_world,vec3(0,0,1));
    double tilt=acos(fmax(-1.0,fmin(1.0,up_world.z)));
    int touching=b->position_world_m.z<=c->ground_clearance_m;
    if(event)*event=0;
    if(previous==SIM_CRASHED){
        if(touching){b->position_world_m.z=c->ground_clearance_m;if(b->velocity_world_m_s.z<0)b->velocity_world_m_s.z=0;}
        return SIM_CRASHED;
    }
    if(touching){
        double impact=b->velocity_world_m_s.z<0?-b->velocity_world_m_s.z:0;
        b->position_world_m.z=c->ground_clearance_m;
        if(impact>c->crash_speed_m_s||tilt>c->crash_tilt_rad){
            b->velocity_world_m_s=vec3(0,0,0);b->omega_body_rad_s=vec3(0,0,0);
            if(event)*event=2;
            return SIM_CRASHED;
        }
        if(upward_force<=weight){
            if(b->velocity_world_m_s.z<0)b->velocity_world_m_s.z=0;
            /* Crude landing-foot friction, explicitly not a collision solver. */
            b->velocity_world_m_s.x*=0.92;b->velocity_world_m_s.y*=0.92;
            b->omega_body_rad_s=vec3_scale(b->omega_body_rad_s,0.90);
            if(previous==SIM_FLYING&&event)*event=1;
            return SIM_GROUNDED;
        }
    }
    if(previous==SIM_GROUNDED&&event)*event=3;
    return SIM_FLYING;
}

