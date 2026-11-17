#include "controller.h"
#include "controls.h"
#include "drone_config.h"
#include <math.h>

static double limit(double v){return v<-1?-1:(v>1?1:v);}
static double clamp01(double v){return v<0?0:(v>1?1:v);}
static double clamp_angle(double v){const double a=.2617993878;return v<-a?-a:(v>a?a:v);}
static double approach(double current,double target,double amount){
    if(current<target)return current+amount<target?current+amount:target;
    return current-amount>target?current-amount:target;
}
static double vertical_rate_request(double collective,double hover){
    const double deadband=.002, span=.12;
    double delta=collective-hover;
    if(delta>deadband)return 2.0*clamp01((delta-deadband)/span);
    if(delta<-deadband)return -1.5*clamp01((-delta-deadband)/span);
    return 0;
}
void controller_reset(ControllerState *c){*c=(ControllerState){0};}
void controller_mix_assisted(ControllerState *c,const PilotInput *p,const RigidBodyState *b,
                             const DroneConfig *d,const EnvironmentConfig *e,double dt,double out[4]){
    PilotInput adjusted=*p;
    Vec3 attitude=quat_to_euler_xyz(b->q_body_to_world);
    Quaternion world_to_body={b->q_body_to_world.w,-b->q_body_to_world.x,-b->q_body_to_world.y,-b->q_body_to_world.z};
    Vec3 velocity_body=quat_rotate(world_to_body,b->velocity_world_m_s);
    double hover=drone_hover_omega(d,e)/d->max_omega_rad_s;
    double lateral_request=2.5*p->roll,forward_request=2.5*p->pitch_forward;
    double vertical_request=vertical_rate_request(p->collective,hover);
    double xy_slew=(1.2+1.8*fmax(fabs(p->roll),fabs(p->pitch_forward)))*dt;
    double vertical_slew=(.8+1.4*fabs(vertical_request)/2.0)*dt;
    double yaw_slew=(.8+1.8*fabs(p->yaw))*dt;
    double desired_roll,desired_pitch,er,ep,ey,collective_target;

    /* The stick magnitude sets a bounded velocity/rate. Each target has its
       own acceleration limit, so both pressing and releasing a key are smooth. */
    c->lateral_speed_target_m_s=approach(c->lateral_speed_target_m_s,lateral_request,xy_slew);
    c->forward_speed_target_m_s=approach(c->forward_speed_target_m_s,forward_request,xy_slew);
    c->vertical_speed_target_m_s=approach(c->vertical_speed_target_m_s,vertical_request,vertical_slew);
    c->yaw_rate_target_rad_s=approach(c->yaw_rate_target_rad_s,.8*p->yaw,yaw_slew);
    desired_roll=clamp_angle(.11*(c->lateral_speed_target_m_s-velocity_body.x));
    desired_pitch=clamp_angle(-.11*(c->forward_speed_target_m_s-velocity_body.y));
    er=desired_roll-attitude.y;
    ep=desired_pitch-attitude.x;
    ey=c->yaw_rate_target_rad_s-b->omega_body_rad_s.z;
    c->roll_integral=limit(c->roll_integral+er*dt);
    c->pitch_integral=limit(c->pitch_integral+ep*dt);
    c->yaw_integral=limit(c->yaw_integral+ey*dt);
    collective_target=clamp01(hover+.045*(c->vertical_speed_target_m_s-b->velocity_world_m_s.z));
    c->collective_limited=approach(c->collective_limited,collective_target,.8*dt);
    adjusted.collective=clamp01(c->collective_limited);
    adjusted.roll=limit(.30*er-.12*b->omega_body_rad_s.y+.02*c->roll_integral);
    adjusted.pitch_forward=limit(-(.30*ep-.12*b->omega_body_rad_s.x+.02*c->pitch_integral));
    adjusted.yaw=limit(.35*ey+.02*c->yaw_integral);
    controls_mix_manual(&adjusted,out);
}
