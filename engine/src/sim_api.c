#include "sim_api.h"
#include "controls.h"
#include "controller.h"
#include "dynamics.h"
#include "ground.h"
#include "integrator.h"
#include "motors.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct SimHandle {
    DroneConfig drone; EnvironmentConfig env; RigidBodyState body;
    MotorState motors; PilotInput input; ControllerState controller;
    double motor_command[4], motor_thrust[4]; DynamicsOutput dynamics;
    Recorder recorder; uint64_t step_count; double fixed_dt; FlightStatus status; FlightMode mode;
    int armed; char error[160];
};
static void set_error(SimHandle *s,const char *m){if(s)snprintf(s->error,sizeof(s->error),"%s",m);}
static void initial_state(SimHandle *s){
    s->body.position_world_m=vec3(0,0,s->drone.ground_clearance_m);
    s->body.velocity_world_m_s=vec3(0,0,0);s->body.q_body_to_world=quat_identity();
    s->body.omega_body_rad_s=vec3(0,0,0);motors_reset(&s->motors);
    s->input=pilot_input_zero();memset(s->motor_command,0,sizeof(s->motor_command));
    memset(s->motor_thrust,0,sizeof(s->motor_thrust));memset(&s->dynamics,0,sizeof(s->dynamics));
    controller_reset(&s->controller);s->step_count=0;s->fixed_dt=0.002;s->status=SIM_GROUNDED;s->armed=0;set_error(s,"ok");
}
SimHandle *sim_create(const DroneConfig *d,const EnvironmentConfig *e){
    char err[160];SimHandle *s;if(drone_config_validate(d,e,err,sizeof(err))!=SIM_OK)return NULL;
    s=(SimHandle*)calloc(1,sizeof(*s));if(!s)return NULL;s->drone=*d;s->env=*e;s->mode=SIM_MODE_MANUAL;
    recorder_init(&s->recorder,0.05);initial_state(s);return s;
}
void sim_destroy(SimHandle *s){free(s);}
SimResult sim_reset(SimHandle *s){if(!s)return SIM_ERR_INVALID_ARGUMENT;initial_state(s);recorder_clear(&s->recorder);return SIM_OK;}
SimResult sim_apply_config(SimHandle *s,const DroneConfig *d,const EnvironmentConfig *e){
    SimResult r;if(!s)return SIM_ERR_INVALID_ARGUMENT;
    if(s->step_count!=0||s->armed){set_error(s,"configuration can only be applied after reset while disarmed");return SIM_ERR_INVALID_STATE;}
    r=drone_config_validate(d,e,s->error,sizeof(s->error));if(r!=SIM_OK)return r;s->drone=*d;s->env=*e;initial_state(s);return SIM_OK;
}
SimResult sim_set_armed(SimHandle *s,int armed){if(!s)return SIM_ERR_INVALID_ARGUMENT;if(s->status==SIM_CRASHED&&armed){set_error(s,"reset is required after a crash");return SIM_ERR_INVALID_STATE;}s->armed=armed?1:0;return SIM_OK;}
SimResult sim_set_mode(SimHandle *s,FlightMode m){if(!s||(m!=SIM_MODE_MANUAL&&m!=SIM_MODE_IDEAL_ASSISTED))return SIM_ERR_INVALID_ARGUMENT;s->mode=m;controller_reset(&s->controller);return SIM_OK;}
SimResult sim_set_input(SimHandle *s,const PilotInput *p){
    unsigned i;if(!s||!p)return SIM_ERR_INVALID_ARGUMENT;
    if(!isfinite(p->collective)||p->collective<0||p->collective>1||!isfinite(p->roll)||!isfinite(p->pitch_forward)||!isfinite(p->yaw))return SIM_ERR_INVALID_ARGUMENT;
    for(i=0;i<4;i++)if(!isfinite(p->motor_override[i])||p->motor_override[i]<0||p->motor_override[i]>1)return SIM_ERR_INVALID_ARGUMENT;
    s->input=*p;return SIM_OK;
}
SimResult sim_step(SimHandle *s,double dt){
    Vec3 force_body,moment_body;int ground_event;SimResult r;unsigned i;
    if(!s||!isfinite(dt)||dt<=0||dt>0.02)return SIM_ERR_INVALID_ARGUMENT;
    if(s->step_count==0)s->fixed_dt=dt;
    else if(fabs(dt-s->fixed_dt)>1e-12){set_error(s,"fixed time step cannot change during a run");return SIM_ERR_INVALID_STATE;}
    if(!s->armed||s->status==SIM_CRASHED)memset(s->motor_command,0,sizeof(s->motor_command));
    else if(s->mode==SIM_MODE_IDEAL_ASSISTED)controller_mix_assisted(&s->controller,&s->input,&s->body,&s->drone,&s->env,dt,s->motor_command);
    else controls_mix_manual(&s->input,s->motor_command);
    motors_advance(&s->motors,s->motor_command,&s->drone,dt);
    motors_compute_wrench(&s->motors,&s->drone,&force_body,&moment_body,s->motor_thrust);
    r=dynamics_calculate(&s->drone,&s->env,&s->body,force_body,moment_body,&s->dynamics);if(r!=SIM_OK)return r;
    integrator_semi_implicit_euler(&s->body,&s->dynamics,dt);
    s->status=ground_resolve(&s->body,&s->drone,s->status,
       quat_rotate(s->body.q_body_to_world,force_body).z,s->drone.mass_kg*s->env.gravity_m_s2,&ground_event);
    if(s->status==SIM_CRASHED){s->armed=0;for(i=0;i<4;i++)s->motor_command[i]=0;}
    s->step_count++;
    {SimSnapshot snap;sim_get_state(s,&snap);recorder_maybe_record(&s->recorder,&snap,dt);}
    return SIM_OK;
}
SimResult sim_get_state(const SimHandle *s,SimSnapshot *o){
    unsigned i;if(!s||!o)return SIM_ERR_INVALID_ARGUMENT;memset(o,0,sizeof(*o));
    o->step_count=s->step_count;o->time_s=s->step_count*s->fixed_dt;
    o->body=s->body;o->acceleration_world_m_s2=s->dynamics.acceleration_world_m_s2;
    o->net_force_world_n=s->dynamics.force_world_n;o->net_moment_body_nm=s->dynamics.moment_body_nm;
    for(i=0;i<4;i++){o->motor_omega_rad_s[i]=s->motors.omega_rad_s[i];o->motor_thrust_n[i]=s->motor_thrust[i];o->motor_command[i]=s->motor_command[i];}
    o->collective_command=s->input.collective;o->status=s->status;o->mode=s->mode;o->armed=s->armed;return SIM_OK;
}
void sim_clear_history(SimHandle *s){if(s)recorder_clear(&s->recorder);}
uint32_t sim_history_count(const SimHandle *s){return s?recorder_count(&s->recorder):0;}
SimResult sim_history_get(const SimHandle *s,uint32_t i,HistorySample *o){return s&&recorder_get(&s->recorder,i,o)?SIM_OK:SIM_ERR_INVALID_ARGUMENT;}
const char *sim_last_error(const SimHandle *s){return s?s->error:"simulator is null";}
const char *sim_result_string(SimResult r){static const char *v[]={"ok","invalid argument","invalid configuration","invalid state","out of memory"};return r>=SIM_OK&&r<=SIM_ERR_OUT_OF_MEMORY?v[r]:"unknown error";}

