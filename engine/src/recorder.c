#include "recorder.h"
#include <math.h>
#include <string.h>

void recorder_init(Recorder *r,double interval){memset(r,0,sizeof(*r));r->sample_interval_s=interval;}
void recorder_clear(Recorder *r){double i=r->sample_interval_s;memset(r,0,sizeof(*r));r->sample_interval_s=i;}
static HistorySample make_sample(const SimSnapshot *s){
    HistorySample h;unsigned i;h.time_s=s->time_s;h.position_world_m=s->body.position_world_m;
    h.q_body_to_world=s->body.q_body_to_world;h.velocity_world_m_s=s->body.velocity_world_m_s;
    h.omega_body_rad_s=s->body.omega_body_rad_s;h.status=s->status;
    for(i=0;i<4;i++){h.motor_rpm[i]=s->motor_omega_rad_s[i]*60.0/(2.0*3.14159265358979323846);h.motor_thrust_n[i]=s->motor_thrust_n[i];h.commands[i]=s->motor_command[i];}
    return h;
}
void recorder_maybe_record(Recorder *r,const SimSnapshot *s,double dt){
    uint64_t interval_steps=(uint64_t)llround(r->sample_interval_s/dt);uint32_t index;
    if(interval_steps<1)interval_steps=1;
    if(s->step_count!=0&&s->step_count-r->last_recorded_step<interval_steps)return;
    r->last_recorded_step=s->step_count;
    if(r->count<SIM_HISTORY_CAPACITY){index=(r->start+r->count)%SIM_HISTORY_CAPACITY;r->count++;}
    else{index=r->start;r->start=(r->start+1)%SIM_HISTORY_CAPACITY;}
    r->samples[index]=make_sample(s);
}
uint32_t recorder_count(const Recorder *r){return r?r->count:0;}
int recorder_get(const Recorder *r,uint32_t i,HistorySample *out){if(!r||!out||i>=r->count)return 0;*out=r->samples[(r->start+i)%SIM_HISTORY_CAPACITY];return 1;}

