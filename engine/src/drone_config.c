#include "drone_config.h"
#include <math.h>
#include <stdio.h>

static void msg(char *out,size_t cap,const char *value){if(out&&cap)snprintf(out,cap,"%s",value);}
DroneConfig drone_config_default(void){
    DroneConfig c; double a=0.45/(2.0*sqrt(2.0));
    c.version=SIM_API_VERSION;c.mass_kg=1.0;c.inertia_kg_m2=mat3_diagonal(0.018,0.018,0.032);
    c.rotor_position_m[0]=vec3( a, a,0); c.rotor_spin[0]= 1; /* front-right */
    c.rotor_position_m[1]=vec3( a,-a,0); c.rotor_spin[1]=-1; /* rear-right */
    c.rotor_position_m[2]=vec3(-a,-a,0); c.rotor_spin[2]= 1; /* rear-left */
    c.rotor_position_m[3]=vec3(-a, a,0); c.rotor_spin[3]=-1; /* front-left */
    c.thrust_coefficient=1.50e-5;c.torque_coefficient=2.2e-7;
    c.max_omega_rad_s=1047.1975512; /* 10,000 RPM */
    c.motor_time_constant_s=0.08;c.body_drag_coefficient=0.12;
    c.ground_clearance_m=0.08;c.crash_speed_m_s=4.0;c.crash_tilt_rad=1.0471975512;
    return c;
}
EnvironmentConfig environment_config_default(void){
    EnvironmentConfig e={SIM_API_VERSION,9.80665,1.225,{0,0,0},0};return e;
}
SimResult drone_config_validate(const DroneConfig *d,const EnvironmentConfig *e,char *m,size_t cap){
    unsigned i; if(!d||!e){msg(m,cap,"configuration pointer is null");return SIM_ERR_INVALID_ARGUMENT;}
    if(d->version!=SIM_API_VERSION||e->version!=SIM_API_VERSION){msg(m,cap,"unsupported config version");return SIM_ERR_INVALID_CONFIGURATION;}
    if(!isfinite(d->mass_kg)||d->mass_kg<=0){msg(m,cap,"mass must be finite and positive");return SIM_ERR_INVALID_CONFIGURATION;}
    if(!mat3_is_symmetric_positive_definite(d->inertia_kg_m2)){msg(m,cap,"inertia must be symmetric positive-definite");return SIM_ERR_INVALID_CONFIGURATION;}
    if(!(d->thrust_coefficient>0)||!(d->torque_coefficient>0)||!(d->max_omega_rad_s>0)||d->motor_time_constant_s<0){msg(m,cap,"propulsion values are invalid");return SIM_ERR_INVALID_CONFIGURATION;}
    for(i=0;i<SIM_ROTOR_COUNT;i++){
        unsigned j;
        if(!vec3_is_finite(d->rotor_position_m[i])||vec3_length(d->rotor_position_m[i])<1e-6||(d->rotor_spin[i]!=1&&d->rotor_spin[i]!=-1)){msg(m,cap,"rotor position or spin direction is invalid");return SIM_ERR_INVALID_CONFIGURATION;}
        for(j=0;j<i;j++)if(vec3_length(vec3_sub(d->rotor_position_m[i],d->rotor_position_m[j]))<1e-6){msg(m,cap,"rotor positions must be distinct");return SIM_ERR_INVALID_CONFIGURATION;}
    }
    if(!(e->gravity_m_s2>0)||!isfinite(e->gravity_m_s2)||!vec3_is_finite(e->wind_m_s)){msg(m,cap,"environment values are invalid");return SIM_ERR_INVALID_CONFIGURATION;}
    if(!(d->ground_clearance_m>=0)||!(d->crash_speed_m_s>0)||!(d->crash_tilt_rad>0)){msg(m,cap,"ground parameters are invalid");return SIM_ERR_INVALID_CONFIGURATION;}
    msg(m,cap,"ok");return SIM_OK;
}
double drone_hover_omega(const DroneConfig *d,const EnvironmentConfig *e){return sqrt(d->mass_kg*e->gravity_m_s2/(4*d->thrust_coefficient));}

