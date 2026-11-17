#include "motors.h"
#include <math.h>
#include <string.h>

void motors_reset(MotorState *m){memset(m,0,sizeof(*m));}
void motors_advance(MotorState *m,const double command[4],const DroneConfig *c,double dt){
    unsigned i; for(i=0;i<4;i++){
        double u=command[i]<0?0:(command[i]>1?1:command[i]);
        double target=u*c->max_omega_rad_s;
        m->omega_rad_s[i]=c->motor_time_constant_s<=0?target:
          target+(m->omega_rad_s[i]-target)*exp(-dt/c->motor_time_constant_s);
    }
}
void motors_compute_wrench(const MotorState *m,const DroneConfig *c,Vec3 *force,Vec3 *moment,double thrust[4]){
    unsigned i;*force=vec3(0,0,0);*moment=vec3(0,0,0);
    for(i=0;i<4;i++){
        Vec3 f; thrust[i]=c->thrust_coefficient*m->omega_rad_s[i]*m->omega_rad_s[i];
        f=vec3(0,0,thrust[i]);*force=vec3_add(*force,f);
        *moment=vec3_add(*moment,vec3_cross(c->rotor_position_m[i],f));
        moment->z+=c->rotor_spin[i]*c->torque_coefficient*m->omega_rad_s[i]*m->omega_rad_s[i];
    }
}

