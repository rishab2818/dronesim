#include "controls.h"

static double clamp(double v){return v<0?0:(v>1?1:v);}
PilotInput pilot_input_zero(void){PilotInput p={0};return p;}
void controls_mix_manual(const PilotInput *p,double o[4]){
    double r=.16*p->roll, f=.16*p->pitch_forward, y=.10*p->yaw,c=clamp(p->collective);
    unsigned i;if(p->use_motor_override){for(i=0;i<4;i++)o[i]=clamp(p->motor_override[i]);return;}
    /* M1 FR(+), M2 RR(-), M3 RL(+), M4 FL(-).
       +roll banks right (+body-Y); +pitch_forward creates -body-X moment. */
    o[0]=clamp(c-r-f+y);o[1]=clamp(c-r+f-y);
    o[2]=clamp(c+r+f+y);o[3]=clamp(c+r-f-y);
}

