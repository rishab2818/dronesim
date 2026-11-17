#include "sim_api.h"
#include "controls.h"
#include <math.h>
#include <stdio.h>
#define PI 3.14159265358979323846

int main(int argc,char **argv){
    DroneConfig d=drone_config_default();EnvironmentConfig e=environment_config_default();
    SimHandle *s=sim_create(&d,&e);PilotInput p=pilot_input_zero();SimSnapshot snap;int i,steps=4000;
    (void)argc;(void)argv;if(!s){fprintf(stderr,"Failed to create simulator\n");return 1;}
    p.collective=drone_hover_omega(&d,&e)/d.max_omega_rad_s+0.04;
    sim_set_input(s,&p);sim_set_armed(s,1);
    printf("time_s,x_m,y_m,z_m,vz_m_s,roll_deg,pitch_deg,yaw_deg,status\n");
    for(i=0;i<steps;i++){
        if(i==1300)p.pitch_forward=0.20;
        if(i==1550)p.pitch_forward=0;
        if(i==2600)p.collective-=0.08;
        sim_set_input(s,&p);sim_step(s,0.002);
        if(i%250==0){Vec3 a;sim_get_state(s,&snap);a=quat_to_euler_xyz(snap.body.q_body_to_world);
          printf("%.3f,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%d\n",snap.time_s,snap.body.position_world_m.x,snap.body.position_world_m.y,snap.body.position_world_m.z,snap.body.velocity_world_m_s.z,a.x*180/PI,a.y*180/PI,a.z*180/PI,(int)snap.status);}
    }
    sim_destroy(s);return 0;
}
