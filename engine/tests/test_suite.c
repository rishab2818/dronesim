#include "sim_api.h"
#include "controls.h"
#include "controller.h"
#include "dynamics.h"
#include "integrator.h"
#include "motors.h"
#include "ground.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int passed=0,failed=0;
#define CHECK(name,expr) do{if(expr){passed++;printf("PASS  %s\n",name);}else{failed++;printf("FAIL  %s (%s:%d)\n",name,__FILE__,__LINE__);}}while(0)
static int near(double a,double b,double eps){return fabs(a-b)<=eps;}

static void test_math(void){
    Vec3 x=vec3(1,0,0),y=vec3(0,1,0),z=vec3_cross(x,y),rot;
    Quaternion q=quat_from_axis_angle(z,3.14159265358979323846/2);
    CHECK("dot orthogonal",near(vec3_dot(x,y),0,1e-12));
    CHECK("right-handed cross XxY=Z",near(z.z,1,1e-12));
    rot=quat_rotate(q,x);CHECK("quaternion +90 Z maps X to Y",near(rot.x,0,1e-12)&&near(rot.y,1,1e-12));
    q=quat_identity();for(int i=0;i<100000;i++)q=quat_integrate_body_rate(q,vec3(.1,.2,.3),.002);
    CHECK("quaternion remains normalized",near(q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z,1,1e-12));
}
static void test_config(void){
    DroneConfig d=drone_config_default();EnvironmentConfig e=environment_config_default();char msg[128];
    CHECK("default config valid",drone_config_validate(&d,&e,msg,sizeof(msg))==SIM_OK);
    d.inertia_kg_m2.m[1][1]=-1;CHECK("negative inertia rejected",drone_config_validate(&d,&e,msg,sizeof(msg))==SIM_ERR_INVALID_CONFIGURATION);
    d=drone_config_default();d.rotor_position_m[1]=d.rotor_position_m[0];CHECK("duplicate rotor positions rejected",drone_config_validate(&d,&e,msg,sizeof(msg))==SIM_ERR_INVALID_CONFIGURATION);
}
static void test_motors(void){
    DroneConfig d=drone_config_default();MotorState m;Vec3 f,t;double thrust[4],cmd[4]={0,0,0,0};
    motors_reset(&m);motors_advance(&m,cmd,&d,.002);motors_compute_wrench(&m,&d,&f,&t,thrust);
    CHECK("motors off zero thrust",near(f.z,0,1e-12));
    for(int i=0;i<4;i++)m.omega_rad_s[i]=500;
    motors_compute_wrench(&m,&d,&f,&t,thrust);
    CHECK("balanced rotors cancel all moments",vec3_length(t)<1e-12);
    CHECK("thrust follows kT omega squared",near(thrust[0],d.thrust_coefficient*250000,1e-12));
    cmd[0]=cmd[1]=cmd[2]=cmd[3]=.7;motors_reset(&m);for(int i=0;i<1000;i++)motors_advance(&m,cmd,&d,.002);
    CHECK("motor converges to target",near(m.omega_rad_s[0],.7*d.max_omega_rad_s,1e-6));
}
static void test_mixer(void){
    PilotInput p=pilot_input_zero();double o[4];DroneConfig d=drone_config_default();MotorState m;Vec3 f,t;double thrust[4];
    p.collective=.5;controls_mix_manual(&p,o);CHECK("neutral mixer balanced",near(o[0],o[3],1e-12)&&near(o[0],.5,1e-12));
    p.roll=1;controls_mix_manual(&p,o);for(int i=0;i<4;i++)m.omega_rad_s[i]=o[i]*d.max_omega_rad_s;motors_compute_wrench(&m,&d,&f,&t,thrust);
    CHECK("roll-right request makes positive body-Y moment",t.y>0);
    p.roll=0;p.pitch_forward=1;controls_mix_manual(&p,o);for(int i=0;i<4;i++)m.omega_rad_s[i]=o[i]*d.max_omega_rad_s;motors_compute_wrench(&m,&d,&f,&t,thrust);
    CHECK("pitch-forward request makes negative body-X moment",t.x<0);
    p.pitch_forward=0;p.yaw=1;controls_mix_manual(&p,o);for(int i=0;i<4;i++)m.omega_rad_s[i]=o[i]*d.max_omega_rad_s;motors_compute_wrench(&m,&d,&f,&t,thrust);
    CHECK("positive yaw request makes positive body-Z moment",t.z>0);
    p.collective=1;p.roll=p.pitch_forward=p.yaw=1;controls_mix_manual(&p,o);CHECK("mixer saturates in range",o[0]>=0&&o[0]<=1&&o[1]>=0&&o[1]<=1&&o[2]>=0&&o[2]<=1&&o[3]>=0&&o[3]<=1);
}
static void test_assisted_controller(void){
    DroneConfig d=drone_config_default();EnvironmentConfig e=environment_config_default();ControllerState c;RigidBodyState b={0};PilotInput p=pilot_input_zero();MotorState m;Vec3 f,t;double out[4],thrust[4],hover=drone_hover_omega(&d,&e)/d.max_omega_rad_s;
    p.collective=hover;b.q_body_to_world=quat_from_axis_angle(vec3(1,0,0),.2);controller_reset(&c);controller_mix_assisted(&c,&p,&b,&d,&e,.002,out);
    for(int i=0;i<4;i++)m.omega_rad_s[i]=out[i]*d.max_omega_rad_s;
    motors_compute_wrench(&m,&d,&f,&t,thrust);
    CHECK("assisted controller levels positive pitch",t.x<0);
    b.q_body_to_world=quat_identity();p.pitch_forward=1;controller_reset(&c);for(int i=0;i<500;i++)controller_mix_assisted(&c,&p,&b,&d,&e,.002,out);
    for(int i=0;i<4;i++)m.omega_rad_s[i]=out[i]*d.max_omega_rad_s;
    motors_compute_wrench(&m,&d,&f,&t,thrust);
    CHECK("assisted forward command produces nose-down moment",t.x<0);
    p=pilot_input_zero();p.collective=hover;b.velocity_world_m_s.z=1;controller_reset(&c);for(int i=0;i<500;i++)controller_mix_assisted(&c,&p,&b,&d,&e,.002,out);
    CHECK("assisted mode reduces collective while climbing",(out[0]+out[1]+out[2]+out[3])/4<hover);
    b.velocity_world_m_s.z=-1;controller_reset(&c);for(int i=0;i<500;i++)controller_mix_assisted(&c,&p,&b,&d,&e,.002,out);
    CHECK("assisted mode raises collective while descending",(out[0]+out[1]+out[2]+out[3])/4>hover);
    b.velocity_world_m_s=vec3(1,0,0);controller_reset(&c);for(int i=0;i<500;i++)controller_mix_assisted(&c,&p,&b,&d,&e,.002,out);
    for(int i=0;i<4;i++)m.omega_rad_s[i]=out[i]*d.max_omega_rad_s;
    motors_compute_wrench(&m,&d,&f,&t,thrust);
    CHECK("assisted mode banks against lateral drift",t.y<0);
    b.velocity_world_m_s=vec3(0,1,0);controller_reset(&c);for(int i=0;i<500;i++)controller_mix_assisted(&c,&p,&b,&d,&e,.002,out);
    for(int i=0;i<4;i++)m.omega_rad_s[i]=out[i]*d.max_omega_rad_s;
    motors_compute_wrench(&m,&d,&f,&t,thrust);
    CHECK("assisted mode pitches against forward drift",t.x>0);
    p.roll=1;p.pitch_forward=0;p.yaw=1;b.velocity_world_m_s=vec3(0,0,0);controller_reset(&c);controller_mix_assisted(&c,&p,&b,&d,&e,.002,out);
    CHECK("horizontal speed target is acceleration limited",c.lateral_speed_target_m_s>0&&c.lateral_speed_target_m_s<.01);
    CHECK("yaw rate target is acceleration limited",c.yaw_rate_target_rad_s>0&&c.yaw_rate_target_rad_s<.01);
    for(int i=0;i<2000;i++)controller_mix_assisted(&c,&p,&b,&d,&e,.002,out);
    CHECK("horizontal target respects speed cap",c.lateral_speed_target_m_s<=2.5);
    CHECK("yaw target respects rate cap",c.yaw_rate_target_rad_s<=.8);
    p.collective=1;p.roll=0;p.yaw=0;for(int i=0;i<2000;i++)controller_mix_assisted(&c,&p,&b,&d,&e,.002,out);
    CHECK("climb target respects rate cap",c.vertical_speed_target_m_s<=2.0);
}
static void test_dynamics(void){
    DroneConfig d=drone_config_default();EnvironmentConfig e=environment_config_default();RigidBodyState b={0};DynamicsOutput out;
    b.q_body_to_world=quat_identity();dynamics_calculate(&d,&e,&b,vec3(0,0,0),vec3(0,0,0),&out);
    CHECK("free flight acceleration is -g",near(out.acceleration_world_m_s2.z,-e.gravity_m_s2,1e-12));
    dynamics_calculate(&d,&e,&b,vec3(0,0,d.mass_kg*e.gravity_m_s2),vec3(0,0,0),&out);
    CHECK("level hover force cancels gravity",fabs(out.acceleration_world_m_s2.z)<1e-12);
    b.q_body_to_world=quat_from_axis_angle(vec3(1,0,0),.3);dynamics_calculate(&d,&e,&b,vec3(0,0,12),vec3(0,0,0),&out);
    CHECK("tilted thrust has horizontal component",fabs(out.acceleration_world_m_s2.y)>1);
    b.q_body_to_world=quat_identity();dynamics_calculate(&d,&e,&b,vec3(0,0,0),vec3(1,0,0),&out);
    CHECK("positive X moment produces positive angular acceleration",out.angular_acceleration_body_rad_s2.x>0);
}
static void test_system(void){
    DroneConfig d=drone_config_default();EnvironmentConfig e=environment_config_default();SimHandle *a=sim_create(&d,&e),*b=sim_create(&d,&e);PilotInput p=pilot_input_zero();SimSnapshot sa,sb;
    p.collective=drone_hover_omega(&d,&e)/d.max_omega_rad_s+.06;sim_set_armed(a,1);sim_set_armed(b,1);sim_set_input(a,&p);sim_set_input(b,&p);
    for(int i=0;i<1800;i++){if(i==900){p.roll=.2;sim_set_input(a,&p);sim_set_input(b,&p);}sim_step(a,.002);sim_step(b,.002);}
    sim_get_state(a,&sa);sim_get_state(b,&sb);
    CHECK("thrust above hover takes off",sa.body.position_world_m.z>d.ground_clearance_m+.1);
    CHECK("repeat scenario deterministic",memcmp(&sa,&sb,sizeof(sa))==0);
    CHECK("history recorded and bounded",sim_history_count(a)>10&&sim_history_count(a)<=SIM_HISTORY_CAPACITY);
    sim_destroy(a);sim_destroy(b);
}
static void test_ground(void){
    DroneConfig d=drone_config_default();RigidBodyState b={0};int event=0;b.q_body_to_world=quat_identity();b.position_world_m=vec3(0,0,d.ground_clearance_m);b.velocity_world_m_s=vec3(0,0,-1);
    CHECK("ordinary ground contact clamps clearance",ground_resolve(&b,&d,SIM_FLYING,0,9.80665,&event)==SIM_GROUNDED&&near(b.position_world_m.z,d.ground_clearance_m,1e-12)&&near(b.velocity_world_m_s.z,0,1e-12));
    b.position_world_m.z=d.ground_clearance_m;b.velocity_world_m_s.z=-(d.crash_speed_m_s+1);
    CHECK("excess impact is a crash",ground_resolve(&b,&d,SIM_FLYING,0,9.80665,&event)==SIM_CRASHED);
}
static void test_reset(void){
    DroneConfig d=drone_config_default();EnvironmentConfig e=environment_config_default();SimHandle *s=sim_create(&d,&e);PilotInput p=pilot_input_zero();SimSnapshot x;p.collective=.6;sim_set_input(s,&p);sim_set_armed(s,1);for(int i=0;i<500;i++)sim_step(s,.002);sim_reset(s);sim_get_state(s,&x);
    CHECK("reset restores initial ground state",x.step_count==0&&x.status==SIM_GROUNDED&&!x.armed&&near(x.body.position_world_m.z,d.ground_clearance_m,1e-12));
    CHECK("reset clears history",sim_history_count(s)==0);sim_destroy(s);
}
static void test_assisted_recovery(void){
    DroneConfig d=drone_config_default();EnvironmentConfig e=environment_config_default();SimHandle *s=sim_create(&d,&e);PilotInput p=pilot_input_zero();SimSnapshot x;Vec3 attitude;
    p.collective=drone_hover_omega(&d,&e)/d.max_omega_rad_s+.03;sim_set_mode(s,SIM_MODE_IDEAL_ASSISTED);sim_set_armed(s,1);sim_set_input(s,&p);
    for(int i=0;i<1000;i++)sim_step(s,.002);
    p.collective-=.03;p.pitch_forward=.6;sim_set_input(s,&p);for(int i=0;i<250;i++)sim_step(s,.002);
    p.pitch_forward=0;sim_set_input(s,&p);for(int i=0;i<1500;i++)sim_step(s,.002);
    sim_get_state(s,&x);attitude=quat_to_euler_xyz(x.body.q_body_to_world);
    CHECK("assisted mode returns pitch near level",fabs(attitude.x)<.10);
    CHECK("assisted mode damps pitch rate",fabs(x.body.omega_body_rad_s.x)<.10);
    sim_destroy(s);
}
int main(void){test_math();test_config();test_motors();test_mixer();test_assisted_controller();test_dynamics();test_ground();test_system();test_reset();test_assisted_recovery();printf("\n%d passed, %d failed\n",passed,failed);return failed?1:0;}
