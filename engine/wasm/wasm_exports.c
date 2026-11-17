#include "sim_api.h"
#include "controls.h"
#include <stdint.h>
#ifdef __EMSCRIPTEN__
#include <emscripten/emscripten.h>
#define EXPORT EMSCRIPTEN_KEEPALIVE
#else
#define EXPORT
#endif

/* The Wasm boundary uses arrays of doubles, never C struct layout. This makes
   padding and alignment irrelevant to JavaScript. See docs/WASM_INTERFACE.md. */
EXPORT SimHandle *wasm_sim_create_default(void){
    DroneConfig d=drone_config_default();EnvironmentConfig e=environment_config_default();
    return sim_create(&d,&e);
}
EXPORT void wasm_sim_destroy(SimHandle *s){sim_destroy(s);}
EXPORT const char *wasm_sim_last_error(SimHandle *s){return sim_last_error(s);}
EXPORT int wasm_sim_reset(SimHandle *s){return (int)sim_reset(s);}
EXPORT int wasm_sim_set_armed(SimHandle *s,int armed){return (int)sim_set_armed(s,armed);}
EXPORT int wasm_sim_set_mode(SimHandle *s,int mode){return (int)sim_set_mode(s,(FlightMode)mode);}
EXPORT int wasm_sim_set_input(SimHandle *s,double collective,double roll,double pitch,double yaw,
 int override_enabled,double m1,double m2,double m3,double m4){
    PilotInput p=pilot_input_zero();p.collective=collective;p.roll=roll;p.pitch_forward=pitch;p.yaw=yaw;
    p.use_motor_override=override_enabled;p.motor_override[0]=m1;p.motor_override[1]=m2;
    p.motor_override[2]=m3;p.motor_override[3]=m4;return (int)sim_set_input(s,&p);
}
EXPORT int wasm_sim_step(SimHandle *s,double dt){return (int)sim_step(s,dt);}
EXPORT int wasm_sim_apply_basic_config(SimHandle *s,double mass,double ixx,double iyy,double izz,
 double kt,double kq,double max_rpm,double motor_tau,double gravity,double drag,int drag_enabled){
    DroneConfig d=drone_config_default();EnvironmentConfig e=environment_config_default();
    d.mass_kg=mass;d.inertia_kg_m2=mat3_diagonal(ixx,iyy,izz);d.thrust_coefficient=kt;
    d.torque_coefficient=kq;d.max_omega_rad_s=max_rpm*2.0*3.14159265358979323846/60.0;
    d.motor_time_constant_s=motor_tau;d.body_drag_coefficient=drag;e.gravity_m_s2=gravity;e.drag_enabled=drag_enabled;
    return (int)sim_apply_config(s,&d,&e);
}
EXPORT int wasm_sim_snapshot_length(void){return 44;}
EXPORT int wasm_sim_get_snapshot(SimHandle *s,double *o,int capacity){
    SimSnapshot x;int n=0,i;if(!o||capacity<44||sim_get_state(s,&x)!=SIM_OK)return 0;
    o[n++]=x.time_s;o[n++]=(double)x.step_count;
    o[n++]=x.body.position_world_m.x;o[n++]=x.body.position_world_m.y;o[n++]=x.body.position_world_m.z;
    o[n++]=x.body.velocity_world_m_s.x;o[n++]=x.body.velocity_world_m_s.y;o[n++]=x.body.velocity_world_m_s.z;
    o[n++]=x.body.q_body_to_world.w;o[n++]=x.body.q_body_to_world.x;o[n++]=x.body.q_body_to_world.y;o[n++]=x.body.q_body_to_world.z;
    o[n++]=x.body.omega_body_rad_s.x;o[n++]=x.body.omega_body_rad_s.y;o[n++]=x.body.omega_body_rad_s.z;
    o[n++]=x.acceleration_world_m_s2.x;o[n++]=x.acceleration_world_m_s2.y;o[n++]=x.acceleration_world_m_s2.z;
    o[n++]=x.net_force_world_n.x;o[n++]=x.net_force_world_n.y;o[n++]=x.net_force_world_n.z;
    o[n++]=x.net_moment_body_nm.x;o[n++]=x.net_moment_body_nm.y;o[n++]=x.net_moment_body_nm.z;
    for(i=0;i<4;i++)o[n++]=x.motor_omega_rad_s[i]*60.0/(2.0*3.14159265358979323846);
    for(i=0;i<4;i++)o[n++]=x.motor_thrust_n[i];
    for(i=0;i<4;i++)o[n++]=x.motor_command[i];
    o[n++]=x.collective_command;o[n++]=(double)x.status;o[n++]=(double)x.mode;o[n++]=(double)x.armed;
    while(n<44)o[n++]=0;return n;
}
EXPORT int wasm_sim_history_count(SimHandle *s){return (int)sim_history_count(s);}
EXPORT void wasm_sim_clear_history(SimHandle *s){sim_clear_history(s);}
EXPORT int wasm_sim_history_row_length(void){return 27;}
EXPORT int wasm_sim_get_history_row(SimHandle *s,int index,double *o,int capacity){
    HistorySample h;int n=0,i;if(!o||capacity<27||index<0||sim_history_get(s,(uint32_t)index,&h)!=SIM_OK)return 0;
    o[n++]=h.time_s;o[n++]=h.position_world_m.x;o[n++]=h.position_world_m.y;o[n++]=h.position_world_m.z;
    o[n++]=h.q_body_to_world.w;o[n++]=h.q_body_to_world.x;o[n++]=h.q_body_to_world.y;o[n++]=h.q_body_to_world.z;
    o[n++]=h.velocity_world_m_s.x;o[n++]=h.velocity_world_m_s.y;o[n++]=h.velocity_world_m_s.z;
    o[n++]=h.omega_body_rad_s.x;o[n++]=h.omega_body_rad_s.y;o[n++]=h.omega_body_rad_s.z;
    for(i=0;i<4;i++)o[n++]=h.motor_rpm[i];for(i=0;i<4;i++)o[n++]=h.motor_thrust_n[i];
    for(i=0;i<4;i++)o[n++]=h.commands[i];o[n++]=(double)h.status;return n;
}

