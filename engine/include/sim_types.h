#ifndef DRONESIM_SIM_TYPES_H
#define DRONESIM_SIM_TYPES_H
#include <stdint.h>
#include "matrix3.h"
#include "quaternion.h"

#define SIM_API_VERSION 1u
#define SIM_ROTOR_COUNT 4u
#define SIM_HISTORY_CAPACITY 6000u

typedef enum { SIM_MODE_MANUAL = 0, SIM_MODE_IDEAL_ASSISTED = 1 } FlightMode;
typedef enum { SIM_STOPPED = 0, SIM_GROUNDED, SIM_FLYING, SIM_CRASHED } FlightStatus;
typedef enum {
    SIM_OK = 0, SIM_ERR_INVALID_ARGUMENT, SIM_ERR_INVALID_CONFIGURATION,
    SIM_ERR_INVALID_STATE, SIM_ERR_OUT_OF_MEMORY
} SimResult;

typedef struct {
    uint32_t version;
    double mass_kg;
    Mat3 inertia_kg_m2;
    Vec3 rotor_position_m[SIM_ROTOR_COUNT];
    int rotor_spin[SIM_ROTOR_COUNT];
    double thrust_coefficient;
    double torque_coefficient;
    double max_omega_rad_s;
    double motor_time_constant_s;
    double body_drag_coefficient;
    double ground_clearance_m;
    double crash_speed_m_s;
    double crash_tilt_rad;
} DroneConfig;

typedef struct {
    uint32_t version;
    double gravity_m_s2;
    double air_density_kg_m3;
    Vec3 wind_m_s;
    int drag_enabled;
} EnvironmentConfig;

/** Persistent collective plus moment requests. roll/pitch/yaw are in [-1,1]. */
typedef struct {
    double collective;
    double roll;
    double pitch_forward;
    double yaw;
    int use_motor_override;
    double motor_override[SIM_ROTOR_COUNT];
} PilotInput;

typedef struct {
    Vec3 position_world_m;
    Vec3 velocity_world_m_s;
    Quaternion q_body_to_world;
    Vec3 omega_body_rad_s;
} RigidBodyState;

typedef struct {
    uint64_t step_count;
    double time_s;
    RigidBodyState body;
    Vec3 acceleration_world_m_s2;
    Vec3 net_force_world_n;
    Vec3 net_moment_body_nm;
    double motor_omega_rad_s[SIM_ROTOR_COUNT];
    double motor_thrust_n[SIM_ROTOR_COUNT];
    double motor_command[SIM_ROTOR_COUNT];
    double collective_command;
    FlightStatus status;
    FlightMode mode;
    int armed;
} SimSnapshot;

typedef struct {
    double time_s;
    Vec3 position_world_m;
    Quaternion q_body_to_world;
    Vec3 velocity_world_m_s;
    Vec3 omega_body_rad_s;
    double motor_rpm[SIM_ROTOR_COUNT];
    double motor_thrust_n[SIM_ROTOR_COUNT];
    double commands[SIM_ROTOR_COUNT];
    FlightStatus status;
} HistorySample;

#endif

