#ifndef DRONESIM_QUATERNION_H
#define DRONESIM_QUATERNION_H
#include "vector3.h"

/** q = w + xi + yj + zk, representing a body-to-world rotation. */
typedef struct { double w, x, y, z; } Quaternion;

Quaternion quat_identity(void);
Quaternion quat_from_axis_angle(Vec3 axis, double radians);
Quaternion quat_multiply(Quaternion a, Quaternion b);
Quaternion quat_normalize(Quaternion q);
Vec3 quat_rotate(Quaternion q_body_to_world, Vec3 body_vector);
Quaternion quat_integrate_body_rate(Quaternion q, Vec3 omega_body, double dt);
Vec3 quat_to_euler_xyz(Quaternion q);

#endif

