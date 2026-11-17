#ifndef DRONESIM_VECTOR3_H
#define DRONESIM_VECTOR3_H

/** Three-component vector. Coordinates and units depend on the caller. */
typedef struct { double x, y, z; } Vec3;

Vec3 vec3(double x, double y, double z);
Vec3 vec3_add(Vec3 a, Vec3 b);
Vec3 vec3_sub(Vec3 a, Vec3 b);
Vec3 vec3_scale(Vec3 v, double scalar);
double vec3_dot(Vec3 a, Vec3 b);
Vec3 vec3_cross(Vec3 a, Vec3 b);
double vec3_length(Vec3 v);
int vec3_is_finite(Vec3 v);

#endif

