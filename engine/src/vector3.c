#include "vector3.h"
#include <math.h>

Vec3 vec3(double x, double y, double z) { Vec3 v = {x,y,z}; return v; }
Vec3 vec3_add(Vec3 a, Vec3 b) { return vec3(a.x+b.x,a.y+b.y,a.z+b.z); }
Vec3 vec3_sub(Vec3 a, Vec3 b) { return vec3(a.x-b.x,a.y-b.y,a.z-b.z); }
Vec3 vec3_scale(Vec3 v, double s) { return vec3(v.x*s,v.y*s,v.z*s); }
double vec3_dot(Vec3 a, Vec3 b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
Vec3 vec3_cross(Vec3 a, Vec3 b) {
    return vec3(a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x);
}
double vec3_length(Vec3 v) { return sqrt(vec3_dot(v,v)); }
int vec3_is_finite(Vec3 v) { return isfinite(v.x)&&isfinite(v.y)&&isfinite(v.z); }

