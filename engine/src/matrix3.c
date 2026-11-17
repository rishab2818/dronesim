#include "matrix3.h"
#include <math.h>

Mat3 mat3_diagonal(double x,double y,double z) {
    Mat3 a={{{x,0,0},{0,y,0},{0,0,z}}}; return a;
}
Vec3 mat3_mul_vec(Mat3 a, Vec3 v) {
    return vec3(a.m[0][0]*v.x+a.m[0][1]*v.y+a.m[0][2]*v.z,
                a.m[1][0]*v.x+a.m[1][1]*v.y+a.m[1][2]*v.z,
                a.m[2][0]*v.x+a.m[2][1]*v.y+a.m[2][2]*v.z);
}
double mat3_determinant(Mat3 a) {
    return a.m[0][0]*(a.m[1][1]*a.m[2][2]-a.m[1][2]*a.m[2][1])
          -a.m[0][1]*(a.m[1][0]*a.m[2][2]-a.m[1][2]*a.m[2][0])
          +a.m[0][2]*(a.m[1][0]*a.m[2][1]-a.m[1][1]*a.m[2][0]);
}
int mat3_inverse(Mat3 a, Mat3 *out) {
    double d=mat3_determinant(a); Mat3 r;
    if(!out || !isfinite(d) || fabs(d)<1e-15) return 0;
    r.m[0][0]=(a.m[1][1]*a.m[2][2]-a.m[1][2]*a.m[2][1])/d;
    r.m[0][1]=(a.m[0][2]*a.m[2][1]-a.m[0][1]*a.m[2][2])/d;
    r.m[0][2]=(a.m[0][1]*a.m[1][2]-a.m[0][2]*a.m[1][1])/d;
    r.m[1][0]=(a.m[1][2]*a.m[2][0]-a.m[1][0]*a.m[2][2])/d;
    r.m[1][1]=(a.m[0][0]*a.m[2][2]-a.m[0][2]*a.m[2][0])/d;
    r.m[1][2]=(a.m[0][2]*a.m[1][0]-a.m[0][0]*a.m[1][2])/d;
    r.m[2][0]=(a.m[1][0]*a.m[2][1]-a.m[1][1]*a.m[2][0])/d;
    r.m[2][1]=(a.m[0][1]*a.m[2][0]-a.m[0][0]*a.m[2][1])/d;
    r.m[2][2]=(a.m[0][0]*a.m[1][1]-a.m[0][1]*a.m[1][0])/d;
    *out=r; return 1;
}
int mat3_is_symmetric_positive_definite(Mat3 a) {
    const double e=1e-12;
    if(fabs(a.m[0][1]-a.m[1][0])>e || fabs(a.m[0][2]-a.m[2][0])>e ||
       fabs(a.m[1][2]-a.m[2][1])>e) return 0;
    /* Sylvester's criterion: all leading principal minors must be positive. */
    return isfinite(mat3_determinant(a)) && a.m[0][0]>e &&
           a.m[0][0]*a.m[1][1]-a.m[0][1]*a.m[1][0]>e &&
           mat3_determinant(a)>e;
}

