#include "quaternion.h"
#include <math.h>

Quaternion quat_identity(void){Quaternion q={1,0,0,0};return q;}
Quaternion quat_from_axis_angle(Vec3 axis,double r){
    double n=vec3_length(axis),s; if(n<1e-15)return quat_identity(); s=sin(r*0.5)/n;
    {Quaternion q={cos(r*0.5),axis.x*s,axis.y*s,axis.z*s};return q;}
}
Quaternion quat_multiply(Quaternion a,Quaternion b){
    Quaternion q={a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z,
      a.w*b.x+a.x*b.w+a.y*b.z-a.z*b.y,
      a.w*b.y-a.x*b.z+a.y*b.w+a.z*b.x,
      a.w*b.z+a.x*b.y-a.y*b.x+a.z*b.w};return q;
}
Quaternion quat_normalize(Quaternion q){
    double n=sqrt(q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z);
    if(!isfinite(n)||n<1e-15)return quat_identity();
    q.w/=n;q.x/=n;q.y/=n;q.z/=n;return q;
}
Vec3 quat_rotate(Quaternion q,Vec3 v){
    /* Equivalent to q*(0,v)*conjugate(q), arranged to avoid temporaries. */
    Vec3 u=vec3(q.x,q.y,q.z);
    return vec3_add(v,vec3_add(vec3_scale(vec3_cross(u,v),2*q.w),
                              vec3_scale(vec3_cross(u,vec3_cross(u,v)),2)));
}
Quaternion quat_integrate_body_rate(Quaternion q,Vec3 w,double dt){
    /* qdot = 0.5*q*(0,omega_body), because q maps body to world. */
    Quaternion o={0,w.x,w.y,w.z},d=quat_multiply(q,o);
    q.w+=0.5*d.w*dt;q.x+=0.5*d.x*dt;q.y+=0.5*d.y*dt;q.z+=0.5*d.z*dt;
    return quat_normalize(q);
}
Vec3 quat_to_euler_xyz(Quaternion q){
    /* Intrinsic rotations about body X(right), Y(forward), Z(up), for display. */
    double sx=2*(q.w*q.x+q.y*q.z),cx=1-2*(q.x*q.x+q.y*q.y);
    double sy=2*(q.w*q.y-q.z*q.x),sz=2*(q.w*q.z+q.x*q.y),cz=1-2*(q.y*q.y+q.z*q.z);
    if(sy>1)sy=1;
    if(sy<-1)sy=-1;
    return vec3(atan2(sx,cx),asin(sy),atan2(sz,cz));
}

