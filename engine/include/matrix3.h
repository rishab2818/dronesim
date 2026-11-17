#ifndef DRONESIM_MATRIX3_H
#define DRONESIM_MATRIX3_H
#include "vector3.h"

/** Row-major 3x3 matrix. Inertia tensors use kg*m^2. */
typedef struct { double m[3][3]; } Mat3;

Mat3 mat3_diagonal(double x, double y, double z);
Vec3 mat3_mul_vec(Mat3 a, Vec3 v);
double mat3_determinant(Mat3 a);
int mat3_inverse(Mat3 a, Mat3 *out);
int mat3_is_symmetric_positive_definite(Mat3 a);

#endif

