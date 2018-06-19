#ifndef __EKF_H
#define __EKF_H

#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_strap.h"

/* Function Declarations */
extern void qkf_run(float X[7], float P[49], const float acc[3],
                               const float gyro[3], const float mag[3], float dt, const float Q[49],
                               const float R[36]);
extern void qkf_initialize(void);

extern void mrdivide(float A[42], const float B[36]);
extern void my_ang2quat(const float ang[3], float q[4]);
extern void my_calcH(float bx, float bz, float q0, float q1, float q2,
                     float q3, float H[42]);
extern void my_quat2att(const float q_data[4], const int q_size[2], float *roll,
                        float *pitch, float *yaw);
extern void my_quat2cbn(const float q[4], float cbn[9]);
extern void my_quatmul(const float q1[4], const float q2[4], float q[4]);
extern float norm(const float x[4]);



#endif	// __EKF_H
