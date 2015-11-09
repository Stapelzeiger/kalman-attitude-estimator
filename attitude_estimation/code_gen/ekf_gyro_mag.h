// This file has been automatically generated
// DO NOT EDIT!

#include <math.h>

const int STATE_DIM = 4;
const int CONTROL_DIM = 3;
const int MEASURE_DIM = 1;



static inline void f(const float *in0_x, const float *in1_u, float in2_Delta_t, float *f_out) {

   f_out[0] = in2_Delta_t*(-0.5*in1_u[0]*in0_x[1] - 0.5*in1_u[1]*in0_x[2] - 0.5*in1_u[2]*in0_x[3]) + in0_x[0];
   f_out[1] = in2_Delta_t*(0.5*in1_u[0]*in0_x[0] - 0.5*in1_u[1]*in0_x[3] + 0.5*in1_u[2]*in0_x[2]) + in0_x[1];
   f_out[2] = in2_Delta_t*(0.5*in1_u[0]*in0_x[3] + 0.5*in1_u[1]*in0_x[0] - 0.5*in1_u[2]*in0_x[1]) + in0_x[2];
   f_out[3] = in2_Delta_t*(-0.5*in1_u[0]*in0_x[2] + 0.5*in1_u[1]*in0_x[1] + 0.5*in1_u[2]*in0_x[0]) + in0_x[3];

}

static inline void F(const float *in0_x, const float *in1_u, float in2_Delta_t, float *F_out) {

   F_out[0] = 1;
   F_out[1] = 0.5*in2_Delta_t*in1_u[0];
   F_out[2] = 0.5*in2_Delta_t*in1_u[1];
   F_out[3] = 0.5*in2_Delta_t*in1_u[2];
   F_out[4] = -0.5*in2_Delta_t*in1_u[0];
   F_out[5] = 1;
   F_out[6] = -0.5*in2_Delta_t*in1_u[2];
   F_out[7] = 0.5*in2_Delta_t*in1_u[1];
   F_out[8] = -0.5*in2_Delta_t*in1_u[1];
   F_out[9] = 0.5*in2_Delta_t*in1_u[2];
   F_out[10] = 1;
   F_out[11] = -0.5*in2_Delta_t*in1_u[0];
   F_out[12] = -0.5*in2_Delta_t*in1_u[2];
   F_out[13] = -0.5*in2_Delta_t*in1_u[1];
   F_out[14] = 0.5*in2_Delta_t*in1_u[0];
   F_out[15] = 1;

}

static inline void h(const float *in0_x, float *h_out) {

   h_out[0] = 0;

}

static inline void H(const float *in0_x, float *H_out) {

   H_out[0] = 2*in0_x[3];
   H_out[1] = 2*in0_x[2];
   H_out[2] = -2*in0_x[1];
   H_out[3] = -2*in0_x[0];

}
static inline void meas_transf(float q0, float q1, float q2, float q3, float z0, float z1, float z2, float *out) {
   out[0] = atan2f(q0*(q0*z1 - q1*z2 + q3*z0) - q1*(q0*z2 + q1*z1 - q2*z0) - q2*(-q1*z0 - q2*z1 - q3*z2) + q3*(q0*z0 + q2*z2 - q3*z1), q0*(q0*z0 + q2*z2 - q3*z1) - q1*(-q1*z0 - q2*z1 - q3*z2) + q2*(q0*z2 + q1*z1 - q2*z0) - q3*(q0*z1 - q1*z2 + q3*z0));
}
static inline void z_inertial(float q0, float q1, float q2, float q3, float z0, float z1, float z2, float *out) {
   out[0] = q0*(q0*z0 + q2*z2 - q3*z1) - q1*(-q1*z0 - q2*z1 - q3*z2) + q2*(q0*z2 + q1*z1 - q2*z0) - q3*(q0*z1 - q1*z2 + q3*z0);
   out[1] = q0*(q0*z1 - q1*z2 + q3*z0) - q1*(q0*z2 + q1*z1 - q2*z0) - q2*(-q1*z0 - q2*z1 - q3*z2) + q3*(q0*z0 + q2*z2 - q3*z1);
   out[2] = q0*(q0*z2 + q1*z1 - q2*z0) + q1*(q0*z1 - q1*z2 + q3*z0) - q2*(q0*z0 + q2*z2 - q3*z1) - q3*(-q1*z0 - q2*z1 - q3*z2);
}
