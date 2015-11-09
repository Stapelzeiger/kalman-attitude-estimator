// This file has been automatically generated
// DO NOT EDIT!

#include <math.h>

const int STATE_DIM = 4;
const int CONTROL_DIM = 3;
const int MEASURE_DIM = 3;



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

   h_out[0] = -19.62*in0_x[2]*in0_x[0] + 19.62*in0_x[3]*in0_x[1];
   h_out[1] = 19.62*in0_x[1]*in0_x[0] + 19.62*in0_x[3]*in0_x[2];
   h_out[2] = 9.81*powf(in0_x[0], 2) - 9.81*powf(in0_x[1], 2) - 9.81*powf(in0_x[2], 2) + 9.81*powf(in0_x[3], 2);

}

static inline void H(const float *in0_x, float *H_out) {

   H_out[0] = -19.62*in0_x[2];
   H_out[1] = 19.62*in0_x[1];
   H_out[2] = 19.62*in0_x[0];
   H_out[3] = 19.62*in0_x[3];
   H_out[4] = 19.62*in0_x[0];
   H_out[5] = -19.62*in0_x[1];
   H_out[6] = -19.62*in0_x[0];
   H_out[7] = 19.62*in0_x[3];
   H_out[8] = -19.62*in0_x[2];
   H_out[9] = 19.62*in0_x[1];
   H_out[10] = 19.62*in0_x[2];
   H_out[11] = 19.62*in0_x[3];

}
