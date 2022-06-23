#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                       Code generated with sympy 1.9                        *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_4899379868417862905) {
   out_4899379868417862905[0] = delta_x[0] + nom_x[0];
   out_4899379868417862905[1] = delta_x[1] + nom_x[1];
   out_4899379868417862905[2] = delta_x[2] + nom_x[2];
   out_4899379868417862905[3] = delta_x[3] + nom_x[3];
   out_4899379868417862905[4] = delta_x[4] + nom_x[4];
   out_4899379868417862905[5] = delta_x[5] + nom_x[5];
   out_4899379868417862905[6] = delta_x[6] + nom_x[6];
   out_4899379868417862905[7] = delta_x[7] + nom_x[7];
   out_4899379868417862905[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_3858087077367475170) {
   out_3858087077367475170[0] = -nom_x[0] + true_x[0];
   out_3858087077367475170[1] = -nom_x[1] + true_x[1];
   out_3858087077367475170[2] = -nom_x[2] + true_x[2];
   out_3858087077367475170[3] = -nom_x[3] + true_x[3];
   out_3858087077367475170[4] = -nom_x[4] + true_x[4];
   out_3858087077367475170[5] = -nom_x[5] + true_x[5];
   out_3858087077367475170[6] = -nom_x[6] + true_x[6];
   out_3858087077367475170[7] = -nom_x[7] + true_x[7];
   out_3858087077367475170[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_565708899702193861) {
   out_565708899702193861[0] = 1.0;
   out_565708899702193861[1] = 0;
   out_565708899702193861[2] = 0;
   out_565708899702193861[3] = 0;
   out_565708899702193861[4] = 0;
   out_565708899702193861[5] = 0;
   out_565708899702193861[6] = 0;
   out_565708899702193861[7] = 0;
   out_565708899702193861[8] = 0;
   out_565708899702193861[9] = 0;
   out_565708899702193861[10] = 1.0;
   out_565708899702193861[11] = 0;
   out_565708899702193861[12] = 0;
   out_565708899702193861[13] = 0;
   out_565708899702193861[14] = 0;
   out_565708899702193861[15] = 0;
   out_565708899702193861[16] = 0;
   out_565708899702193861[17] = 0;
   out_565708899702193861[18] = 0;
   out_565708899702193861[19] = 0;
   out_565708899702193861[20] = 1.0;
   out_565708899702193861[21] = 0;
   out_565708899702193861[22] = 0;
   out_565708899702193861[23] = 0;
   out_565708899702193861[24] = 0;
   out_565708899702193861[25] = 0;
   out_565708899702193861[26] = 0;
   out_565708899702193861[27] = 0;
   out_565708899702193861[28] = 0;
   out_565708899702193861[29] = 0;
   out_565708899702193861[30] = 1.0;
   out_565708899702193861[31] = 0;
   out_565708899702193861[32] = 0;
   out_565708899702193861[33] = 0;
   out_565708899702193861[34] = 0;
   out_565708899702193861[35] = 0;
   out_565708899702193861[36] = 0;
   out_565708899702193861[37] = 0;
   out_565708899702193861[38] = 0;
   out_565708899702193861[39] = 0;
   out_565708899702193861[40] = 1.0;
   out_565708899702193861[41] = 0;
   out_565708899702193861[42] = 0;
   out_565708899702193861[43] = 0;
   out_565708899702193861[44] = 0;
   out_565708899702193861[45] = 0;
   out_565708899702193861[46] = 0;
   out_565708899702193861[47] = 0;
   out_565708899702193861[48] = 0;
   out_565708899702193861[49] = 0;
   out_565708899702193861[50] = 1.0;
   out_565708899702193861[51] = 0;
   out_565708899702193861[52] = 0;
   out_565708899702193861[53] = 0;
   out_565708899702193861[54] = 0;
   out_565708899702193861[55] = 0;
   out_565708899702193861[56] = 0;
   out_565708899702193861[57] = 0;
   out_565708899702193861[58] = 0;
   out_565708899702193861[59] = 0;
   out_565708899702193861[60] = 1.0;
   out_565708899702193861[61] = 0;
   out_565708899702193861[62] = 0;
   out_565708899702193861[63] = 0;
   out_565708899702193861[64] = 0;
   out_565708899702193861[65] = 0;
   out_565708899702193861[66] = 0;
   out_565708899702193861[67] = 0;
   out_565708899702193861[68] = 0;
   out_565708899702193861[69] = 0;
   out_565708899702193861[70] = 1.0;
   out_565708899702193861[71] = 0;
   out_565708899702193861[72] = 0;
   out_565708899702193861[73] = 0;
   out_565708899702193861[74] = 0;
   out_565708899702193861[75] = 0;
   out_565708899702193861[76] = 0;
   out_565708899702193861[77] = 0;
   out_565708899702193861[78] = 0;
   out_565708899702193861[79] = 0;
   out_565708899702193861[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_8210882771349018177) {
   out_8210882771349018177[0] = state[0];
   out_8210882771349018177[1] = state[1];
   out_8210882771349018177[2] = state[2];
   out_8210882771349018177[3] = state[3];
   out_8210882771349018177[4] = state[4];
   out_8210882771349018177[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_8210882771349018177[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_8210882771349018177[7] = state[7];
   out_8210882771349018177[8] = state[8];
}
void F_fun(double *state, double dt, double *out_3432224512469045389) {
   out_3432224512469045389[0] = 1;
   out_3432224512469045389[1] = 0;
   out_3432224512469045389[2] = 0;
   out_3432224512469045389[3] = 0;
   out_3432224512469045389[4] = 0;
   out_3432224512469045389[5] = 0;
   out_3432224512469045389[6] = 0;
   out_3432224512469045389[7] = 0;
   out_3432224512469045389[8] = 0;
   out_3432224512469045389[9] = 0;
   out_3432224512469045389[10] = 1;
   out_3432224512469045389[11] = 0;
   out_3432224512469045389[12] = 0;
   out_3432224512469045389[13] = 0;
   out_3432224512469045389[14] = 0;
   out_3432224512469045389[15] = 0;
   out_3432224512469045389[16] = 0;
   out_3432224512469045389[17] = 0;
   out_3432224512469045389[18] = 0;
   out_3432224512469045389[19] = 0;
   out_3432224512469045389[20] = 1;
   out_3432224512469045389[21] = 0;
   out_3432224512469045389[22] = 0;
   out_3432224512469045389[23] = 0;
   out_3432224512469045389[24] = 0;
   out_3432224512469045389[25] = 0;
   out_3432224512469045389[26] = 0;
   out_3432224512469045389[27] = 0;
   out_3432224512469045389[28] = 0;
   out_3432224512469045389[29] = 0;
   out_3432224512469045389[30] = 1;
   out_3432224512469045389[31] = 0;
   out_3432224512469045389[32] = 0;
   out_3432224512469045389[33] = 0;
   out_3432224512469045389[34] = 0;
   out_3432224512469045389[35] = 0;
   out_3432224512469045389[36] = 0;
   out_3432224512469045389[37] = 0;
   out_3432224512469045389[38] = 0;
   out_3432224512469045389[39] = 0;
   out_3432224512469045389[40] = 1;
   out_3432224512469045389[41] = 0;
   out_3432224512469045389[42] = 0;
   out_3432224512469045389[43] = 0;
   out_3432224512469045389[44] = 0;
   out_3432224512469045389[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_3432224512469045389[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_3432224512469045389[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3432224512469045389[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3432224512469045389[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_3432224512469045389[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_3432224512469045389[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_3432224512469045389[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_3432224512469045389[53] = -9.8000000000000007*dt;
   out_3432224512469045389[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_3432224512469045389[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_3432224512469045389[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3432224512469045389[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3432224512469045389[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_3432224512469045389[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_3432224512469045389[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_3432224512469045389[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3432224512469045389[62] = 0;
   out_3432224512469045389[63] = 0;
   out_3432224512469045389[64] = 0;
   out_3432224512469045389[65] = 0;
   out_3432224512469045389[66] = 0;
   out_3432224512469045389[67] = 0;
   out_3432224512469045389[68] = 0;
   out_3432224512469045389[69] = 0;
   out_3432224512469045389[70] = 1;
   out_3432224512469045389[71] = 0;
   out_3432224512469045389[72] = 0;
   out_3432224512469045389[73] = 0;
   out_3432224512469045389[74] = 0;
   out_3432224512469045389[75] = 0;
   out_3432224512469045389[76] = 0;
   out_3432224512469045389[77] = 0;
   out_3432224512469045389[78] = 0;
   out_3432224512469045389[79] = 0;
   out_3432224512469045389[80] = 1;
}
void h_25(double *state, double *unused, double *out_6485200509708027430) {
   out_6485200509708027430[0] = state[6];
}
void H_25(double *state, double *unused, double *out_7060147963260463716) {
   out_7060147963260463716[0] = 0;
   out_7060147963260463716[1] = 0;
   out_7060147963260463716[2] = 0;
   out_7060147963260463716[3] = 0;
   out_7060147963260463716[4] = 0;
   out_7060147963260463716[5] = 0;
   out_7060147963260463716[6] = 1;
   out_7060147963260463716[7] = 0;
   out_7060147963260463716[8] = 0;
}
void h_24(double *state, double *unused, double *out_5991557838608407533) {
   out_5991557838608407533[0] = state[4];
   out_5991557838608407533[1] = state[5];
}
void H_24(double *state, double *unused, double *out_7828670511646764764) {
   out_7828670511646764764[0] = 0;
   out_7828670511646764764[1] = 0;
   out_7828670511646764764[2] = 0;
   out_7828670511646764764[3] = 0;
   out_7828670511646764764[4] = 1;
   out_7828670511646764764[5] = 0;
   out_7828670511646764764[6] = 0;
   out_7828670511646764764[7] = 0;
   out_7828670511646764764[8] = 0;
   out_7828670511646764764[9] = 0;
   out_7828670511646764764[10] = 0;
   out_7828670511646764764[11] = 0;
   out_7828670511646764764[12] = 0;
   out_7828670511646764764[13] = 0;
   out_7828670511646764764[14] = 1;
   out_7828670511646764764[15] = 0;
   out_7828670511646764764[16] = 0;
   out_7828670511646764764[17] = 0;
}
void h_30(double *state, double *unused, double *out_5190708337651173250) {
   out_5190708337651173250[0] = state[4];
}
void H_30(double *state, double *unused, double *out_8868263151941839273) {
   out_8868263151941839273[0] = 0;
   out_8868263151941839273[1] = 0;
   out_8868263151941839273[2] = 0;
   out_8868263151941839273[3] = 0;
   out_8868263151941839273[4] = 1;
   out_8868263151941839273[5] = 0;
   out_8868263151941839273[6] = 0;
   out_8868263151941839273[7] = 0;
   out_8868263151941839273[8] = 0;
}
void h_26(double *state, double *unused, double *out_6881156172386854210) {
   out_6881156172386854210[0] = state[7];
}
void H_26(double *state, double *unused, double *out_8082070140688287299) {
   out_8082070140688287299[0] = 0;
   out_8082070140688287299[1] = 0;
   out_8082070140688287299[2] = 0;
   out_8082070140688287299[3] = 0;
   out_8082070140688287299[4] = 0;
   out_8082070140688287299[5] = 0;
   out_8082070140688287299[6] = 0;
   out_8082070140688287299[7] = 1;
   out_8082070140688287299[8] = 0;
}
void h_27(double *state, double *unused, double *out_1483527242047181785) {
   out_1483527242047181785[0] = state[3];
}
void H_27(double *state, double *unused, double *out_6644669080757896056) {
   out_6644669080757896056[0] = 0;
   out_6644669080757896056[1] = 0;
   out_6644669080757896056[2] = 0;
   out_6644669080757896056[3] = 1;
   out_6644669080757896056[4] = 0;
   out_6644669080757896056[5] = 0;
   out_6644669080757896056[6] = 0;
   out_6644669080757896056[7] = 0;
   out_6644669080757896056[8] = 0;
}
void h_29(double *state, double *unused, double *out_2153922286206840565) {
   out_2153922286206840565[0] = state[1];
}
void H_29(double *state, double *unused, double *out_8358031807627447089) {
   out_8358031807627447089[0] = 0;
   out_8358031807627447089[1] = 1;
   out_8358031807627447089[2] = 0;
   out_8358031807627447089[3] = 0;
   out_8358031807627447089[4] = 0;
   out_8358031807627447089[5] = 0;
   out_8358031807627447089[6] = 0;
   out_8358031807627447089[7] = 0;
   out_8358031807627447089[8] = 0;
}
void h_28(double *state, double *unused, double *out_2427037094948844220) {
   out_2427037094948844220[0] = state[0];
}
void H_28(double *state, double *unused, double *out_5006313249012573953) {
   out_5006313249012573953[0] = 1;
   out_5006313249012573953[1] = 0;
   out_5006313249012573953[2] = 0;
   out_5006313249012573953[3] = 0;
   out_5006313249012573953[4] = 0;
   out_5006313249012573953[5] = 0;
   out_5006313249012573953[6] = 0;
   out_5006313249012573953[7] = 0;
   out_5006313249012573953[8] = 0;
}
void h_31(double *state, double *unused, double *out_8324094035747501836) {
   out_8324094035747501836[0] = state[8];
}
void H_31(double *state, double *unused, double *out_2692436542153056016) {
   out_2692436542153056016[0] = 0;
   out_2692436542153056016[1] = 0;
   out_2692436542153056016[2] = 0;
   out_2692436542153056016[3] = 0;
   out_2692436542153056016[4] = 0;
   out_2692436542153056016[5] = 0;
   out_2692436542153056016[6] = 0;
   out_2692436542153056016[7] = 0;
   out_2692436542153056016[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_4899379868417862905) {
  err_fun(nom_x, delta_x, out_4899379868417862905);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_3858087077367475170) {
  inv_err_fun(nom_x, true_x, out_3858087077367475170);
}
void car_H_mod_fun(double *state, double *out_565708899702193861) {
  H_mod_fun(state, out_565708899702193861);
}
void car_f_fun(double *state, double dt, double *out_8210882771349018177) {
  f_fun(state,  dt, out_8210882771349018177);
}
void car_F_fun(double *state, double dt, double *out_3432224512469045389) {
  F_fun(state,  dt, out_3432224512469045389);
}
void car_h_25(double *state, double *unused, double *out_6485200509708027430) {
  h_25(state, unused, out_6485200509708027430);
}
void car_H_25(double *state, double *unused, double *out_7060147963260463716) {
  H_25(state, unused, out_7060147963260463716);
}
void car_h_24(double *state, double *unused, double *out_5991557838608407533) {
  h_24(state, unused, out_5991557838608407533);
}
void car_H_24(double *state, double *unused, double *out_7828670511646764764) {
  H_24(state, unused, out_7828670511646764764);
}
void car_h_30(double *state, double *unused, double *out_5190708337651173250) {
  h_30(state, unused, out_5190708337651173250);
}
void car_H_30(double *state, double *unused, double *out_8868263151941839273) {
  H_30(state, unused, out_8868263151941839273);
}
void car_h_26(double *state, double *unused, double *out_6881156172386854210) {
  h_26(state, unused, out_6881156172386854210);
}
void car_H_26(double *state, double *unused, double *out_8082070140688287299) {
  H_26(state, unused, out_8082070140688287299);
}
void car_h_27(double *state, double *unused, double *out_1483527242047181785) {
  h_27(state, unused, out_1483527242047181785);
}
void car_H_27(double *state, double *unused, double *out_6644669080757896056) {
  H_27(state, unused, out_6644669080757896056);
}
void car_h_29(double *state, double *unused, double *out_2153922286206840565) {
  h_29(state, unused, out_2153922286206840565);
}
void car_H_29(double *state, double *unused, double *out_8358031807627447089) {
  H_29(state, unused, out_8358031807627447089);
}
void car_h_28(double *state, double *unused, double *out_2427037094948844220) {
  h_28(state, unused, out_2427037094948844220);
}
void car_H_28(double *state, double *unused, double *out_5006313249012573953) {
  H_28(state, unused, out_5006313249012573953);
}
void car_h_31(double *state, double *unused, double *out_8324094035747501836) {
  h_31(state, unused, out_8324094035747501836);
}
void car_H_31(double *state, double *unused, double *out_2692436542153056016) {
  H_31(state, unused, out_2692436542153056016);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_init(car);
