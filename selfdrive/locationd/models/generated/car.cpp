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
void err_fun(double *nom_x, double *delta_x, double *out_8410643270261249886) {
   out_8410643270261249886[0] = delta_x[0] + nom_x[0];
   out_8410643270261249886[1] = delta_x[1] + nom_x[1];
   out_8410643270261249886[2] = delta_x[2] + nom_x[2];
   out_8410643270261249886[3] = delta_x[3] + nom_x[3];
   out_8410643270261249886[4] = delta_x[4] + nom_x[4];
   out_8410643270261249886[5] = delta_x[5] + nom_x[5];
   out_8410643270261249886[6] = delta_x[6] + nom_x[6];
   out_8410643270261249886[7] = delta_x[7] + nom_x[7];
   out_8410643270261249886[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_2170846217420803909) {
   out_2170846217420803909[0] = -nom_x[0] + true_x[0];
   out_2170846217420803909[1] = -nom_x[1] + true_x[1];
   out_2170846217420803909[2] = -nom_x[2] + true_x[2];
   out_2170846217420803909[3] = -nom_x[3] + true_x[3];
   out_2170846217420803909[4] = -nom_x[4] + true_x[4];
   out_2170846217420803909[5] = -nom_x[5] + true_x[5];
   out_2170846217420803909[6] = -nom_x[6] + true_x[6];
   out_2170846217420803909[7] = -nom_x[7] + true_x[7];
   out_2170846217420803909[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_5654866458334913771) {
   out_5654866458334913771[0] = 1.0;
   out_5654866458334913771[1] = 0;
   out_5654866458334913771[2] = 0;
   out_5654866458334913771[3] = 0;
   out_5654866458334913771[4] = 0;
   out_5654866458334913771[5] = 0;
   out_5654866458334913771[6] = 0;
   out_5654866458334913771[7] = 0;
   out_5654866458334913771[8] = 0;
   out_5654866458334913771[9] = 0;
   out_5654866458334913771[10] = 1.0;
   out_5654866458334913771[11] = 0;
   out_5654866458334913771[12] = 0;
   out_5654866458334913771[13] = 0;
   out_5654866458334913771[14] = 0;
   out_5654866458334913771[15] = 0;
   out_5654866458334913771[16] = 0;
   out_5654866458334913771[17] = 0;
   out_5654866458334913771[18] = 0;
   out_5654866458334913771[19] = 0;
   out_5654866458334913771[20] = 1.0;
   out_5654866458334913771[21] = 0;
   out_5654866458334913771[22] = 0;
   out_5654866458334913771[23] = 0;
   out_5654866458334913771[24] = 0;
   out_5654866458334913771[25] = 0;
   out_5654866458334913771[26] = 0;
   out_5654866458334913771[27] = 0;
   out_5654866458334913771[28] = 0;
   out_5654866458334913771[29] = 0;
   out_5654866458334913771[30] = 1.0;
   out_5654866458334913771[31] = 0;
   out_5654866458334913771[32] = 0;
   out_5654866458334913771[33] = 0;
   out_5654866458334913771[34] = 0;
   out_5654866458334913771[35] = 0;
   out_5654866458334913771[36] = 0;
   out_5654866458334913771[37] = 0;
   out_5654866458334913771[38] = 0;
   out_5654866458334913771[39] = 0;
   out_5654866458334913771[40] = 1.0;
   out_5654866458334913771[41] = 0;
   out_5654866458334913771[42] = 0;
   out_5654866458334913771[43] = 0;
   out_5654866458334913771[44] = 0;
   out_5654866458334913771[45] = 0;
   out_5654866458334913771[46] = 0;
   out_5654866458334913771[47] = 0;
   out_5654866458334913771[48] = 0;
   out_5654866458334913771[49] = 0;
   out_5654866458334913771[50] = 1.0;
   out_5654866458334913771[51] = 0;
   out_5654866458334913771[52] = 0;
   out_5654866458334913771[53] = 0;
   out_5654866458334913771[54] = 0;
   out_5654866458334913771[55] = 0;
   out_5654866458334913771[56] = 0;
   out_5654866458334913771[57] = 0;
   out_5654866458334913771[58] = 0;
   out_5654866458334913771[59] = 0;
   out_5654866458334913771[60] = 1.0;
   out_5654866458334913771[61] = 0;
   out_5654866458334913771[62] = 0;
   out_5654866458334913771[63] = 0;
   out_5654866458334913771[64] = 0;
   out_5654866458334913771[65] = 0;
   out_5654866458334913771[66] = 0;
   out_5654866458334913771[67] = 0;
   out_5654866458334913771[68] = 0;
   out_5654866458334913771[69] = 0;
   out_5654866458334913771[70] = 1.0;
   out_5654866458334913771[71] = 0;
   out_5654866458334913771[72] = 0;
   out_5654866458334913771[73] = 0;
   out_5654866458334913771[74] = 0;
   out_5654866458334913771[75] = 0;
   out_5654866458334913771[76] = 0;
   out_5654866458334913771[77] = 0;
   out_5654866458334913771[78] = 0;
   out_5654866458334913771[79] = 0;
   out_5654866458334913771[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_5195591810852336672) {
   out_5195591810852336672[0] = state[0];
   out_5195591810852336672[1] = state[1];
   out_5195591810852336672[2] = state[2];
   out_5195591810852336672[3] = state[3];
   out_5195591810852336672[4] = state[4];
   out_5195591810852336672[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_5195591810852336672[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_5195591810852336672[7] = state[7];
   out_5195591810852336672[8] = state[8];
}
void F_fun(double *state, double dt, double *out_7896467353570579206) {
   out_7896467353570579206[0] = 1;
   out_7896467353570579206[1] = 0;
   out_7896467353570579206[2] = 0;
   out_7896467353570579206[3] = 0;
   out_7896467353570579206[4] = 0;
   out_7896467353570579206[5] = 0;
   out_7896467353570579206[6] = 0;
   out_7896467353570579206[7] = 0;
   out_7896467353570579206[8] = 0;
   out_7896467353570579206[9] = 0;
   out_7896467353570579206[10] = 1;
   out_7896467353570579206[11] = 0;
   out_7896467353570579206[12] = 0;
   out_7896467353570579206[13] = 0;
   out_7896467353570579206[14] = 0;
   out_7896467353570579206[15] = 0;
   out_7896467353570579206[16] = 0;
   out_7896467353570579206[17] = 0;
   out_7896467353570579206[18] = 0;
   out_7896467353570579206[19] = 0;
   out_7896467353570579206[20] = 1;
   out_7896467353570579206[21] = 0;
   out_7896467353570579206[22] = 0;
   out_7896467353570579206[23] = 0;
   out_7896467353570579206[24] = 0;
   out_7896467353570579206[25] = 0;
   out_7896467353570579206[26] = 0;
   out_7896467353570579206[27] = 0;
   out_7896467353570579206[28] = 0;
   out_7896467353570579206[29] = 0;
   out_7896467353570579206[30] = 1;
   out_7896467353570579206[31] = 0;
   out_7896467353570579206[32] = 0;
   out_7896467353570579206[33] = 0;
   out_7896467353570579206[34] = 0;
   out_7896467353570579206[35] = 0;
   out_7896467353570579206[36] = 0;
   out_7896467353570579206[37] = 0;
   out_7896467353570579206[38] = 0;
   out_7896467353570579206[39] = 0;
   out_7896467353570579206[40] = 1;
   out_7896467353570579206[41] = 0;
   out_7896467353570579206[42] = 0;
   out_7896467353570579206[43] = 0;
   out_7896467353570579206[44] = 0;
   out_7896467353570579206[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_7896467353570579206[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_7896467353570579206[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7896467353570579206[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7896467353570579206[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_7896467353570579206[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_7896467353570579206[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_7896467353570579206[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_7896467353570579206[53] = -9.8000000000000007*dt;
   out_7896467353570579206[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_7896467353570579206[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_7896467353570579206[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7896467353570579206[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7896467353570579206[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_7896467353570579206[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_7896467353570579206[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_7896467353570579206[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7896467353570579206[62] = 0;
   out_7896467353570579206[63] = 0;
   out_7896467353570579206[64] = 0;
   out_7896467353570579206[65] = 0;
   out_7896467353570579206[66] = 0;
   out_7896467353570579206[67] = 0;
   out_7896467353570579206[68] = 0;
   out_7896467353570579206[69] = 0;
   out_7896467353570579206[70] = 1;
   out_7896467353570579206[71] = 0;
   out_7896467353570579206[72] = 0;
   out_7896467353570579206[73] = 0;
   out_7896467353570579206[74] = 0;
   out_7896467353570579206[75] = 0;
   out_7896467353570579206[76] = 0;
   out_7896467353570579206[77] = 0;
   out_7896467353570579206[78] = 0;
   out_7896467353570579206[79] = 0;
   out_7896467353570579206[80] = 1;
}
void h_25(double *state, double *unused, double *out_4670350168092863626) {
   out_4670350168092863626[0] = state[6];
}
void H_25(double *state, double *unused, double *out_1965496603878225476) {
   out_1965496603878225476[0] = 0;
   out_1965496603878225476[1] = 0;
   out_1965496603878225476[2] = 0;
   out_1965496603878225476[3] = 0;
   out_1965496603878225476[4] = 0;
   out_1965496603878225476[5] = 0;
   out_1965496603878225476[6] = 1;
   out_1965496603878225476[7] = 0;
   out_1965496603878225476[8] = 0;
}
void h_24(double *state, double *unused, double *out_4805896676923701039) {
   out_4805896676923701039[0] = state[4];
   out_4805896676923701039[1] = state[5];
}
void H_24(double *state, double *unused, double *out_4138146202883725042) {
   out_4138146202883725042[0] = 0;
   out_4138146202883725042[1] = 0;
   out_4138146202883725042[2] = 0;
   out_4138146202883725042[3] = 0;
   out_4138146202883725042[4] = 1;
   out_4138146202883725042[5] = 0;
   out_4138146202883725042[6] = 0;
   out_4138146202883725042[7] = 0;
   out_4138146202883725042[8] = 0;
   out_4138146202883725042[9] = 0;
   out_4138146202883725042[10] = 0;
   out_4138146202883725042[11] = 0;
   out_4138146202883725042[12] = 0;
   out_4138146202883725042[13] = 0;
   out_4138146202883725042[14] = 1;
   out_4138146202883725042[15] = 0;
   out_4138146202883725042[16] = 0;
   out_4138146202883725042[17] = 0;
}
void h_30(double *state, double *unused, double *out_7005558679266337054) {
   out_7005558679266337054[0] = state[4];
}
void H_30(double *state, double *unused, double *out_4951193737613391279) {
   out_4951193737613391279[0] = 0;
   out_4951193737613391279[1] = 0;
   out_4951193737613391279[2] = 0;
   out_4951193737613391279[3] = 0;
   out_4951193737613391279[4] = 1;
   out_4951193737613391279[5] = 0;
   out_4951193737613391279[6] = 0;
   out_4951193737613391279[7] = 0;
   out_4951193737613391279[8] = 0;
}
void h_26(double *state, double *unused, double *out_1790750784999660086) {
   out_1790750784999660086[0] = state[7];
}
void H_26(double *state, double *unused, double *out_5706999922752281700) {
   out_5706999922752281700[0] = 0;
   out_5706999922752281700[1] = 0;
   out_5706999922752281700[2] = 0;
   out_5706999922752281700[3] = 0;
   out_5706999922752281700[4] = 0;
   out_5706999922752281700[5] = 0;
   out_5706999922752281700[6] = 0;
   out_5706999922752281700[7] = 1;
   out_5706999922752281700[8] = 0;
}
void h_27(double *state, double *unused, double *out_474539895022783911) {
   out_474539895022783911[0] = state[3];
}
void H_27(double *state, double *unused, double *out_2776430425812966368) {
   out_2776430425812966368[0] = 0;
   out_2776430425812966368[1] = 0;
   out_2776430425812966368[2] = 0;
   out_2776430425812966368[3] = 1;
   out_2776430425812966368[4] = 0;
   out_2776430425812966368[5] = 0;
   out_2776430425812966368[6] = 0;
   out_2776430425812966368[7] = 0;
   out_2776430425812966368[8] = 0;
}
void h_29(double *state, double *unused, double *out_7608874563942814571) {
   out_7608874563942814571[0] = state[1];
}
void H_29(double *state, double *unused, double *out_1063067698943415335) {
   out_1063067698943415335[0] = 0;
   out_1063067698943415335[1] = 1;
   out_1063067698943415335[2] = 0;
   out_1063067698943415335[3] = 0;
   out_1063067698943415335[4] = 0;
   out_1063067698943415335[5] = 0;
   out_1063067698943415335[6] = 0;
   out_1063067698943415335[7] = 0;
   out_1063067698943415335[8] = 0;
}
void h_28(double *state, double *unused, double *out_5010544136318048544) {
   out_5010544136318048544[0] = state[0];
}
void H_28(double *state, double *unused, double *out_4019331318126115239) {
   out_4019331318126115239[0] = 1;
   out_4019331318126115239[1] = 0;
   out_4019331318126115239[2] = 0;
   out_4019331318126115239[3] = 0;
   out_4019331318126115239[4] = 0;
   out_4019331318126115239[5] = 0;
   out_4019331318126115239[6] = 0;
   out_4019331318126115239[7] = 0;
   out_4019331318126115239[8] = 0;
}
void h_31(double *state, double *unused, double *out_3092915088727808815) {
   out_3092915088727808815[0] = state[8];
}
void H_31(double *state, double *unused, double *out_1934850642001265048) {
   out_1934850642001265048[0] = 0;
   out_1934850642001265048[1] = 0;
   out_1934850642001265048[2] = 0;
   out_1934850642001265048[3] = 0;
   out_1934850642001265048[4] = 0;
   out_1934850642001265048[5] = 0;
   out_1934850642001265048[6] = 0;
   out_1934850642001265048[7] = 0;
   out_1934850642001265048[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_8410643270261249886) {
  err_fun(nom_x, delta_x, out_8410643270261249886);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_2170846217420803909) {
  inv_err_fun(nom_x, true_x, out_2170846217420803909);
}
void car_H_mod_fun(double *state, double *out_5654866458334913771) {
  H_mod_fun(state, out_5654866458334913771);
}
void car_f_fun(double *state, double dt, double *out_5195591810852336672) {
  f_fun(state,  dt, out_5195591810852336672);
}
void car_F_fun(double *state, double dt, double *out_7896467353570579206) {
  F_fun(state,  dt, out_7896467353570579206);
}
void car_h_25(double *state, double *unused, double *out_4670350168092863626) {
  h_25(state, unused, out_4670350168092863626);
}
void car_H_25(double *state, double *unused, double *out_1965496603878225476) {
  H_25(state, unused, out_1965496603878225476);
}
void car_h_24(double *state, double *unused, double *out_4805896676923701039) {
  h_24(state, unused, out_4805896676923701039);
}
void car_H_24(double *state, double *unused, double *out_4138146202883725042) {
  H_24(state, unused, out_4138146202883725042);
}
void car_h_30(double *state, double *unused, double *out_7005558679266337054) {
  h_30(state, unused, out_7005558679266337054);
}
void car_H_30(double *state, double *unused, double *out_4951193737613391279) {
  H_30(state, unused, out_4951193737613391279);
}
void car_h_26(double *state, double *unused, double *out_1790750784999660086) {
  h_26(state, unused, out_1790750784999660086);
}
void car_H_26(double *state, double *unused, double *out_5706999922752281700) {
  H_26(state, unused, out_5706999922752281700);
}
void car_h_27(double *state, double *unused, double *out_474539895022783911) {
  h_27(state, unused, out_474539895022783911);
}
void car_H_27(double *state, double *unused, double *out_2776430425812966368) {
  H_27(state, unused, out_2776430425812966368);
}
void car_h_29(double *state, double *unused, double *out_7608874563942814571) {
  h_29(state, unused, out_7608874563942814571);
}
void car_H_29(double *state, double *unused, double *out_1063067698943415335) {
  H_29(state, unused, out_1063067698943415335);
}
void car_h_28(double *state, double *unused, double *out_5010544136318048544) {
  h_28(state, unused, out_5010544136318048544);
}
void car_H_28(double *state, double *unused, double *out_4019331318126115239) {
  H_28(state, unused, out_4019331318126115239);
}
void car_h_31(double *state, double *unused, double *out_3092915088727808815) {
  h_31(state, unused, out_3092915088727808815);
}
void car_H_31(double *state, double *unused, double *out_1934850642001265048) {
  H_31(state, unused, out_1934850642001265048);
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
