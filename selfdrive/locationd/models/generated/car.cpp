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
void err_fun(double *nom_x, double *delta_x, double *out_7831334908946486631) {
   out_7831334908946486631[0] = delta_x[0] + nom_x[0];
   out_7831334908946486631[1] = delta_x[1] + nom_x[1];
   out_7831334908946486631[2] = delta_x[2] + nom_x[2];
   out_7831334908946486631[3] = delta_x[3] + nom_x[3];
   out_7831334908946486631[4] = delta_x[4] + nom_x[4];
   out_7831334908946486631[5] = delta_x[5] + nom_x[5];
   out_7831334908946486631[6] = delta_x[6] + nom_x[6];
   out_7831334908946486631[7] = delta_x[7] + nom_x[7];
   out_7831334908946486631[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_1412484805370031907) {
   out_1412484805370031907[0] = -nom_x[0] + true_x[0];
   out_1412484805370031907[1] = -nom_x[1] + true_x[1];
   out_1412484805370031907[2] = -nom_x[2] + true_x[2];
   out_1412484805370031907[3] = -nom_x[3] + true_x[3];
   out_1412484805370031907[4] = -nom_x[4] + true_x[4];
   out_1412484805370031907[5] = -nom_x[5] + true_x[5];
   out_1412484805370031907[6] = -nom_x[6] + true_x[6];
   out_1412484805370031907[7] = -nom_x[7] + true_x[7];
   out_1412484805370031907[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_722397761437678664) {
   out_722397761437678664[0] = 1.0;
   out_722397761437678664[1] = 0;
   out_722397761437678664[2] = 0;
   out_722397761437678664[3] = 0;
   out_722397761437678664[4] = 0;
   out_722397761437678664[5] = 0;
   out_722397761437678664[6] = 0;
   out_722397761437678664[7] = 0;
   out_722397761437678664[8] = 0;
   out_722397761437678664[9] = 0;
   out_722397761437678664[10] = 1.0;
   out_722397761437678664[11] = 0;
   out_722397761437678664[12] = 0;
   out_722397761437678664[13] = 0;
   out_722397761437678664[14] = 0;
   out_722397761437678664[15] = 0;
   out_722397761437678664[16] = 0;
   out_722397761437678664[17] = 0;
   out_722397761437678664[18] = 0;
   out_722397761437678664[19] = 0;
   out_722397761437678664[20] = 1.0;
   out_722397761437678664[21] = 0;
   out_722397761437678664[22] = 0;
   out_722397761437678664[23] = 0;
   out_722397761437678664[24] = 0;
   out_722397761437678664[25] = 0;
   out_722397761437678664[26] = 0;
   out_722397761437678664[27] = 0;
   out_722397761437678664[28] = 0;
   out_722397761437678664[29] = 0;
   out_722397761437678664[30] = 1.0;
   out_722397761437678664[31] = 0;
   out_722397761437678664[32] = 0;
   out_722397761437678664[33] = 0;
   out_722397761437678664[34] = 0;
   out_722397761437678664[35] = 0;
   out_722397761437678664[36] = 0;
   out_722397761437678664[37] = 0;
   out_722397761437678664[38] = 0;
   out_722397761437678664[39] = 0;
   out_722397761437678664[40] = 1.0;
   out_722397761437678664[41] = 0;
   out_722397761437678664[42] = 0;
   out_722397761437678664[43] = 0;
   out_722397761437678664[44] = 0;
   out_722397761437678664[45] = 0;
   out_722397761437678664[46] = 0;
   out_722397761437678664[47] = 0;
   out_722397761437678664[48] = 0;
   out_722397761437678664[49] = 0;
   out_722397761437678664[50] = 1.0;
   out_722397761437678664[51] = 0;
   out_722397761437678664[52] = 0;
   out_722397761437678664[53] = 0;
   out_722397761437678664[54] = 0;
   out_722397761437678664[55] = 0;
   out_722397761437678664[56] = 0;
   out_722397761437678664[57] = 0;
   out_722397761437678664[58] = 0;
   out_722397761437678664[59] = 0;
   out_722397761437678664[60] = 1.0;
   out_722397761437678664[61] = 0;
   out_722397761437678664[62] = 0;
   out_722397761437678664[63] = 0;
   out_722397761437678664[64] = 0;
   out_722397761437678664[65] = 0;
   out_722397761437678664[66] = 0;
   out_722397761437678664[67] = 0;
   out_722397761437678664[68] = 0;
   out_722397761437678664[69] = 0;
   out_722397761437678664[70] = 1.0;
   out_722397761437678664[71] = 0;
   out_722397761437678664[72] = 0;
   out_722397761437678664[73] = 0;
   out_722397761437678664[74] = 0;
   out_722397761437678664[75] = 0;
   out_722397761437678664[76] = 0;
   out_722397761437678664[77] = 0;
   out_722397761437678664[78] = 0;
   out_722397761437678664[79] = 0;
   out_722397761437678664[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_7415195887102028552) {
   out_7415195887102028552[0] = state[0];
   out_7415195887102028552[1] = state[1];
   out_7415195887102028552[2] = state[2];
   out_7415195887102028552[3] = state[3];
   out_7415195887102028552[4] = state[4];
   out_7415195887102028552[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_7415195887102028552[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_7415195887102028552[7] = state[7];
   out_7415195887102028552[8] = state[8];
}
void F_fun(double *state, double dt, double *out_7715249174232332377) {
   out_7715249174232332377[0] = 1;
   out_7715249174232332377[1] = 0;
   out_7715249174232332377[2] = 0;
   out_7715249174232332377[3] = 0;
   out_7715249174232332377[4] = 0;
   out_7715249174232332377[5] = 0;
   out_7715249174232332377[6] = 0;
   out_7715249174232332377[7] = 0;
   out_7715249174232332377[8] = 0;
   out_7715249174232332377[9] = 0;
   out_7715249174232332377[10] = 1;
   out_7715249174232332377[11] = 0;
   out_7715249174232332377[12] = 0;
   out_7715249174232332377[13] = 0;
   out_7715249174232332377[14] = 0;
   out_7715249174232332377[15] = 0;
   out_7715249174232332377[16] = 0;
   out_7715249174232332377[17] = 0;
   out_7715249174232332377[18] = 0;
   out_7715249174232332377[19] = 0;
   out_7715249174232332377[20] = 1;
   out_7715249174232332377[21] = 0;
   out_7715249174232332377[22] = 0;
   out_7715249174232332377[23] = 0;
   out_7715249174232332377[24] = 0;
   out_7715249174232332377[25] = 0;
   out_7715249174232332377[26] = 0;
   out_7715249174232332377[27] = 0;
   out_7715249174232332377[28] = 0;
   out_7715249174232332377[29] = 0;
   out_7715249174232332377[30] = 1;
   out_7715249174232332377[31] = 0;
   out_7715249174232332377[32] = 0;
   out_7715249174232332377[33] = 0;
   out_7715249174232332377[34] = 0;
   out_7715249174232332377[35] = 0;
   out_7715249174232332377[36] = 0;
   out_7715249174232332377[37] = 0;
   out_7715249174232332377[38] = 0;
   out_7715249174232332377[39] = 0;
   out_7715249174232332377[40] = 1;
   out_7715249174232332377[41] = 0;
   out_7715249174232332377[42] = 0;
   out_7715249174232332377[43] = 0;
   out_7715249174232332377[44] = 0;
   out_7715249174232332377[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_7715249174232332377[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_7715249174232332377[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7715249174232332377[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7715249174232332377[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_7715249174232332377[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_7715249174232332377[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_7715249174232332377[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_7715249174232332377[53] = -9.8000000000000007*dt;
   out_7715249174232332377[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_7715249174232332377[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_7715249174232332377[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7715249174232332377[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7715249174232332377[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_7715249174232332377[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_7715249174232332377[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_7715249174232332377[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7715249174232332377[62] = 0;
   out_7715249174232332377[63] = 0;
   out_7715249174232332377[64] = 0;
   out_7715249174232332377[65] = 0;
   out_7715249174232332377[66] = 0;
   out_7715249174232332377[67] = 0;
   out_7715249174232332377[68] = 0;
   out_7715249174232332377[69] = 0;
   out_7715249174232332377[70] = 1;
   out_7715249174232332377[71] = 0;
   out_7715249174232332377[72] = 0;
   out_7715249174232332377[73] = 0;
   out_7715249174232332377[74] = 0;
   out_7715249174232332377[75] = 0;
   out_7715249174232332377[76] = 0;
   out_7715249174232332377[77] = 0;
   out_7715249174232332377[78] = 0;
   out_7715249174232332377[79] = 0;
   out_7715249174232332377[80] = 1;
}
void h_25(double *state, double *unused, double *out_1809245855196595095) {
   out_1809245855196595095[0] = state[6];
}
void H_25(double *state, double *unused, double *out_3284134045966130737) {
   out_3284134045966130737[0] = 0;
   out_3284134045966130737[1] = 0;
   out_3284134045966130737[2] = 0;
   out_3284134045966130737[3] = 0;
   out_3284134045966130737[4] = 0;
   out_3284134045966130737[5] = 0;
   out_3284134045966130737[6] = 1;
   out_3284134045966130737[7] = 0;
   out_3284134045966130737[8] = 0;
}
void h_24(double *state, double *unused, double *out_5296156693199309723) {
   out_5296156693199309723[0] = state[4];
   out_5296156693199309723[1] = state[5];
}
void H_24(double *state, double *unused, double *out_3754591528009469461) {
   out_3754591528009469461[0] = 0;
   out_3754591528009469461[1] = 0;
   out_3754591528009469461[2] = 0;
   out_3754591528009469461[3] = 0;
   out_3754591528009469461[4] = 1;
   out_3754591528009469461[5] = 0;
   out_3754591528009469461[6] = 0;
   out_3754591528009469461[7] = 0;
   out_3754591528009469461[8] = 0;
   out_3754591528009469461[9] = 0;
   out_3754591528009469461[10] = 0;
   out_3754591528009469461[11] = 0;
   out_3754591528009469461[12] = 0;
   out_3754591528009469461[13] = 0;
   out_3754591528009469461[14] = 1;
   out_3754591528009469461[15] = 0;
   out_3754591528009469461[16] = 0;
   out_3754591528009469461[17] = 0;
}
void h_30(double *state, double *unused, double *out_2079843709083647517) {
   out_2079843709083647517[0] = state[4];
}
void H_30(double *state, double *unused, double *out_3413472993109370807) {
   out_3413472993109370807[0] = 0;
   out_3413472993109370807[1] = 0;
   out_3413472993109370807[2] = 0;
   out_3413472993109370807[3] = 0;
   out_3413472993109370807[4] = 1;
   out_3413472993109370807[5] = 0;
   out_3413472993109370807[6] = 0;
   out_3413472993109370807[7] = 0;
   out_3413472993109370807[8] = 0;
}
void h_26(double *state, double *unused, double *out_8326294766543820985) {
   out_8326294766543820985[0] = state[7];
}
void H_26(double *state, double *unused, double *out_7025637364840186961) {
   out_7025637364840186961[0] = 0;
   out_7025637364840186961[1] = 0;
   out_7025637364840186961[2] = 0;
   out_7025637364840186961[3] = 0;
   out_7025637364840186961[4] = 0;
   out_7025637364840186961[5] = 0;
   out_7025637364840186961[6] = 0;
   out_7025637364840186961[7] = 1;
   out_7025637364840186961[8] = 0;
}
void h_27(double *state, double *unused, double *out_8208287372610444241) {
   out_8208287372610444241[0] = state[3];
}
void H_27(double *state, double *unused, double *out_5588236304909795718) {
   out_5588236304909795718[0] = 0;
   out_5588236304909795718[1] = 0;
   out_5588236304909795718[2] = 0;
   out_5588236304909795718[3] = 1;
   out_5588236304909795718[4] = 0;
   out_5588236304909795718[5] = 0;
   out_5588236304909795718[6] = 0;
   out_5588236304909795718[7] = 0;
   out_5588236304909795718[8] = 0;
}
void h_29(double *state, double *unused, double *out_6140153402819735511) {
   out_6140153402819735511[0] = state[1];
}
void H_29(double *state, double *unused, double *out_2903241648794978623) {
   out_2903241648794978623[0] = 0;
   out_2903241648794978623[1] = 1;
   out_2903241648794978623[2] = 0;
   out_2903241648794978623[3] = 0;
   out_2903241648794978623[4] = 0;
   out_2903241648794978623[5] = 0;
   out_2903241648794978623[6] = 0;
   out_2903241648794978623[7] = 0;
   out_2903241648794978623[8] = 0;
}
void h_28(double *state, double *unused, double *out_3206614104949598596) {
   out_3206614104949598596[0] = state[0];
}
void H_28(double *state, double *unused, double *out_5337968760214020500) {
   out_5337968760214020500[0] = 1;
   out_5337968760214020500[1] = 0;
   out_5337968760214020500[2] = 0;
   out_5337968760214020500[3] = 0;
   out_5337968760214020500[4] = 0;
   out_5337968760214020500[5] = 0;
   out_5337968760214020500[6] = 0;
   out_5337968760214020500[7] = 0;
   out_5337968760214020500[8] = 0;
}
void h_31(double *state, double *unused, double *out_5446695383450617445) {
   out_5446695383450617445[0] = state[8];
}
void H_31(double *state, double *unused, double *out_3253488084089170309) {
   out_3253488084089170309[0] = 0;
   out_3253488084089170309[1] = 0;
   out_3253488084089170309[2] = 0;
   out_3253488084089170309[3] = 0;
   out_3253488084089170309[4] = 0;
   out_3253488084089170309[5] = 0;
   out_3253488084089170309[6] = 0;
   out_3253488084089170309[7] = 0;
   out_3253488084089170309[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_7831334908946486631) {
  err_fun(nom_x, delta_x, out_7831334908946486631);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1412484805370031907) {
  inv_err_fun(nom_x, true_x, out_1412484805370031907);
}
void car_H_mod_fun(double *state, double *out_722397761437678664) {
  H_mod_fun(state, out_722397761437678664);
}
void car_f_fun(double *state, double dt, double *out_7415195887102028552) {
  f_fun(state,  dt, out_7415195887102028552);
}
void car_F_fun(double *state, double dt, double *out_7715249174232332377) {
  F_fun(state,  dt, out_7715249174232332377);
}
void car_h_25(double *state, double *unused, double *out_1809245855196595095) {
  h_25(state, unused, out_1809245855196595095);
}
void car_H_25(double *state, double *unused, double *out_3284134045966130737) {
  H_25(state, unused, out_3284134045966130737);
}
void car_h_24(double *state, double *unused, double *out_5296156693199309723) {
  h_24(state, unused, out_5296156693199309723);
}
void car_H_24(double *state, double *unused, double *out_3754591528009469461) {
  H_24(state, unused, out_3754591528009469461);
}
void car_h_30(double *state, double *unused, double *out_2079843709083647517) {
  h_30(state, unused, out_2079843709083647517);
}
void car_H_30(double *state, double *unused, double *out_3413472993109370807) {
  H_30(state, unused, out_3413472993109370807);
}
void car_h_26(double *state, double *unused, double *out_8326294766543820985) {
  h_26(state, unused, out_8326294766543820985);
}
void car_H_26(double *state, double *unused, double *out_7025637364840186961) {
  H_26(state, unused, out_7025637364840186961);
}
void car_h_27(double *state, double *unused, double *out_8208287372610444241) {
  h_27(state, unused, out_8208287372610444241);
}
void car_H_27(double *state, double *unused, double *out_5588236304909795718) {
  H_27(state, unused, out_5588236304909795718);
}
void car_h_29(double *state, double *unused, double *out_6140153402819735511) {
  h_29(state, unused, out_6140153402819735511);
}
void car_H_29(double *state, double *unused, double *out_2903241648794978623) {
  H_29(state, unused, out_2903241648794978623);
}
void car_h_28(double *state, double *unused, double *out_3206614104949598596) {
  h_28(state, unused, out_3206614104949598596);
}
void car_H_28(double *state, double *unused, double *out_5337968760214020500) {
  H_28(state, unused, out_5337968760214020500);
}
void car_h_31(double *state, double *unused, double *out_5446695383450617445) {
  h_31(state, unused, out_5446695383450617445);
}
void car_H_31(double *state, double *unused, double *out_3253488084089170309) {
  H_31(state, unused, out_3253488084089170309);
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
