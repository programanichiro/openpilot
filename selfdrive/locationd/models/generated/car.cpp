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
void err_fun(double *nom_x, double *delta_x, double *out_6961449098991356967) {
   out_6961449098991356967[0] = delta_x[0] + nom_x[0];
   out_6961449098991356967[1] = delta_x[1] + nom_x[1];
   out_6961449098991356967[2] = delta_x[2] + nom_x[2];
   out_6961449098991356967[3] = delta_x[3] + nom_x[3];
   out_6961449098991356967[4] = delta_x[4] + nom_x[4];
   out_6961449098991356967[5] = delta_x[5] + nom_x[5];
   out_6961449098991356967[6] = delta_x[6] + nom_x[6];
   out_6961449098991356967[7] = delta_x[7] + nom_x[7];
   out_6961449098991356967[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7191079212549050895) {
   out_7191079212549050895[0] = -nom_x[0] + true_x[0];
   out_7191079212549050895[1] = -nom_x[1] + true_x[1];
   out_7191079212549050895[2] = -nom_x[2] + true_x[2];
   out_7191079212549050895[3] = -nom_x[3] + true_x[3];
   out_7191079212549050895[4] = -nom_x[4] + true_x[4];
   out_7191079212549050895[5] = -nom_x[5] + true_x[5];
   out_7191079212549050895[6] = -nom_x[6] + true_x[6];
   out_7191079212549050895[7] = -nom_x[7] + true_x[7];
   out_7191079212549050895[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_292831340756603194) {
   out_292831340756603194[0] = 1.0;
   out_292831340756603194[1] = 0;
   out_292831340756603194[2] = 0;
   out_292831340756603194[3] = 0;
   out_292831340756603194[4] = 0;
   out_292831340756603194[5] = 0;
   out_292831340756603194[6] = 0;
   out_292831340756603194[7] = 0;
   out_292831340756603194[8] = 0;
   out_292831340756603194[9] = 0;
   out_292831340756603194[10] = 1.0;
   out_292831340756603194[11] = 0;
   out_292831340756603194[12] = 0;
   out_292831340756603194[13] = 0;
   out_292831340756603194[14] = 0;
   out_292831340756603194[15] = 0;
   out_292831340756603194[16] = 0;
   out_292831340756603194[17] = 0;
   out_292831340756603194[18] = 0;
   out_292831340756603194[19] = 0;
   out_292831340756603194[20] = 1.0;
   out_292831340756603194[21] = 0;
   out_292831340756603194[22] = 0;
   out_292831340756603194[23] = 0;
   out_292831340756603194[24] = 0;
   out_292831340756603194[25] = 0;
   out_292831340756603194[26] = 0;
   out_292831340756603194[27] = 0;
   out_292831340756603194[28] = 0;
   out_292831340756603194[29] = 0;
   out_292831340756603194[30] = 1.0;
   out_292831340756603194[31] = 0;
   out_292831340756603194[32] = 0;
   out_292831340756603194[33] = 0;
   out_292831340756603194[34] = 0;
   out_292831340756603194[35] = 0;
   out_292831340756603194[36] = 0;
   out_292831340756603194[37] = 0;
   out_292831340756603194[38] = 0;
   out_292831340756603194[39] = 0;
   out_292831340756603194[40] = 1.0;
   out_292831340756603194[41] = 0;
   out_292831340756603194[42] = 0;
   out_292831340756603194[43] = 0;
   out_292831340756603194[44] = 0;
   out_292831340756603194[45] = 0;
   out_292831340756603194[46] = 0;
   out_292831340756603194[47] = 0;
   out_292831340756603194[48] = 0;
   out_292831340756603194[49] = 0;
   out_292831340756603194[50] = 1.0;
   out_292831340756603194[51] = 0;
   out_292831340756603194[52] = 0;
   out_292831340756603194[53] = 0;
   out_292831340756603194[54] = 0;
   out_292831340756603194[55] = 0;
   out_292831340756603194[56] = 0;
   out_292831340756603194[57] = 0;
   out_292831340756603194[58] = 0;
   out_292831340756603194[59] = 0;
   out_292831340756603194[60] = 1.0;
   out_292831340756603194[61] = 0;
   out_292831340756603194[62] = 0;
   out_292831340756603194[63] = 0;
   out_292831340756603194[64] = 0;
   out_292831340756603194[65] = 0;
   out_292831340756603194[66] = 0;
   out_292831340756603194[67] = 0;
   out_292831340756603194[68] = 0;
   out_292831340756603194[69] = 0;
   out_292831340756603194[70] = 1.0;
   out_292831340756603194[71] = 0;
   out_292831340756603194[72] = 0;
   out_292831340756603194[73] = 0;
   out_292831340756603194[74] = 0;
   out_292831340756603194[75] = 0;
   out_292831340756603194[76] = 0;
   out_292831340756603194[77] = 0;
   out_292831340756603194[78] = 0;
   out_292831340756603194[79] = 0;
   out_292831340756603194[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_8924841413416666487) {
   out_8924841413416666487[0] = state[0];
   out_8924841413416666487[1] = state[1];
   out_8924841413416666487[2] = state[2];
   out_8924841413416666487[3] = state[3];
   out_8924841413416666487[4] = state[4];
   out_8924841413416666487[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_8924841413416666487[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_8924841413416666487[7] = state[7];
   out_8924841413416666487[8] = state[8];
}
void F_fun(double *state, double dt, double *out_631402002945752087) {
   out_631402002945752087[0] = 1;
   out_631402002945752087[1] = 0;
   out_631402002945752087[2] = 0;
   out_631402002945752087[3] = 0;
   out_631402002945752087[4] = 0;
   out_631402002945752087[5] = 0;
   out_631402002945752087[6] = 0;
   out_631402002945752087[7] = 0;
   out_631402002945752087[8] = 0;
   out_631402002945752087[9] = 0;
   out_631402002945752087[10] = 1;
   out_631402002945752087[11] = 0;
   out_631402002945752087[12] = 0;
   out_631402002945752087[13] = 0;
   out_631402002945752087[14] = 0;
   out_631402002945752087[15] = 0;
   out_631402002945752087[16] = 0;
   out_631402002945752087[17] = 0;
   out_631402002945752087[18] = 0;
   out_631402002945752087[19] = 0;
   out_631402002945752087[20] = 1;
   out_631402002945752087[21] = 0;
   out_631402002945752087[22] = 0;
   out_631402002945752087[23] = 0;
   out_631402002945752087[24] = 0;
   out_631402002945752087[25] = 0;
   out_631402002945752087[26] = 0;
   out_631402002945752087[27] = 0;
   out_631402002945752087[28] = 0;
   out_631402002945752087[29] = 0;
   out_631402002945752087[30] = 1;
   out_631402002945752087[31] = 0;
   out_631402002945752087[32] = 0;
   out_631402002945752087[33] = 0;
   out_631402002945752087[34] = 0;
   out_631402002945752087[35] = 0;
   out_631402002945752087[36] = 0;
   out_631402002945752087[37] = 0;
   out_631402002945752087[38] = 0;
   out_631402002945752087[39] = 0;
   out_631402002945752087[40] = 1;
   out_631402002945752087[41] = 0;
   out_631402002945752087[42] = 0;
   out_631402002945752087[43] = 0;
   out_631402002945752087[44] = 0;
   out_631402002945752087[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_631402002945752087[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_631402002945752087[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_631402002945752087[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_631402002945752087[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_631402002945752087[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_631402002945752087[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_631402002945752087[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_631402002945752087[53] = -9.8000000000000007*dt;
   out_631402002945752087[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_631402002945752087[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_631402002945752087[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_631402002945752087[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_631402002945752087[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_631402002945752087[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_631402002945752087[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_631402002945752087[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_631402002945752087[62] = 0;
   out_631402002945752087[63] = 0;
   out_631402002945752087[64] = 0;
   out_631402002945752087[65] = 0;
   out_631402002945752087[66] = 0;
   out_631402002945752087[67] = 0;
   out_631402002945752087[68] = 0;
   out_631402002945752087[69] = 0;
   out_631402002945752087[70] = 1;
   out_631402002945752087[71] = 0;
   out_631402002945752087[72] = 0;
   out_631402002945752087[73] = 0;
   out_631402002945752087[74] = 0;
   out_631402002945752087[75] = 0;
   out_631402002945752087[76] = 0;
   out_631402002945752087[77] = 0;
   out_631402002945752087[78] = 0;
   out_631402002945752087[79] = 0;
   out_631402002945752087[80] = 1;
}
void h_25(double *state, double *unused, double *out_5191763995009145519) {
   out_5191763995009145519[0] = state[6];
}
void H_25(double *state, double *unused, double *out_9205295690685381982) {
   out_9205295690685381982[0] = 0;
   out_9205295690685381982[1] = 0;
   out_9205295690685381982[2] = 0;
   out_9205295690685381982[3] = 0;
   out_9205295690685381982[4] = 0;
   out_9205295690685381982[5] = 0;
   out_9205295690685381982[6] = 1;
   out_9205295690685381982[7] = 0;
   out_9205295690685381982[8] = 0;
}
void h_24(double *state, double *unused, double *out_4678822133761862964) {
   out_4678822133761862964[0] = state[4];
   out_4678822133761862964[1] = state[5];
}
void H_24(double *state, double *unused, double *out_7028081267078232009) {
   out_7028081267078232009[0] = 0;
   out_7028081267078232009[1] = 0;
   out_7028081267078232009[2] = 0;
   out_7028081267078232009[3] = 0;
   out_7028081267078232009[4] = 1;
   out_7028081267078232009[5] = 0;
   out_7028081267078232009[6] = 0;
   out_7028081267078232009[7] = 0;
   out_7028081267078232009[8] = 0;
   out_7028081267078232009[9] = 0;
   out_7028081267078232009[10] = 0;
   out_7028081267078232009[11] = 0;
   out_7028081267078232009[12] = 0;
   out_7028081267078232009[13] = 0;
   out_7028081267078232009[14] = 1;
   out_7028081267078232009[15] = 0;
   out_7028081267078232009[16] = 0;
   out_7028081267078232009[17] = 0;
}
void h_30(double *state, double *unused, double *out_1396063931467149644) {
   out_1396063931467149644[0] = state[4];
}
void H_30(double *state, double *unused, double *out_6686962732178133355) {
   out_6686962732178133355[0] = 0;
   out_6686962732178133355[1] = 0;
   out_6686962732178133355[2] = 0;
   out_6686962732178133355[3] = 0;
   out_6686962732178133355[4] = 1;
   out_6686962732178133355[5] = 0;
   out_6686962732178133355[6] = 0;
   out_6686962732178133355[7] = 0;
   out_6686962732178133355[8] = 0;
}
void h_26(double *state, double *unused, double *out_4246665939613807191) {
   out_4246665939613807191[0] = state[7];
}
void H_26(double *state, double *unused, double *out_5499945064150113410) {
   out_5499945064150113410[0] = 0;
   out_5499945064150113410[1] = 0;
   out_5499945064150113410[2] = 0;
   out_5499945064150113410[3] = 0;
   out_5499945064150113410[4] = 0;
   out_5499945064150113410[5] = 0;
   out_5499945064150113410[6] = 0;
   out_5499945064150113410[7] = 1;
   out_5499945064150113410[8] = 0;
}
void h_27(double *state, double *unused, double *out_334022349075278952) {
   out_334022349075278952[0] = state[3];
}
void H_27(double *state, double *unused, double *out_6937346124080504653) {
   out_6937346124080504653[0] = 0;
   out_6937346124080504653[1] = 0;
   out_6937346124080504653[2] = 0;
   out_6937346124080504653[3] = 1;
   out_6937346124080504653[4] = 0;
   out_6937346124080504653[5] = 0;
   out_6937346124080504653[6] = 0;
   out_6937346124080504653[7] = 0;
   out_6937346124080504653[8] = 0;
}
void h_29(double *state, double *unused, double *out_8332843222373709646) {
   out_8332843222373709646[0] = state[1];
}
void H_29(double *state, double *unused, double *out_5223983397210953620) {
   out_5223983397210953620[0] = 0;
   out_5223983397210953620[1] = 1;
   out_5223983397210953620[2] = 0;
   out_5223983397210953620[3] = 0;
   out_5223983397210953620[4] = 0;
   out_5223983397210953620[5] = 0;
   out_5223983397210953620[6] = 0;
   out_5223983397210953620[7] = 0;
   out_5223983397210953620[8] = 0;
}
void h_28(double *state, double *unused, double *out_5308707522005677883) {
   out_5308707522005677883[0] = state[0];
}
void H_28(double *state, double *unused, double *out_7187613668776279871) {
   out_7187613668776279871[0] = 1;
   out_7187613668776279871[1] = 0;
   out_7187613668776279871[2] = 0;
   out_7187613668776279871[3] = 0;
   out_7187613668776279871[4] = 0;
   out_7187613668776279871[5] = 0;
   out_7187613668776279871[6] = 0;
   out_7187613668776279871[7] = 0;
   out_7187613668776279871[8] = 0;
}
void h_31(double *state, double *unused, double *out_2427569640084547844) {
   out_2427569640084547844[0] = state[8];
}
void H_31(double *state, double *unused, double *out_9174649728808421554) {
   out_9174649728808421554[0] = 0;
   out_9174649728808421554[1] = 0;
   out_9174649728808421554[2] = 0;
   out_9174649728808421554[3] = 0;
   out_9174649728808421554[4] = 0;
   out_9174649728808421554[5] = 0;
   out_9174649728808421554[6] = 0;
   out_9174649728808421554[7] = 0;
   out_9174649728808421554[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_6961449098991356967) {
  err_fun(nom_x, delta_x, out_6961449098991356967);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7191079212549050895) {
  inv_err_fun(nom_x, true_x, out_7191079212549050895);
}
void car_H_mod_fun(double *state, double *out_292831340756603194) {
  H_mod_fun(state, out_292831340756603194);
}
void car_f_fun(double *state, double dt, double *out_8924841413416666487) {
  f_fun(state,  dt, out_8924841413416666487);
}
void car_F_fun(double *state, double dt, double *out_631402002945752087) {
  F_fun(state,  dt, out_631402002945752087);
}
void car_h_25(double *state, double *unused, double *out_5191763995009145519) {
  h_25(state, unused, out_5191763995009145519);
}
void car_H_25(double *state, double *unused, double *out_9205295690685381982) {
  H_25(state, unused, out_9205295690685381982);
}
void car_h_24(double *state, double *unused, double *out_4678822133761862964) {
  h_24(state, unused, out_4678822133761862964);
}
void car_H_24(double *state, double *unused, double *out_7028081267078232009) {
  H_24(state, unused, out_7028081267078232009);
}
void car_h_30(double *state, double *unused, double *out_1396063931467149644) {
  h_30(state, unused, out_1396063931467149644);
}
void car_H_30(double *state, double *unused, double *out_6686962732178133355) {
  H_30(state, unused, out_6686962732178133355);
}
void car_h_26(double *state, double *unused, double *out_4246665939613807191) {
  h_26(state, unused, out_4246665939613807191);
}
void car_H_26(double *state, double *unused, double *out_5499945064150113410) {
  H_26(state, unused, out_5499945064150113410);
}
void car_h_27(double *state, double *unused, double *out_334022349075278952) {
  h_27(state, unused, out_334022349075278952);
}
void car_H_27(double *state, double *unused, double *out_6937346124080504653) {
  H_27(state, unused, out_6937346124080504653);
}
void car_h_29(double *state, double *unused, double *out_8332843222373709646) {
  h_29(state, unused, out_8332843222373709646);
}
void car_H_29(double *state, double *unused, double *out_5223983397210953620) {
  H_29(state, unused, out_5223983397210953620);
}
void car_h_28(double *state, double *unused, double *out_5308707522005677883) {
  h_28(state, unused, out_5308707522005677883);
}
void car_H_28(double *state, double *unused, double *out_7187613668776279871) {
  H_28(state, unused, out_7187613668776279871);
}
void car_h_31(double *state, double *unused, double *out_2427569640084547844) {
  h_31(state, unused, out_2427569640084547844);
}
void car_H_31(double *state, double *unused, double *out_9174649728808421554) {
  H_31(state, unused, out_9174649728808421554);
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
