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
void err_fun(double *nom_x, double *delta_x, double *out_8978105205623168168) {
   out_8978105205623168168[0] = delta_x[0] + nom_x[0];
   out_8978105205623168168[1] = delta_x[1] + nom_x[1];
   out_8978105205623168168[2] = delta_x[2] + nom_x[2];
   out_8978105205623168168[3] = delta_x[3] + nom_x[3];
   out_8978105205623168168[4] = delta_x[4] + nom_x[4];
   out_8978105205623168168[5] = delta_x[5] + nom_x[5];
   out_8978105205623168168[6] = delta_x[6] + nom_x[6];
   out_8978105205623168168[7] = delta_x[7] + nom_x[7];
   out_8978105205623168168[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_527484150705711459) {
   out_527484150705711459[0] = -nom_x[0] + true_x[0];
   out_527484150705711459[1] = -nom_x[1] + true_x[1];
   out_527484150705711459[2] = -nom_x[2] + true_x[2];
   out_527484150705711459[3] = -nom_x[3] + true_x[3];
   out_527484150705711459[4] = -nom_x[4] + true_x[4];
   out_527484150705711459[5] = -nom_x[5] + true_x[5];
   out_527484150705711459[6] = -nom_x[6] + true_x[6];
   out_527484150705711459[7] = -nom_x[7] + true_x[7];
   out_527484150705711459[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_5422076647877894621) {
   out_5422076647877894621[0] = 1.0;
   out_5422076647877894621[1] = 0;
   out_5422076647877894621[2] = 0;
   out_5422076647877894621[3] = 0;
   out_5422076647877894621[4] = 0;
   out_5422076647877894621[5] = 0;
   out_5422076647877894621[6] = 0;
   out_5422076647877894621[7] = 0;
   out_5422076647877894621[8] = 0;
   out_5422076647877894621[9] = 0;
   out_5422076647877894621[10] = 1.0;
   out_5422076647877894621[11] = 0;
   out_5422076647877894621[12] = 0;
   out_5422076647877894621[13] = 0;
   out_5422076647877894621[14] = 0;
   out_5422076647877894621[15] = 0;
   out_5422076647877894621[16] = 0;
   out_5422076647877894621[17] = 0;
   out_5422076647877894621[18] = 0;
   out_5422076647877894621[19] = 0;
   out_5422076647877894621[20] = 1.0;
   out_5422076647877894621[21] = 0;
   out_5422076647877894621[22] = 0;
   out_5422076647877894621[23] = 0;
   out_5422076647877894621[24] = 0;
   out_5422076647877894621[25] = 0;
   out_5422076647877894621[26] = 0;
   out_5422076647877894621[27] = 0;
   out_5422076647877894621[28] = 0;
   out_5422076647877894621[29] = 0;
   out_5422076647877894621[30] = 1.0;
   out_5422076647877894621[31] = 0;
   out_5422076647877894621[32] = 0;
   out_5422076647877894621[33] = 0;
   out_5422076647877894621[34] = 0;
   out_5422076647877894621[35] = 0;
   out_5422076647877894621[36] = 0;
   out_5422076647877894621[37] = 0;
   out_5422076647877894621[38] = 0;
   out_5422076647877894621[39] = 0;
   out_5422076647877894621[40] = 1.0;
   out_5422076647877894621[41] = 0;
   out_5422076647877894621[42] = 0;
   out_5422076647877894621[43] = 0;
   out_5422076647877894621[44] = 0;
   out_5422076647877894621[45] = 0;
   out_5422076647877894621[46] = 0;
   out_5422076647877894621[47] = 0;
   out_5422076647877894621[48] = 0;
   out_5422076647877894621[49] = 0;
   out_5422076647877894621[50] = 1.0;
   out_5422076647877894621[51] = 0;
   out_5422076647877894621[52] = 0;
   out_5422076647877894621[53] = 0;
   out_5422076647877894621[54] = 0;
   out_5422076647877894621[55] = 0;
   out_5422076647877894621[56] = 0;
   out_5422076647877894621[57] = 0;
   out_5422076647877894621[58] = 0;
   out_5422076647877894621[59] = 0;
   out_5422076647877894621[60] = 1.0;
   out_5422076647877894621[61] = 0;
   out_5422076647877894621[62] = 0;
   out_5422076647877894621[63] = 0;
   out_5422076647877894621[64] = 0;
   out_5422076647877894621[65] = 0;
   out_5422076647877894621[66] = 0;
   out_5422076647877894621[67] = 0;
   out_5422076647877894621[68] = 0;
   out_5422076647877894621[69] = 0;
   out_5422076647877894621[70] = 1.0;
   out_5422076647877894621[71] = 0;
   out_5422076647877894621[72] = 0;
   out_5422076647877894621[73] = 0;
   out_5422076647877894621[74] = 0;
   out_5422076647877894621[75] = 0;
   out_5422076647877894621[76] = 0;
   out_5422076647877894621[77] = 0;
   out_5422076647877894621[78] = 0;
   out_5422076647877894621[79] = 0;
   out_5422076647877894621[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_3002906037626941687) {
   out_3002906037626941687[0] = state[0];
   out_3002906037626941687[1] = state[1];
   out_3002906037626941687[2] = state[2];
   out_3002906037626941687[3] = state[3];
   out_3002906037626941687[4] = state[4];
   out_3002906037626941687[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_3002906037626941687[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_3002906037626941687[7] = state[7];
   out_3002906037626941687[8] = state[8];
}
void F_fun(double *state, double dt, double *out_5137753966668609063) {
   out_5137753966668609063[0] = 1;
   out_5137753966668609063[1] = 0;
   out_5137753966668609063[2] = 0;
   out_5137753966668609063[3] = 0;
   out_5137753966668609063[4] = 0;
   out_5137753966668609063[5] = 0;
   out_5137753966668609063[6] = 0;
   out_5137753966668609063[7] = 0;
   out_5137753966668609063[8] = 0;
   out_5137753966668609063[9] = 0;
   out_5137753966668609063[10] = 1;
   out_5137753966668609063[11] = 0;
   out_5137753966668609063[12] = 0;
   out_5137753966668609063[13] = 0;
   out_5137753966668609063[14] = 0;
   out_5137753966668609063[15] = 0;
   out_5137753966668609063[16] = 0;
   out_5137753966668609063[17] = 0;
   out_5137753966668609063[18] = 0;
   out_5137753966668609063[19] = 0;
   out_5137753966668609063[20] = 1;
   out_5137753966668609063[21] = 0;
   out_5137753966668609063[22] = 0;
   out_5137753966668609063[23] = 0;
   out_5137753966668609063[24] = 0;
   out_5137753966668609063[25] = 0;
   out_5137753966668609063[26] = 0;
   out_5137753966668609063[27] = 0;
   out_5137753966668609063[28] = 0;
   out_5137753966668609063[29] = 0;
   out_5137753966668609063[30] = 1;
   out_5137753966668609063[31] = 0;
   out_5137753966668609063[32] = 0;
   out_5137753966668609063[33] = 0;
   out_5137753966668609063[34] = 0;
   out_5137753966668609063[35] = 0;
   out_5137753966668609063[36] = 0;
   out_5137753966668609063[37] = 0;
   out_5137753966668609063[38] = 0;
   out_5137753966668609063[39] = 0;
   out_5137753966668609063[40] = 1;
   out_5137753966668609063[41] = 0;
   out_5137753966668609063[42] = 0;
   out_5137753966668609063[43] = 0;
   out_5137753966668609063[44] = 0;
   out_5137753966668609063[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_5137753966668609063[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_5137753966668609063[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5137753966668609063[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5137753966668609063[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_5137753966668609063[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_5137753966668609063[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_5137753966668609063[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_5137753966668609063[53] = -9.8000000000000007*dt;
   out_5137753966668609063[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_5137753966668609063[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_5137753966668609063[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5137753966668609063[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5137753966668609063[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_5137753966668609063[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_5137753966668609063[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_5137753966668609063[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5137753966668609063[62] = 0;
   out_5137753966668609063[63] = 0;
   out_5137753966668609063[64] = 0;
   out_5137753966668609063[65] = 0;
   out_5137753966668609063[66] = 0;
   out_5137753966668609063[67] = 0;
   out_5137753966668609063[68] = 0;
   out_5137753966668609063[69] = 0;
   out_5137753966668609063[70] = 1;
   out_5137753966668609063[71] = 0;
   out_5137753966668609063[72] = 0;
   out_5137753966668609063[73] = 0;
   out_5137753966668609063[74] = 0;
   out_5137753966668609063[75] = 0;
   out_5137753966668609063[76] = 0;
   out_5137753966668609063[77] = 0;
   out_5137753966668609063[78] = 0;
   out_5137753966668609063[79] = 0;
   out_5137753966668609063[80] = 1;
}
void h_25(double *state, double *unused, double *out_4442074441953279896) {
   out_4442074441953279896[0] = state[6];
}
void H_25(double *state, double *unused, double *out_5376650681047099606) {
   out_5376650681047099606[0] = 0;
   out_5376650681047099606[1] = 0;
   out_5376650681047099606[2] = 0;
   out_5376650681047099606[3] = 0;
   out_5376650681047099606[4] = 0;
   out_5376650681047099606[5] = 0;
   out_5376650681047099606[6] = 1;
   out_5376650681047099606[7] = 0;
   out_5376650681047099606[8] = 0;
}
void h_24(double *state, double *unused, double *out_8121433539224248093) {
   out_8121433539224248093[0] = state[4];
   out_8121433539224248093[1] = state[5];
}
void H_24(double *state, double *unused, double *out_7549300280052599172) {
   out_7549300280052599172[0] = 0;
   out_7549300280052599172[1] = 0;
   out_7549300280052599172[2] = 0;
   out_7549300280052599172[3] = 0;
   out_7549300280052599172[4] = 1;
   out_7549300280052599172[5] = 0;
   out_7549300280052599172[6] = 0;
   out_7549300280052599172[7] = 0;
   out_7549300280052599172[8] = 0;
   out_7549300280052599172[9] = 0;
   out_7549300280052599172[10] = 0;
   out_7549300280052599172[11] = 0;
   out_7549300280052599172[12] = 0;
   out_7549300280052599172[13] = 0;
   out_7549300280052599172[14] = 1;
   out_7549300280052599172[15] = 0;
   out_7549300280052599172[16] = 0;
   out_7549300280052599172[17] = 0;
}
void h_30(double *state, double *unused, double *out_4166880379668774007) {
   out_4166880379668774007[0] = state[4];
}
void H_30(double *state, double *unused, double *out_2858317722539850979) {
   out_2858317722539850979[0] = 0;
   out_2858317722539850979[1] = 0;
   out_2858317722539850979[2] = 0;
   out_2858317722539850979[3] = 0;
   out_2858317722539850979[4] = 1;
   out_2858317722539850979[5] = 0;
   out_2858317722539850979[6] = 0;
   out_2858317722539850979[7] = 0;
   out_2858317722539850979[8] = 0;
}
void h_26(double *state, double *unused, double *out_7487620720409045830) {
   out_7487620720409045830[0] = state[7];
}
void H_26(double *state, double *unused, double *out_9118153999921155830) {
   out_9118153999921155830[0] = 0;
   out_9118153999921155830[1] = 0;
   out_9118153999921155830[2] = 0;
   out_9118153999921155830[3] = 0;
   out_9118153999921155830[4] = 0;
   out_9118153999921155830[5] = 0;
   out_9118153999921155830[6] = 0;
   out_9118153999921155830[7] = 1;
   out_9118153999921155830[8] = 0;
}
void h_27(double *state, double *unused, double *out_559598825707565749) {
   out_559598825707565749[0] = state[3];
}
void H_27(double *state, double *unused, double *out_5033081034340275890) {
   out_5033081034340275890[0] = 0;
   out_5033081034340275890[1] = 0;
   out_5033081034340275890[2] = 0;
   out_5033081034340275890[3] = 1;
   out_5033081034340275890[4] = 0;
   out_5033081034340275890[5] = 0;
   out_5033081034340275890[6] = 0;
   out_5033081034340275890[7] = 0;
   out_5033081034340275890[8] = 0;
}
void h_29(double *state, double *unused, double *out_3507324816063050710) {
   out_3507324816063050710[0] = state[1];
}
void H_29(double *state, double *unused, double *out_2348086378225458795) {
   out_2348086378225458795[0] = 0;
   out_2348086378225458795[1] = 1;
   out_2348086378225458795[2] = 0;
   out_2348086378225458795[3] = 0;
   out_2348086378225458795[4] = 0;
   out_2348086378225458795[5] = 0;
   out_2348086378225458795[6] = 0;
   out_2348086378225458795[7] = 0;
   out_2348086378225458795[8] = 0;
}
void h_28(double *state, double *unused, double *out_5839442691706283397) {
   out_5839442691706283397[0] = state[0];
}
void H_28(double *state, double *unused, double *out_7430485395294989369) {
   out_7430485395294989369[0] = 1;
   out_7430485395294989369[1] = 0;
   out_7430485395294989369[2] = 0;
   out_7430485395294989369[3] = 0;
   out_7430485395294989369[4] = 0;
   out_7430485395294989369[5] = 0;
   out_7430485395294989369[6] = 0;
   out_7430485395294989369[7] = 0;
   out_7430485395294989369[8] = 0;
}
void h_31(double *state, double *unused, double *out_8079523970207302246) {
   out_8079523970207302246[0] = state[8];
}
void H_31(double *state, double *unused, double *out_8702381971555044310) {
   out_8702381971555044310[0] = 0;
   out_8702381971555044310[1] = 0;
   out_8702381971555044310[2] = 0;
   out_8702381971555044310[3] = 0;
   out_8702381971555044310[4] = 0;
   out_8702381971555044310[5] = 0;
   out_8702381971555044310[6] = 0;
   out_8702381971555044310[7] = 0;
   out_8702381971555044310[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_8978105205623168168) {
  err_fun(nom_x, delta_x, out_8978105205623168168);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_527484150705711459) {
  inv_err_fun(nom_x, true_x, out_527484150705711459);
}
void car_H_mod_fun(double *state, double *out_5422076647877894621) {
  H_mod_fun(state, out_5422076647877894621);
}
void car_f_fun(double *state, double dt, double *out_3002906037626941687) {
  f_fun(state,  dt, out_3002906037626941687);
}
void car_F_fun(double *state, double dt, double *out_5137753966668609063) {
  F_fun(state,  dt, out_5137753966668609063);
}
void car_h_25(double *state, double *unused, double *out_4442074441953279896) {
  h_25(state, unused, out_4442074441953279896);
}
void car_H_25(double *state, double *unused, double *out_5376650681047099606) {
  H_25(state, unused, out_5376650681047099606);
}
void car_h_24(double *state, double *unused, double *out_8121433539224248093) {
  h_24(state, unused, out_8121433539224248093);
}
void car_H_24(double *state, double *unused, double *out_7549300280052599172) {
  H_24(state, unused, out_7549300280052599172);
}
void car_h_30(double *state, double *unused, double *out_4166880379668774007) {
  h_30(state, unused, out_4166880379668774007);
}
void car_H_30(double *state, double *unused, double *out_2858317722539850979) {
  H_30(state, unused, out_2858317722539850979);
}
void car_h_26(double *state, double *unused, double *out_7487620720409045830) {
  h_26(state, unused, out_7487620720409045830);
}
void car_H_26(double *state, double *unused, double *out_9118153999921155830) {
  H_26(state, unused, out_9118153999921155830);
}
void car_h_27(double *state, double *unused, double *out_559598825707565749) {
  h_27(state, unused, out_559598825707565749);
}
void car_H_27(double *state, double *unused, double *out_5033081034340275890) {
  H_27(state, unused, out_5033081034340275890);
}
void car_h_29(double *state, double *unused, double *out_3507324816063050710) {
  h_29(state, unused, out_3507324816063050710);
}
void car_H_29(double *state, double *unused, double *out_2348086378225458795) {
  H_29(state, unused, out_2348086378225458795);
}
void car_h_28(double *state, double *unused, double *out_5839442691706283397) {
  h_28(state, unused, out_5839442691706283397);
}
void car_H_28(double *state, double *unused, double *out_7430485395294989369) {
  H_28(state, unused, out_7430485395294989369);
}
void car_h_31(double *state, double *unused, double *out_8079523970207302246) {
  h_31(state, unused, out_8079523970207302246);
}
void car_H_31(double *state, double *unused, double *out_8702381971555044310) {
  H_31(state, unused, out_8702381971555044310);
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
