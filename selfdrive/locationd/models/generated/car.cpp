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
 *                       Code generated with SymPy 1.12                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_8937458499683816514) {
   out_8937458499683816514[0] = delta_x[0] + nom_x[0];
   out_8937458499683816514[1] = delta_x[1] + nom_x[1];
   out_8937458499683816514[2] = delta_x[2] + nom_x[2];
   out_8937458499683816514[3] = delta_x[3] + nom_x[3];
   out_8937458499683816514[4] = delta_x[4] + nom_x[4];
   out_8937458499683816514[5] = delta_x[5] + nom_x[5];
   out_8937458499683816514[6] = delta_x[6] + nom_x[6];
   out_8937458499683816514[7] = delta_x[7] + nom_x[7];
   out_8937458499683816514[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_9068690400416595123) {
   out_9068690400416595123[0] = -nom_x[0] + true_x[0];
   out_9068690400416595123[1] = -nom_x[1] + true_x[1];
   out_9068690400416595123[2] = -nom_x[2] + true_x[2];
   out_9068690400416595123[3] = -nom_x[3] + true_x[3];
   out_9068690400416595123[4] = -nom_x[4] + true_x[4];
   out_9068690400416595123[5] = -nom_x[5] + true_x[5];
   out_9068690400416595123[6] = -nom_x[6] + true_x[6];
   out_9068690400416595123[7] = -nom_x[7] + true_x[7];
   out_9068690400416595123[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_5097223253014355728) {
   out_5097223253014355728[0] = 1.0;
   out_5097223253014355728[1] = 0;
   out_5097223253014355728[2] = 0;
   out_5097223253014355728[3] = 0;
   out_5097223253014355728[4] = 0;
   out_5097223253014355728[5] = 0;
   out_5097223253014355728[6] = 0;
   out_5097223253014355728[7] = 0;
   out_5097223253014355728[8] = 0;
   out_5097223253014355728[9] = 0;
   out_5097223253014355728[10] = 1.0;
   out_5097223253014355728[11] = 0;
   out_5097223253014355728[12] = 0;
   out_5097223253014355728[13] = 0;
   out_5097223253014355728[14] = 0;
   out_5097223253014355728[15] = 0;
   out_5097223253014355728[16] = 0;
   out_5097223253014355728[17] = 0;
   out_5097223253014355728[18] = 0;
   out_5097223253014355728[19] = 0;
   out_5097223253014355728[20] = 1.0;
   out_5097223253014355728[21] = 0;
   out_5097223253014355728[22] = 0;
   out_5097223253014355728[23] = 0;
   out_5097223253014355728[24] = 0;
   out_5097223253014355728[25] = 0;
   out_5097223253014355728[26] = 0;
   out_5097223253014355728[27] = 0;
   out_5097223253014355728[28] = 0;
   out_5097223253014355728[29] = 0;
   out_5097223253014355728[30] = 1.0;
   out_5097223253014355728[31] = 0;
   out_5097223253014355728[32] = 0;
   out_5097223253014355728[33] = 0;
   out_5097223253014355728[34] = 0;
   out_5097223253014355728[35] = 0;
   out_5097223253014355728[36] = 0;
   out_5097223253014355728[37] = 0;
   out_5097223253014355728[38] = 0;
   out_5097223253014355728[39] = 0;
   out_5097223253014355728[40] = 1.0;
   out_5097223253014355728[41] = 0;
   out_5097223253014355728[42] = 0;
   out_5097223253014355728[43] = 0;
   out_5097223253014355728[44] = 0;
   out_5097223253014355728[45] = 0;
   out_5097223253014355728[46] = 0;
   out_5097223253014355728[47] = 0;
   out_5097223253014355728[48] = 0;
   out_5097223253014355728[49] = 0;
   out_5097223253014355728[50] = 1.0;
   out_5097223253014355728[51] = 0;
   out_5097223253014355728[52] = 0;
   out_5097223253014355728[53] = 0;
   out_5097223253014355728[54] = 0;
   out_5097223253014355728[55] = 0;
   out_5097223253014355728[56] = 0;
   out_5097223253014355728[57] = 0;
   out_5097223253014355728[58] = 0;
   out_5097223253014355728[59] = 0;
   out_5097223253014355728[60] = 1.0;
   out_5097223253014355728[61] = 0;
   out_5097223253014355728[62] = 0;
   out_5097223253014355728[63] = 0;
   out_5097223253014355728[64] = 0;
   out_5097223253014355728[65] = 0;
   out_5097223253014355728[66] = 0;
   out_5097223253014355728[67] = 0;
   out_5097223253014355728[68] = 0;
   out_5097223253014355728[69] = 0;
   out_5097223253014355728[70] = 1.0;
   out_5097223253014355728[71] = 0;
   out_5097223253014355728[72] = 0;
   out_5097223253014355728[73] = 0;
   out_5097223253014355728[74] = 0;
   out_5097223253014355728[75] = 0;
   out_5097223253014355728[76] = 0;
   out_5097223253014355728[77] = 0;
   out_5097223253014355728[78] = 0;
   out_5097223253014355728[79] = 0;
   out_5097223253014355728[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_3149203961878712284) {
   out_3149203961878712284[0] = state[0];
   out_3149203961878712284[1] = state[1];
   out_3149203961878712284[2] = state[2];
   out_3149203961878712284[3] = state[3];
   out_3149203961878712284[4] = state[4];
   out_3149203961878712284[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_3149203961878712284[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_3149203961878712284[7] = state[7];
   out_3149203961878712284[8] = state[8];
}
void F_fun(double *state, double dt, double *out_5048345358166039832) {
   out_5048345358166039832[0] = 1;
   out_5048345358166039832[1] = 0;
   out_5048345358166039832[2] = 0;
   out_5048345358166039832[3] = 0;
   out_5048345358166039832[4] = 0;
   out_5048345358166039832[5] = 0;
   out_5048345358166039832[6] = 0;
   out_5048345358166039832[7] = 0;
   out_5048345358166039832[8] = 0;
   out_5048345358166039832[9] = 0;
   out_5048345358166039832[10] = 1;
   out_5048345358166039832[11] = 0;
   out_5048345358166039832[12] = 0;
   out_5048345358166039832[13] = 0;
   out_5048345358166039832[14] = 0;
   out_5048345358166039832[15] = 0;
   out_5048345358166039832[16] = 0;
   out_5048345358166039832[17] = 0;
   out_5048345358166039832[18] = 0;
   out_5048345358166039832[19] = 0;
   out_5048345358166039832[20] = 1;
   out_5048345358166039832[21] = 0;
   out_5048345358166039832[22] = 0;
   out_5048345358166039832[23] = 0;
   out_5048345358166039832[24] = 0;
   out_5048345358166039832[25] = 0;
   out_5048345358166039832[26] = 0;
   out_5048345358166039832[27] = 0;
   out_5048345358166039832[28] = 0;
   out_5048345358166039832[29] = 0;
   out_5048345358166039832[30] = 1;
   out_5048345358166039832[31] = 0;
   out_5048345358166039832[32] = 0;
   out_5048345358166039832[33] = 0;
   out_5048345358166039832[34] = 0;
   out_5048345358166039832[35] = 0;
   out_5048345358166039832[36] = 0;
   out_5048345358166039832[37] = 0;
   out_5048345358166039832[38] = 0;
   out_5048345358166039832[39] = 0;
   out_5048345358166039832[40] = 1;
   out_5048345358166039832[41] = 0;
   out_5048345358166039832[42] = 0;
   out_5048345358166039832[43] = 0;
   out_5048345358166039832[44] = 0;
   out_5048345358166039832[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_5048345358166039832[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_5048345358166039832[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5048345358166039832[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5048345358166039832[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_5048345358166039832[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_5048345358166039832[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_5048345358166039832[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_5048345358166039832[53] = -9.8000000000000007*dt;
   out_5048345358166039832[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_5048345358166039832[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_5048345358166039832[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5048345358166039832[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5048345358166039832[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_5048345358166039832[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_5048345358166039832[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_5048345358166039832[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5048345358166039832[62] = 0;
   out_5048345358166039832[63] = 0;
   out_5048345358166039832[64] = 0;
   out_5048345358166039832[65] = 0;
   out_5048345358166039832[66] = 0;
   out_5048345358166039832[67] = 0;
   out_5048345358166039832[68] = 0;
   out_5048345358166039832[69] = 0;
   out_5048345358166039832[70] = 1;
   out_5048345358166039832[71] = 0;
   out_5048345358166039832[72] = 0;
   out_5048345358166039832[73] = 0;
   out_5048345358166039832[74] = 0;
   out_5048345358166039832[75] = 0;
   out_5048345358166039832[76] = 0;
   out_5048345358166039832[77] = 0;
   out_5048345358166039832[78] = 0;
   out_5048345358166039832[79] = 0;
   out_5048345358166039832[80] = 1;
}
void h_25(double *state, double *unused, double *out_8982389284843880955) {
   out_8982389284843880955[0] = state[6];
}
void H_25(double *state, double *unused, double *out_5853167493858090130) {
   out_5853167493858090130[0] = 0;
   out_5853167493858090130[1] = 0;
   out_5853167493858090130[2] = 0;
   out_5853167493858090130[3] = 0;
   out_5853167493858090130[4] = 0;
   out_5853167493858090130[5] = 0;
   out_5853167493858090130[6] = 1;
   out_5853167493858090130[7] = 0;
   out_5853167493858090130[8] = 0;
}
void h_24(double *state, double *unused, double *out_1960999661702281101) {
   out_1960999661702281101[0] = state[4];
   out_1960999661702281101[1] = state[5];
}
void H_24(double *state, double *unused, double *out_9035650981049138350) {
   out_9035650981049138350[0] = 0;
   out_9035650981049138350[1] = 0;
   out_9035650981049138350[2] = 0;
   out_9035650981049138350[3] = 0;
   out_9035650981049138350[4] = 1;
   out_9035650981049138350[5] = 0;
   out_9035650981049138350[6] = 0;
   out_9035650981049138350[7] = 0;
   out_9035650981049138350[8] = 0;
   out_9035650981049138350[9] = 0;
   out_9035650981049138350[10] = 0;
   out_9035650981049138350[11] = 0;
   out_9035650981049138350[12] = 0;
   out_9035650981049138350[13] = 0;
   out_9035650981049138350[14] = 1;
   out_9035650981049138350[15] = 0;
   out_9035650981049138350[16] = 0;
   out_9035650981049138350[17] = 0;
}
void h_30(double *state, double *unused, double *out_1701089532044998220) {
   out_1701089532044998220[0] = state[4];
}
void H_30(double *state, double *unused, double *out_8371500452365338757) {
   out_8371500452365338757[0] = 0;
   out_8371500452365338757[1] = 0;
   out_8371500452365338757[2] = 0;
   out_8371500452365338757[3] = 0;
   out_8371500452365338757[4] = 1;
   out_8371500452365338757[5] = 0;
   out_8371500452365338757[6] = 0;
   out_8371500452365338757[7] = 0;
   out_8371500452365338757[8] = 0;
}
void h_26(double *state, double *unused, double *out_4737875583489330905) {
   out_4737875583489330905[0] = state[7];
}
void H_26(double *state, double *unused, double *out_2111664174984033906) {
   out_2111664174984033906[0] = 0;
   out_2111664174984033906[1] = 0;
   out_2111664174984033906[2] = 0;
   out_2111664174984033906[3] = 0;
   out_2111664174984033906[4] = 0;
   out_2111664174984033906[5] = 0;
   out_2111664174984033906[6] = 0;
   out_2111664174984033906[7] = 1;
   out_2111664174984033906[8] = 0;
}
void h_27(double *state, double *unused, double *out_8520844935964008226) {
   out_8520844935964008226[0] = state[3];
}
void H_27(double *state, double *unused, double *out_6196737140564913846) {
   out_6196737140564913846[0] = 0;
   out_6196737140564913846[1] = 0;
   out_6196737140564913846[2] = 0;
   out_6196737140564913846[3] = 1;
   out_6196737140564913846[4] = 0;
   out_6196737140564913846[5] = 0;
   out_6196737140564913846[6] = 0;
   out_6196737140564913846[7] = 0;
   out_6196737140564913846[8] = 0;
}
void h_29(double *state, double *unused, double *out_757579679143335785) {
   out_757579679143335785[0] = state[1];
}
void H_29(double *state, double *unused, double *out_8881731796679730941) {
   out_8881731796679730941[0] = 0;
   out_8881731796679730941[1] = 1;
   out_8881731796679730941[2] = 0;
   out_8881731796679730941[3] = 0;
   out_8881731796679730941[4] = 0;
   out_8881731796679730941[5] = 0;
   out_8881731796679730941[6] = 0;
   out_8881731796679730941[7] = 0;
   out_8881731796679730941[8] = 0;
}
void h_28(double *state, double *unused, double *out_2122019703949867755) {
   out_2122019703949867755[0] = state[0];
}
void H_28(double *state, double *unused, double *out_3799332779610200367) {
   out_3799332779610200367[0] = 1;
   out_3799332779610200367[1] = 0;
   out_3799332779610200367[2] = 0;
   out_3799332779610200367[3] = 0;
   out_3799332779610200367[4] = 0;
   out_3799332779610200367[5] = 0;
   out_3799332779610200367[6] = 0;
   out_3799332779610200367[7] = 0;
   out_3799332779610200367[8] = 0;
}
void h_31(double *state, double *unused, double *out_3068656258173132582) {
   out_3068656258173132582[0] = state[8];
}
void H_31(double *state, double *unused, double *out_1485456072750682430) {
   out_1485456072750682430[0] = 0;
   out_1485456072750682430[1] = 0;
   out_1485456072750682430[2] = 0;
   out_1485456072750682430[3] = 0;
   out_1485456072750682430[4] = 0;
   out_1485456072750682430[5] = 0;
   out_1485456072750682430[6] = 0;
   out_1485456072750682430[7] = 0;
   out_1485456072750682430[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_8937458499683816514) {
  err_fun(nom_x, delta_x, out_8937458499683816514);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_9068690400416595123) {
  inv_err_fun(nom_x, true_x, out_9068690400416595123);
}
void car_H_mod_fun(double *state, double *out_5097223253014355728) {
  H_mod_fun(state, out_5097223253014355728);
}
void car_f_fun(double *state, double dt, double *out_3149203961878712284) {
  f_fun(state,  dt, out_3149203961878712284);
}
void car_F_fun(double *state, double dt, double *out_5048345358166039832) {
  F_fun(state,  dt, out_5048345358166039832);
}
void car_h_25(double *state, double *unused, double *out_8982389284843880955) {
  h_25(state, unused, out_8982389284843880955);
}
void car_H_25(double *state, double *unused, double *out_5853167493858090130) {
  H_25(state, unused, out_5853167493858090130);
}
void car_h_24(double *state, double *unused, double *out_1960999661702281101) {
  h_24(state, unused, out_1960999661702281101);
}
void car_H_24(double *state, double *unused, double *out_9035650981049138350) {
  H_24(state, unused, out_9035650981049138350);
}
void car_h_30(double *state, double *unused, double *out_1701089532044998220) {
  h_30(state, unused, out_1701089532044998220);
}
void car_H_30(double *state, double *unused, double *out_8371500452365338757) {
  H_30(state, unused, out_8371500452365338757);
}
void car_h_26(double *state, double *unused, double *out_4737875583489330905) {
  h_26(state, unused, out_4737875583489330905);
}
void car_H_26(double *state, double *unused, double *out_2111664174984033906) {
  H_26(state, unused, out_2111664174984033906);
}
void car_h_27(double *state, double *unused, double *out_8520844935964008226) {
  h_27(state, unused, out_8520844935964008226);
}
void car_H_27(double *state, double *unused, double *out_6196737140564913846) {
  H_27(state, unused, out_6196737140564913846);
}
void car_h_29(double *state, double *unused, double *out_757579679143335785) {
  h_29(state, unused, out_757579679143335785);
}
void car_H_29(double *state, double *unused, double *out_8881731796679730941) {
  H_29(state, unused, out_8881731796679730941);
}
void car_h_28(double *state, double *unused, double *out_2122019703949867755) {
  h_28(state, unused, out_2122019703949867755);
}
void car_H_28(double *state, double *unused, double *out_3799332779610200367) {
  H_28(state, unused, out_3799332779610200367);
}
void car_h_31(double *state, double *unused, double *out_3068656258173132582) {
  h_31(state, unused, out_3068656258173132582);
}
void car_H_31(double *state, double *unused, double *out_1485456072750682430) {
  H_31(state, unused, out_1485456072750682430);
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

ekf_lib_init(car)
