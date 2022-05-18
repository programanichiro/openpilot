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
void err_fun(double *nom_x, double *delta_x, double *out_5671110135909666668) {
   out_5671110135909666668[0] = delta_x[0] + nom_x[0];
   out_5671110135909666668[1] = delta_x[1] + nom_x[1];
   out_5671110135909666668[2] = delta_x[2] + nom_x[2];
   out_5671110135909666668[3] = delta_x[3] + nom_x[3];
   out_5671110135909666668[4] = delta_x[4] + nom_x[4];
   out_5671110135909666668[5] = delta_x[5] + nom_x[5];
   out_5671110135909666668[6] = delta_x[6] + nom_x[6];
   out_5671110135909666668[7] = delta_x[7] + nom_x[7];
   out_5671110135909666668[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_3548638300579484155) {
   out_3548638300579484155[0] = -nom_x[0] + true_x[0];
   out_3548638300579484155[1] = -nom_x[1] + true_x[1];
   out_3548638300579484155[2] = -nom_x[2] + true_x[2];
   out_3548638300579484155[3] = -nom_x[3] + true_x[3];
   out_3548638300579484155[4] = -nom_x[4] + true_x[4];
   out_3548638300579484155[5] = -nom_x[5] + true_x[5];
   out_3548638300579484155[6] = -nom_x[6] + true_x[6];
   out_3548638300579484155[7] = -nom_x[7] + true_x[7];
   out_3548638300579484155[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_5423950865144251747) {
   out_5423950865144251747[0] = 1.0;
   out_5423950865144251747[1] = 0;
   out_5423950865144251747[2] = 0;
   out_5423950865144251747[3] = 0;
   out_5423950865144251747[4] = 0;
   out_5423950865144251747[5] = 0;
   out_5423950865144251747[6] = 0;
   out_5423950865144251747[7] = 0;
   out_5423950865144251747[8] = 0;
   out_5423950865144251747[9] = 0;
   out_5423950865144251747[10] = 1.0;
   out_5423950865144251747[11] = 0;
   out_5423950865144251747[12] = 0;
   out_5423950865144251747[13] = 0;
   out_5423950865144251747[14] = 0;
   out_5423950865144251747[15] = 0;
   out_5423950865144251747[16] = 0;
   out_5423950865144251747[17] = 0;
   out_5423950865144251747[18] = 0;
   out_5423950865144251747[19] = 0;
   out_5423950865144251747[20] = 1.0;
   out_5423950865144251747[21] = 0;
   out_5423950865144251747[22] = 0;
   out_5423950865144251747[23] = 0;
   out_5423950865144251747[24] = 0;
   out_5423950865144251747[25] = 0;
   out_5423950865144251747[26] = 0;
   out_5423950865144251747[27] = 0;
   out_5423950865144251747[28] = 0;
   out_5423950865144251747[29] = 0;
   out_5423950865144251747[30] = 1.0;
   out_5423950865144251747[31] = 0;
   out_5423950865144251747[32] = 0;
   out_5423950865144251747[33] = 0;
   out_5423950865144251747[34] = 0;
   out_5423950865144251747[35] = 0;
   out_5423950865144251747[36] = 0;
   out_5423950865144251747[37] = 0;
   out_5423950865144251747[38] = 0;
   out_5423950865144251747[39] = 0;
   out_5423950865144251747[40] = 1.0;
   out_5423950865144251747[41] = 0;
   out_5423950865144251747[42] = 0;
   out_5423950865144251747[43] = 0;
   out_5423950865144251747[44] = 0;
   out_5423950865144251747[45] = 0;
   out_5423950865144251747[46] = 0;
   out_5423950865144251747[47] = 0;
   out_5423950865144251747[48] = 0;
   out_5423950865144251747[49] = 0;
   out_5423950865144251747[50] = 1.0;
   out_5423950865144251747[51] = 0;
   out_5423950865144251747[52] = 0;
   out_5423950865144251747[53] = 0;
   out_5423950865144251747[54] = 0;
   out_5423950865144251747[55] = 0;
   out_5423950865144251747[56] = 0;
   out_5423950865144251747[57] = 0;
   out_5423950865144251747[58] = 0;
   out_5423950865144251747[59] = 0;
   out_5423950865144251747[60] = 1.0;
   out_5423950865144251747[61] = 0;
   out_5423950865144251747[62] = 0;
   out_5423950865144251747[63] = 0;
   out_5423950865144251747[64] = 0;
   out_5423950865144251747[65] = 0;
   out_5423950865144251747[66] = 0;
   out_5423950865144251747[67] = 0;
   out_5423950865144251747[68] = 0;
   out_5423950865144251747[69] = 0;
   out_5423950865144251747[70] = 1.0;
   out_5423950865144251747[71] = 0;
   out_5423950865144251747[72] = 0;
   out_5423950865144251747[73] = 0;
   out_5423950865144251747[74] = 0;
   out_5423950865144251747[75] = 0;
   out_5423950865144251747[76] = 0;
   out_5423950865144251747[77] = 0;
   out_5423950865144251747[78] = 0;
   out_5423950865144251747[79] = 0;
   out_5423950865144251747[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_4814793839893572125) {
   out_4814793839893572125[0] = state[0];
   out_4814793839893572125[1] = state[1];
   out_4814793839893572125[2] = state[2];
   out_4814793839893572125[3] = state[3];
   out_4814793839893572125[4] = state[4];
   out_4814793839893572125[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_4814793839893572125[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_4814793839893572125[7] = state[7];
   out_4814793839893572125[8] = state[8];
}
void F_fun(double *state, double dt, double *out_4081315827943822484) {
   out_4081315827943822484[0] = 1;
   out_4081315827943822484[1] = 0;
   out_4081315827943822484[2] = 0;
   out_4081315827943822484[3] = 0;
   out_4081315827943822484[4] = 0;
   out_4081315827943822484[5] = 0;
   out_4081315827943822484[6] = 0;
   out_4081315827943822484[7] = 0;
   out_4081315827943822484[8] = 0;
   out_4081315827943822484[9] = 0;
   out_4081315827943822484[10] = 1;
   out_4081315827943822484[11] = 0;
   out_4081315827943822484[12] = 0;
   out_4081315827943822484[13] = 0;
   out_4081315827943822484[14] = 0;
   out_4081315827943822484[15] = 0;
   out_4081315827943822484[16] = 0;
   out_4081315827943822484[17] = 0;
   out_4081315827943822484[18] = 0;
   out_4081315827943822484[19] = 0;
   out_4081315827943822484[20] = 1;
   out_4081315827943822484[21] = 0;
   out_4081315827943822484[22] = 0;
   out_4081315827943822484[23] = 0;
   out_4081315827943822484[24] = 0;
   out_4081315827943822484[25] = 0;
   out_4081315827943822484[26] = 0;
   out_4081315827943822484[27] = 0;
   out_4081315827943822484[28] = 0;
   out_4081315827943822484[29] = 0;
   out_4081315827943822484[30] = 1;
   out_4081315827943822484[31] = 0;
   out_4081315827943822484[32] = 0;
   out_4081315827943822484[33] = 0;
   out_4081315827943822484[34] = 0;
   out_4081315827943822484[35] = 0;
   out_4081315827943822484[36] = 0;
   out_4081315827943822484[37] = 0;
   out_4081315827943822484[38] = 0;
   out_4081315827943822484[39] = 0;
   out_4081315827943822484[40] = 1;
   out_4081315827943822484[41] = 0;
   out_4081315827943822484[42] = 0;
   out_4081315827943822484[43] = 0;
   out_4081315827943822484[44] = 0;
   out_4081315827943822484[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_4081315827943822484[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_4081315827943822484[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_4081315827943822484[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_4081315827943822484[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_4081315827943822484[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_4081315827943822484[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_4081315827943822484[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_4081315827943822484[53] = -9.8000000000000007*dt;
   out_4081315827943822484[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_4081315827943822484[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_4081315827943822484[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4081315827943822484[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4081315827943822484[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_4081315827943822484[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_4081315827943822484[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_4081315827943822484[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4081315827943822484[62] = 0;
   out_4081315827943822484[63] = 0;
   out_4081315827943822484[64] = 0;
   out_4081315827943822484[65] = 0;
   out_4081315827943822484[66] = 0;
   out_4081315827943822484[67] = 0;
   out_4081315827943822484[68] = 0;
   out_4081315827943822484[69] = 0;
   out_4081315827943822484[70] = 1;
   out_4081315827943822484[71] = 0;
   out_4081315827943822484[72] = 0;
   out_4081315827943822484[73] = 0;
   out_4081315827943822484[74] = 0;
   out_4081315827943822484[75] = 0;
   out_4081315827943822484[76] = 0;
   out_4081315827943822484[77] = 0;
   out_4081315827943822484[78] = 0;
   out_4081315827943822484[79] = 0;
   out_4081315827943822484[80] = 1;
}
void h_25(double *state, double *unused, double *out_2182817408992859716) {
   out_2182817408992859716[0] = state[6];
}
void H_25(double *state, double *unused, double *out_4915178676720596354) {
   out_4915178676720596354[0] = 0;
   out_4915178676720596354[1] = 0;
   out_4915178676720596354[2] = 0;
   out_4915178676720596354[3] = 0;
   out_4915178676720596354[4] = 0;
   out_4915178676720596354[5] = 0;
   out_4915178676720596354[6] = 1;
   out_4915178676720596354[7] = 0;
   out_4915178676720596354[8] = 0;
}
void h_24(double *state, double *unused, double *out_5657492609903548931) {
   out_5657492609903548931[0] = state[4];
   out_5657492609903548931[1] = state[5];
}
void H_24(double *state, double *unused, double *out_4386698687038078100) {
   out_4386698687038078100[0] = 0;
   out_4386698687038078100[1] = 0;
   out_4386698687038078100[2] = 0;
   out_4386698687038078100[3] = 0;
   out_4386698687038078100[4] = 1;
   out_4386698687038078100[5] = 0;
   out_4386698687038078100[6] = 0;
   out_4386698687038078100[7] = 0;
   out_4386698687038078100[8] = 0;
   out_4386698687038078100[9] = 0;
   out_4386698687038078100[10] = 0;
   out_4386698687038078100[11] = 0;
   out_4386698687038078100[12] = 0;
   out_4386698687038078100[13] = 0;
   out_4386698687038078100[14] = 1;
   out_4386698687038078100[15] = 0;
   out_4386698687038078100[16] = 0;
   out_4386698687038078100[17] = 0;
}
void h_30(double *state, double *unused, double *out_5702259543313505322) {
   out_5702259543313505322[0] = state[4];
}
void H_30(double *state, double *unused, double *out_2396845718213347727) {
   out_2396845718213347727[0] = 0;
   out_2396845718213347727[1] = 0;
   out_2396845718213347727[2] = 0;
   out_2396845718213347727[3] = 0;
   out_2396845718213347727[4] = 1;
   out_2396845718213347727[5] = 0;
   out_2396845718213347727[6] = 0;
   out_2396845718213347727[7] = 0;
   out_2396845718213347727[8] = 0;
}
void h_26(double *state, double *unused, double *out_7406421596062331584) {
   out_7406421596062331584[0] = state[7];
}
void H_26(double *state, double *unused, double *out_8656681995594652578) {
   out_8656681995594652578[0] = 0;
   out_8656681995594652578[1] = 0;
   out_8656681995594652578[2] = 0;
   out_8656681995594652578[3] = 0;
   out_8656681995594652578[4] = 0;
   out_8656681995594652578[5] = 0;
   out_8656681995594652578[6] = 0;
   out_8656681995594652578[7] = 1;
   out_8656681995594652578[8] = 0;
}
void h_27(double *state, double *unused, double *out_8581858926406708862) {
   out_8581858926406708862[0] = state[3];
}
void H_27(double *state, double *unused, double *out_4571609030013772638) {
   out_4571609030013772638[0] = 0;
   out_4571609030013772638[1] = 0;
   out_4571609030013772638[2] = 0;
   out_4571609030013772638[3] = 1;
   out_4571609030013772638[4] = 0;
   out_4571609030013772638[5] = 0;
   out_4571609030013772638[6] = 0;
   out_4571609030013772638[7] = 0;
   out_4571609030013772638[8] = 0;
}
void h_29(double *state, double *unused, double *out_4944409398152686512) {
   out_4944409398152686512[0] = state[1];
}
void H_29(double *state, double *unused, double *out_1886614373898955543) {
   out_1886614373898955543[0] = 0;
   out_1886614373898955543[1] = 1;
   out_1886614373898955543[2] = 0;
   out_1886614373898955543[3] = 0;
   out_1886614373898955543[4] = 0;
   out_1886614373898955543[5] = 0;
   out_1886614373898955543[6] = 0;
   out_1886614373898955543[7] = 0;
   out_1886614373898955543[8] = 0;
}
void h_28(double *state, double *unused, double *out_3580185658745863217) {
   out_3580185658745863217[0] = state[0];
}
void H_28(double *state, double *unused, double *out_6969013390968486117) {
   out_6969013390968486117[0] = 1;
   out_6969013390968486117[1] = 0;
   out_6969013390968486117[2] = 0;
   out_6969013390968486117[3] = 0;
   out_6969013390968486117[4] = 0;
   out_6969013390968486117[5] = 0;
   out_6969013390968486117[6] = 0;
   out_6969013390968486117[7] = 0;
   out_6969013390968486117[8] = 0;
}
void h_31(double *state, double *unused, double *out_6313843272270880276) {
   out_6313843272270880276[0] = state[8];
}
void H_31(double *state, double *unused, double *out_9163853975881547562) {
   out_9163853975881547562[0] = 0;
   out_9163853975881547562[1] = 0;
   out_9163853975881547562[2] = 0;
   out_9163853975881547562[3] = 0;
   out_9163853975881547562[4] = 0;
   out_9163853975881547562[5] = 0;
   out_9163853975881547562[6] = 0;
   out_9163853975881547562[7] = 0;
   out_9163853975881547562[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_5671110135909666668) {
  err_fun(nom_x, delta_x, out_5671110135909666668);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_3548638300579484155) {
  inv_err_fun(nom_x, true_x, out_3548638300579484155);
}
void car_H_mod_fun(double *state, double *out_5423950865144251747) {
  H_mod_fun(state, out_5423950865144251747);
}
void car_f_fun(double *state, double dt, double *out_4814793839893572125) {
  f_fun(state,  dt, out_4814793839893572125);
}
void car_F_fun(double *state, double dt, double *out_4081315827943822484) {
  F_fun(state,  dt, out_4081315827943822484);
}
void car_h_25(double *state, double *unused, double *out_2182817408992859716) {
  h_25(state, unused, out_2182817408992859716);
}
void car_H_25(double *state, double *unused, double *out_4915178676720596354) {
  H_25(state, unused, out_4915178676720596354);
}
void car_h_24(double *state, double *unused, double *out_5657492609903548931) {
  h_24(state, unused, out_5657492609903548931);
}
void car_H_24(double *state, double *unused, double *out_4386698687038078100) {
  H_24(state, unused, out_4386698687038078100);
}
void car_h_30(double *state, double *unused, double *out_5702259543313505322) {
  h_30(state, unused, out_5702259543313505322);
}
void car_H_30(double *state, double *unused, double *out_2396845718213347727) {
  H_30(state, unused, out_2396845718213347727);
}
void car_h_26(double *state, double *unused, double *out_7406421596062331584) {
  h_26(state, unused, out_7406421596062331584);
}
void car_H_26(double *state, double *unused, double *out_8656681995594652578) {
  H_26(state, unused, out_8656681995594652578);
}
void car_h_27(double *state, double *unused, double *out_8581858926406708862) {
  h_27(state, unused, out_8581858926406708862);
}
void car_H_27(double *state, double *unused, double *out_4571609030013772638) {
  H_27(state, unused, out_4571609030013772638);
}
void car_h_29(double *state, double *unused, double *out_4944409398152686512) {
  h_29(state, unused, out_4944409398152686512);
}
void car_H_29(double *state, double *unused, double *out_1886614373898955543) {
  H_29(state, unused, out_1886614373898955543);
}
void car_h_28(double *state, double *unused, double *out_3580185658745863217) {
  h_28(state, unused, out_3580185658745863217);
}
void car_H_28(double *state, double *unused, double *out_6969013390968486117) {
  H_28(state, unused, out_6969013390968486117);
}
void car_h_31(double *state, double *unused, double *out_6313843272270880276) {
  h_31(state, unused, out_6313843272270880276);
}
void car_H_31(double *state, double *unused, double *out_9163853975881547562) {
  H_31(state, unused, out_9163853975881547562);
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
