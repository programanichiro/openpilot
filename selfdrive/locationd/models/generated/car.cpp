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
void err_fun(double *nom_x, double *delta_x, double *out_4325689932387369841) {
   out_4325689932387369841[0] = delta_x[0] + nom_x[0];
   out_4325689932387369841[1] = delta_x[1] + nom_x[1];
   out_4325689932387369841[2] = delta_x[2] + nom_x[2];
   out_4325689932387369841[3] = delta_x[3] + nom_x[3];
   out_4325689932387369841[4] = delta_x[4] + nom_x[4];
   out_4325689932387369841[5] = delta_x[5] + nom_x[5];
   out_4325689932387369841[6] = delta_x[6] + nom_x[6];
   out_4325689932387369841[7] = delta_x[7] + nom_x[7];
   out_4325689932387369841[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_1119892225369903103) {
   out_1119892225369903103[0] = -nom_x[0] + true_x[0];
   out_1119892225369903103[1] = -nom_x[1] + true_x[1];
   out_1119892225369903103[2] = -nom_x[2] + true_x[2];
   out_1119892225369903103[3] = -nom_x[3] + true_x[3];
   out_1119892225369903103[4] = -nom_x[4] + true_x[4];
   out_1119892225369903103[5] = -nom_x[5] + true_x[5];
   out_1119892225369903103[6] = -nom_x[6] + true_x[6];
   out_1119892225369903103[7] = -nom_x[7] + true_x[7];
   out_1119892225369903103[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_4832036451820639239) {
   out_4832036451820639239[0] = 1.0;
   out_4832036451820639239[1] = 0;
   out_4832036451820639239[2] = 0;
   out_4832036451820639239[3] = 0;
   out_4832036451820639239[4] = 0;
   out_4832036451820639239[5] = 0;
   out_4832036451820639239[6] = 0;
   out_4832036451820639239[7] = 0;
   out_4832036451820639239[8] = 0;
   out_4832036451820639239[9] = 0;
   out_4832036451820639239[10] = 1.0;
   out_4832036451820639239[11] = 0;
   out_4832036451820639239[12] = 0;
   out_4832036451820639239[13] = 0;
   out_4832036451820639239[14] = 0;
   out_4832036451820639239[15] = 0;
   out_4832036451820639239[16] = 0;
   out_4832036451820639239[17] = 0;
   out_4832036451820639239[18] = 0;
   out_4832036451820639239[19] = 0;
   out_4832036451820639239[20] = 1.0;
   out_4832036451820639239[21] = 0;
   out_4832036451820639239[22] = 0;
   out_4832036451820639239[23] = 0;
   out_4832036451820639239[24] = 0;
   out_4832036451820639239[25] = 0;
   out_4832036451820639239[26] = 0;
   out_4832036451820639239[27] = 0;
   out_4832036451820639239[28] = 0;
   out_4832036451820639239[29] = 0;
   out_4832036451820639239[30] = 1.0;
   out_4832036451820639239[31] = 0;
   out_4832036451820639239[32] = 0;
   out_4832036451820639239[33] = 0;
   out_4832036451820639239[34] = 0;
   out_4832036451820639239[35] = 0;
   out_4832036451820639239[36] = 0;
   out_4832036451820639239[37] = 0;
   out_4832036451820639239[38] = 0;
   out_4832036451820639239[39] = 0;
   out_4832036451820639239[40] = 1.0;
   out_4832036451820639239[41] = 0;
   out_4832036451820639239[42] = 0;
   out_4832036451820639239[43] = 0;
   out_4832036451820639239[44] = 0;
   out_4832036451820639239[45] = 0;
   out_4832036451820639239[46] = 0;
   out_4832036451820639239[47] = 0;
   out_4832036451820639239[48] = 0;
   out_4832036451820639239[49] = 0;
   out_4832036451820639239[50] = 1.0;
   out_4832036451820639239[51] = 0;
   out_4832036451820639239[52] = 0;
   out_4832036451820639239[53] = 0;
   out_4832036451820639239[54] = 0;
   out_4832036451820639239[55] = 0;
   out_4832036451820639239[56] = 0;
   out_4832036451820639239[57] = 0;
   out_4832036451820639239[58] = 0;
   out_4832036451820639239[59] = 0;
   out_4832036451820639239[60] = 1.0;
   out_4832036451820639239[61] = 0;
   out_4832036451820639239[62] = 0;
   out_4832036451820639239[63] = 0;
   out_4832036451820639239[64] = 0;
   out_4832036451820639239[65] = 0;
   out_4832036451820639239[66] = 0;
   out_4832036451820639239[67] = 0;
   out_4832036451820639239[68] = 0;
   out_4832036451820639239[69] = 0;
   out_4832036451820639239[70] = 1.0;
   out_4832036451820639239[71] = 0;
   out_4832036451820639239[72] = 0;
   out_4832036451820639239[73] = 0;
   out_4832036451820639239[74] = 0;
   out_4832036451820639239[75] = 0;
   out_4832036451820639239[76] = 0;
   out_4832036451820639239[77] = 0;
   out_4832036451820639239[78] = 0;
   out_4832036451820639239[79] = 0;
   out_4832036451820639239[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_6035807853235887415) {
   out_6035807853235887415[0] = state[0];
   out_6035807853235887415[1] = state[1];
   out_6035807853235887415[2] = state[2];
   out_6035807853235887415[3] = state[3];
   out_6035807853235887415[4] = state[4];
   out_6035807853235887415[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_6035807853235887415[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_6035807853235887415[7] = state[7];
   out_6035807853235887415[8] = state[8];
}
void F_fun(double *state, double dt, double *out_8459570567044433633) {
   out_8459570567044433633[0] = 1;
   out_8459570567044433633[1] = 0;
   out_8459570567044433633[2] = 0;
   out_8459570567044433633[3] = 0;
   out_8459570567044433633[4] = 0;
   out_8459570567044433633[5] = 0;
   out_8459570567044433633[6] = 0;
   out_8459570567044433633[7] = 0;
   out_8459570567044433633[8] = 0;
   out_8459570567044433633[9] = 0;
   out_8459570567044433633[10] = 1;
   out_8459570567044433633[11] = 0;
   out_8459570567044433633[12] = 0;
   out_8459570567044433633[13] = 0;
   out_8459570567044433633[14] = 0;
   out_8459570567044433633[15] = 0;
   out_8459570567044433633[16] = 0;
   out_8459570567044433633[17] = 0;
   out_8459570567044433633[18] = 0;
   out_8459570567044433633[19] = 0;
   out_8459570567044433633[20] = 1;
   out_8459570567044433633[21] = 0;
   out_8459570567044433633[22] = 0;
   out_8459570567044433633[23] = 0;
   out_8459570567044433633[24] = 0;
   out_8459570567044433633[25] = 0;
   out_8459570567044433633[26] = 0;
   out_8459570567044433633[27] = 0;
   out_8459570567044433633[28] = 0;
   out_8459570567044433633[29] = 0;
   out_8459570567044433633[30] = 1;
   out_8459570567044433633[31] = 0;
   out_8459570567044433633[32] = 0;
   out_8459570567044433633[33] = 0;
   out_8459570567044433633[34] = 0;
   out_8459570567044433633[35] = 0;
   out_8459570567044433633[36] = 0;
   out_8459570567044433633[37] = 0;
   out_8459570567044433633[38] = 0;
   out_8459570567044433633[39] = 0;
   out_8459570567044433633[40] = 1;
   out_8459570567044433633[41] = 0;
   out_8459570567044433633[42] = 0;
   out_8459570567044433633[43] = 0;
   out_8459570567044433633[44] = 0;
   out_8459570567044433633[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_8459570567044433633[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_8459570567044433633[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8459570567044433633[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8459570567044433633[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_8459570567044433633[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_8459570567044433633[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_8459570567044433633[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_8459570567044433633[53] = -9.8000000000000007*dt;
   out_8459570567044433633[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_8459570567044433633[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_8459570567044433633[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8459570567044433633[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8459570567044433633[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_8459570567044433633[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_8459570567044433633[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_8459570567044433633[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8459570567044433633[62] = 0;
   out_8459570567044433633[63] = 0;
   out_8459570567044433633[64] = 0;
   out_8459570567044433633[65] = 0;
   out_8459570567044433633[66] = 0;
   out_8459570567044433633[67] = 0;
   out_8459570567044433633[68] = 0;
   out_8459570567044433633[69] = 0;
   out_8459570567044433633[70] = 1;
   out_8459570567044433633[71] = 0;
   out_8459570567044433633[72] = 0;
   out_8459570567044433633[73] = 0;
   out_8459570567044433633[74] = 0;
   out_8459570567044433633[75] = 0;
   out_8459570567044433633[76] = 0;
   out_8459570567044433633[77] = 0;
   out_8459570567044433633[78] = 0;
   out_8459570567044433633[79] = 0;
   out_8459570567044433633[80] = 1;
}
void h_25(double *state, double *unused, double *out_4434622194633419438) {
   out_4434622194633419438[0] = state[6];
}
void H_25(double *state, double *unused, double *out_279236249526357609) {
   out_279236249526357609[0] = 0;
   out_279236249526357609[1] = 0;
   out_279236249526357609[2] = 0;
   out_279236249526357609[3] = 0;
   out_279236249526357609[4] = 0;
   out_279236249526357609[5] = 0;
   out_279236249526357609[6] = 1;
   out_279236249526357609[7] = 0;
   out_279236249526357609[8] = 0;
}
void h_24(double *state, double *unused, double *out_5396490682701100739) {
   out_5396490682701100739[0] = state[4];
   out_5396490682701100739[1] = state[5];
}
void H_24(double *state, double *unused, double *out_6854808056117875710) {
   out_6854808056117875710[0] = 0;
   out_6854808056117875710[1] = 0;
   out_6854808056117875710[2] = 0;
   out_6854808056117875710[3] = 0;
   out_6854808056117875710[4] = 1;
   out_6854808056117875710[5] = 0;
   out_6854808056117875710[6] = 0;
   out_6854808056117875710[7] = 0;
   out_6854808056117875710[8] = 0;
   out_6854808056117875710[9] = 0;
   out_6854808056117875710[10] = 0;
   out_6854808056117875710[11] = 0;
   out_6854808056117875710[12] = 0;
   out_6854808056117875710[13] = 0;
   out_6854808056117875710[14] = 1;
   out_6854808056117875710[15] = 0;
   out_6854808056117875710[16] = 0;
   out_6854808056117875710[17] = 0;
}
void h_30(double *state, double *unused, double *out_8072071722887441788) {
   out_8072071722887441788[0] = state[4];
}
void H_30(double *state, double *unused, double *out_7195926591017974364) {
   out_7195926591017974364[0] = 0;
   out_7195926591017974364[1] = 0;
   out_7195926591017974364[2] = 0;
   out_7195926591017974364[3] = 0;
   out_7195926591017974364[4] = 1;
   out_7195926591017974364[5] = 0;
   out_7195926591017974364[6] = 0;
   out_7195926591017974364[7] = 0;
   out_7195926591017974364[8] = 0;
}
void h_26(double *state, double *unused, double *out_7337886299377777143) {
   out_7337886299377777143[0] = state[7];
}
void H_26(double *state, double *unused, double *out_3462267069347698615) {
   out_3462267069347698615[0] = 0;
   out_3462267069347698615[1] = 0;
   out_3462267069347698615[2] = 0;
   out_3462267069347698615[3] = 0;
   out_3462267069347698615[4] = 0;
   out_3462267069347698615[5] = 0;
   out_3462267069347698615[6] = 0;
   out_3462267069347698615[7] = 1;
   out_3462267069347698615[8] = 0;
}
void h_27(double *state, double *unused, double *out_3764718201522586999) {
   out_3764718201522586999[0] = state[3];
}
void H_27(double *state, double *unused, double *out_5021163279217549453) {
   out_5021163279217549453[0] = 0;
   out_5021163279217549453[1] = 0;
   out_5021163279217549453[2] = 0;
   out_5021163279217549453[3] = 1;
   out_5021163279217549453[4] = 0;
   out_5021163279217549453[5] = 0;
   out_5021163279217549453[6] = 0;
   out_5021163279217549453[7] = 0;
   out_5021163279217549453[8] = 0;
}
void h_29(double *state, double *unused, double *out_5372999083161264183) {
   out_5372999083161264183[0] = state[1];
}
void H_29(double *state, double *unused, double *out_7706157935332366548) {
   out_7706157935332366548[0] = 0;
   out_7706157935332366548[1] = 1;
   out_7706157935332366548[2] = 0;
   out_7706157935332366548[3] = 0;
   out_7706157935332366548[4] = 0;
   out_7706157935332366548[5] = 0;
   out_7706157935332366548[6] = 0;
   out_7706157935332366548[7] = 0;
   out_7706157935332366548[8] = 0;
}
void h_28(double *state, double *unused, double *out_3944031092696943810) {
   out_3944031092696943810[0] = state[0];
}
void H_28(double *state, double *unused, double *out_2623758918262835974) {
   out_2623758918262835974[0] = 1;
   out_2623758918262835974[1] = 0;
   out_2623758918262835974[2] = 0;
   out_2623758918262835974[3] = 0;
   out_2623758918262835974[4] = 0;
   out_2623758918262835974[5] = 0;
   out_2623758918262835974[6] = 0;
   out_2623758918262835974[7] = 0;
   out_2623758918262835974[8] = 0;
}
void h_31(double *state, double *unused, double *out_6526310190682413795) {
   out_6526310190682413795[0] = state[8];
}
void H_31(double *state, double *unused, double *out_309882211403318037) {
   out_309882211403318037[0] = 0;
   out_309882211403318037[1] = 0;
   out_309882211403318037[2] = 0;
   out_309882211403318037[3] = 0;
   out_309882211403318037[4] = 0;
   out_309882211403318037[5] = 0;
   out_309882211403318037[6] = 0;
   out_309882211403318037[7] = 0;
   out_309882211403318037[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_4325689932387369841) {
  err_fun(nom_x, delta_x, out_4325689932387369841);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1119892225369903103) {
  inv_err_fun(nom_x, true_x, out_1119892225369903103);
}
void car_H_mod_fun(double *state, double *out_4832036451820639239) {
  H_mod_fun(state, out_4832036451820639239);
}
void car_f_fun(double *state, double dt, double *out_6035807853235887415) {
  f_fun(state,  dt, out_6035807853235887415);
}
void car_F_fun(double *state, double dt, double *out_8459570567044433633) {
  F_fun(state,  dt, out_8459570567044433633);
}
void car_h_25(double *state, double *unused, double *out_4434622194633419438) {
  h_25(state, unused, out_4434622194633419438);
}
void car_H_25(double *state, double *unused, double *out_279236249526357609) {
  H_25(state, unused, out_279236249526357609);
}
void car_h_24(double *state, double *unused, double *out_5396490682701100739) {
  h_24(state, unused, out_5396490682701100739);
}
void car_H_24(double *state, double *unused, double *out_6854808056117875710) {
  H_24(state, unused, out_6854808056117875710);
}
void car_h_30(double *state, double *unused, double *out_8072071722887441788) {
  h_30(state, unused, out_8072071722887441788);
}
void car_H_30(double *state, double *unused, double *out_7195926591017974364) {
  H_30(state, unused, out_7195926591017974364);
}
void car_h_26(double *state, double *unused, double *out_7337886299377777143) {
  h_26(state, unused, out_7337886299377777143);
}
void car_H_26(double *state, double *unused, double *out_3462267069347698615) {
  H_26(state, unused, out_3462267069347698615);
}
void car_h_27(double *state, double *unused, double *out_3764718201522586999) {
  h_27(state, unused, out_3764718201522586999);
}
void car_H_27(double *state, double *unused, double *out_5021163279217549453) {
  H_27(state, unused, out_5021163279217549453);
}
void car_h_29(double *state, double *unused, double *out_5372999083161264183) {
  h_29(state, unused, out_5372999083161264183);
}
void car_H_29(double *state, double *unused, double *out_7706157935332366548) {
  H_29(state, unused, out_7706157935332366548);
}
void car_h_28(double *state, double *unused, double *out_3944031092696943810) {
  h_28(state, unused, out_3944031092696943810);
}
void car_H_28(double *state, double *unused, double *out_2623758918262835974) {
  H_28(state, unused, out_2623758918262835974);
}
void car_h_31(double *state, double *unused, double *out_6526310190682413795) {
  h_31(state, unused, out_6526310190682413795);
}
void car_H_31(double *state, double *unused, double *out_309882211403318037) {
  H_31(state, unused, out_309882211403318037);
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
