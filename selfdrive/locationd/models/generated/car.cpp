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
void err_fun(double *nom_x, double *delta_x, double *out_3171762008471037189) {
   out_3171762008471037189[0] = delta_x[0] + nom_x[0];
   out_3171762008471037189[1] = delta_x[1] + nom_x[1];
   out_3171762008471037189[2] = delta_x[2] + nom_x[2];
   out_3171762008471037189[3] = delta_x[3] + nom_x[3];
   out_3171762008471037189[4] = delta_x[4] + nom_x[4];
   out_3171762008471037189[5] = delta_x[5] + nom_x[5];
   out_3171762008471037189[6] = delta_x[6] + nom_x[6];
   out_3171762008471037189[7] = delta_x[7] + nom_x[7];
   out_3171762008471037189[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_4110702914314724221) {
   out_4110702914314724221[0] = -nom_x[0] + true_x[0];
   out_4110702914314724221[1] = -nom_x[1] + true_x[1];
   out_4110702914314724221[2] = -nom_x[2] + true_x[2];
   out_4110702914314724221[3] = -nom_x[3] + true_x[3];
   out_4110702914314724221[4] = -nom_x[4] + true_x[4];
   out_4110702914314724221[5] = -nom_x[5] + true_x[5];
   out_4110702914314724221[6] = -nom_x[6] + true_x[6];
   out_4110702914314724221[7] = -nom_x[7] + true_x[7];
   out_4110702914314724221[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_5033876641403212536) {
   out_5033876641403212536[0] = 1.0;
   out_5033876641403212536[1] = 0;
   out_5033876641403212536[2] = 0;
   out_5033876641403212536[3] = 0;
   out_5033876641403212536[4] = 0;
   out_5033876641403212536[5] = 0;
   out_5033876641403212536[6] = 0;
   out_5033876641403212536[7] = 0;
   out_5033876641403212536[8] = 0;
   out_5033876641403212536[9] = 0;
   out_5033876641403212536[10] = 1.0;
   out_5033876641403212536[11] = 0;
   out_5033876641403212536[12] = 0;
   out_5033876641403212536[13] = 0;
   out_5033876641403212536[14] = 0;
   out_5033876641403212536[15] = 0;
   out_5033876641403212536[16] = 0;
   out_5033876641403212536[17] = 0;
   out_5033876641403212536[18] = 0;
   out_5033876641403212536[19] = 0;
   out_5033876641403212536[20] = 1.0;
   out_5033876641403212536[21] = 0;
   out_5033876641403212536[22] = 0;
   out_5033876641403212536[23] = 0;
   out_5033876641403212536[24] = 0;
   out_5033876641403212536[25] = 0;
   out_5033876641403212536[26] = 0;
   out_5033876641403212536[27] = 0;
   out_5033876641403212536[28] = 0;
   out_5033876641403212536[29] = 0;
   out_5033876641403212536[30] = 1.0;
   out_5033876641403212536[31] = 0;
   out_5033876641403212536[32] = 0;
   out_5033876641403212536[33] = 0;
   out_5033876641403212536[34] = 0;
   out_5033876641403212536[35] = 0;
   out_5033876641403212536[36] = 0;
   out_5033876641403212536[37] = 0;
   out_5033876641403212536[38] = 0;
   out_5033876641403212536[39] = 0;
   out_5033876641403212536[40] = 1.0;
   out_5033876641403212536[41] = 0;
   out_5033876641403212536[42] = 0;
   out_5033876641403212536[43] = 0;
   out_5033876641403212536[44] = 0;
   out_5033876641403212536[45] = 0;
   out_5033876641403212536[46] = 0;
   out_5033876641403212536[47] = 0;
   out_5033876641403212536[48] = 0;
   out_5033876641403212536[49] = 0;
   out_5033876641403212536[50] = 1.0;
   out_5033876641403212536[51] = 0;
   out_5033876641403212536[52] = 0;
   out_5033876641403212536[53] = 0;
   out_5033876641403212536[54] = 0;
   out_5033876641403212536[55] = 0;
   out_5033876641403212536[56] = 0;
   out_5033876641403212536[57] = 0;
   out_5033876641403212536[58] = 0;
   out_5033876641403212536[59] = 0;
   out_5033876641403212536[60] = 1.0;
   out_5033876641403212536[61] = 0;
   out_5033876641403212536[62] = 0;
   out_5033876641403212536[63] = 0;
   out_5033876641403212536[64] = 0;
   out_5033876641403212536[65] = 0;
   out_5033876641403212536[66] = 0;
   out_5033876641403212536[67] = 0;
   out_5033876641403212536[68] = 0;
   out_5033876641403212536[69] = 0;
   out_5033876641403212536[70] = 1.0;
   out_5033876641403212536[71] = 0;
   out_5033876641403212536[72] = 0;
   out_5033876641403212536[73] = 0;
   out_5033876641403212536[74] = 0;
   out_5033876641403212536[75] = 0;
   out_5033876641403212536[76] = 0;
   out_5033876641403212536[77] = 0;
   out_5033876641403212536[78] = 0;
   out_5033876641403212536[79] = 0;
   out_5033876641403212536[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_3454824215855402291) {
   out_3454824215855402291[0] = state[0];
   out_3454824215855402291[1] = state[1];
   out_3454824215855402291[2] = state[2];
   out_3454824215855402291[3] = state[3];
   out_3454824215855402291[4] = state[4];
   out_3454824215855402291[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_3454824215855402291[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_3454824215855402291[7] = state[7];
   out_3454824215855402291[8] = state[8];
}
void F_fun(double *state, double dt, double *out_1974642873389597258) {
   out_1974642873389597258[0] = 1;
   out_1974642873389597258[1] = 0;
   out_1974642873389597258[2] = 0;
   out_1974642873389597258[3] = 0;
   out_1974642873389597258[4] = 0;
   out_1974642873389597258[5] = 0;
   out_1974642873389597258[6] = 0;
   out_1974642873389597258[7] = 0;
   out_1974642873389597258[8] = 0;
   out_1974642873389597258[9] = 0;
   out_1974642873389597258[10] = 1;
   out_1974642873389597258[11] = 0;
   out_1974642873389597258[12] = 0;
   out_1974642873389597258[13] = 0;
   out_1974642873389597258[14] = 0;
   out_1974642873389597258[15] = 0;
   out_1974642873389597258[16] = 0;
   out_1974642873389597258[17] = 0;
   out_1974642873389597258[18] = 0;
   out_1974642873389597258[19] = 0;
   out_1974642873389597258[20] = 1;
   out_1974642873389597258[21] = 0;
   out_1974642873389597258[22] = 0;
   out_1974642873389597258[23] = 0;
   out_1974642873389597258[24] = 0;
   out_1974642873389597258[25] = 0;
   out_1974642873389597258[26] = 0;
   out_1974642873389597258[27] = 0;
   out_1974642873389597258[28] = 0;
   out_1974642873389597258[29] = 0;
   out_1974642873389597258[30] = 1;
   out_1974642873389597258[31] = 0;
   out_1974642873389597258[32] = 0;
   out_1974642873389597258[33] = 0;
   out_1974642873389597258[34] = 0;
   out_1974642873389597258[35] = 0;
   out_1974642873389597258[36] = 0;
   out_1974642873389597258[37] = 0;
   out_1974642873389597258[38] = 0;
   out_1974642873389597258[39] = 0;
   out_1974642873389597258[40] = 1;
   out_1974642873389597258[41] = 0;
   out_1974642873389597258[42] = 0;
   out_1974642873389597258[43] = 0;
   out_1974642873389597258[44] = 0;
   out_1974642873389597258[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_1974642873389597258[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_1974642873389597258[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_1974642873389597258[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_1974642873389597258[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_1974642873389597258[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_1974642873389597258[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_1974642873389597258[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_1974642873389597258[53] = -9.8000000000000007*dt;
   out_1974642873389597258[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_1974642873389597258[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_1974642873389597258[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1974642873389597258[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1974642873389597258[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_1974642873389597258[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_1974642873389597258[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_1974642873389597258[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1974642873389597258[62] = 0;
   out_1974642873389597258[63] = 0;
   out_1974642873389597258[64] = 0;
   out_1974642873389597258[65] = 0;
   out_1974642873389597258[66] = 0;
   out_1974642873389597258[67] = 0;
   out_1974642873389597258[68] = 0;
   out_1974642873389597258[69] = 0;
   out_1974642873389597258[70] = 1;
   out_1974642873389597258[71] = 0;
   out_1974642873389597258[72] = 0;
   out_1974642873389597258[73] = 0;
   out_1974642873389597258[74] = 0;
   out_1974642873389597258[75] = 0;
   out_1974642873389597258[76] = 0;
   out_1974642873389597258[77] = 0;
   out_1974642873389597258[78] = 0;
   out_1974642873389597258[79] = 0;
   out_1974642873389597258[80] = 1;
}
void h_25(double *state, double *unused, double *out_1355303837527753590) {
   out_1355303837527753590[0] = state[6];
}
void H_25(double *state, double *unused, double *out_3910608959992168667) {
   out_3910608959992168667[0] = 0;
   out_3910608959992168667[1] = 0;
   out_3910608959992168667[2] = 0;
   out_3910608959992168667[3] = 0;
   out_3910608959992168667[4] = 0;
   out_3910608959992168667[5] = 0;
   out_3910608959992168667[6] = 1;
   out_3910608959992168667[7] = 0;
   out_3910608959992168667[8] = 0;
}
void h_24(double *state, double *unused, double *out_3737888511771369975) {
   out_3737888511771369975[0] = state[4];
   out_3737888511771369975[1] = state[5];
}
void H_24(double *state, double *unused, double *out_4381066442035507391) {
   out_4381066442035507391[0] = 0;
   out_4381066442035507391[1] = 0;
   out_4381066442035507391[2] = 0;
   out_4381066442035507391[3] = 0;
   out_4381066442035507391[4] = 1;
   out_4381066442035507391[5] = 0;
   out_4381066442035507391[6] = 0;
   out_4381066442035507391[7] = 0;
   out_4381066442035507391[8] = 0;
   out_4381066442035507391[9] = 0;
   out_4381066442035507391[10] = 0;
   out_4381066442035507391[11] = 0;
   out_4381066442035507391[12] = 0;
   out_4381066442035507391[13] = 0;
   out_4381066442035507391[14] = 1;
   out_4381066442035507391[15] = 0;
   out_4381066442035507391[16] = 0;
   out_4381066442035507391[17] = 0;
}
void h_30(double *state, double *unused, double *out_753885679914390036) {
   out_753885679914390036[0] = state[4];
}
void H_30(double *state, double *unused, double *out_8438305290119776865) {
   out_8438305290119776865[0] = 0;
   out_8438305290119776865[1] = 0;
   out_8438305290119776865[2] = 0;
   out_8438305290119776865[3] = 0;
   out_8438305290119776865[4] = 1;
   out_8438305290119776865[5] = 0;
   out_8438305290119776865[6] = 0;
   out_8438305290119776865[7] = 0;
   out_8438305290119776865[8] = 0;
}
void h_26(double *state, double *unused, double *out_4247787587746455609) {
   out_4247787587746455609[0] = state[7];
}
void H_26(double *state, double *unused, double *out_7652112278866224891) {
   out_7652112278866224891[0] = 0;
   out_7652112278866224891[1] = 0;
   out_7652112278866224891[2] = 0;
   out_7652112278866224891[3] = 0;
   out_7652112278866224891[4] = 0;
   out_7652112278866224891[5] = 0;
   out_7652112278866224891[6] = 0;
   out_7652112278866224891[7] = 1;
   out_7652112278866224891[8] = 0;
}
void h_27(double *state, double *unused, double *out_6356977105188599235) {
   out_6356977105188599235[0] = state[3];
}
void H_27(double *state, double *unused, double *out_6214711218935833648) {
   out_6214711218935833648[0] = 0;
   out_6214711218935833648[1] = 0;
   out_6214711218935833648[2] = 0;
   out_6214711218935833648[3] = 1;
   out_6214711218935833648[4] = 0;
   out_6214711218935833648[5] = 0;
   out_6214711218935833648[6] = 0;
   out_6214711218935833648[7] = 0;
   out_6214711218935833648[8] = 0;
}
void h_29(double *state, double *unused, double *out_6632171167473105124) {
   out_6632171167473105124[0] = state[1];
}
void H_29(double *state, double *unused, double *out_7928073945805384681) {
   out_7928073945805384681[0] = 0;
   out_7928073945805384681[1] = 1;
   out_7928073945805384681[2] = 0;
   out_7928073945805384681[3] = 0;
   out_7928073945805384681[4] = 0;
   out_7928073945805384681[5] = 0;
   out_7928073945805384681[6] = 0;
   out_7928073945805384681[7] = 0;
   out_7928073945805384681[8] = 0;
}
void h_28(double *state, double *unused, double *out_6863859752685627434) {
   out_6863859752685627434[0] = state[0];
}
void H_28(double *state, double *unused, double *out_5436271110834636361) {
   out_5436271110834636361[0] = 1;
   out_5436271110834636361[1] = 0;
   out_5436271110834636361[2] = 0;
   out_5436271110834636361[3] = 0;
   out_5436271110834636361[4] = 0;
   out_5436271110834636361[5] = 0;
   out_5436271110834636361[6] = 0;
   out_5436271110834636361[7] = 0;
   out_5436271110834636361[8] = 0;
}
void h_31(double *state, double *unused, double *out_2282145690726268760) {
   out_2282145690726268760[0] = state[8];
}
void H_31(double *state, double *unused, double *out_3879962998115208239) {
   out_3879962998115208239[0] = 0;
   out_3879962998115208239[1] = 0;
   out_3879962998115208239[2] = 0;
   out_3879962998115208239[3] = 0;
   out_3879962998115208239[4] = 0;
   out_3879962998115208239[5] = 0;
   out_3879962998115208239[6] = 0;
   out_3879962998115208239[7] = 0;
   out_3879962998115208239[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_3171762008471037189) {
  err_fun(nom_x, delta_x, out_3171762008471037189);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4110702914314724221) {
  inv_err_fun(nom_x, true_x, out_4110702914314724221);
}
void car_H_mod_fun(double *state, double *out_5033876641403212536) {
  H_mod_fun(state, out_5033876641403212536);
}
void car_f_fun(double *state, double dt, double *out_3454824215855402291) {
  f_fun(state,  dt, out_3454824215855402291);
}
void car_F_fun(double *state, double dt, double *out_1974642873389597258) {
  F_fun(state,  dt, out_1974642873389597258);
}
void car_h_25(double *state, double *unused, double *out_1355303837527753590) {
  h_25(state, unused, out_1355303837527753590);
}
void car_H_25(double *state, double *unused, double *out_3910608959992168667) {
  H_25(state, unused, out_3910608959992168667);
}
void car_h_24(double *state, double *unused, double *out_3737888511771369975) {
  h_24(state, unused, out_3737888511771369975);
}
void car_H_24(double *state, double *unused, double *out_4381066442035507391) {
  H_24(state, unused, out_4381066442035507391);
}
void car_h_30(double *state, double *unused, double *out_753885679914390036) {
  h_30(state, unused, out_753885679914390036);
}
void car_H_30(double *state, double *unused, double *out_8438305290119776865) {
  H_30(state, unused, out_8438305290119776865);
}
void car_h_26(double *state, double *unused, double *out_4247787587746455609) {
  h_26(state, unused, out_4247787587746455609);
}
void car_H_26(double *state, double *unused, double *out_7652112278866224891) {
  H_26(state, unused, out_7652112278866224891);
}
void car_h_27(double *state, double *unused, double *out_6356977105188599235) {
  h_27(state, unused, out_6356977105188599235);
}
void car_H_27(double *state, double *unused, double *out_6214711218935833648) {
  H_27(state, unused, out_6214711218935833648);
}
void car_h_29(double *state, double *unused, double *out_6632171167473105124) {
  h_29(state, unused, out_6632171167473105124);
}
void car_H_29(double *state, double *unused, double *out_7928073945805384681) {
  H_29(state, unused, out_7928073945805384681);
}
void car_h_28(double *state, double *unused, double *out_6863859752685627434) {
  h_28(state, unused, out_6863859752685627434);
}
void car_H_28(double *state, double *unused, double *out_5436271110834636361) {
  H_28(state, unused, out_5436271110834636361);
}
void car_h_31(double *state, double *unused, double *out_2282145690726268760) {
  h_31(state, unused, out_2282145690726268760);
}
void car_H_31(double *state, double *unused, double *out_3879962998115208239) {
  H_31(state, unused, out_3879962998115208239);
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
