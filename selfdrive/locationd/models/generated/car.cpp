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
void err_fun(double *nom_x, double *delta_x, double *out_1259430672998403639) {
   out_1259430672998403639[0] = delta_x[0] + nom_x[0];
   out_1259430672998403639[1] = delta_x[1] + nom_x[1];
   out_1259430672998403639[2] = delta_x[2] + nom_x[2];
   out_1259430672998403639[3] = delta_x[3] + nom_x[3];
   out_1259430672998403639[4] = delta_x[4] + nom_x[4];
   out_1259430672998403639[5] = delta_x[5] + nom_x[5];
   out_1259430672998403639[6] = delta_x[6] + nom_x[6];
   out_1259430672998403639[7] = delta_x[7] + nom_x[7];
   out_1259430672998403639[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6486332291499723158) {
   out_6486332291499723158[0] = -nom_x[0] + true_x[0];
   out_6486332291499723158[1] = -nom_x[1] + true_x[1];
   out_6486332291499723158[2] = -nom_x[2] + true_x[2];
   out_6486332291499723158[3] = -nom_x[3] + true_x[3];
   out_6486332291499723158[4] = -nom_x[4] + true_x[4];
   out_6486332291499723158[5] = -nom_x[5] + true_x[5];
   out_6486332291499723158[6] = -nom_x[6] + true_x[6];
   out_6486332291499723158[7] = -nom_x[7] + true_x[7];
   out_6486332291499723158[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_1916627579971738044) {
   out_1916627579971738044[0] = 1.0;
   out_1916627579971738044[1] = 0;
   out_1916627579971738044[2] = 0;
   out_1916627579971738044[3] = 0;
   out_1916627579971738044[4] = 0;
   out_1916627579971738044[5] = 0;
   out_1916627579971738044[6] = 0;
   out_1916627579971738044[7] = 0;
   out_1916627579971738044[8] = 0;
   out_1916627579971738044[9] = 0;
   out_1916627579971738044[10] = 1.0;
   out_1916627579971738044[11] = 0;
   out_1916627579971738044[12] = 0;
   out_1916627579971738044[13] = 0;
   out_1916627579971738044[14] = 0;
   out_1916627579971738044[15] = 0;
   out_1916627579971738044[16] = 0;
   out_1916627579971738044[17] = 0;
   out_1916627579971738044[18] = 0;
   out_1916627579971738044[19] = 0;
   out_1916627579971738044[20] = 1.0;
   out_1916627579971738044[21] = 0;
   out_1916627579971738044[22] = 0;
   out_1916627579971738044[23] = 0;
   out_1916627579971738044[24] = 0;
   out_1916627579971738044[25] = 0;
   out_1916627579971738044[26] = 0;
   out_1916627579971738044[27] = 0;
   out_1916627579971738044[28] = 0;
   out_1916627579971738044[29] = 0;
   out_1916627579971738044[30] = 1.0;
   out_1916627579971738044[31] = 0;
   out_1916627579971738044[32] = 0;
   out_1916627579971738044[33] = 0;
   out_1916627579971738044[34] = 0;
   out_1916627579971738044[35] = 0;
   out_1916627579971738044[36] = 0;
   out_1916627579971738044[37] = 0;
   out_1916627579971738044[38] = 0;
   out_1916627579971738044[39] = 0;
   out_1916627579971738044[40] = 1.0;
   out_1916627579971738044[41] = 0;
   out_1916627579971738044[42] = 0;
   out_1916627579971738044[43] = 0;
   out_1916627579971738044[44] = 0;
   out_1916627579971738044[45] = 0;
   out_1916627579971738044[46] = 0;
   out_1916627579971738044[47] = 0;
   out_1916627579971738044[48] = 0;
   out_1916627579971738044[49] = 0;
   out_1916627579971738044[50] = 1.0;
   out_1916627579971738044[51] = 0;
   out_1916627579971738044[52] = 0;
   out_1916627579971738044[53] = 0;
   out_1916627579971738044[54] = 0;
   out_1916627579971738044[55] = 0;
   out_1916627579971738044[56] = 0;
   out_1916627579971738044[57] = 0;
   out_1916627579971738044[58] = 0;
   out_1916627579971738044[59] = 0;
   out_1916627579971738044[60] = 1.0;
   out_1916627579971738044[61] = 0;
   out_1916627579971738044[62] = 0;
   out_1916627579971738044[63] = 0;
   out_1916627579971738044[64] = 0;
   out_1916627579971738044[65] = 0;
   out_1916627579971738044[66] = 0;
   out_1916627579971738044[67] = 0;
   out_1916627579971738044[68] = 0;
   out_1916627579971738044[69] = 0;
   out_1916627579971738044[70] = 1.0;
   out_1916627579971738044[71] = 0;
   out_1916627579971738044[72] = 0;
   out_1916627579971738044[73] = 0;
   out_1916627579971738044[74] = 0;
   out_1916627579971738044[75] = 0;
   out_1916627579971738044[76] = 0;
   out_1916627579971738044[77] = 0;
   out_1916627579971738044[78] = 0;
   out_1916627579971738044[79] = 0;
   out_1916627579971738044[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_225145524562687816) {
   out_225145524562687816[0] = state[0];
   out_225145524562687816[1] = state[1];
   out_225145524562687816[2] = state[2];
   out_225145524562687816[3] = state[3];
   out_225145524562687816[4] = state[4];
   out_225145524562687816[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_225145524562687816[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_225145524562687816[7] = state[7];
   out_225145524562687816[8] = state[8];
}
void F_fun(double *state, double dt, double *out_4004130248429785926) {
   out_4004130248429785926[0] = 1;
   out_4004130248429785926[1] = 0;
   out_4004130248429785926[2] = 0;
   out_4004130248429785926[3] = 0;
   out_4004130248429785926[4] = 0;
   out_4004130248429785926[5] = 0;
   out_4004130248429785926[6] = 0;
   out_4004130248429785926[7] = 0;
   out_4004130248429785926[8] = 0;
   out_4004130248429785926[9] = 0;
   out_4004130248429785926[10] = 1;
   out_4004130248429785926[11] = 0;
   out_4004130248429785926[12] = 0;
   out_4004130248429785926[13] = 0;
   out_4004130248429785926[14] = 0;
   out_4004130248429785926[15] = 0;
   out_4004130248429785926[16] = 0;
   out_4004130248429785926[17] = 0;
   out_4004130248429785926[18] = 0;
   out_4004130248429785926[19] = 0;
   out_4004130248429785926[20] = 1;
   out_4004130248429785926[21] = 0;
   out_4004130248429785926[22] = 0;
   out_4004130248429785926[23] = 0;
   out_4004130248429785926[24] = 0;
   out_4004130248429785926[25] = 0;
   out_4004130248429785926[26] = 0;
   out_4004130248429785926[27] = 0;
   out_4004130248429785926[28] = 0;
   out_4004130248429785926[29] = 0;
   out_4004130248429785926[30] = 1;
   out_4004130248429785926[31] = 0;
   out_4004130248429785926[32] = 0;
   out_4004130248429785926[33] = 0;
   out_4004130248429785926[34] = 0;
   out_4004130248429785926[35] = 0;
   out_4004130248429785926[36] = 0;
   out_4004130248429785926[37] = 0;
   out_4004130248429785926[38] = 0;
   out_4004130248429785926[39] = 0;
   out_4004130248429785926[40] = 1;
   out_4004130248429785926[41] = 0;
   out_4004130248429785926[42] = 0;
   out_4004130248429785926[43] = 0;
   out_4004130248429785926[44] = 0;
   out_4004130248429785926[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_4004130248429785926[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_4004130248429785926[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_4004130248429785926[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_4004130248429785926[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_4004130248429785926[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_4004130248429785926[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_4004130248429785926[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_4004130248429785926[53] = -9.8000000000000007*dt;
   out_4004130248429785926[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_4004130248429785926[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_4004130248429785926[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4004130248429785926[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4004130248429785926[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_4004130248429785926[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_4004130248429785926[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_4004130248429785926[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4004130248429785926[62] = 0;
   out_4004130248429785926[63] = 0;
   out_4004130248429785926[64] = 0;
   out_4004130248429785926[65] = 0;
   out_4004130248429785926[66] = 0;
   out_4004130248429785926[67] = 0;
   out_4004130248429785926[68] = 0;
   out_4004130248429785926[69] = 0;
   out_4004130248429785926[70] = 1;
   out_4004130248429785926[71] = 0;
   out_4004130248429785926[72] = 0;
   out_4004130248429785926[73] = 0;
   out_4004130248429785926[74] = 0;
   out_4004130248429785926[75] = 0;
   out_4004130248429785926[76] = 0;
   out_4004130248429785926[77] = 0;
   out_4004130248429785926[78] = 0;
   out_4004130248429785926[79] = 0;
   out_4004130248429785926[80] = 1;
}
void h_25(double *state, double *unused, double *out_3571277728892257344) {
   out_3571277728892257344[0] = state[6];
}
void H_25(double *state, double *unused, double *out_3606441959916610189) {
   out_3606441959916610189[0] = 0;
   out_3606441959916610189[1] = 0;
   out_3606441959916610189[2] = 0;
   out_3606441959916610189[3] = 0;
   out_3606441959916610189[4] = 0;
   out_3606441959916610189[5] = 0;
   out_3606441959916610189[6] = 1;
   out_3606441959916610189[7] = 0;
   out_3606441959916610189[8] = 0;
}
void h_24(double *state, double *unused, double *out_8046866872121177892) {
   out_8046866872121177892[0] = state[4];
   out_8046866872121177892[1] = state[5];
}
void H_24(double *state, double *unused, double *out_8426763464572598452) {
   out_8426763464572598452[0] = 0;
   out_8426763464572598452[1] = 0;
   out_8426763464572598452[2] = 0;
   out_8426763464572598452[3] = 0;
   out_8426763464572598452[4] = 1;
   out_8426763464572598452[5] = 0;
   out_8426763464572598452[6] = 0;
   out_8426763464572598452[7] = 0;
   out_8426763464572598452[8] = 0;
   out_8426763464572598452[9] = 0;
   out_8426763464572598452[10] = 0;
   out_8426763464572598452[11] = 0;
   out_8426763464572598452[12] = 0;
   out_8426763464572598452[13] = 0;
   out_8426763464572598452[14] = 1;
   out_8426763464572598452[15] = 0;
   out_8426763464572598452[16] = 0;
   out_8426763464572598452[17] = 0;
}
void h_30(double *state, double *unused, double *out_6994193694063245087) {
   out_6994193694063245087[0] = state[4];
}
void H_30(double *state, double *unused, double *out_3735780907059850259) {
   out_3735780907059850259[0] = 0;
   out_3735780907059850259[1] = 0;
   out_3735780907059850259[2] = 0;
   out_3735780907059850259[3] = 0;
   out_3735780907059850259[4] = 1;
   out_3735780907059850259[5] = 0;
   out_3735780907059850259[6] = 0;
   out_3735780907059850259[7] = 0;
   out_3735780907059850259[8] = 0;
}
void h_26(double *state, double *unused, double *out_6450877111985460884) {
   out_6450877111985460884[0] = state[7];
}
void H_26(double *state, double *unused, double *out_7347945278790666413) {
   out_7347945278790666413[0] = 0;
   out_7347945278790666413[1] = 0;
   out_7347945278790666413[2] = 0;
   out_7347945278790666413[3] = 0;
   out_7347945278790666413[4] = 0;
   out_7347945278790666413[5] = 0;
   out_7347945278790666413[6] = 0;
   out_7347945278790666413[7] = 1;
   out_7347945278790666413[8] = 0;
}
void h_27(double *state, double *unused, double *out_8629015287357120804) {
   out_8629015287357120804[0] = state[3];
}
void H_27(double *state, double *unused, double *out_5910544218860275170) {
   out_5910544218860275170[0] = 0;
   out_5910544218860275170[1] = 0;
   out_5910544218860275170[2] = 0;
   out_5910544218860275170[3] = 1;
   out_5910544218860275170[4] = 0;
   out_5910544218860275170[5] = 0;
   out_5910544218860275170[6] = 0;
   out_5910544218860275170[7] = 0;
   out_5910544218860275170[8] = 0;
}
void h_29(double *state, double *unused, double *out_6012109600727946975) {
   out_6012109600727946975[0] = state[1];
}
void H_29(double *state, double *unused, double *out_3225549562745458075) {
   out_3225549562745458075[0] = 0;
   out_3225549562745458075[1] = 1;
   out_3225549562745458075[2] = 0;
   out_3225549562745458075[3] = 0;
   out_3225549562745458075[4] = 0;
   out_3225549562745458075[5] = 0;
   out_3225549562745458075[6] = 0;
   out_3225549562745458075[7] = 0;
   out_3225549562745458075[8] = 0;
}
void h_28(double *state, double *unused, double *out_7629441143651440554) {
   out_7629441143651440554[0] = state[0];
}
void H_28(double *state, double *unused, double *out_5740438110910194839) {
   out_5740438110910194839[0] = 1;
   out_5740438110910194839[1] = 0;
   out_5740438110910194839[2] = 0;
   out_5740438110910194839[3] = 0;
   out_5740438110910194839[4] = 0;
   out_5740438110910194839[5] = 0;
   out_5740438110910194839[6] = 0;
   out_5740438110910194839[7] = 0;
   out_5740438110910194839[8] = 0;
}
void h_31(double *state, double *unused, double *out_4973242187042714312) {
   out_4973242187042714312[0] = state[8];
}
void H_31(double *state, double *unused, double *out_3575795998039649761) {
   out_3575795998039649761[0] = 0;
   out_3575795998039649761[1] = 0;
   out_3575795998039649761[2] = 0;
   out_3575795998039649761[3] = 0;
   out_3575795998039649761[4] = 0;
   out_3575795998039649761[5] = 0;
   out_3575795998039649761[6] = 0;
   out_3575795998039649761[7] = 0;
   out_3575795998039649761[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_1259430672998403639) {
  err_fun(nom_x, delta_x, out_1259430672998403639);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6486332291499723158) {
  inv_err_fun(nom_x, true_x, out_6486332291499723158);
}
void car_H_mod_fun(double *state, double *out_1916627579971738044) {
  H_mod_fun(state, out_1916627579971738044);
}
void car_f_fun(double *state, double dt, double *out_225145524562687816) {
  f_fun(state,  dt, out_225145524562687816);
}
void car_F_fun(double *state, double dt, double *out_4004130248429785926) {
  F_fun(state,  dt, out_4004130248429785926);
}
void car_h_25(double *state, double *unused, double *out_3571277728892257344) {
  h_25(state, unused, out_3571277728892257344);
}
void car_H_25(double *state, double *unused, double *out_3606441959916610189) {
  H_25(state, unused, out_3606441959916610189);
}
void car_h_24(double *state, double *unused, double *out_8046866872121177892) {
  h_24(state, unused, out_8046866872121177892);
}
void car_H_24(double *state, double *unused, double *out_8426763464572598452) {
  H_24(state, unused, out_8426763464572598452);
}
void car_h_30(double *state, double *unused, double *out_6994193694063245087) {
  h_30(state, unused, out_6994193694063245087);
}
void car_H_30(double *state, double *unused, double *out_3735780907059850259) {
  H_30(state, unused, out_3735780907059850259);
}
void car_h_26(double *state, double *unused, double *out_6450877111985460884) {
  h_26(state, unused, out_6450877111985460884);
}
void car_H_26(double *state, double *unused, double *out_7347945278790666413) {
  H_26(state, unused, out_7347945278790666413);
}
void car_h_27(double *state, double *unused, double *out_8629015287357120804) {
  h_27(state, unused, out_8629015287357120804);
}
void car_H_27(double *state, double *unused, double *out_5910544218860275170) {
  H_27(state, unused, out_5910544218860275170);
}
void car_h_29(double *state, double *unused, double *out_6012109600727946975) {
  h_29(state, unused, out_6012109600727946975);
}
void car_H_29(double *state, double *unused, double *out_3225549562745458075) {
  H_29(state, unused, out_3225549562745458075);
}
void car_h_28(double *state, double *unused, double *out_7629441143651440554) {
  h_28(state, unused, out_7629441143651440554);
}
void car_H_28(double *state, double *unused, double *out_5740438110910194839) {
  H_28(state, unused, out_5740438110910194839);
}
void car_h_31(double *state, double *unused, double *out_4973242187042714312) {
  h_31(state, unused, out_4973242187042714312);
}
void car_H_31(double *state, double *unused, double *out_3575795998039649761) {
  H_31(state, unused, out_3575795998039649761);
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
