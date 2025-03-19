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
 *                      Code generated with SymPy 1.13.2                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_7960405237366517205) {
   out_7960405237366517205[0] = delta_x[0] + nom_x[0];
   out_7960405237366517205[1] = delta_x[1] + nom_x[1];
   out_7960405237366517205[2] = delta_x[2] + nom_x[2];
   out_7960405237366517205[3] = delta_x[3] + nom_x[3];
   out_7960405237366517205[4] = delta_x[4] + nom_x[4];
   out_7960405237366517205[5] = delta_x[5] + nom_x[5];
   out_7960405237366517205[6] = delta_x[6] + nom_x[6];
   out_7960405237366517205[7] = delta_x[7] + nom_x[7];
   out_7960405237366517205[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7862762090034680680) {
   out_7862762090034680680[0] = -nom_x[0] + true_x[0];
   out_7862762090034680680[1] = -nom_x[1] + true_x[1];
   out_7862762090034680680[2] = -nom_x[2] + true_x[2];
   out_7862762090034680680[3] = -nom_x[3] + true_x[3];
   out_7862762090034680680[4] = -nom_x[4] + true_x[4];
   out_7862762090034680680[5] = -nom_x[5] + true_x[5];
   out_7862762090034680680[6] = -nom_x[6] + true_x[6];
   out_7862762090034680680[7] = -nom_x[7] + true_x[7];
   out_7862762090034680680[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_4138065836317693599) {
   out_4138065836317693599[0] = 1.0;
   out_4138065836317693599[1] = 0.0;
   out_4138065836317693599[2] = 0.0;
   out_4138065836317693599[3] = 0.0;
   out_4138065836317693599[4] = 0.0;
   out_4138065836317693599[5] = 0.0;
   out_4138065836317693599[6] = 0.0;
   out_4138065836317693599[7] = 0.0;
   out_4138065836317693599[8] = 0.0;
   out_4138065836317693599[9] = 0.0;
   out_4138065836317693599[10] = 1.0;
   out_4138065836317693599[11] = 0.0;
   out_4138065836317693599[12] = 0.0;
   out_4138065836317693599[13] = 0.0;
   out_4138065836317693599[14] = 0.0;
   out_4138065836317693599[15] = 0.0;
   out_4138065836317693599[16] = 0.0;
   out_4138065836317693599[17] = 0.0;
   out_4138065836317693599[18] = 0.0;
   out_4138065836317693599[19] = 0.0;
   out_4138065836317693599[20] = 1.0;
   out_4138065836317693599[21] = 0.0;
   out_4138065836317693599[22] = 0.0;
   out_4138065836317693599[23] = 0.0;
   out_4138065836317693599[24] = 0.0;
   out_4138065836317693599[25] = 0.0;
   out_4138065836317693599[26] = 0.0;
   out_4138065836317693599[27] = 0.0;
   out_4138065836317693599[28] = 0.0;
   out_4138065836317693599[29] = 0.0;
   out_4138065836317693599[30] = 1.0;
   out_4138065836317693599[31] = 0.0;
   out_4138065836317693599[32] = 0.0;
   out_4138065836317693599[33] = 0.0;
   out_4138065836317693599[34] = 0.0;
   out_4138065836317693599[35] = 0.0;
   out_4138065836317693599[36] = 0.0;
   out_4138065836317693599[37] = 0.0;
   out_4138065836317693599[38] = 0.0;
   out_4138065836317693599[39] = 0.0;
   out_4138065836317693599[40] = 1.0;
   out_4138065836317693599[41] = 0.0;
   out_4138065836317693599[42] = 0.0;
   out_4138065836317693599[43] = 0.0;
   out_4138065836317693599[44] = 0.0;
   out_4138065836317693599[45] = 0.0;
   out_4138065836317693599[46] = 0.0;
   out_4138065836317693599[47] = 0.0;
   out_4138065836317693599[48] = 0.0;
   out_4138065836317693599[49] = 0.0;
   out_4138065836317693599[50] = 1.0;
   out_4138065836317693599[51] = 0.0;
   out_4138065836317693599[52] = 0.0;
   out_4138065836317693599[53] = 0.0;
   out_4138065836317693599[54] = 0.0;
   out_4138065836317693599[55] = 0.0;
   out_4138065836317693599[56] = 0.0;
   out_4138065836317693599[57] = 0.0;
   out_4138065836317693599[58] = 0.0;
   out_4138065836317693599[59] = 0.0;
   out_4138065836317693599[60] = 1.0;
   out_4138065836317693599[61] = 0.0;
   out_4138065836317693599[62] = 0.0;
   out_4138065836317693599[63] = 0.0;
   out_4138065836317693599[64] = 0.0;
   out_4138065836317693599[65] = 0.0;
   out_4138065836317693599[66] = 0.0;
   out_4138065836317693599[67] = 0.0;
   out_4138065836317693599[68] = 0.0;
   out_4138065836317693599[69] = 0.0;
   out_4138065836317693599[70] = 1.0;
   out_4138065836317693599[71] = 0.0;
   out_4138065836317693599[72] = 0.0;
   out_4138065836317693599[73] = 0.0;
   out_4138065836317693599[74] = 0.0;
   out_4138065836317693599[75] = 0.0;
   out_4138065836317693599[76] = 0.0;
   out_4138065836317693599[77] = 0.0;
   out_4138065836317693599[78] = 0.0;
   out_4138065836317693599[79] = 0.0;
   out_4138065836317693599[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_1608403082623815818) {
   out_1608403082623815818[0] = state[0];
   out_1608403082623815818[1] = state[1];
   out_1608403082623815818[2] = state[2];
   out_1608403082623815818[3] = state[3];
   out_1608403082623815818[4] = state[4];
   out_1608403082623815818[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_1608403082623815818[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_1608403082623815818[7] = state[7];
   out_1608403082623815818[8] = state[8];
}
void F_fun(double *state, double dt, double *out_72870844652074545) {
   out_72870844652074545[0] = 1;
   out_72870844652074545[1] = 0;
   out_72870844652074545[2] = 0;
   out_72870844652074545[3] = 0;
   out_72870844652074545[4] = 0;
   out_72870844652074545[5] = 0;
   out_72870844652074545[6] = 0;
   out_72870844652074545[7] = 0;
   out_72870844652074545[8] = 0;
   out_72870844652074545[9] = 0;
   out_72870844652074545[10] = 1;
   out_72870844652074545[11] = 0;
   out_72870844652074545[12] = 0;
   out_72870844652074545[13] = 0;
   out_72870844652074545[14] = 0;
   out_72870844652074545[15] = 0;
   out_72870844652074545[16] = 0;
   out_72870844652074545[17] = 0;
   out_72870844652074545[18] = 0;
   out_72870844652074545[19] = 0;
   out_72870844652074545[20] = 1;
   out_72870844652074545[21] = 0;
   out_72870844652074545[22] = 0;
   out_72870844652074545[23] = 0;
   out_72870844652074545[24] = 0;
   out_72870844652074545[25] = 0;
   out_72870844652074545[26] = 0;
   out_72870844652074545[27] = 0;
   out_72870844652074545[28] = 0;
   out_72870844652074545[29] = 0;
   out_72870844652074545[30] = 1;
   out_72870844652074545[31] = 0;
   out_72870844652074545[32] = 0;
   out_72870844652074545[33] = 0;
   out_72870844652074545[34] = 0;
   out_72870844652074545[35] = 0;
   out_72870844652074545[36] = 0;
   out_72870844652074545[37] = 0;
   out_72870844652074545[38] = 0;
   out_72870844652074545[39] = 0;
   out_72870844652074545[40] = 1;
   out_72870844652074545[41] = 0;
   out_72870844652074545[42] = 0;
   out_72870844652074545[43] = 0;
   out_72870844652074545[44] = 0;
   out_72870844652074545[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_72870844652074545[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_72870844652074545[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_72870844652074545[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_72870844652074545[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_72870844652074545[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_72870844652074545[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_72870844652074545[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_72870844652074545[53] = -9.8000000000000007*dt;
   out_72870844652074545[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_72870844652074545[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_72870844652074545[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_72870844652074545[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_72870844652074545[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_72870844652074545[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_72870844652074545[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_72870844652074545[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_72870844652074545[62] = 0;
   out_72870844652074545[63] = 0;
   out_72870844652074545[64] = 0;
   out_72870844652074545[65] = 0;
   out_72870844652074545[66] = 0;
   out_72870844652074545[67] = 0;
   out_72870844652074545[68] = 0;
   out_72870844652074545[69] = 0;
   out_72870844652074545[70] = 1;
   out_72870844652074545[71] = 0;
   out_72870844652074545[72] = 0;
   out_72870844652074545[73] = 0;
   out_72870844652074545[74] = 0;
   out_72870844652074545[75] = 0;
   out_72870844652074545[76] = 0;
   out_72870844652074545[77] = 0;
   out_72870844652074545[78] = 0;
   out_72870844652074545[79] = 0;
   out_72870844652074545[80] = 1;
}
void h_25(double *state, double *unused, double *out_3299501452529380860) {
   out_3299501452529380860[0] = state[6];
}
void H_25(double *state, double *unused, double *out_7273763929392063695) {
   out_7273763929392063695[0] = 0;
   out_7273763929392063695[1] = 0;
   out_7273763929392063695[2] = 0;
   out_7273763929392063695[3] = 0;
   out_7273763929392063695[4] = 0;
   out_7273763929392063695[5] = 0;
   out_7273763929392063695[6] = 1;
   out_7273763929392063695[7] = 0;
   out_7273763929392063695[8] = 0;
}
void h_24(double *state, double *unused, double *out_8845649651338186010) {
   out_8845649651338186010[0] = state[4];
   out_8845649651338186010[1] = state[5];
}
void H_24(double *state, double *unused, double *out_2942840412474121214) {
   out_2942840412474121214[0] = 0;
   out_2942840412474121214[1] = 0;
   out_2942840412474121214[2] = 0;
   out_2942840412474121214[3] = 0;
   out_2942840412474121214[4] = 1;
   out_2942840412474121214[5] = 0;
   out_2942840412474121214[6] = 0;
   out_2942840412474121214[7] = 0;
   out_2942840412474121214[8] = 0;
   out_2942840412474121214[9] = 0;
   out_2942840412474121214[10] = 0;
   out_2942840412474121214[11] = 0;
   out_2942840412474121214[12] = 0;
   out_2942840412474121214[13] = 0;
   out_2942840412474121214[14] = 1;
   out_2942840412474121214[15] = 0;
   out_2942840412474121214[16] = 0;
   out_2942840412474121214[17] = 0;
}
void h_30(double *state, double *unused, double *out_4125153558317223716) {
   out_4125153558317223716[0] = state[4];
}
void H_30(double *state, double *unused, double *out_7144424982248823625) {
   out_7144424982248823625[0] = 0;
   out_7144424982248823625[1] = 0;
   out_7144424982248823625[2] = 0;
   out_7144424982248823625[3] = 0;
   out_7144424982248823625[4] = 1;
   out_7144424982248823625[5] = 0;
   out_7144424982248823625[6] = 0;
   out_7144424982248823625[7] = 0;
   out_7144424982248823625[8] = 0;
}
void h_26(double *state, double *unused, double *out_9126826825978069361) {
   out_9126826825978069361[0] = state[7];
}
void H_26(double *state, double *unused, double *out_7930617993502375599) {
   out_7930617993502375599[0] = 0;
   out_7930617993502375599[1] = 0;
   out_7930617993502375599[2] = 0;
   out_7930617993502375599[3] = 0;
   out_7930617993502375599[4] = 0;
   out_7930617993502375599[5] = 0;
   out_7930617993502375599[6] = 0;
   out_7930617993502375599[7] = 1;
   out_7930617993502375599[8] = 0;
}
void h_27(double *state, double *unused, double *out_1702171815131464785) {
   out_1702171815131464785[0] = state[3];
}
void H_27(double *state, double *unused, double *out_4969661670448398714) {
   out_4969661670448398714[0] = 0;
   out_4969661670448398714[1] = 0;
   out_4969661670448398714[2] = 0;
   out_4969661670448398714[3] = 1;
   out_4969661670448398714[4] = 0;
   out_4969661670448398714[5] = 0;
   out_4969661670448398714[6] = 0;
   out_4969661670448398714[7] = 0;
   out_4969661670448398714[8] = 0;
}
void h_29(double *state, double *unused, double *out_5068663411218886151) {
   out_5068663411218886151[0] = state[1];
}
void H_29(double *state, double *unused, double *out_7654656326563215809) {
   out_7654656326563215809[0] = 0;
   out_7654656326563215809[1] = 1;
   out_7654656326563215809[2] = 0;
   out_7654656326563215809[3] = 0;
   out_7654656326563215809[4] = 0;
   out_7654656326563215809[5] = 0;
   out_7654656326563215809[6] = 0;
   out_7654656326563215809[7] = 0;
   out_7654656326563215809[8] = 0;
}
void h_28(double *state, double *unused, double *out_2305487699807942302) {
   out_2305487699807942302[0] = state[0];
}
void H_28(double *state, double *unused, double *out_2572257309493685235) {
   out_2572257309493685235[0] = 1;
   out_2572257309493685235[1] = 0;
   out_2572257309493685235[2] = 0;
   out_2572257309493685235[3] = 0;
   out_2572257309493685235[4] = 0;
   out_2572257309493685235[5] = 0;
   out_2572257309493685235[6] = 0;
   out_2572257309493685235[7] = 0;
   out_2572257309493685235[8] = 0;
}
void h_31(double *state, double *unused, double *out_7161184928957882512) {
   out_7161184928957882512[0] = state[8];
}
void H_31(double *state, double *unused, double *out_7304409891269024123) {
   out_7304409891269024123[0] = 0;
   out_7304409891269024123[1] = 0;
   out_7304409891269024123[2] = 0;
   out_7304409891269024123[3] = 0;
   out_7304409891269024123[4] = 0;
   out_7304409891269024123[5] = 0;
   out_7304409891269024123[6] = 0;
   out_7304409891269024123[7] = 0;
   out_7304409891269024123[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_7960405237366517205) {
  err_fun(nom_x, delta_x, out_7960405237366517205);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7862762090034680680) {
  inv_err_fun(nom_x, true_x, out_7862762090034680680);
}
void car_H_mod_fun(double *state, double *out_4138065836317693599) {
  H_mod_fun(state, out_4138065836317693599);
}
void car_f_fun(double *state, double dt, double *out_1608403082623815818) {
  f_fun(state,  dt, out_1608403082623815818);
}
void car_F_fun(double *state, double dt, double *out_72870844652074545) {
  F_fun(state,  dt, out_72870844652074545);
}
void car_h_25(double *state, double *unused, double *out_3299501452529380860) {
  h_25(state, unused, out_3299501452529380860);
}
void car_H_25(double *state, double *unused, double *out_7273763929392063695) {
  H_25(state, unused, out_7273763929392063695);
}
void car_h_24(double *state, double *unused, double *out_8845649651338186010) {
  h_24(state, unused, out_8845649651338186010);
}
void car_H_24(double *state, double *unused, double *out_2942840412474121214) {
  H_24(state, unused, out_2942840412474121214);
}
void car_h_30(double *state, double *unused, double *out_4125153558317223716) {
  h_30(state, unused, out_4125153558317223716);
}
void car_H_30(double *state, double *unused, double *out_7144424982248823625) {
  H_30(state, unused, out_7144424982248823625);
}
void car_h_26(double *state, double *unused, double *out_9126826825978069361) {
  h_26(state, unused, out_9126826825978069361);
}
void car_H_26(double *state, double *unused, double *out_7930617993502375599) {
  H_26(state, unused, out_7930617993502375599);
}
void car_h_27(double *state, double *unused, double *out_1702171815131464785) {
  h_27(state, unused, out_1702171815131464785);
}
void car_H_27(double *state, double *unused, double *out_4969661670448398714) {
  H_27(state, unused, out_4969661670448398714);
}
void car_h_29(double *state, double *unused, double *out_5068663411218886151) {
  h_29(state, unused, out_5068663411218886151);
}
void car_H_29(double *state, double *unused, double *out_7654656326563215809) {
  H_29(state, unused, out_7654656326563215809);
}
void car_h_28(double *state, double *unused, double *out_2305487699807942302) {
  h_28(state, unused, out_2305487699807942302);
}
void car_H_28(double *state, double *unused, double *out_2572257309493685235) {
  H_28(state, unused, out_2572257309493685235);
}
void car_h_31(double *state, double *unused, double *out_7161184928957882512) {
  h_31(state, unused, out_7161184928957882512);
}
void car_H_31(double *state, double *unused, double *out_7304409891269024123) {
  H_31(state, unused, out_7304409891269024123);
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
