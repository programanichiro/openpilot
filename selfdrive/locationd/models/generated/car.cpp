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
void err_fun(double *nom_x, double *delta_x, double *out_8750905003075963447) {
   out_8750905003075963447[0] = delta_x[0] + nom_x[0];
   out_8750905003075963447[1] = delta_x[1] + nom_x[1];
   out_8750905003075963447[2] = delta_x[2] + nom_x[2];
   out_8750905003075963447[3] = delta_x[3] + nom_x[3];
   out_8750905003075963447[4] = delta_x[4] + nom_x[4];
   out_8750905003075963447[5] = delta_x[5] + nom_x[5];
   out_8750905003075963447[6] = delta_x[6] + nom_x[6];
   out_8750905003075963447[7] = delta_x[7] + nom_x[7];
   out_8750905003075963447[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_5158842673956621266) {
   out_5158842673956621266[0] = -nom_x[0] + true_x[0];
   out_5158842673956621266[1] = -nom_x[1] + true_x[1];
   out_5158842673956621266[2] = -nom_x[2] + true_x[2];
   out_5158842673956621266[3] = -nom_x[3] + true_x[3];
   out_5158842673956621266[4] = -nom_x[4] + true_x[4];
   out_5158842673956621266[5] = -nom_x[5] + true_x[5];
   out_5158842673956621266[6] = -nom_x[6] + true_x[6];
   out_5158842673956621266[7] = -nom_x[7] + true_x[7];
   out_5158842673956621266[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_6032308480841622457) {
   out_6032308480841622457[0] = 1.0;
   out_6032308480841622457[1] = 0;
   out_6032308480841622457[2] = 0;
   out_6032308480841622457[3] = 0;
   out_6032308480841622457[4] = 0;
   out_6032308480841622457[5] = 0;
   out_6032308480841622457[6] = 0;
   out_6032308480841622457[7] = 0;
   out_6032308480841622457[8] = 0;
   out_6032308480841622457[9] = 0;
   out_6032308480841622457[10] = 1.0;
   out_6032308480841622457[11] = 0;
   out_6032308480841622457[12] = 0;
   out_6032308480841622457[13] = 0;
   out_6032308480841622457[14] = 0;
   out_6032308480841622457[15] = 0;
   out_6032308480841622457[16] = 0;
   out_6032308480841622457[17] = 0;
   out_6032308480841622457[18] = 0;
   out_6032308480841622457[19] = 0;
   out_6032308480841622457[20] = 1.0;
   out_6032308480841622457[21] = 0;
   out_6032308480841622457[22] = 0;
   out_6032308480841622457[23] = 0;
   out_6032308480841622457[24] = 0;
   out_6032308480841622457[25] = 0;
   out_6032308480841622457[26] = 0;
   out_6032308480841622457[27] = 0;
   out_6032308480841622457[28] = 0;
   out_6032308480841622457[29] = 0;
   out_6032308480841622457[30] = 1.0;
   out_6032308480841622457[31] = 0;
   out_6032308480841622457[32] = 0;
   out_6032308480841622457[33] = 0;
   out_6032308480841622457[34] = 0;
   out_6032308480841622457[35] = 0;
   out_6032308480841622457[36] = 0;
   out_6032308480841622457[37] = 0;
   out_6032308480841622457[38] = 0;
   out_6032308480841622457[39] = 0;
   out_6032308480841622457[40] = 1.0;
   out_6032308480841622457[41] = 0;
   out_6032308480841622457[42] = 0;
   out_6032308480841622457[43] = 0;
   out_6032308480841622457[44] = 0;
   out_6032308480841622457[45] = 0;
   out_6032308480841622457[46] = 0;
   out_6032308480841622457[47] = 0;
   out_6032308480841622457[48] = 0;
   out_6032308480841622457[49] = 0;
   out_6032308480841622457[50] = 1.0;
   out_6032308480841622457[51] = 0;
   out_6032308480841622457[52] = 0;
   out_6032308480841622457[53] = 0;
   out_6032308480841622457[54] = 0;
   out_6032308480841622457[55] = 0;
   out_6032308480841622457[56] = 0;
   out_6032308480841622457[57] = 0;
   out_6032308480841622457[58] = 0;
   out_6032308480841622457[59] = 0;
   out_6032308480841622457[60] = 1.0;
   out_6032308480841622457[61] = 0;
   out_6032308480841622457[62] = 0;
   out_6032308480841622457[63] = 0;
   out_6032308480841622457[64] = 0;
   out_6032308480841622457[65] = 0;
   out_6032308480841622457[66] = 0;
   out_6032308480841622457[67] = 0;
   out_6032308480841622457[68] = 0;
   out_6032308480841622457[69] = 0;
   out_6032308480841622457[70] = 1.0;
   out_6032308480841622457[71] = 0;
   out_6032308480841622457[72] = 0;
   out_6032308480841622457[73] = 0;
   out_6032308480841622457[74] = 0;
   out_6032308480841622457[75] = 0;
   out_6032308480841622457[76] = 0;
   out_6032308480841622457[77] = 0;
   out_6032308480841622457[78] = 0;
   out_6032308480841622457[79] = 0;
   out_6032308480841622457[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_2168661768497310835) {
   out_2168661768497310835[0] = state[0];
   out_2168661768497310835[1] = state[1];
   out_2168661768497310835[2] = state[2];
   out_2168661768497310835[3] = state[3];
   out_2168661768497310835[4] = state[4];
   out_2168661768497310835[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_2168661768497310835[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_2168661768497310835[7] = state[7];
   out_2168661768497310835[8] = state[8];
}
void F_fun(double *state, double dt, double *out_1487349331672120911) {
   out_1487349331672120911[0] = 1;
   out_1487349331672120911[1] = 0;
   out_1487349331672120911[2] = 0;
   out_1487349331672120911[3] = 0;
   out_1487349331672120911[4] = 0;
   out_1487349331672120911[5] = 0;
   out_1487349331672120911[6] = 0;
   out_1487349331672120911[7] = 0;
   out_1487349331672120911[8] = 0;
   out_1487349331672120911[9] = 0;
   out_1487349331672120911[10] = 1;
   out_1487349331672120911[11] = 0;
   out_1487349331672120911[12] = 0;
   out_1487349331672120911[13] = 0;
   out_1487349331672120911[14] = 0;
   out_1487349331672120911[15] = 0;
   out_1487349331672120911[16] = 0;
   out_1487349331672120911[17] = 0;
   out_1487349331672120911[18] = 0;
   out_1487349331672120911[19] = 0;
   out_1487349331672120911[20] = 1;
   out_1487349331672120911[21] = 0;
   out_1487349331672120911[22] = 0;
   out_1487349331672120911[23] = 0;
   out_1487349331672120911[24] = 0;
   out_1487349331672120911[25] = 0;
   out_1487349331672120911[26] = 0;
   out_1487349331672120911[27] = 0;
   out_1487349331672120911[28] = 0;
   out_1487349331672120911[29] = 0;
   out_1487349331672120911[30] = 1;
   out_1487349331672120911[31] = 0;
   out_1487349331672120911[32] = 0;
   out_1487349331672120911[33] = 0;
   out_1487349331672120911[34] = 0;
   out_1487349331672120911[35] = 0;
   out_1487349331672120911[36] = 0;
   out_1487349331672120911[37] = 0;
   out_1487349331672120911[38] = 0;
   out_1487349331672120911[39] = 0;
   out_1487349331672120911[40] = 1;
   out_1487349331672120911[41] = 0;
   out_1487349331672120911[42] = 0;
   out_1487349331672120911[43] = 0;
   out_1487349331672120911[44] = 0;
   out_1487349331672120911[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_1487349331672120911[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_1487349331672120911[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_1487349331672120911[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_1487349331672120911[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_1487349331672120911[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_1487349331672120911[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_1487349331672120911[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_1487349331672120911[53] = -9.8000000000000007*dt;
   out_1487349331672120911[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_1487349331672120911[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_1487349331672120911[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1487349331672120911[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1487349331672120911[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_1487349331672120911[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_1487349331672120911[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_1487349331672120911[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1487349331672120911[62] = 0;
   out_1487349331672120911[63] = 0;
   out_1487349331672120911[64] = 0;
   out_1487349331672120911[65] = 0;
   out_1487349331672120911[66] = 0;
   out_1487349331672120911[67] = 0;
   out_1487349331672120911[68] = 0;
   out_1487349331672120911[69] = 0;
   out_1487349331672120911[70] = 1;
   out_1487349331672120911[71] = 0;
   out_1487349331672120911[72] = 0;
   out_1487349331672120911[73] = 0;
   out_1487349331672120911[74] = 0;
   out_1487349331672120911[75] = 0;
   out_1487349331672120911[76] = 0;
   out_1487349331672120911[77] = 0;
   out_1487349331672120911[78] = 0;
   out_1487349331672120911[79] = 0;
   out_1487349331672120911[80] = 1;
}
void h_25(double *state, double *unused, double *out_5440738245400390420) {
   out_5440738245400390420[0] = state[6];
}
void H_25(double *state, double *unused, double *out_1227009239193474661) {
   out_1227009239193474661[0] = 0;
   out_1227009239193474661[1] = 0;
   out_1227009239193474661[2] = 0;
   out_1227009239193474661[3] = 0;
   out_1227009239193474661[4] = 0;
   out_1227009239193474661[5] = 0;
   out_1227009239193474661[6] = 1;
   out_1227009239193474661[7] = 0;
   out_1227009239193474661[8] = 0;
}
void h_24(double *state, double *unused, double *out_5094988861551536833) {
   out_5094988861551536833[0] = state[4];
   out_5094988861551536833[1] = state[5];
}
void H_24(double *state, double *unused, double *out_3399658838198974227) {
   out_3399658838198974227[0] = 0;
   out_3399658838198974227[1] = 0;
   out_3399658838198974227[2] = 0;
   out_3399658838198974227[3] = 0;
   out_3399658838198974227[4] = 1;
   out_3399658838198974227[5] = 0;
   out_3399658838198974227[6] = 0;
   out_3399658838198974227[7] = 0;
   out_3399658838198974227[8] = 0;
   out_3399658838198974227[9] = 0;
   out_3399658838198974227[10] = 0;
   out_3399658838198974227[11] = 0;
   out_3399658838198974227[12] = 0;
   out_3399658838198974227[13] = 0;
   out_3399658838198974227[14] = 1;
   out_3399658838198974227[15] = 0;
   out_3399658838198974227[16] = 0;
   out_3399658838198974227[17] = 0;
}
void h_30(double *state, double *unused, double *out_6356988911105007499) {
   out_6356988911105007499[0] = state[4];
}
void H_30(double *state, double *unused, double *out_5689681102298142094) {
   out_5689681102298142094[0] = 0;
   out_5689681102298142094[1] = 0;
   out_5689681102298142094[2] = 0;
   out_5689681102298142094[3] = 0;
   out_5689681102298142094[4] = 1;
   out_5689681102298142094[5] = 0;
   out_5689681102298142094[6] = 0;
   out_5689681102298142094[7] = 0;
   out_5689681102298142094[8] = 0;
}
void h_26(double *state, double *unused, double *out_6331770248610806161) {
   out_6331770248610806161[0] = state[7];
}
void H_26(double *state, double *unused, double *out_4968512558067530885) {
   out_4968512558067530885[0] = 0;
   out_4968512558067530885[1] = 0;
   out_4968512558067530885[2] = 0;
   out_4968512558067530885[3] = 0;
   out_4968512558067530885[4] = 0;
   out_4968512558067530885[5] = 0;
   out_4968512558067530885[6] = 0;
   out_4968512558067530885[7] = 1;
   out_4968512558067530885[8] = 0;
}
void h_27(double *state, double *unused, double *out_4770834252289557981) {
   out_4770834252289557981[0] = state[3];
}
void H_27(double *state, double *unused, double *out_3514917790497717183) {
   out_3514917790497717183[0] = 0;
   out_3514917790497717183[1] = 0;
   out_3514917790497717183[2] = 0;
   out_3514917790497717183[3] = 1;
   out_3514917790497717183[4] = 0;
   out_3514917790497717183[5] = 0;
   out_3514917790497717183[6] = 0;
   out_3514917790497717183[7] = 0;
   out_3514917790497717183[8] = 0;
}
void h_29(double *state, double *unused, double *out_8765240415378661329) {
   out_8765240415378661329[0] = state[1];
}
void H_29(double *state, double *unused, double *out_1801555063628166150) {
   out_1801555063628166150[0] = 0;
   out_1801555063628166150[1] = 1;
   out_1801555063628166150[2] = 0;
   out_1801555063628166150[3] = 0;
   out_1801555063628166150[4] = 0;
   out_1801555063628166150[5] = 0;
   out_1801555063628166150[6] = 0;
   out_1801555063628166150[7] = 0;
   out_1801555063628166150[8] = 0;
}
void h_28(double *state, double *unused, double *out_2937915041929972828) {
   out_2937915041929972828[0] = state[0];
}
void H_28(double *state, double *unused, double *out_3280843953441364424) {
   out_3280843953441364424[0] = 1;
   out_3280843953441364424[1] = 0;
   out_3280843953441364424[2] = 0;
   out_3280843953441364424[3] = 0;
   out_3280843953441364424[4] = 0;
   out_3280843953441364424[5] = 0;
   out_3280843953441364424[6] = 0;
   out_3280843953441364424[7] = 0;
   out_3280843953441364424[8] = 0;
}
void h_31(double *state, double *unused, double *out_3868288543625310014) {
   out_3868288543625310014[0] = state[8];
}
void H_31(double *state, double *unused, double *out_1196363277316514233) {
   out_1196363277316514233[0] = 0;
   out_1196363277316514233[1] = 0;
   out_1196363277316514233[2] = 0;
   out_1196363277316514233[3] = 0;
   out_1196363277316514233[4] = 0;
   out_1196363277316514233[5] = 0;
   out_1196363277316514233[6] = 0;
   out_1196363277316514233[7] = 0;
   out_1196363277316514233[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_8750905003075963447) {
  err_fun(nom_x, delta_x, out_8750905003075963447);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5158842673956621266) {
  inv_err_fun(nom_x, true_x, out_5158842673956621266);
}
void car_H_mod_fun(double *state, double *out_6032308480841622457) {
  H_mod_fun(state, out_6032308480841622457);
}
void car_f_fun(double *state, double dt, double *out_2168661768497310835) {
  f_fun(state,  dt, out_2168661768497310835);
}
void car_F_fun(double *state, double dt, double *out_1487349331672120911) {
  F_fun(state,  dt, out_1487349331672120911);
}
void car_h_25(double *state, double *unused, double *out_5440738245400390420) {
  h_25(state, unused, out_5440738245400390420);
}
void car_H_25(double *state, double *unused, double *out_1227009239193474661) {
  H_25(state, unused, out_1227009239193474661);
}
void car_h_24(double *state, double *unused, double *out_5094988861551536833) {
  h_24(state, unused, out_5094988861551536833);
}
void car_H_24(double *state, double *unused, double *out_3399658838198974227) {
  H_24(state, unused, out_3399658838198974227);
}
void car_h_30(double *state, double *unused, double *out_6356988911105007499) {
  h_30(state, unused, out_6356988911105007499);
}
void car_H_30(double *state, double *unused, double *out_5689681102298142094) {
  H_30(state, unused, out_5689681102298142094);
}
void car_h_26(double *state, double *unused, double *out_6331770248610806161) {
  h_26(state, unused, out_6331770248610806161);
}
void car_H_26(double *state, double *unused, double *out_4968512558067530885) {
  H_26(state, unused, out_4968512558067530885);
}
void car_h_27(double *state, double *unused, double *out_4770834252289557981) {
  h_27(state, unused, out_4770834252289557981);
}
void car_H_27(double *state, double *unused, double *out_3514917790497717183) {
  H_27(state, unused, out_3514917790497717183);
}
void car_h_29(double *state, double *unused, double *out_8765240415378661329) {
  h_29(state, unused, out_8765240415378661329);
}
void car_H_29(double *state, double *unused, double *out_1801555063628166150) {
  H_29(state, unused, out_1801555063628166150);
}
void car_h_28(double *state, double *unused, double *out_2937915041929972828) {
  h_28(state, unused, out_2937915041929972828);
}
void car_H_28(double *state, double *unused, double *out_3280843953441364424) {
  H_28(state, unused, out_3280843953441364424);
}
void car_h_31(double *state, double *unused, double *out_3868288543625310014) {
  h_31(state, unused, out_3868288543625310014);
}
void car_H_31(double *state, double *unused, double *out_1196363277316514233) {
  H_31(state, unused, out_1196363277316514233);
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
