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
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_2070268012761229651) {
   out_2070268012761229651[0] = delta_x[0] + nom_x[0];
   out_2070268012761229651[1] = delta_x[1] + nom_x[1];
   out_2070268012761229651[2] = delta_x[2] + nom_x[2];
   out_2070268012761229651[3] = delta_x[3] + nom_x[3];
   out_2070268012761229651[4] = delta_x[4] + nom_x[4];
   out_2070268012761229651[5] = delta_x[5] + nom_x[5];
   out_2070268012761229651[6] = delta_x[6] + nom_x[6];
   out_2070268012761229651[7] = delta_x[7] + nom_x[7];
   out_2070268012761229651[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_4265412688424853444) {
   out_4265412688424853444[0] = -nom_x[0] + true_x[0];
   out_4265412688424853444[1] = -nom_x[1] + true_x[1];
   out_4265412688424853444[2] = -nom_x[2] + true_x[2];
   out_4265412688424853444[3] = -nom_x[3] + true_x[3];
   out_4265412688424853444[4] = -nom_x[4] + true_x[4];
   out_4265412688424853444[5] = -nom_x[5] + true_x[5];
   out_4265412688424853444[6] = -nom_x[6] + true_x[6];
   out_4265412688424853444[7] = -nom_x[7] + true_x[7];
   out_4265412688424853444[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_2584612022031943484) {
   out_2584612022031943484[0] = 1.0;
   out_2584612022031943484[1] = 0.0;
   out_2584612022031943484[2] = 0.0;
   out_2584612022031943484[3] = 0.0;
   out_2584612022031943484[4] = 0.0;
   out_2584612022031943484[5] = 0.0;
   out_2584612022031943484[6] = 0.0;
   out_2584612022031943484[7] = 0.0;
   out_2584612022031943484[8] = 0.0;
   out_2584612022031943484[9] = 0.0;
   out_2584612022031943484[10] = 1.0;
   out_2584612022031943484[11] = 0.0;
   out_2584612022031943484[12] = 0.0;
   out_2584612022031943484[13] = 0.0;
   out_2584612022031943484[14] = 0.0;
   out_2584612022031943484[15] = 0.0;
   out_2584612022031943484[16] = 0.0;
   out_2584612022031943484[17] = 0.0;
   out_2584612022031943484[18] = 0.0;
   out_2584612022031943484[19] = 0.0;
   out_2584612022031943484[20] = 1.0;
   out_2584612022031943484[21] = 0.0;
   out_2584612022031943484[22] = 0.0;
   out_2584612022031943484[23] = 0.0;
   out_2584612022031943484[24] = 0.0;
   out_2584612022031943484[25] = 0.0;
   out_2584612022031943484[26] = 0.0;
   out_2584612022031943484[27] = 0.0;
   out_2584612022031943484[28] = 0.0;
   out_2584612022031943484[29] = 0.0;
   out_2584612022031943484[30] = 1.0;
   out_2584612022031943484[31] = 0.0;
   out_2584612022031943484[32] = 0.0;
   out_2584612022031943484[33] = 0.0;
   out_2584612022031943484[34] = 0.0;
   out_2584612022031943484[35] = 0.0;
   out_2584612022031943484[36] = 0.0;
   out_2584612022031943484[37] = 0.0;
   out_2584612022031943484[38] = 0.0;
   out_2584612022031943484[39] = 0.0;
   out_2584612022031943484[40] = 1.0;
   out_2584612022031943484[41] = 0.0;
   out_2584612022031943484[42] = 0.0;
   out_2584612022031943484[43] = 0.0;
   out_2584612022031943484[44] = 0.0;
   out_2584612022031943484[45] = 0.0;
   out_2584612022031943484[46] = 0.0;
   out_2584612022031943484[47] = 0.0;
   out_2584612022031943484[48] = 0.0;
   out_2584612022031943484[49] = 0.0;
   out_2584612022031943484[50] = 1.0;
   out_2584612022031943484[51] = 0.0;
   out_2584612022031943484[52] = 0.0;
   out_2584612022031943484[53] = 0.0;
   out_2584612022031943484[54] = 0.0;
   out_2584612022031943484[55] = 0.0;
   out_2584612022031943484[56] = 0.0;
   out_2584612022031943484[57] = 0.0;
   out_2584612022031943484[58] = 0.0;
   out_2584612022031943484[59] = 0.0;
   out_2584612022031943484[60] = 1.0;
   out_2584612022031943484[61] = 0.0;
   out_2584612022031943484[62] = 0.0;
   out_2584612022031943484[63] = 0.0;
   out_2584612022031943484[64] = 0.0;
   out_2584612022031943484[65] = 0.0;
   out_2584612022031943484[66] = 0.0;
   out_2584612022031943484[67] = 0.0;
   out_2584612022031943484[68] = 0.0;
   out_2584612022031943484[69] = 0.0;
   out_2584612022031943484[70] = 1.0;
   out_2584612022031943484[71] = 0.0;
   out_2584612022031943484[72] = 0.0;
   out_2584612022031943484[73] = 0.0;
   out_2584612022031943484[74] = 0.0;
   out_2584612022031943484[75] = 0.0;
   out_2584612022031943484[76] = 0.0;
   out_2584612022031943484[77] = 0.0;
   out_2584612022031943484[78] = 0.0;
   out_2584612022031943484[79] = 0.0;
   out_2584612022031943484[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_7883455534529976806) {
   out_7883455534529976806[0] = state[0];
   out_7883455534529976806[1] = state[1];
   out_7883455534529976806[2] = state[2];
   out_7883455534529976806[3] = state[3];
   out_7883455534529976806[4] = state[4];
   out_7883455534529976806[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_7883455534529976806[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_7883455534529976806[7] = state[7];
   out_7883455534529976806[8] = state[8];
}
void F_fun(double *state, double dt, double *out_3864057917650639482) {
   out_3864057917650639482[0] = 1;
   out_3864057917650639482[1] = 0;
   out_3864057917650639482[2] = 0;
   out_3864057917650639482[3] = 0;
   out_3864057917650639482[4] = 0;
   out_3864057917650639482[5] = 0;
   out_3864057917650639482[6] = 0;
   out_3864057917650639482[7] = 0;
   out_3864057917650639482[8] = 0;
   out_3864057917650639482[9] = 0;
   out_3864057917650639482[10] = 1;
   out_3864057917650639482[11] = 0;
   out_3864057917650639482[12] = 0;
   out_3864057917650639482[13] = 0;
   out_3864057917650639482[14] = 0;
   out_3864057917650639482[15] = 0;
   out_3864057917650639482[16] = 0;
   out_3864057917650639482[17] = 0;
   out_3864057917650639482[18] = 0;
   out_3864057917650639482[19] = 0;
   out_3864057917650639482[20] = 1;
   out_3864057917650639482[21] = 0;
   out_3864057917650639482[22] = 0;
   out_3864057917650639482[23] = 0;
   out_3864057917650639482[24] = 0;
   out_3864057917650639482[25] = 0;
   out_3864057917650639482[26] = 0;
   out_3864057917650639482[27] = 0;
   out_3864057917650639482[28] = 0;
   out_3864057917650639482[29] = 0;
   out_3864057917650639482[30] = 1;
   out_3864057917650639482[31] = 0;
   out_3864057917650639482[32] = 0;
   out_3864057917650639482[33] = 0;
   out_3864057917650639482[34] = 0;
   out_3864057917650639482[35] = 0;
   out_3864057917650639482[36] = 0;
   out_3864057917650639482[37] = 0;
   out_3864057917650639482[38] = 0;
   out_3864057917650639482[39] = 0;
   out_3864057917650639482[40] = 1;
   out_3864057917650639482[41] = 0;
   out_3864057917650639482[42] = 0;
   out_3864057917650639482[43] = 0;
   out_3864057917650639482[44] = 0;
   out_3864057917650639482[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_3864057917650639482[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_3864057917650639482[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3864057917650639482[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3864057917650639482[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_3864057917650639482[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_3864057917650639482[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_3864057917650639482[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_3864057917650639482[53] = -9.8000000000000007*dt;
   out_3864057917650639482[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_3864057917650639482[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_3864057917650639482[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3864057917650639482[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3864057917650639482[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_3864057917650639482[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_3864057917650639482[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_3864057917650639482[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3864057917650639482[62] = 0;
   out_3864057917650639482[63] = 0;
   out_3864057917650639482[64] = 0;
   out_3864057917650639482[65] = 0;
   out_3864057917650639482[66] = 0;
   out_3864057917650639482[67] = 0;
   out_3864057917650639482[68] = 0;
   out_3864057917650639482[69] = 0;
   out_3864057917650639482[70] = 1;
   out_3864057917650639482[71] = 0;
   out_3864057917650639482[72] = 0;
   out_3864057917650639482[73] = 0;
   out_3864057917650639482[74] = 0;
   out_3864057917650639482[75] = 0;
   out_3864057917650639482[76] = 0;
   out_3864057917650639482[77] = 0;
   out_3864057917650639482[78] = 0;
   out_3864057917650639482[79] = 0;
   out_3864057917650639482[80] = 1;
}
void h_25(double *state, double *unused, double *out_7452911399691068209) {
   out_7452911399691068209[0] = state[6];
}
void H_25(double *state, double *unused, double *out_7488303847216608324) {
   out_7488303847216608324[0] = 0;
   out_7488303847216608324[1] = 0;
   out_7488303847216608324[2] = 0;
   out_7488303847216608324[3] = 0;
   out_7488303847216608324[4] = 0;
   out_7488303847216608324[5] = 0;
   out_7488303847216608324[6] = 1;
   out_7488303847216608324[7] = 0;
   out_7488303847216608324[8] = 0;
}
void h_24(double *state, double *unused, double *out_1394037061893423660) {
   out_1394037061893423660[0] = state[4];
   out_1394037061893423660[1] = state[5];
}
void H_24(double *state, double *unused, double *out_4382868419901425191) {
   out_4382868419901425191[0] = 0;
   out_4382868419901425191[1] = 0;
   out_4382868419901425191[2] = 0;
   out_4382868419901425191[3] = 0;
   out_4382868419901425191[4] = 1;
   out_4382868419901425191[5] = 0;
   out_4382868419901425191[6] = 0;
   out_4382868419901425191[7] = 0;
   out_4382868419901425191[8] = 0;
   out_4382868419901425191[9] = 0;
   out_4382868419901425191[10] = 0;
   out_4382868419901425191[11] = 0;
   out_4382868419901425191[12] = 0;
   out_4382868419901425191[13] = 0;
   out_4382868419901425191[14] = 1;
   out_4382868419901425191[15] = 0;
   out_4382868419901425191[16] = 0;
   out_4382868419901425191[17] = 0;
}
void h_30(double *state, double *unused, double *out_3591227923262566557) {
   out_3591227923262566557[0] = state[4];
}
void H_30(double *state, double *unused, double *out_8440107267985694665) {
   out_8440107267985694665[0] = 0;
   out_8440107267985694665[1] = 0;
   out_8440107267985694665[2] = 0;
   out_8440107267985694665[3] = 0;
   out_8440107267985694665[4] = 1;
   out_8440107267985694665[5] = 0;
   out_8440107267985694665[6] = 0;
   out_8440107267985694665[7] = 0;
   out_8440107267985694665[8] = 0;
}
void h_26(double *state, double *unused, double *out_3145557878326213420) {
   out_3145557878326213420[0] = state[7];
}
void H_26(double *state, double *unused, double *out_3746800528342552100) {
   out_3746800528342552100[0] = 0;
   out_3746800528342552100[1] = 0;
   out_3746800528342552100[2] = 0;
   out_3746800528342552100[3] = 0;
   out_3746800528342552100[4] = 0;
   out_3746800528342552100[5] = 0;
   out_3746800528342552100[6] = 0;
   out_3746800528342552100[7] = 1;
   out_3746800528342552100[8] = 0;
}
void h_27(double *state, double *unused, double *out_3027550484392836676) {
   out_3027550484392836676[0] = state[3];
}
void H_27(double *state, double *unused, double *out_6216513196801751448) {
   out_6216513196801751448[0] = 0;
   out_6216513196801751448[1] = 0;
   out_6216513196801751448[2] = 0;
   out_6216513196801751448[3] = 1;
   out_6216513196801751448[4] = 0;
   out_6216513196801751448[5] = 0;
   out_6216513196801751448[6] = 0;
   out_6216513196801751448[7] = 0;
   out_6216513196801751448[8] = 0;
}
void h_29(double *state, double *unused, double *out_3485751846551398338) {
   out_3485751846551398338[0] = state[1];
}
void H_29(double *state, double *unused, double *out_7929875923671302481) {
   out_7929875923671302481[0] = 0;
   out_7929875923671302481[1] = 1;
   out_7929875923671302481[2] = 0;
   out_7929875923671302481[3] = 0;
   out_7929875923671302481[4] = 0;
   out_7929875923671302481[5] = 0;
   out_7929875923671302481[6] = 0;
   out_7929875923671302481[7] = 0;
   out_7929875923671302481[8] = 0;
}
void h_28(double *state, double *unused, double *out_2615179386688120159) {
   out_2615179386688120159[0] = state[0];
}
void H_28(double *state, double *unused, double *out_5434469132968718561) {
   out_5434469132968718561[0] = 1;
   out_5434469132968718561[1] = 0;
   out_5434469132968718561[2] = 0;
   out_5434469132968718561[3] = 0;
   out_5434469132968718561[4] = 0;
   out_5434469132968718561[5] = 0;
   out_5434469132968718561[6] = 0;
   out_5434469132968718561[7] = 0;
   out_5434469132968718561[8] = 0;
}
void h_31(double *state, double *unused, double *out_265958495233009880) {
   out_265958495233009880[0] = state[8];
}
void H_31(double *state, double *unused, double *out_7518949809093568752) {
   out_7518949809093568752[0] = 0;
   out_7518949809093568752[1] = 0;
   out_7518949809093568752[2] = 0;
   out_7518949809093568752[3] = 0;
   out_7518949809093568752[4] = 0;
   out_7518949809093568752[5] = 0;
   out_7518949809093568752[6] = 0;
   out_7518949809093568752[7] = 0;
   out_7518949809093568752[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_2070268012761229651) {
  err_fun(nom_x, delta_x, out_2070268012761229651);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4265412688424853444) {
  inv_err_fun(nom_x, true_x, out_4265412688424853444);
}
void car_H_mod_fun(double *state, double *out_2584612022031943484) {
  H_mod_fun(state, out_2584612022031943484);
}
void car_f_fun(double *state, double dt, double *out_7883455534529976806) {
  f_fun(state,  dt, out_7883455534529976806);
}
void car_F_fun(double *state, double dt, double *out_3864057917650639482) {
  F_fun(state,  dt, out_3864057917650639482);
}
void car_h_25(double *state, double *unused, double *out_7452911399691068209) {
  h_25(state, unused, out_7452911399691068209);
}
void car_H_25(double *state, double *unused, double *out_7488303847216608324) {
  H_25(state, unused, out_7488303847216608324);
}
void car_h_24(double *state, double *unused, double *out_1394037061893423660) {
  h_24(state, unused, out_1394037061893423660);
}
void car_H_24(double *state, double *unused, double *out_4382868419901425191) {
  H_24(state, unused, out_4382868419901425191);
}
void car_h_30(double *state, double *unused, double *out_3591227923262566557) {
  h_30(state, unused, out_3591227923262566557);
}
void car_H_30(double *state, double *unused, double *out_8440107267985694665) {
  H_30(state, unused, out_8440107267985694665);
}
void car_h_26(double *state, double *unused, double *out_3145557878326213420) {
  h_26(state, unused, out_3145557878326213420);
}
void car_H_26(double *state, double *unused, double *out_3746800528342552100) {
  H_26(state, unused, out_3746800528342552100);
}
void car_h_27(double *state, double *unused, double *out_3027550484392836676) {
  h_27(state, unused, out_3027550484392836676);
}
void car_H_27(double *state, double *unused, double *out_6216513196801751448) {
  H_27(state, unused, out_6216513196801751448);
}
void car_h_29(double *state, double *unused, double *out_3485751846551398338) {
  h_29(state, unused, out_3485751846551398338);
}
void car_H_29(double *state, double *unused, double *out_7929875923671302481) {
  H_29(state, unused, out_7929875923671302481);
}
void car_h_28(double *state, double *unused, double *out_2615179386688120159) {
  h_28(state, unused, out_2615179386688120159);
}
void car_H_28(double *state, double *unused, double *out_5434469132968718561) {
  H_28(state, unused, out_5434469132968718561);
}
void car_h_31(double *state, double *unused, double *out_265958495233009880) {
  h_31(state, unused, out_265958495233009880);
}
void car_H_31(double *state, double *unused, double *out_7518949809093568752) {
  H_31(state, unused, out_7518949809093568752);
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
