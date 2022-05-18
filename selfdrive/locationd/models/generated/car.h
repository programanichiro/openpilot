#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_5671110135909666668);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_3548638300579484155);
void car_H_mod_fun(double *state, double *out_5423950865144251747);
void car_f_fun(double *state, double dt, double *out_4814793839893572125);
void car_F_fun(double *state, double dt, double *out_4081315827943822484);
void car_h_25(double *state, double *unused, double *out_2182817408992859716);
void car_H_25(double *state, double *unused, double *out_4915178676720596354);
void car_h_24(double *state, double *unused, double *out_5657492609903548931);
void car_H_24(double *state, double *unused, double *out_4386698687038078100);
void car_h_30(double *state, double *unused, double *out_5702259543313505322);
void car_H_30(double *state, double *unused, double *out_2396845718213347727);
void car_h_26(double *state, double *unused, double *out_7406421596062331584);
void car_H_26(double *state, double *unused, double *out_8656681995594652578);
void car_h_27(double *state, double *unused, double *out_8581858926406708862);
void car_H_27(double *state, double *unused, double *out_4571609030013772638);
void car_h_29(double *state, double *unused, double *out_4944409398152686512);
void car_H_29(double *state, double *unused, double *out_1886614373898955543);
void car_h_28(double *state, double *unused, double *out_3580185658745863217);
void car_H_28(double *state, double *unused, double *out_6969013390968486117);
void car_h_31(double *state, double *unused, double *out_6313843272270880276);
void car_H_31(double *state, double *unused, double *out_9163853975881547562);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}