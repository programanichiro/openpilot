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
void car_err_fun(double *nom_x, double *delta_x, double *out_1876979179817506794);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_738824714567483071);
void car_H_mod_fun(double *state, double *out_1020147720070790775);
void car_f_fun(double *state, double dt, double *out_1837887467846292217);
void car_F_fun(double *state, double dt, double *out_5787251712374232048);
void car_h_25(double *state, double *unused, double *out_2627157697858734525);
void car_H_25(double *state, double *unused, double *out_3076857351056495453);
void car_h_24(double *state, double *unused, double *out_235510140924072653);
void car_H_24(double *state, double *unused, double *out_1669651975910013581);
void car_h_30(double *state, double *unused, double *out_4077789090487617692);
void car_H_30(double *state, double *unused, double *out_5595190309563744080);
void car_h_26(double *state, double *unused, double *out_543163231077975709);
void car_H_26(double *state, double *unused, double *out_664645967817560771);
void car_h_27(double *state, double *unused, double *out_8160908663677993179);
void car_H_27(double *state, double *unused, double *out_7818784380747687297);
void car_h_29(double *state, double *unused, double *out_5768236925223298652);
void car_H_29(double *state, double *unused, double *out_6105421653878136264);
void car_h_28(double *state, double *unused, double *out_6004271568787897929);
void car_H_28(double *state, double *unused, double *out_1023022636808605690);
void car_h_31(double *state, double *unused, double *out_3764190290286879080);
void car_H_31(double *state, double *unused, double *out_1290854070050912247);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}