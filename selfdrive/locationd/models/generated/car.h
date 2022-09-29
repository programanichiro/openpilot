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
void car_err_fun(double *nom_x, double *delta_x, double *out_6961449098991356967);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7191079212549050895);
void car_H_mod_fun(double *state, double *out_292831340756603194);
void car_f_fun(double *state, double dt, double *out_8924841413416666487);
void car_F_fun(double *state, double dt, double *out_631402002945752087);
void car_h_25(double *state, double *unused, double *out_5191763995009145519);
void car_H_25(double *state, double *unused, double *out_9205295690685381982);
void car_h_24(double *state, double *unused, double *out_4678822133761862964);
void car_H_24(double *state, double *unused, double *out_7028081267078232009);
void car_h_30(double *state, double *unused, double *out_1396063931467149644);
void car_H_30(double *state, double *unused, double *out_6686962732178133355);
void car_h_26(double *state, double *unused, double *out_4246665939613807191);
void car_H_26(double *state, double *unused, double *out_5499945064150113410);
void car_h_27(double *state, double *unused, double *out_334022349075278952);
void car_H_27(double *state, double *unused, double *out_6937346124080504653);
void car_h_29(double *state, double *unused, double *out_8332843222373709646);
void car_H_29(double *state, double *unused, double *out_5223983397210953620);
void car_h_28(double *state, double *unused, double *out_5308707522005677883);
void car_H_28(double *state, double *unused, double *out_7187613668776279871);
void car_h_31(double *state, double *unused, double *out_2427569640084547844);
void car_H_31(double *state, double *unused, double *out_9174649728808421554);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}