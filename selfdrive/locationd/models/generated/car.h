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
void car_err_fun(double *nom_x, double *delta_x, double *out_8410643270261249886);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_2170846217420803909);
void car_H_mod_fun(double *state, double *out_5654866458334913771);
void car_f_fun(double *state, double dt, double *out_5195591810852336672);
void car_F_fun(double *state, double dt, double *out_7896467353570579206);
void car_h_25(double *state, double *unused, double *out_4670350168092863626);
void car_H_25(double *state, double *unused, double *out_1965496603878225476);
void car_h_24(double *state, double *unused, double *out_4805896676923701039);
void car_H_24(double *state, double *unused, double *out_4138146202883725042);
void car_h_30(double *state, double *unused, double *out_7005558679266337054);
void car_H_30(double *state, double *unused, double *out_4951193737613391279);
void car_h_26(double *state, double *unused, double *out_1790750784999660086);
void car_H_26(double *state, double *unused, double *out_5706999922752281700);
void car_h_27(double *state, double *unused, double *out_474539895022783911);
void car_H_27(double *state, double *unused, double *out_2776430425812966368);
void car_h_29(double *state, double *unused, double *out_7608874563942814571);
void car_H_29(double *state, double *unused, double *out_1063067698943415335);
void car_h_28(double *state, double *unused, double *out_5010544136318048544);
void car_H_28(double *state, double *unused, double *out_4019331318126115239);
void car_h_31(double *state, double *unused, double *out_3092915088727808815);
void car_H_31(double *state, double *unused, double *out_1934850642001265048);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}