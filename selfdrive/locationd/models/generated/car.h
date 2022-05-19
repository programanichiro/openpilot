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
void car_err_fun(double *nom_x, double *delta_x, double *out_7831334908946486631);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1412484805370031907);
void car_H_mod_fun(double *state, double *out_722397761437678664);
void car_f_fun(double *state, double dt, double *out_7415195887102028552);
void car_F_fun(double *state, double dt, double *out_7715249174232332377);
void car_h_25(double *state, double *unused, double *out_1809245855196595095);
void car_H_25(double *state, double *unused, double *out_3284134045966130737);
void car_h_24(double *state, double *unused, double *out_5296156693199309723);
void car_H_24(double *state, double *unused, double *out_3754591528009469461);
void car_h_30(double *state, double *unused, double *out_2079843709083647517);
void car_H_30(double *state, double *unused, double *out_3413472993109370807);
void car_h_26(double *state, double *unused, double *out_8326294766543820985);
void car_H_26(double *state, double *unused, double *out_7025637364840186961);
void car_h_27(double *state, double *unused, double *out_8208287372610444241);
void car_H_27(double *state, double *unused, double *out_5588236304909795718);
void car_h_29(double *state, double *unused, double *out_6140153402819735511);
void car_H_29(double *state, double *unused, double *out_2903241648794978623);
void car_h_28(double *state, double *unused, double *out_3206614104949598596);
void car_H_28(double *state, double *unused, double *out_5337968760214020500);
void car_h_31(double *state, double *unused, double *out_5446695383450617445);
void car_H_31(double *state, double *unused, double *out_3253488084089170309);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}