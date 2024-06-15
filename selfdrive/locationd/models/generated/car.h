#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_8937458499683816514);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_9068690400416595123);
void car_H_mod_fun(double *state, double *out_5097223253014355728);
void car_f_fun(double *state, double dt, double *out_3149203961878712284);
void car_F_fun(double *state, double dt, double *out_5048345358166039832);
void car_h_25(double *state, double *unused, double *out_8982389284843880955);
void car_H_25(double *state, double *unused, double *out_5853167493858090130);
void car_h_24(double *state, double *unused, double *out_1960999661702281101);
void car_H_24(double *state, double *unused, double *out_9035650981049138350);
void car_h_30(double *state, double *unused, double *out_1701089532044998220);
void car_H_30(double *state, double *unused, double *out_8371500452365338757);
void car_h_26(double *state, double *unused, double *out_4737875583489330905);
void car_H_26(double *state, double *unused, double *out_2111664174984033906);
void car_h_27(double *state, double *unused, double *out_8520844935964008226);
void car_H_27(double *state, double *unused, double *out_6196737140564913846);
void car_h_29(double *state, double *unused, double *out_757579679143335785);
void car_H_29(double *state, double *unused, double *out_8881731796679730941);
void car_h_28(double *state, double *unused, double *out_2122019703949867755);
void car_H_28(double *state, double *unused, double *out_3799332779610200367);
void car_h_31(double *state, double *unused, double *out_3068656258173132582);
void car_H_31(double *state, double *unused, double *out_1485456072750682430);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}