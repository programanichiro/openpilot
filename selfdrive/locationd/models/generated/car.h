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
void car_err_fun(double *nom_x, double *delta_x, double *out_4325689932387369841);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1119892225369903103);
void car_H_mod_fun(double *state, double *out_4832036451820639239);
void car_f_fun(double *state, double dt, double *out_6035807853235887415);
void car_F_fun(double *state, double dt, double *out_8459570567044433633);
void car_h_25(double *state, double *unused, double *out_4434622194633419438);
void car_H_25(double *state, double *unused, double *out_279236249526357609);
void car_h_24(double *state, double *unused, double *out_5396490682701100739);
void car_H_24(double *state, double *unused, double *out_6854808056117875710);
void car_h_30(double *state, double *unused, double *out_8072071722887441788);
void car_H_30(double *state, double *unused, double *out_7195926591017974364);
void car_h_26(double *state, double *unused, double *out_7337886299377777143);
void car_H_26(double *state, double *unused, double *out_3462267069347698615);
void car_h_27(double *state, double *unused, double *out_3764718201522586999);
void car_H_27(double *state, double *unused, double *out_5021163279217549453);
void car_h_29(double *state, double *unused, double *out_5372999083161264183);
void car_H_29(double *state, double *unused, double *out_7706157935332366548);
void car_h_28(double *state, double *unused, double *out_3944031092696943810);
void car_H_28(double *state, double *unused, double *out_2623758918262835974);
void car_h_31(double *state, double *unused, double *out_6526310190682413795);
void car_H_31(double *state, double *unused, double *out_309882211403318037);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}