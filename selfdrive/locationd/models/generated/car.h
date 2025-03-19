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
void car_err_fun(double *nom_x, double *delta_x, double *out_7960405237366517205);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7862762090034680680);
void car_H_mod_fun(double *state, double *out_4138065836317693599);
void car_f_fun(double *state, double dt, double *out_1608403082623815818);
void car_F_fun(double *state, double dt, double *out_72870844652074545);
void car_h_25(double *state, double *unused, double *out_3299501452529380860);
void car_H_25(double *state, double *unused, double *out_7273763929392063695);
void car_h_24(double *state, double *unused, double *out_8845649651338186010);
void car_H_24(double *state, double *unused, double *out_2942840412474121214);
void car_h_30(double *state, double *unused, double *out_4125153558317223716);
void car_H_30(double *state, double *unused, double *out_7144424982248823625);
void car_h_26(double *state, double *unused, double *out_9126826825978069361);
void car_H_26(double *state, double *unused, double *out_7930617993502375599);
void car_h_27(double *state, double *unused, double *out_1702171815131464785);
void car_H_27(double *state, double *unused, double *out_4969661670448398714);
void car_h_29(double *state, double *unused, double *out_5068663411218886151);
void car_H_29(double *state, double *unused, double *out_7654656326563215809);
void car_h_28(double *state, double *unused, double *out_2305487699807942302);
void car_H_28(double *state, double *unused, double *out_2572257309493685235);
void car_h_31(double *state, double *unused, double *out_7161184928957882512);
void car_H_31(double *state, double *unused, double *out_7304409891269024123);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}