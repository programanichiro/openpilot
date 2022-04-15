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
void car_err_fun(double *nom_x, double *delta_x, double *out_8978105205623168168);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_527484150705711459);
void car_H_mod_fun(double *state, double *out_5422076647877894621);
void car_f_fun(double *state, double dt, double *out_3002906037626941687);
void car_F_fun(double *state, double dt, double *out_5137753966668609063);
void car_h_25(double *state, double *unused, double *out_4442074441953279896);
void car_H_25(double *state, double *unused, double *out_5376650681047099606);
void car_h_24(double *state, double *unused, double *out_8121433539224248093);
void car_H_24(double *state, double *unused, double *out_7549300280052599172);
void car_h_30(double *state, double *unused, double *out_4166880379668774007);
void car_H_30(double *state, double *unused, double *out_2858317722539850979);
void car_h_26(double *state, double *unused, double *out_7487620720409045830);
void car_H_26(double *state, double *unused, double *out_9118153999921155830);
void car_h_27(double *state, double *unused, double *out_559598825707565749);
void car_H_27(double *state, double *unused, double *out_5033081034340275890);
void car_h_29(double *state, double *unused, double *out_3507324816063050710);
void car_H_29(double *state, double *unused, double *out_2348086378225458795);
void car_h_28(double *state, double *unused, double *out_5839442691706283397);
void car_H_28(double *state, double *unused, double *out_7430485395294989369);
void car_h_31(double *state, double *unused, double *out_8079523970207302246);
void car_H_31(double *state, double *unused, double *out_8702381971555044310);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}