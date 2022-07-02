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
void car_err_fun(double *nom_x, double *delta_x, double *out_3171762008471037189);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4110702914314724221);
void car_H_mod_fun(double *state, double *out_5033876641403212536);
void car_f_fun(double *state, double dt, double *out_3454824215855402291);
void car_F_fun(double *state, double dt, double *out_1974642873389597258);
void car_h_25(double *state, double *unused, double *out_1355303837527753590);
void car_H_25(double *state, double *unused, double *out_3910608959992168667);
void car_h_24(double *state, double *unused, double *out_3737888511771369975);
void car_H_24(double *state, double *unused, double *out_4381066442035507391);
void car_h_30(double *state, double *unused, double *out_753885679914390036);
void car_H_30(double *state, double *unused, double *out_8438305290119776865);
void car_h_26(double *state, double *unused, double *out_4247787587746455609);
void car_H_26(double *state, double *unused, double *out_7652112278866224891);
void car_h_27(double *state, double *unused, double *out_6356977105188599235);
void car_H_27(double *state, double *unused, double *out_6214711218935833648);
void car_h_29(double *state, double *unused, double *out_6632171167473105124);
void car_H_29(double *state, double *unused, double *out_7928073945805384681);
void car_h_28(double *state, double *unused, double *out_6863859752685627434);
void car_H_28(double *state, double *unused, double *out_5436271110834636361);
void car_h_31(double *state, double *unused, double *out_2282145690726268760);
void car_H_31(double *state, double *unused, double *out_3879962998115208239);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}