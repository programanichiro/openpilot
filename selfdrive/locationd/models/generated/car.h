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
void car_err_fun(double *nom_x, double *delta_x, double *out_1259430672998403639);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6486332291499723158);
void car_H_mod_fun(double *state, double *out_1916627579971738044);
void car_f_fun(double *state, double dt, double *out_225145524562687816);
void car_F_fun(double *state, double dt, double *out_4004130248429785926);
void car_h_25(double *state, double *unused, double *out_3571277728892257344);
void car_H_25(double *state, double *unused, double *out_3606441959916610189);
void car_h_24(double *state, double *unused, double *out_8046866872121177892);
void car_H_24(double *state, double *unused, double *out_8426763464572598452);
void car_h_30(double *state, double *unused, double *out_6994193694063245087);
void car_H_30(double *state, double *unused, double *out_3735780907059850259);
void car_h_26(double *state, double *unused, double *out_6450877111985460884);
void car_H_26(double *state, double *unused, double *out_7347945278790666413);
void car_h_27(double *state, double *unused, double *out_8629015287357120804);
void car_H_27(double *state, double *unused, double *out_5910544218860275170);
void car_h_29(double *state, double *unused, double *out_6012109600727946975);
void car_H_29(double *state, double *unused, double *out_3225549562745458075);
void car_h_28(double *state, double *unused, double *out_7629441143651440554);
void car_H_28(double *state, double *unused, double *out_5740438110910194839);
void car_h_31(double *state, double *unused, double *out_4973242187042714312);
void car_H_31(double *state, double *unused, double *out_3575795998039649761);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}