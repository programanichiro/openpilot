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
void car_err_fun(double *nom_x, double *delta_x, double *out_8750905003075963447);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5158842673956621266);
void car_H_mod_fun(double *state, double *out_6032308480841622457);
void car_f_fun(double *state, double dt, double *out_2168661768497310835);
void car_F_fun(double *state, double dt, double *out_1487349331672120911);
void car_h_25(double *state, double *unused, double *out_5440738245400390420);
void car_H_25(double *state, double *unused, double *out_1227009239193474661);
void car_h_24(double *state, double *unused, double *out_5094988861551536833);
void car_H_24(double *state, double *unused, double *out_3399658838198974227);
void car_h_30(double *state, double *unused, double *out_6356988911105007499);
void car_H_30(double *state, double *unused, double *out_5689681102298142094);
void car_h_26(double *state, double *unused, double *out_6331770248610806161);
void car_H_26(double *state, double *unused, double *out_4968512558067530885);
void car_h_27(double *state, double *unused, double *out_4770834252289557981);
void car_H_27(double *state, double *unused, double *out_3514917790497717183);
void car_h_29(double *state, double *unused, double *out_8765240415378661329);
void car_H_29(double *state, double *unused, double *out_1801555063628166150);
void car_h_28(double *state, double *unused, double *out_2937915041929972828);
void car_H_28(double *state, double *unused, double *out_3280843953441364424);
void car_h_31(double *state, double *unused, double *out_3868288543625310014);
void car_H_31(double *state, double *unused, double *out_1196363277316514233);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}