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
void car_err_fun(double *nom_x, double *delta_x, double *out_2070268012761229651);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4265412688424853444);
void car_H_mod_fun(double *state, double *out_2584612022031943484);
void car_f_fun(double *state, double dt, double *out_7883455534529976806);
void car_F_fun(double *state, double dt, double *out_3864057917650639482);
void car_h_25(double *state, double *unused, double *out_7452911399691068209);
void car_H_25(double *state, double *unused, double *out_7488303847216608324);
void car_h_24(double *state, double *unused, double *out_1394037061893423660);
void car_H_24(double *state, double *unused, double *out_4382868419901425191);
void car_h_30(double *state, double *unused, double *out_3591227923262566557);
void car_H_30(double *state, double *unused, double *out_8440107267985694665);
void car_h_26(double *state, double *unused, double *out_3145557878326213420);
void car_H_26(double *state, double *unused, double *out_3746800528342552100);
void car_h_27(double *state, double *unused, double *out_3027550484392836676);
void car_H_27(double *state, double *unused, double *out_6216513196801751448);
void car_h_29(double *state, double *unused, double *out_3485751846551398338);
void car_H_29(double *state, double *unused, double *out_7929875923671302481);
void car_h_28(double *state, double *unused, double *out_2615179386688120159);
void car_H_28(double *state, double *unused, double *out_5434469132968718561);
void car_h_31(double *state, double *unused, double *out_265958495233009880);
void car_H_31(double *state, double *unused, double *out_7518949809093568752);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}