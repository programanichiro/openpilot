#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_6287082876705174730);
void live_err_fun(double *nom_x, double *delta_x, double *out_6622887547268195339);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_2469532870010074731);
void live_H_mod_fun(double *state, double *out_1372219795177703307);
void live_f_fun(double *state, double dt, double *out_6471053790479580686);
void live_F_fun(double *state, double dt, double *out_8723779082052171387);
void live_h_4(double *state, double *unused, double *out_602150755429471006);
void live_H_4(double *state, double *unused, double *out_1181153021740974696);
void live_h_9(double *state, double *unused, double *out_8053080118545079418);
void live_H_9(double *state, double *unused, double *out_6106065913523472774);
void live_h_10(double *state, double *unused, double *out_8952859443434322822);
void live_H_10(double *state, double *unused, double *out_8116439110641593908);
void live_h_12(double *state, double *unused, double *out_5876765162710571962);
void live_H_12(double *state, double *unused, double *out_7562411398783707692);
void live_h_35(double *state, double *unused, double *out_8882447461541958668);
void live_H_35(double *state, double *unused, double *out_9215205749443062111);
void live_h_32(double *state, double *unused, double *out_4113087374974639312);
void live_H_32(double *state, double *unused, double *out_2033190202040276390);
void live_h_13(double *state, double *unused, double *out_9058530126902937662);
void live_H_13(double *state, double *unused, double *out_2889039755446710196);
void live_h_14(double *state, double *unused, double *out_8053080118545079418);
void live_H_14(double *state, double *unused, double *out_6106065913523472774);
void live_h_33(double *state, double *unused, double *out_6825187964767546674);
void live_H_33(double *state, double *unused, double *out_6064648744804204507);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}