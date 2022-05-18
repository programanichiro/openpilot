#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_203763629049945927);
void live_err_fun(double *nom_x, double *delta_x, double *out_6615215334932537642);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_3427045033875063057);
void live_H_mod_fun(double *state, double *out_152905880027603843);
void live_f_fun(double *state, double dt, double *out_6640930378359727142);
void live_F_fun(double *state, double dt, double *out_8636870859560165645);
void live_h_4(double *state, double *unused, double *out_7945567079173314142);
void live_H_4(double *state, double *unused, double *out_5298139631988645786);
void live_h_9(double *state, double *unused, double *out_1312581086390258992);
void live_H_9(double *state, double *unused, double *out_5861385506456458360);
void live_h_10(double *state, double *unused, double *out_6351328039698630590);
void live_H_10(double *state, double *unused, double *out_8663732705569998162);
void live_h_12(double *state, double *unused, double *out_609511315937755073);
void live_H_12(double *state, double *unused, double *out_1083118745054087210);
void live_h_31(double *state, double *unused, double *out_4168344227315293461);
void live_H_31(double *state, double *unused, double *out_1662444287270926499);
void live_h_32(double *state, double *unused, double *out_2208594815178244637);
void live_H_32(double *state, double *unused, double *out_1224037135577527997);
void live_h_13(double *state, double *unused, double *out_1390305149260195769);
void live_H_13(double *state, double *unused, double *out_6026262890563242966);
void live_h_14(double *state, double *unused, double *out_1312581086390258992);
void live_H_14(double *state, double *unused, double *out_5861385506456458360);
void live_h_33(double *state, double *unused, double *out_5108101792277788640);
void live_H_33(double *state, double *unused, double *out_414643908925415975);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}