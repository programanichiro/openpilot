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
void live_H(double *in_vec, double *out_4569387946329419208);
void live_err_fun(double *nom_x, double *delta_x, double *out_6556537210215607924);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_7856042354861612950);
void live_H_mod_fun(double *state, double *out_7556345036683071491);
void live_f_fun(double *state, double dt, double *out_1640672703619824350);
void live_F_fun(double *state, double dt, double *out_579857707936974651);
void live_h_4(double *state, double *unused, double *out_3157812660945645194);
void live_H_4(double *state, double *unused, double *out_2799050509402396472);
void live_h_9(double *state, double *unused, double *out_5347814014346992222);
void live_H_9(double *state, double *unused, double *out_3040240156031987117);
void live_h_10(double *state, double *unused, double *out_2516387686715293528);
void live_H_10(double *state, double *unused, double *out_5199155354689267728);
void live_h_12(double *state, double *unused, double *out_4403747035606413249);
void live_H_12(double *state, double *unused, double *out_7818506917434358267);
void live_h_31(double *state, double *unused, double *out_1924041066080295585);
void live_H_31(double *state, double *unused, double *out_7882674123950179640);
void live_h_32(double *state, double *unused, double *out_3062932800655568819);
void live_H_32(double *state, double *unused, double *out_1947013329103094778);
void live_h_13(double *state, double *unused, double *out_1363461724325150419);
void live_H_13(double *state, double *unused, double *out_4943892349359658287);
void live_h_14(double *state, double *unused, double *out_5347814014346992222);
void live_H_14(double *state, double *unused, double *out_3040240156031987117);
void live_h_33(double *state, double *unused, double *out_6673789227314719959);
void live_H_33(double *state, double *unused, double *out_4732117119311322036);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}