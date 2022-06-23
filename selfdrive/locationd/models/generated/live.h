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
void live_H(double *in_vec, double *out_8112787183996767333);
void live_err_fun(double *nom_x, double *delta_x, double *out_7438452076199970549);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_9072751160980696045);
void live_H_mod_fun(double *state, double *out_3531246140483666849);
void live_f_fun(double *state, double dt, double *out_5640748228499018316);
void live_F_fun(double *state, double dt, double *out_40891815030632395);
void live_h_4(double *state, double *unused, double *out_5552634738632533904);
void live_H_4(double *state, double *unused, double *out_7051147384659624149);
void live_h_9(double *state, double *unused, double *out_8377485234475286224);
void live_H_9(double *state, double *unused, double *out_236071550604823321);
void live_h_10(double *state, double *unused, double *out_743876404820101682);
void live_H_10(double *state, double *unused, double *out_3520204588481565727);
void live_h_12(double *state, double *unused, double *out_3690241237526513347);
void live_H_12(double *state, double *unused, double *out_2031690976627662354);
void live_h_31(double *state, double *unused, double *out_453781667833494065);
void live_H_31(double *state, double *unused, double *out_713872055697351355);
void live_h_32(double *state, double *unused, double *out_6886599282477753184);
void live_H_32(double *state, double *unused, double *out_7903184564958925843);
void live_h_13(double *state, double *unused, double *out_323612180662106075);
void live_H_13(double *state, double *unused, double *out_2926532109473779390);
void live_h_14(double *state, double *unused, double *out_8377485234475286224);
void live_H_14(double *state, double *unused, double *out_236071550604823321);
void live_h_33(double *state, double *unused, double *out_4490303204228291593);
void live_H_33(double *state, double *unused, double *out_3864429060336208959);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}