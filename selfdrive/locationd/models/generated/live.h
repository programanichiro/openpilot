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
void live_H(double *in_vec, double *out_8763911302294510439);
void live_err_fun(double *nom_x, double *delta_x, double *out_4694240674060362296);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_8987034116723791834);
void live_H_mod_fun(double *state, double *out_4062262789697768732);
void live_f_fun(double *state, double dt, double *out_5839784521047323211);
void live_F_fun(double *state, double dt, double *out_8192204327667344615);
void live_h_4(double *state, double *unused, double *out_8608266300252683133);
void live_H_4(double *state, double *unused, double *out_3822630080322556112);
void live_h_9(double *state, double *unused, double *out_1588695200126532466);
void live_H_9(double *state, double *unused, double *out_7336895058122548034);
void live_h_10(double *state, double *unused, double *out_2409671769183914665);
void live_H_10(double *state, double *unused, double *out_1976361460291977877);
void live_h_12(double *state, double *unused, double *out_3729976212899161138);
void live_H_12(double *state, double *unused, double *out_2558628296720176884);
void live_h_31(double *state, double *unused, double *out_4896809077366527731);
void live_H_31(double *state, double *unused, double *out_6859094553030020000);
void live_h_32(double *state, double *unused, double *out_4233969394671175643);
void live_H_32(double *state, double *unused, double *out_7320456922635903957);
void live_h_13(double *state, double *unused, double *out_4310268936889545101);
void live_H_13(double *state, double *unused, double *out_7768491640206014986);
void live_h_14(double *state, double *unused, double *out_1588695200126532466);
void live_H_14(double *state, double *unused, double *out_7336895058122548034);
void live_h_33(double *state, double *unused, double *out_4534444071083632422);
void live_H_33(double *state, double *unused, double *out_3708537548391162396);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}