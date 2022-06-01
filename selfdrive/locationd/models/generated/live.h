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
void live_H(double *in_vec, double *out_5537917853957494860);
void live_err_fun(double *nom_x, double *delta_x, double *out_3173224133217314314);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_2801905154792875483);
void live_H_mod_fun(double *state, double *out_74819044353985183);
void live_f_fun(double *state, double dt, double *out_4760523032026918799);
void live_F_fun(double *state, double dt, double *out_8496492668489745222);
void live_h_4(double *state, double *unused, double *out_4917282499255070247);
void live_H_4(double *state, double *unused, double *out_2921092052629370473);
void live_h_9(double *state, double *unused, double *out_6325298498358679280);
void live_H_9(double *state, double *unused, double *out_3162281699258961118);
void live_h_10(double *state, double *unused, double *out_156214839265580708);
void live_H_10(double *state, double *unused, double *out_5729337145004090259);
void live_h_12(double *state, double *unused, double *out_6605349573946297244);
void live_H_12(double *state, double *unused, double *out_7940548460661332268);
void live_h_31(double *state, double *unused, double *out_1511533530509665495);
void live_H_31(double *state, double *unused, double *out_6287754110001977849);
void live_h_32(double *state, double *unused, double *out_8963794230603142874);
void live_H_32(double *state, double *unused, double *out_8073254275610650667);
void live_h_13(double *state, double *unused, double *out_8288584196068907752);
void live_H_13(double *state, double *unused, double *out_1494949637232207687);
void live_h_14(double *state, double *unused, double *out_6325298498358679280);
void live_H_14(double *state, double *unused, double *out_3162281699258961118);
void live_h_33(double *state, double *unused, double *out_5618792197983551011);
void live_H_33(double *state, double *unused, double *out_9008432959068716163);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}