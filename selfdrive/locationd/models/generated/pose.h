#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_3762798617585209229);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_5516892092138461329);
void pose_H_mod_fun(double *state, double *out_7476032322802575233);
void pose_f_fun(double *state, double dt, double *out_2337861120406759811);
void pose_F_fun(double *state, double dt, double *out_7110526711015872718);
void pose_h_4(double *state, double *unused, double *out_5575517766908778607);
void pose_H_4(double *state, double *unused, double *out_8230193324857287840);
void pose_h_10(double *state, double *unused, double *out_231064800032479191);
void pose_H_10(double *state, double *unused, double *out_809148575071622373);
void pose_h_13(double *state, double *unused, double *out_6428852753307240167);
void pose_H_13(double *state, double *unused, double *out_2605919540535562847);
void pose_h_14(double *state, double *unused, double *out_6226409763366102898);
void pose_H_14(double *state, double *unused, double *out_6253309892512779247);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}