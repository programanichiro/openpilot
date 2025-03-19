#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_6279086065746722501);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_2166314779381952832);
void pose_H_mod_fun(double *state, double *out_5648904405508167058);
void pose_f_fun(double *state, double dt, double *out_5535553793790539232);
void pose_F_fun(double *state, double dt, double *out_3418835783146762417);
void pose_h_4(double *state, double *unused, double *out_8681835026761677731);
void pose_H_4(double *state, double *unused, double *out_3295457260549654284);
void pose_h_10(double *state, double *unused, double *out_5359658747254490916);
void pose_H_10(double *state, double *unused, double *out_4562305178044036433);
void pose_h_13(double *state, double *unused, double *out_2317300498181279990);
void pose_H_13(double *state, double *unused, double *out_83183435217321483);
void pose_h_14(double *state, double *unused, double *out_7153354114899869002);
void pose_H_14(double *state, double *unused, double *out_667783595789830245);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}