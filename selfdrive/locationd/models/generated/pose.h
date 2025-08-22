#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_4197240932685800719);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_8091385002608574816);
void pose_H_mod_fun(double *state, double *out_209432845494472972);
void pose_f_fun(double *state, double dt, double *out_7558387581431867623);
void pose_F_fun(double *state, double dt, double *out_5546184258405748350);
void pose_h_4(double *state, double *unused, double *out_1928798689699948695);
void pose_H_4(double *state, double *unused, double *out_8528763386588516166);
void pose_h_10(double *state, double *unused, double *out_3989753312953530471);
void pose_H_10(double *state, double *unused, double *out_6637515270266097066);
void pose_h_13(double *state, double *unused, double *out_1637769040693296482);
void pose_H_13(double *state, double *unused, double *out_2307349478804334521);
void pose_h_14(double *state, double *unused, double *out_2199265346921650482);
void pose_H_14(double *state, double *unused, double *out_5445974954293143870);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}