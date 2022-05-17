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
void live_H(double *in_vec, double *out_3483100787921365159);
void live_err_fun(double *nom_x, double *delta_x, double *out_6250513862940019271);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_2438799314880860478);
void live_H_mod_fun(double *state, double *out_6134002557142884385);
void live_f_fun(double *state, double dt, double *out_5714529502036206260);
void live_F_fun(double *state, double dt, double *out_7501004244675927010);
void live_h_4(double *state, double *unused, double *out_6934495951339207705);
void live_H_4(double *state, double *unused, double *out_4394660202728893828);
void live_h_9(double *state, double *unused, double *out_7387929548916810335);
void live_H_9(double *state, double *unused, double *out_1505798650448814486);
void live_h_10(double *state, double *unused, double *out_6022400556223056722);
void live_H_10(double *state, double *unused, double *out_1725494204179519976);
void live_h_12(double *state, double *unused, double *out_2512725506475548655);
void live_H_12(double *state, double *unused, double *out_3773561177681300161);
void live_h_31(double *state, double *unused, double *out_9122883903975310898);
void live_H_31(double *state, double *unused, double *out_1027998145356286452);
void live_h_32(double *state, double *unused, double *out_4592011657649615550);
void live_H_32(double *state, double *unused, double *out_896833360415545983);
void live_h_13(double *state, double *unused, double *out_3711051745725782804);
void live_H_13(double *state, double *unused, double *out_8612442169034944907);
void live_h_14(double *state, double *unused, double *out_7387929548916810335);
void live_H_14(double *state, double *unused, double *out_1505798650448814486);
void live_h_33(double *state, double *unused, double *out_3390743547420780895);
void live_H_33(double *state, double *unused, double *out_2122558859282571152);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}