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
void live_H(double *in_vec, double *out_4561219149607775257);
void live_err_fun(double *nom_x, double *delta_x, double *out_5714388780541513559);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_2722425663997044208);
void live_H_mod_fun(double *state, double *out_7048599423514714999);
void live_f_fun(double *state, double dt, double *out_1817294066734651182);
void live_F_fun(double *state, double dt, double *out_4034785705381798254);
void live_h_4(double *state, double *unused, double *out_6581286902000104292);
void live_H_4(double *state, double *unused, double *out_8537832987797683486);
void live_h_9(double *state, double *unused, double *out_1016674407698141864);
void live_H_9(double *state, double *unused, double *out_8296643341168092841);
void live_h_10(double *state, double *unused, double *out_4115902024506385539);
void live_H_10(double *state, double *unused, double *out_6970232624066594959);
void live_h_12(double *state, double *unused, double *out_687751758443620076);
void live_H_12(double *state, double *unused, double *out_3518376579765721691);
void live_h_31(double *state, double *unused, double *out_1281509209662324245);
void live_H_31(double *state, double *unused, double *out_5171170930425076110);
void live_h_32(double *state, double *unused, double *out_4292634007224965488);
void live_H_32(double *state, double *unused, double *out_2010844616977709611);
void live_h_13(double *state, double *unused, double *out_5591226189252689606);
void live_H_13(double *state, double *unused, double *out_8965909042930030514);
void live_h_14(double *state, double *unused, double *out_1016674407698141864);
void live_H_14(double *state, double *unused, double *out_8296643341168092841);
void live_h_33(double *state, double *unused, double *out_8226086700540071770);
void live_H_33(double *state, double *unused, double *out_2020613925786218506);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}