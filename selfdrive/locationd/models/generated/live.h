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
void live_H(double *in_vec, double *out_7025793885702551025);
void live_err_fun(double *nom_x, double *delta_x, double *out_8842764689132081094);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_5199305465308273602);
void live_H_mod_fun(double *state, double *out_3685767440720203406);
void live_f_fun(double *state, double dt, double *out_3828607459824926178);
void live_F_fun(double *state, double dt, double *out_5165582692164970601);
void live_h_4(double *state, double *unused, double *out_7417913370402788155);
void live_H_4(double *state, double *unused, double *out_1136689340074395490);
void live_h_9(double *state, double *unused, double *out_3542522809502864859);
void live_H_9(double *state, double *unused, double *out_3020478396280381993);
void live_h_10(double *state, double *unused, double *out_5170121947243021887);
void live_H_10(double *state, double *unused, double *out_733443939580960134);
void live_h_12(double *state, double *unused, double *out_2346182449451327159);
void live_H_12(double *state, double *unused, double *out_1757788365121989157);
void live_h_31(double *state, double *unused, double *out_9147156794790558350);
void live_H_31(double *state, double *unused, double *out_4503351397447002866);
void live_h_32(double *state, double *unused, double *out_2517444789799649268);
void live_H_32(double *state, double *unused, double *out_4113705223209274332);
void live_h_13(double *state, double *unused, double *out_8232464278499455166);
void live_H_13(double *state, double *unused, double *out_2378879281699706885);
void live_h_14(double *state, double *unused, double *out_3542522809502864859);
void live_H_14(double *state, double *unused, double *out_3020478396280381993);
void live_h_33(double *state, double *unused, double *out_8436989049089448161);
void live_H_33(double *state, double *unused, double *out_7653908402085860470);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}