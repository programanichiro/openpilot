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
void live_H(double *in_vec, double *out_664870032555571696);
void live_err_fun(double *nom_x, double *delta_x, double *out_515850225930482518);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_5678167244921416369);
void live_H_mod_fun(double *state, double *out_7147677672942268557);
void live_f_fun(double *state, double dt, double *out_792155603275649113);
void live_F_fun(double *state, double dt, double *out_4421713706834801602);
void live_h_4(double *state, double *unused, double *out_7627615084599271767);
void live_H_4(double *state, double *unused, double *out_660110156520424923);
void live_h_9(double *state, double *unused, double *out_1645483601582419251);
void live_H_9(double *state, double *unused, double *out_7947329091784872393);
void live_h_10(double *state, double *unused, double *out_5341847282855306581);
void live_H_10(double *state, double *unused, double *out_3248568946624199813);
void live_h_12(double *state, double *unused, double *out_3244330558587092924);
void live_H_12(double *state, double *unused, double *out_5679566564552386718);
void live_h_31(double *state, double *unused, double *out_4675634228744983158);
void live_H_31(double *state, double *unused, double *out_4026772213893032299);
void live_h_32(double *state, double *unused, double *out_249585325447370776);
void live_H_32(double *state, double *unused, double *out_2381932704182132697);
void live_h_13(double *state, double *unused, double *out_6925078624835144738);
void live_H_13(double *state, double *unused, double *out_7496803675139196446);
void live_h_14(double *state, double *unused, double *out_1645483601582419251);
void live_H_14(double *state, double *unused, double *out_7947329091784872393);
void live_h_33(double *state, double *unused, double *out_1678862661483315476);
void live_H_33(double *state, double *unused, double *out_7177329218531889903);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}