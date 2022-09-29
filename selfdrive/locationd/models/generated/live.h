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
void live_H(double *in_vec, double *out_6155196363117996682);
void live_err_fun(double *nom_x, double *delta_x, double *out_6075871963894260130);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_4690067397995263188);
void live_H_mod_fun(double *state, double *out_7115582354504771251);
void live_f_fun(double *state, double dt, double *out_1332665565425515852);
void live_F_fun(double *state, double dt, double *out_968664180753864894);
void live_h_4(double *state, double *unused, double *out_5981084790993228995);
void live_H_4(double *state, double *unused, double *out_9085027438210769922);
void live_h_9(double *state, double *unused, double *out_7747596541618275538);
void live_H_9(double *state, double *unused, double *out_2074497700234334224);
void live_h_10(double *state, double *unused, double *out_2174078754877144842);
void live_H_10(double *state, double *unused, double *out_230320362877323902);
void live_h_12(double *state, double *unused, double *out_2233554327596514923);
void live_H_12(double *state, double *unused, double *out_1694588321816331202);
void live_h_31(double *state, double *unused, double *out_8840709976543803019);
void live_H_31(double *state, double *unused, double *out_1050974710508682507);
void live_h_32(double *state, double *unused, double *out_4661925462396763423);
void live_H_32(double *state, double *unused, double *out_6852279082716295065);
void live_h_13(double *state, double *unused, double *out_5639857450949673610);
void live_H_13(double *state, double *unused, double *out_8375523914364483466);
void live_h_14(double *state, double *unused, double *out_7747596541618275538);
void live_H_14(double *state, double *unused, double *out_2074497700234334224);
void live_h_33(double *state, double *unused, double *out_4654553732768129461);
void live_H_33(double *state, double *unused, double *out_4201531715147540111);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}