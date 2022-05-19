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
void live_H(double *in_vec, double *out_7457422784797960521);
void live_err_fun(double *nom_x, double *delta_x, double *out_3470959261784518961);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_1797346124048533595);
void live_H_mod_fun(double *state, double *out_8553784297177108843);
void live_f_fun(double *state, double dt, double *out_8550223565152907424);
void live_F_fun(double *state, double dt, double *out_2687924198656063120);
void live_h_4(double *state, double *unused, double *out_7204517438101592670);
void live_H_4(double *state, double *unused, double *out_7325098029227874372);
void live_h_9(double *state, double *unused, double *out_9147253032187164228);
void live_H_9(double *state, double *unused, double *out_7566287675857465017);
void live_h_10(double *state, double *unused, double *out_5220684388615811881);
void live_H_10(double *state, double *unused, double *out_2284770634911041421);
void live_h_12(double *state, double *unused, double *out_1549710670992380127);
void live_H_12(double *state, double *unused, double *out_6102189636449715449);
void live_h_31(double *state, double *unused, double *out_4733520265523424407);
void live_H_31(double *state, double *unused, double *out_7754983987109069868);
void live_h_32(double *state, double *unused, double *out_1456985045138936089);
void live_H_32(double *state, double *unused, double *out_1953679614901457310);
void live_h_13(double *state, double *unused, double *out_1435016025948278781);
void live_H_13(double *state, double *unused, double *out_5192973418431082081);
void live_h_14(double *state, double *unused, double *out_9147253032187164228);
void live_H_14(double *state, double *unused, double *out_7566287675857465017);
void live_h_33(double *state, double *unused, double *out_1896070629801626530);
void live_H_33(double *state, double *unused, double *out_4604426982470212264);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}