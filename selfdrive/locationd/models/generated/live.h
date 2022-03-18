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
void live_H(double *in_vec, double *out_8958589685007469941);
void live_err_fun(double *nom_x, double *delta_x, double *out_3025361583143407068);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_4198850217462499107);
void live_H_mod_fun(double *state, double *out_8679930981745783122);
void live_f_fun(double *state, double dt, double *out_3683900094095511410);
void live_F_fun(double *state, double dt, double *out_5388026379385406588);
void live_h_4(double *state, double *unused, double *out_1715536465522411978);
void live_H_4(double *state, double *unused, double *out_3719156232158759943);
void live_h_9(double *state, double *unused, double *out_975100759360487818);
void live_H_9(double *state, double *unused, double *out_3568062703105687527);
void live_h_10(double *state, double *unused, double *out_2896629913052241188);
void live_H_10(double *state, double *unused, double *out_7733782407281295071);
void live_h_12(double *state, double *unused, double *out_1790768434786706157);
void live_H_12(double *state, double *unused, double *out_1300300175873201852);
void live_h_31(double *state, double *unused, double *out_5470712750729101726);
void live_H_31(double *state, double *unused, double *out_4045863208198215561);
void live_h_32(double *state, double *unused, double *out_7781329644986494312);
void live_H_32(double *state, double *unused, double *out_6750887721099965498);
void live_h_13(double *state, double *unused, double *out_8121777646425007865);
void live_H_13(double *state, double *unused, double *out_1674701136754016706);
void live_h_14(double *state, double *unused, double *out_975100759360487818);
void live_H_14(double *state, double *unused, double *out_3568062703105687527);
void live_h_33(double *state, double *unused, double *out_1822743366871707549);
void live_H_33(double *state, double *unused, double *out_7196420212837073165);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}