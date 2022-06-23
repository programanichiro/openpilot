#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_4899379868417862905);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_3858087077367475170);
void car_H_mod_fun(double *state, double *out_565708899702193861);
void car_f_fun(double *state, double dt, double *out_8210882771349018177);
void car_F_fun(double *state, double dt, double *out_3432224512469045389);
void car_h_25(double *state, double *unused, double *out_6485200509708027430);
void car_H_25(double *state, double *unused, double *out_7060147963260463716);
void car_h_24(double *state, double *unused, double *out_5991557838608407533);
void car_H_24(double *state, double *unused, double *out_7828670511646764764);
void car_h_30(double *state, double *unused, double *out_5190708337651173250);
void car_H_30(double *state, double *unused, double *out_8868263151941839273);
void car_h_26(double *state, double *unused, double *out_6881156172386854210);
void car_H_26(double *state, double *unused, double *out_8082070140688287299);
void car_h_27(double *state, double *unused, double *out_1483527242047181785);
void car_H_27(double *state, double *unused, double *out_6644669080757896056);
void car_h_29(double *state, double *unused, double *out_2153922286206840565);
void car_H_29(double *state, double *unused, double *out_8358031807627447089);
void car_h_28(double *state, double *unused, double *out_2427037094948844220);
void car_H_28(double *state, double *unused, double *out_5006313249012573953);
void car_h_31(double *state, double *unused, double *out_8324094035747501836);
void car_H_31(double *state, double *unused, double *out_2692436542153056016);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}