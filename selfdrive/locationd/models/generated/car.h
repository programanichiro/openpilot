#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_6563120477627388557);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_292638181712566491);
void car_H_mod_fun(double *state, double *out_2066885570274751950);
void car_f_fun(double *state, double dt, double *out_5067628721497108259);
void car_F_fun(double *state, double dt, double *out_4991524933764804051);
void car_h_25(double *state, double *unused, double *out_313793849787633926);
void car_H_25(double *state, double *unused, double *out_8302795210282871364);
void car_h_24(double *state, double *unused, double *out_5687283266067473238);
void car_H_24(double *state, double *unused, double *out_9147791819066566763);
void car_h_30(double *state, double *unused, double *out_5839851286338657039);
void car_H_30(double *state, double *unused, double *out_3227258521935063497);
void car_h_26(double *state, double *unused, double *out_2565805533305569614);
void car_H_26(double *state, double *unused, double *out_8959649274393183268);
void car_h_27(double *state, double *unused, double *out_6478449123844097853);
void car_H_27(double *state, double *unused, double *out_5402021833735488408);
void car_h_29(double *state, double *unused, double *out_6623968948064752824);
void car_H_29(double *state, double *unused, double *out_2717027177620671313);
void car_h_28(double *state, double *unused, double *out_6966621682204608118);
void car_H_28(double *state, double *unused, double *out_7799426194690201887);
void car_h_31(double *state, double *unused, double *out_2514581858309100362);
void car_H_31(double *state, double *unused, double *out_8333441172159831792);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}