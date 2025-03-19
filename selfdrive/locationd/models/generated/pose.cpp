#include "pose.h"

namespace {
#define DIM 18
#define EDIM 18
#define MEDIM 18
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_4 = 7.814727903251177;
const static double MAHA_THRESH_10 = 7.814727903251177;
const static double MAHA_THRESH_13 = 7.814727903251177;
const static double MAHA_THRESH_14 = 7.814727903251177;

/******************************************************************************
 *                      Code generated with SymPy 1.13.2                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_6279086065746722501) {
   out_6279086065746722501[0] = delta_x[0] + nom_x[0];
   out_6279086065746722501[1] = delta_x[1] + nom_x[1];
   out_6279086065746722501[2] = delta_x[2] + nom_x[2];
   out_6279086065746722501[3] = delta_x[3] + nom_x[3];
   out_6279086065746722501[4] = delta_x[4] + nom_x[4];
   out_6279086065746722501[5] = delta_x[5] + nom_x[5];
   out_6279086065746722501[6] = delta_x[6] + nom_x[6];
   out_6279086065746722501[7] = delta_x[7] + nom_x[7];
   out_6279086065746722501[8] = delta_x[8] + nom_x[8];
   out_6279086065746722501[9] = delta_x[9] + nom_x[9];
   out_6279086065746722501[10] = delta_x[10] + nom_x[10];
   out_6279086065746722501[11] = delta_x[11] + nom_x[11];
   out_6279086065746722501[12] = delta_x[12] + nom_x[12];
   out_6279086065746722501[13] = delta_x[13] + nom_x[13];
   out_6279086065746722501[14] = delta_x[14] + nom_x[14];
   out_6279086065746722501[15] = delta_x[15] + nom_x[15];
   out_6279086065746722501[16] = delta_x[16] + nom_x[16];
   out_6279086065746722501[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_2166314779381952832) {
   out_2166314779381952832[0] = -nom_x[0] + true_x[0];
   out_2166314779381952832[1] = -nom_x[1] + true_x[1];
   out_2166314779381952832[2] = -nom_x[2] + true_x[2];
   out_2166314779381952832[3] = -nom_x[3] + true_x[3];
   out_2166314779381952832[4] = -nom_x[4] + true_x[4];
   out_2166314779381952832[5] = -nom_x[5] + true_x[5];
   out_2166314779381952832[6] = -nom_x[6] + true_x[6];
   out_2166314779381952832[7] = -nom_x[7] + true_x[7];
   out_2166314779381952832[8] = -nom_x[8] + true_x[8];
   out_2166314779381952832[9] = -nom_x[9] + true_x[9];
   out_2166314779381952832[10] = -nom_x[10] + true_x[10];
   out_2166314779381952832[11] = -nom_x[11] + true_x[11];
   out_2166314779381952832[12] = -nom_x[12] + true_x[12];
   out_2166314779381952832[13] = -nom_x[13] + true_x[13];
   out_2166314779381952832[14] = -nom_x[14] + true_x[14];
   out_2166314779381952832[15] = -nom_x[15] + true_x[15];
   out_2166314779381952832[16] = -nom_x[16] + true_x[16];
   out_2166314779381952832[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_5648904405508167058) {
   out_5648904405508167058[0] = 1.0;
   out_5648904405508167058[1] = 0.0;
   out_5648904405508167058[2] = 0.0;
   out_5648904405508167058[3] = 0.0;
   out_5648904405508167058[4] = 0.0;
   out_5648904405508167058[5] = 0.0;
   out_5648904405508167058[6] = 0.0;
   out_5648904405508167058[7] = 0.0;
   out_5648904405508167058[8] = 0.0;
   out_5648904405508167058[9] = 0.0;
   out_5648904405508167058[10] = 0.0;
   out_5648904405508167058[11] = 0.0;
   out_5648904405508167058[12] = 0.0;
   out_5648904405508167058[13] = 0.0;
   out_5648904405508167058[14] = 0.0;
   out_5648904405508167058[15] = 0.0;
   out_5648904405508167058[16] = 0.0;
   out_5648904405508167058[17] = 0.0;
   out_5648904405508167058[18] = 0.0;
   out_5648904405508167058[19] = 1.0;
   out_5648904405508167058[20] = 0.0;
   out_5648904405508167058[21] = 0.0;
   out_5648904405508167058[22] = 0.0;
   out_5648904405508167058[23] = 0.0;
   out_5648904405508167058[24] = 0.0;
   out_5648904405508167058[25] = 0.0;
   out_5648904405508167058[26] = 0.0;
   out_5648904405508167058[27] = 0.0;
   out_5648904405508167058[28] = 0.0;
   out_5648904405508167058[29] = 0.0;
   out_5648904405508167058[30] = 0.0;
   out_5648904405508167058[31] = 0.0;
   out_5648904405508167058[32] = 0.0;
   out_5648904405508167058[33] = 0.0;
   out_5648904405508167058[34] = 0.0;
   out_5648904405508167058[35] = 0.0;
   out_5648904405508167058[36] = 0.0;
   out_5648904405508167058[37] = 0.0;
   out_5648904405508167058[38] = 1.0;
   out_5648904405508167058[39] = 0.0;
   out_5648904405508167058[40] = 0.0;
   out_5648904405508167058[41] = 0.0;
   out_5648904405508167058[42] = 0.0;
   out_5648904405508167058[43] = 0.0;
   out_5648904405508167058[44] = 0.0;
   out_5648904405508167058[45] = 0.0;
   out_5648904405508167058[46] = 0.0;
   out_5648904405508167058[47] = 0.0;
   out_5648904405508167058[48] = 0.0;
   out_5648904405508167058[49] = 0.0;
   out_5648904405508167058[50] = 0.0;
   out_5648904405508167058[51] = 0.0;
   out_5648904405508167058[52] = 0.0;
   out_5648904405508167058[53] = 0.0;
   out_5648904405508167058[54] = 0.0;
   out_5648904405508167058[55] = 0.0;
   out_5648904405508167058[56] = 0.0;
   out_5648904405508167058[57] = 1.0;
   out_5648904405508167058[58] = 0.0;
   out_5648904405508167058[59] = 0.0;
   out_5648904405508167058[60] = 0.0;
   out_5648904405508167058[61] = 0.0;
   out_5648904405508167058[62] = 0.0;
   out_5648904405508167058[63] = 0.0;
   out_5648904405508167058[64] = 0.0;
   out_5648904405508167058[65] = 0.0;
   out_5648904405508167058[66] = 0.0;
   out_5648904405508167058[67] = 0.0;
   out_5648904405508167058[68] = 0.0;
   out_5648904405508167058[69] = 0.0;
   out_5648904405508167058[70] = 0.0;
   out_5648904405508167058[71] = 0.0;
   out_5648904405508167058[72] = 0.0;
   out_5648904405508167058[73] = 0.0;
   out_5648904405508167058[74] = 0.0;
   out_5648904405508167058[75] = 0.0;
   out_5648904405508167058[76] = 1.0;
   out_5648904405508167058[77] = 0.0;
   out_5648904405508167058[78] = 0.0;
   out_5648904405508167058[79] = 0.0;
   out_5648904405508167058[80] = 0.0;
   out_5648904405508167058[81] = 0.0;
   out_5648904405508167058[82] = 0.0;
   out_5648904405508167058[83] = 0.0;
   out_5648904405508167058[84] = 0.0;
   out_5648904405508167058[85] = 0.0;
   out_5648904405508167058[86] = 0.0;
   out_5648904405508167058[87] = 0.0;
   out_5648904405508167058[88] = 0.0;
   out_5648904405508167058[89] = 0.0;
   out_5648904405508167058[90] = 0.0;
   out_5648904405508167058[91] = 0.0;
   out_5648904405508167058[92] = 0.0;
   out_5648904405508167058[93] = 0.0;
   out_5648904405508167058[94] = 0.0;
   out_5648904405508167058[95] = 1.0;
   out_5648904405508167058[96] = 0.0;
   out_5648904405508167058[97] = 0.0;
   out_5648904405508167058[98] = 0.0;
   out_5648904405508167058[99] = 0.0;
   out_5648904405508167058[100] = 0.0;
   out_5648904405508167058[101] = 0.0;
   out_5648904405508167058[102] = 0.0;
   out_5648904405508167058[103] = 0.0;
   out_5648904405508167058[104] = 0.0;
   out_5648904405508167058[105] = 0.0;
   out_5648904405508167058[106] = 0.0;
   out_5648904405508167058[107] = 0.0;
   out_5648904405508167058[108] = 0.0;
   out_5648904405508167058[109] = 0.0;
   out_5648904405508167058[110] = 0.0;
   out_5648904405508167058[111] = 0.0;
   out_5648904405508167058[112] = 0.0;
   out_5648904405508167058[113] = 0.0;
   out_5648904405508167058[114] = 1.0;
   out_5648904405508167058[115] = 0.0;
   out_5648904405508167058[116] = 0.0;
   out_5648904405508167058[117] = 0.0;
   out_5648904405508167058[118] = 0.0;
   out_5648904405508167058[119] = 0.0;
   out_5648904405508167058[120] = 0.0;
   out_5648904405508167058[121] = 0.0;
   out_5648904405508167058[122] = 0.0;
   out_5648904405508167058[123] = 0.0;
   out_5648904405508167058[124] = 0.0;
   out_5648904405508167058[125] = 0.0;
   out_5648904405508167058[126] = 0.0;
   out_5648904405508167058[127] = 0.0;
   out_5648904405508167058[128] = 0.0;
   out_5648904405508167058[129] = 0.0;
   out_5648904405508167058[130] = 0.0;
   out_5648904405508167058[131] = 0.0;
   out_5648904405508167058[132] = 0.0;
   out_5648904405508167058[133] = 1.0;
   out_5648904405508167058[134] = 0.0;
   out_5648904405508167058[135] = 0.0;
   out_5648904405508167058[136] = 0.0;
   out_5648904405508167058[137] = 0.0;
   out_5648904405508167058[138] = 0.0;
   out_5648904405508167058[139] = 0.0;
   out_5648904405508167058[140] = 0.0;
   out_5648904405508167058[141] = 0.0;
   out_5648904405508167058[142] = 0.0;
   out_5648904405508167058[143] = 0.0;
   out_5648904405508167058[144] = 0.0;
   out_5648904405508167058[145] = 0.0;
   out_5648904405508167058[146] = 0.0;
   out_5648904405508167058[147] = 0.0;
   out_5648904405508167058[148] = 0.0;
   out_5648904405508167058[149] = 0.0;
   out_5648904405508167058[150] = 0.0;
   out_5648904405508167058[151] = 0.0;
   out_5648904405508167058[152] = 1.0;
   out_5648904405508167058[153] = 0.0;
   out_5648904405508167058[154] = 0.0;
   out_5648904405508167058[155] = 0.0;
   out_5648904405508167058[156] = 0.0;
   out_5648904405508167058[157] = 0.0;
   out_5648904405508167058[158] = 0.0;
   out_5648904405508167058[159] = 0.0;
   out_5648904405508167058[160] = 0.0;
   out_5648904405508167058[161] = 0.0;
   out_5648904405508167058[162] = 0.0;
   out_5648904405508167058[163] = 0.0;
   out_5648904405508167058[164] = 0.0;
   out_5648904405508167058[165] = 0.0;
   out_5648904405508167058[166] = 0.0;
   out_5648904405508167058[167] = 0.0;
   out_5648904405508167058[168] = 0.0;
   out_5648904405508167058[169] = 0.0;
   out_5648904405508167058[170] = 0.0;
   out_5648904405508167058[171] = 1.0;
   out_5648904405508167058[172] = 0.0;
   out_5648904405508167058[173] = 0.0;
   out_5648904405508167058[174] = 0.0;
   out_5648904405508167058[175] = 0.0;
   out_5648904405508167058[176] = 0.0;
   out_5648904405508167058[177] = 0.0;
   out_5648904405508167058[178] = 0.0;
   out_5648904405508167058[179] = 0.0;
   out_5648904405508167058[180] = 0.0;
   out_5648904405508167058[181] = 0.0;
   out_5648904405508167058[182] = 0.0;
   out_5648904405508167058[183] = 0.0;
   out_5648904405508167058[184] = 0.0;
   out_5648904405508167058[185] = 0.0;
   out_5648904405508167058[186] = 0.0;
   out_5648904405508167058[187] = 0.0;
   out_5648904405508167058[188] = 0.0;
   out_5648904405508167058[189] = 0.0;
   out_5648904405508167058[190] = 1.0;
   out_5648904405508167058[191] = 0.0;
   out_5648904405508167058[192] = 0.0;
   out_5648904405508167058[193] = 0.0;
   out_5648904405508167058[194] = 0.0;
   out_5648904405508167058[195] = 0.0;
   out_5648904405508167058[196] = 0.0;
   out_5648904405508167058[197] = 0.0;
   out_5648904405508167058[198] = 0.0;
   out_5648904405508167058[199] = 0.0;
   out_5648904405508167058[200] = 0.0;
   out_5648904405508167058[201] = 0.0;
   out_5648904405508167058[202] = 0.0;
   out_5648904405508167058[203] = 0.0;
   out_5648904405508167058[204] = 0.0;
   out_5648904405508167058[205] = 0.0;
   out_5648904405508167058[206] = 0.0;
   out_5648904405508167058[207] = 0.0;
   out_5648904405508167058[208] = 0.0;
   out_5648904405508167058[209] = 1.0;
   out_5648904405508167058[210] = 0.0;
   out_5648904405508167058[211] = 0.0;
   out_5648904405508167058[212] = 0.0;
   out_5648904405508167058[213] = 0.0;
   out_5648904405508167058[214] = 0.0;
   out_5648904405508167058[215] = 0.0;
   out_5648904405508167058[216] = 0.0;
   out_5648904405508167058[217] = 0.0;
   out_5648904405508167058[218] = 0.0;
   out_5648904405508167058[219] = 0.0;
   out_5648904405508167058[220] = 0.0;
   out_5648904405508167058[221] = 0.0;
   out_5648904405508167058[222] = 0.0;
   out_5648904405508167058[223] = 0.0;
   out_5648904405508167058[224] = 0.0;
   out_5648904405508167058[225] = 0.0;
   out_5648904405508167058[226] = 0.0;
   out_5648904405508167058[227] = 0.0;
   out_5648904405508167058[228] = 1.0;
   out_5648904405508167058[229] = 0.0;
   out_5648904405508167058[230] = 0.0;
   out_5648904405508167058[231] = 0.0;
   out_5648904405508167058[232] = 0.0;
   out_5648904405508167058[233] = 0.0;
   out_5648904405508167058[234] = 0.0;
   out_5648904405508167058[235] = 0.0;
   out_5648904405508167058[236] = 0.0;
   out_5648904405508167058[237] = 0.0;
   out_5648904405508167058[238] = 0.0;
   out_5648904405508167058[239] = 0.0;
   out_5648904405508167058[240] = 0.0;
   out_5648904405508167058[241] = 0.0;
   out_5648904405508167058[242] = 0.0;
   out_5648904405508167058[243] = 0.0;
   out_5648904405508167058[244] = 0.0;
   out_5648904405508167058[245] = 0.0;
   out_5648904405508167058[246] = 0.0;
   out_5648904405508167058[247] = 1.0;
   out_5648904405508167058[248] = 0.0;
   out_5648904405508167058[249] = 0.0;
   out_5648904405508167058[250] = 0.0;
   out_5648904405508167058[251] = 0.0;
   out_5648904405508167058[252] = 0.0;
   out_5648904405508167058[253] = 0.0;
   out_5648904405508167058[254] = 0.0;
   out_5648904405508167058[255] = 0.0;
   out_5648904405508167058[256] = 0.0;
   out_5648904405508167058[257] = 0.0;
   out_5648904405508167058[258] = 0.0;
   out_5648904405508167058[259] = 0.0;
   out_5648904405508167058[260] = 0.0;
   out_5648904405508167058[261] = 0.0;
   out_5648904405508167058[262] = 0.0;
   out_5648904405508167058[263] = 0.0;
   out_5648904405508167058[264] = 0.0;
   out_5648904405508167058[265] = 0.0;
   out_5648904405508167058[266] = 1.0;
   out_5648904405508167058[267] = 0.0;
   out_5648904405508167058[268] = 0.0;
   out_5648904405508167058[269] = 0.0;
   out_5648904405508167058[270] = 0.0;
   out_5648904405508167058[271] = 0.0;
   out_5648904405508167058[272] = 0.0;
   out_5648904405508167058[273] = 0.0;
   out_5648904405508167058[274] = 0.0;
   out_5648904405508167058[275] = 0.0;
   out_5648904405508167058[276] = 0.0;
   out_5648904405508167058[277] = 0.0;
   out_5648904405508167058[278] = 0.0;
   out_5648904405508167058[279] = 0.0;
   out_5648904405508167058[280] = 0.0;
   out_5648904405508167058[281] = 0.0;
   out_5648904405508167058[282] = 0.0;
   out_5648904405508167058[283] = 0.0;
   out_5648904405508167058[284] = 0.0;
   out_5648904405508167058[285] = 1.0;
   out_5648904405508167058[286] = 0.0;
   out_5648904405508167058[287] = 0.0;
   out_5648904405508167058[288] = 0.0;
   out_5648904405508167058[289] = 0.0;
   out_5648904405508167058[290] = 0.0;
   out_5648904405508167058[291] = 0.0;
   out_5648904405508167058[292] = 0.0;
   out_5648904405508167058[293] = 0.0;
   out_5648904405508167058[294] = 0.0;
   out_5648904405508167058[295] = 0.0;
   out_5648904405508167058[296] = 0.0;
   out_5648904405508167058[297] = 0.0;
   out_5648904405508167058[298] = 0.0;
   out_5648904405508167058[299] = 0.0;
   out_5648904405508167058[300] = 0.0;
   out_5648904405508167058[301] = 0.0;
   out_5648904405508167058[302] = 0.0;
   out_5648904405508167058[303] = 0.0;
   out_5648904405508167058[304] = 1.0;
   out_5648904405508167058[305] = 0.0;
   out_5648904405508167058[306] = 0.0;
   out_5648904405508167058[307] = 0.0;
   out_5648904405508167058[308] = 0.0;
   out_5648904405508167058[309] = 0.0;
   out_5648904405508167058[310] = 0.0;
   out_5648904405508167058[311] = 0.0;
   out_5648904405508167058[312] = 0.0;
   out_5648904405508167058[313] = 0.0;
   out_5648904405508167058[314] = 0.0;
   out_5648904405508167058[315] = 0.0;
   out_5648904405508167058[316] = 0.0;
   out_5648904405508167058[317] = 0.0;
   out_5648904405508167058[318] = 0.0;
   out_5648904405508167058[319] = 0.0;
   out_5648904405508167058[320] = 0.0;
   out_5648904405508167058[321] = 0.0;
   out_5648904405508167058[322] = 0.0;
   out_5648904405508167058[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_5535553793790539232) {
   out_5535553793790539232[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_5535553793790539232[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_5535553793790539232[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_5535553793790539232[3] = dt*state[12] + state[3];
   out_5535553793790539232[4] = dt*state[13] + state[4];
   out_5535553793790539232[5] = dt*state[14] + state[5];
   out_5535553793790539232[6] = state[6];
   out_5535553793790539232[7] = state[7];
   out_5535553793790539232[8] = state[8];
   out_5535553793790539232[9] = state[9];
   out_5535553793790539232[10] = state[10];
   out_5535553793790539232[11] = state[11];
   out_5535553793790539232[12] = state[12];
   out_5535553793790539232[13] = state[13];
   out_5535553793790539232[14] = state[14];
   out_5535553793790539232[15] = state[15];
   out_5535553793790539232[16] = state[16];
   out_5535553793790539232[17] = state[17];
}
void F_fun(double *state, double dt, double *out_3418835783146762417) {
   out_3418835783146762417[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3418835783146762417[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3418835783146762417[2] = 0;
   out_3418835783146762417[3] = 0;
   out_3418835783146762417[4] = 0;
   out_3418835783146762417[5] = 0;
   out_3418835783146762417[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3418835783146762417[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3418835783146762417[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3418835783146762417[9] = 0;
   out_3418835783146762417[10] = 0;
   out_3418835783146762417[11] = 0;
   out_3418835783146762417[12] = 0;
   out_3418835783146762417[13] = 0;
   out_3418835783146762417[14] = 0;
   out_3418835783146762417[15] = 0;
   out_3418835783146762417[16] = 0;
   out_3418835783146762417[17] = 0;
   out_3418835783146762417[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_3418835783146762417[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_3418835783146762417[20] = 0;
   out_3418835783146762417[21] = 0;
   out_3418835783146762417[22] = 0;
   out_3418835783146762417[23] = 0;
   out_3418835783146762417[24] = 0;
   out_3418835783146762417[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_3418835783146762417[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_3418835783146762417[27] = 0;
   out_3418835783146762417[28] = 0;
   out_3418835783146762417[29] = 0;
   out_3418835783146762417[30] = 0;
   out_3418835783146762417[31] = 0;
   out_3418835783146762417[32] = 0;
   out_3418835783146762417[33] = 0;
   out_3418835783146762417[34] = 0;
   out_3418835783146762417[35] = 0;
   out_3418835783146762417[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3418835783146762417[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3418835783146762417[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3418835783146762417[39] = 0;
   out_3418835783146762417[40] = 0;
   out_3418835783146762417[41] = 0;
   out_3418835783146762417[42] = 0;
   out_3418835783146762417[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3418835783146762417[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3418835783146762417[45] = 0;
   out_3418835783146762417[46] = 0;
   out_3418835783146762417[47] = 0;
   out_3418835783146762417[48] = 0;
   out_3418835783146762417[49] = 0;
   out_3418835783146762417[50] = 0;
   out_3418835783146762417[51] = 0;
   out_3418835783146762417[52] = 0;
   out_3418835783146762417[53] = 0;
   out_3418835783146762417[54] = 0;
   out_3418835783146762417[55] = 0;
   out_3418835783146762417[56] = 0;
   out_3418835783146762417[57] = 1;
   out_3418835783146762417[58] = 0;
   out_3418835783146762417[59] = 0;
   out_3418835783146762417[60] = 0;
   out_3418835783146762417[61] = 0;
   out_3418835783146762417[62] = 0;
   out_3418835783146762417[63] = 0;
   out_3418835783146762417[64] = 0;
   out_3418835783146762417[65] = 0;
   out_3418835783146762417[66] = dt;
   out_3418835783146762417[67] = 0;
   out_3418835783146762417[68] = 0;
   out_3418835783146762417[69] = 0;
   out_3418835783146762417[70] = 0;
   out_3418835783146762417[71] = 0;
   out_3418835783146762417[72] = 0;
   out_3418835783146762417[73] = 0;
   out_3418835783146762417[74] = 0;
   out_3418835783146762417[75] = 0;
   out_3418835783146762417[76] = 1;
   out_3418835783146762417[77] = 0;
   out_3418835783146762417[78] = 0;
   out_3418835783146762417[79] = 0;
   out_3418835783146762417[80] = 0;
   out_3418835783146762417[81] = 0;
   out_3418835783146762417[82] = 0;
   out_3418835783146762417[83] = 0;
   out_3418835783146762417[84] = 0;
   out_3418835783146762417[85] = dt;
   out_3418835783146762417[86] = 0;
   out_3418835783146762417[87] = 0;
   out_3418835783146762417[88] = 0;
   out_3418835783146762417[89] = 0;
   out_3418835783146762417[90] = 0;
   out_3418835783146762417[91] = 0;
   out_3418835783146762417[92] = 0;
   out_3418835783146762417[93] = 0;
   out_3418835783146762417[94] = 0;
   out_3418835783146762417[95] = 1;
   out_3418835783146762417[96] = 0;
   out_3418835783146762417[97] = 0;
   out_3418835783146762417[98] = 0;
   out_3418835783146762417[99] = 0;
   out_3418835783146762417[100] = 0;
   out_3418835783146762417[101] = 0;
   out_3418835783146762417[102] = 0;
   out_3418835783146762417[103] = 0;
   out_3418835783146762417[104] = dt;
   out_3418835783146762417[105] = 0;
   out_3418835783146762417[106] = 0;
   out_3418835783146762417[107] = 0;
   out_3418835783146762417[108] = 0;
   out_3418835783146762417[109] = 0;
   out_3418835783146762417[110] = 0;
   out_3418835783146762417[111] = 0;
   out_3418835783146762417[112] = 0;
   out_3418835783146762417[113] = 0;
   out_3418835783146762417[114] = 1;
   out_3418835783146762417[115] = 0;
   out_3418835783146762417[116] = 0;
   out_3418835783146762417[117] = 0;
   out_3418835783146762417[118] = 0;
   out_3418835783146762417[119] = 0;
   out_3418835783146762417[120] = 0;
   out_3418835783146762417[121] = 0;
   out_3418835783146762417[122] = 0;
   out_3418835783146762417[123] = 0;
   out_3418835783146762417[124] = 0;
   out_3418835783146762417[125] = 0;
   out_3418835783146762417[126] = 0;
   out_3418835783146762417[127] = 0;
   out_3418835783146762417[128] = 0;
   out_3418835783146762417[129] = 0;
   out_3418835783146762417[130] = 0;
   out_3418835783146762417[131] = 0;
   out_3418835783146762417[132] = 0;
   out_3418835783146762417[133] = 1;
   out_3418835783146762417[134] = 0;
   out_3418835783146762417[135] = 0;
   out_3418835783146762417[136] = 0;
   out_3418835783146762417[137] = 0;
   out_3418835783146762417[138] = 0;
   out_3418835783146762417[139] = 0;
   out_3418835783146762417[140] = 0;
   out_3418835783146762417[141] = 0;
   out_3418835783146762417[142] = 0;
   out_3418835783146762417[143] = 0;
   out_3418835783146762417[144] = 0;
   out_3418835783146762417[145] = 0;
   out_3418835783146762417[146] = 0;
   out_3418835783146762417[147] = 0;
   out_3418835783146762417[148] = 0;
   out_3418835783146762417[149] = 0;
   out_3418835783146762417[150] = 0;
   out_3418835783146762417[151] = 0;
   out_3418835783146762417[152] = 1;
   out_3418835783146762417[153] = 0;
   out_3418835783146762417[154] = 0;
   out_3418835783146762417[155] = 0;
   out_3418835783146762417[156] = 0;
   out_3418835783146762417[157] = 0;
   out_3418835783146762417[158] = 0;
   out_3418835783146762417[159] = 0;
   out_3418835783146762417[160] = 0;
   out_3418835783146762417[161] = 0;
   out_3418835783146762417[162] = 0;
   out_3418835783146762417[163] = 0;
   out_3418835783146762417[164] = 0;
   out_3418835783146762417[165] = 0;
   out_3418835783146762417[166] = 0;
   out_3418835783146762417[167] = 0;
   out_3418835783146762417[168] = 0;
   out_3418835783146762417[169] = 0;
   out_3418835783146762417[170] = 0;
   out_3418835783146762417[171] = 1;
   out_3418835783146762417[172] = 0;
   out_3418835783146762417[173] = 0;
   out_3418835783146762417[174] = 0;
   out_3418835783146762417[175] = 0;
   out_3418835783146762417[176] = 0;
   out_3418835783146762417[177] = 0;
   out_3418835783146762417[178] = 0;
   out_3418835783146762417[179] = 0;
   out_3418835783146762417[180] = 0;
   out_3418835783146762417[181] = 0;
   out_3418835783146762417[182] = 0;
   out_3418835783146762417[183] = 0;
   out_3418835783146762417[184] = 0;
   out_3418835783146762417[185] = 0;
   out_3418835783146762417[186] = 0;
   out_3418835783146762417[187] = 0;
   out_3418835783146762417[188] = 0;
   out_3418835783146762417[189] = 0;
   out_3418835783146762417[190] = 1;
   out_3418835783146762417[191] = 0;
   out_3418835783146762417[192] = 0;
   out_3418835783146762417[193] = 0;
   out_3418835783146762417[194] = 0;
   out_3418835783146762417[195] = 0;
   out_3418835783146762417[196] = 0;
   out_3418835783146762417[197] = 0;
   out_3418835783146762417[198] = 0;
   out_3418835783146762417[199] = 0;
   out_3418835783146762417[200] = 0;
   out_3418835783146762417[201] = 0;
   out_3418835783146762417[202] = 0;
   out_3418835783146762417[203] = 0;
   out_3418835783146762417[204] = 0;
   out_3418835783146762417[205] = 0;
   out_3418835783146762417[206] = 0;
   out_3418835783146762417[207] = 0;
   out_3418835783146762417[208] = 0;
   out_3418835783146762417[209] = 1;
   out_3418835783146762417[210] = 0;
   out_3418835783146762417[211] = 0;
   out_3418835783146762417[212] = 0;
   out_3418835783146762417[213] = 0;
   out_3418835783146762417[214] = 0;
   out_3418835783146762417[215] = 0;
   out_3418835783146762417[216] = 0;
   out_3418835783146762417[217] = 0;
   out_3418835783146762417[218] = 0;
   out_3418835783146762417[219] = 0;
   out_3418835783146762417[220] = 0;
   out_3418835783146762417[221] = 0;
   out_3418835783146762417[222] = 0;
   out_3418835783146762417[223] = 0;
   out_3418835783146762417[224] = 0;
   out_3418835783146762417[225] = 0;
   out_3418835783146762417[226] = 0;
   out_3418835783146762417[227] = 0;
   out_3418835783146762417[228] = 1;
   out_3418835783146762417[229] = 0;
   out_3418835783146762417[230] = 0;
   out_3418835783146762417[231] = 0;
   out_3418835783146762417[232] = 0;
   out_3418835783146762417[233] = 0;
   out_3418835783146762417[234] = 0;
   out_3418835783146762417[235] = 0;
   out_3418835783146762417[236] = 0;
   out_3418835783146762417[237] = 0;
   out_3418835783146762417[238] = 0;
   out_3418835783146762417[239] = 0;
   out_3418835783146762417[240] = 0;
   out_3418835783146762417[241] = 0;
   out_3418835783146762417[242] = 0;
   out_3418835783146762417[243] = 0;
   out_3418835783146762417[244] = 0;
   out_3418835783146762417[245] = 0;
   out_3418835783146762417[246] = 0;
   out_3418835783146762417[247] = 1;
   out_3418835783146762417[248] = 0;
   out_3418835783146762417[249] = 0;
   out_3418835783146762417[250] = 0;
   out_3418835783146762417[251] = 0;
   out_3418835783146762417[252] = 0;
   out_3418835783146762417[253] = 0;
   out_3418835783146762417[254] = 0;
   out_3418835783146762417[255] = 0;
   out_3418835783146762417[256] = 0;
   out_3418835783146762417[257] = 0;
   out_3418835783146762417[258] = 0;
   out_3418835783146762417[259] = 0;
   out_3418835783146762417[260] = 0;
   out_3418835783146762417[261] = 0;
   out_3418835783146762417[262] = 0;
   out_3418835783146762417[263] = 0;
   out_3418835783146762417[264] = 0;
   out_3418835783146762417[265] = 0;
   out_3418835783146762417[266] = 1;
   out_3418835783146762417[267] = 0;
   out_3418835783146762417[268] = 0;
   out_3418835783146762417[269] = 0;
   out_3418835783146762417[270] = 0;
   out_3418835783146762417[271] = 0;
   out_3418835783146762417[272] = 0;
   out_3418835783146762417[273] = 0;
   out_3418835783146762417[274] = 0;
   out_3418835783146762417[275] = 0;
   out_3418835783146762417[276] = 0;
   out_3418835783146762417[277] = 0;
   out_3418835783146762417[278] = 0;
   out_3418835783146762417[279] = 0;
   out_3418835783146762417[280] = 0;
   out_3418835783146762417[281] = 0;
   out_3418835783146762417[282] = 0;
   out_3418835783146762417[283] = 0;
   out_3418835783146762417[284] = 0;
   out_3418835783146762417[285] = 1;
   out_3418835783146762417[286] = 0;
   out_3418835783146762417[287] = 0;
   out_3418835783146762417[288] = 0;
   out_3418835783146762417[289] = 0;
   out_3418835783146762417[290] = 0;
   out_3418835783146762417[291] = 0;
   out_3418835783146762417[292] = 0;
   out_3418835783146762417[293] = 0;
   out_3418835783146762417[294] = 0;
   out_3418835783146762417[295] = 0;
   out_3418835783146762417[296] = 0;
   out_3418835783146762417[297] = 0;
   out_3418835783146762417[298] = 0;
   out_3418835783146762417[299] = 0;
   out_3418835783146762417[300] = 0;
   out_3418835783146762417[301] = 0;
   out_3418835783146762417[302] = 0;
   out_3418835783146762417[303] = 0;
   out_3418835783146762417[304] = 1;
   out_3418835783146762417[305] = 0;
   out_3418835783146762417[306] = 0;
   out_3418835783146762417[307] = 0;
   out_3418835783146762417[308] = 0;
   out_3418835783146762417[309] = 0;
   out_3418835783146762417[310] = 0;
   out_3418835783146762417[311] = 0;
   out_3418835783146762417[312] = 0;
   out_3418835783146762417[313] = 0;
   out_3418835783146762417[314] = 0;
   out_3418835783146762417[315] = 0;
   out_3418835783146762417[316] = 0;
   out_3418835783146762417[317] = 0;
   out_3418835783146762417[318] = 0;
   out_3418835783146762417[319] = 0;
   out_3418835783146762417[320] = 0;
   out_3418835783146762417[321] = 0;
   out_3418835783146762417[322] = 0;
   out_3418835783146762417[323] = 1;
}
void h_4(double *state, double *unused, double *out_8681835026761677731) {
   out_8681835026761677731[0] = state[6] + state[9];
   out_8681835026761677731[1] = state[7] + state[10];
   out_8681835026761677731[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_3295457260549654284) {
   out_3295457260549654284[0] = 0;
   out_3295457260549654284[1] = 0;
   out_3295457260549654284[2] = 0;
   out_3295457260549654284[3] = 0;
   out_3295457260549654284[4] = 0;
   out_3295457260549654284[5] = 0;
   out_3295457260549654284[6] = 1;
   out_3295457260549654284[7] = 0;
   out_3295457260549654284[8] = 0;
   out_3295457260549654284[9] = 1;
   out_3295457260549654284[10] = 0;
   out_3295457260549654284[11] = 0;
   out_3295457260549654284[12] = 0;
   out_3295457260549654284[13] = 0;
   out_3295457260549654284[14] = 0;
   out_3295457260549654284[15] = 0;
   out_3295457260549654284[16] = 0;
   out_3295457260549654284[17] = 0;
   out_3295457260549654284[18] = 0;
   out_3295457260549654284[19] = 0;
   out_3295457260549654284[20] = 0;
   out_3295457260549654284[21] = 0;
   out_3295457260549654284[22] = 0;
   out_3295457260549654284[23] = 0;
   out_3295457260549654284[24] = 0;
   out_3295457260549654284[25] = 1;
   out_3295457260549654284[26] = 0;
   out_3295457260549654284[27] = 0;
   out_3295457260549654284[28] = 1;
   out_3295457260549654284[29] = 0;
   out_3295457260549654284[30] = 0;
   out_3295457260549654284[31] = 0;
   out_3295457260549654284[32] = 0;
   out_3295457260549654284[33] = 0;
   out_3295457260549654284[34] = 0;
   out_3295457260549654284[35] = 0;
   out_3295457260549654284[36] = 0;
   out_3295457260549654284[37] = 0;
   out_3295457260549654284[38] = 0;
   out_3295457260549654284[39] = 0;
   out_3295457260549654284[40] = 0;
   out_3295457260549654284[41] = 0;
   out_3295457260549654284[42] = 0;
   out_3295457260549654284[43] = 0;
   out_3295457260549654284[44] = 1;
   out_3295457260549654284[45] = 0;
   out_3295457260549654284[46] = 0;
   out_3295457260549654284[47] = 1;
   out_3295457260549654284[48] = 0;
   out_3295457260549654284[49] = 0;
   out_3295457260549654284[50] = 0;
   out_3295457260549654284[51] = 0;
   out_3295457260549654284[52] = 0;
   out_3295457260549654284[53] = 0;
}
void h_10(double *state, double *unused, double *out_5359658747254490916) {
   out_5359658747254490916[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_5359658747254490916[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_5359658747254490916[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_4562305178044036433) {
   out_4562305178044036433[0] = 0;
   out_4562305178044036433[1] = 9.8100000000000005*cos(state[1]);
   out_4562305178044036433[2] = 0;
   out_4562305178044036433[3] = 0;
   out_4562305178044036433[4] = -state[8];
   out_4562305178044036433[5] = state[7];
   out_4562305178044036433[6] = 0;
   out_4562305178044036433[7] = state[5];
   out_4562305178044036433[8] = -state[4];
   out_4562305178044036433[9] = 0;
   out_4562305178044036433[10] = 0;
   out_4562305178044036433[11] = 0;
   out_4562305178044036433[12] = 1;
   out_4562305178044036433[13] = 0;
   out_4562305178044036433[14] = 0;
   out_4562305178044036433[15] = 1;
   out_4562305178044036433[16] = 0;
   out_4562305178044036433[17] = 0;
   out_4562305178044036433[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_4562305178044036433[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_4562305178044036433[20] = 0;
   out_4562305178044036433[21] = state[8];
   out_4562305178044036433[22] = 0;
   out_4562305178044036433[23] = -state[6];
   out_4562305178044036433[24] = -state[5];
   out_4562305178044036433[25] = 0;
   out_4562305178044036433[26] = state[3];
   out_4562305178044036433[27] = 0;
   out_4562305178044036433[28] = 0;
   out_4562305178044036433[29] = 0;
   out_4562305178044036433[30] = 0;
   out_4562305178044036433[31] = 1;
   out_4562305178044036433[32] = 0;
   out_4562305178044036433[33] = 0;
   out_4562305178044036433[34] = 1;
   out_4562305178044036433[35] = 0;
   out_4562305178044036433[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_4562305178044036433[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_4562305178044036433[38] = 0;
   out_4562305178044036433[39] = -state[7];
   out_4562305178044036433[40] = state[6];
   out_4562305178044036433[41] = 0;
   out_4562305178044036433[42] = state[4];
   out_4562305178044036433[43] = -state[3];
   out_4562305178044036433[44] = 0;
   out_4562305178044036433[45] = 0;
   out_4562305178044036433[46] = 0;
   out_4562305178044036433[47] = 0;
   out_4562305178044036433[48] = 0;
   out_4562305178044036433[49] = 0;
   out_4562305178044036433[50] = 1;
   out_4562305178044036433[51] = 0;
   out_4562305178044036433[52] = 0;
   out_4562305178044036433[53] = 1;
}
void h_13(double *state, double *unused, double *out_2317300498181279990) {
   out_2317300498181279990[0] = state[3];
   out_2317300498181279990[1] = state[4];
   out_2317300498181279990[2] = state[5];
}
void H_13(double *state, double *unused, double *out_83183435217321483) {
   out_83183435217321483[0] = 0;
   out_83183435217321483[1] = 0;
   out_83183435217321483[2] = 0;
   out_83183435217321483[3] = 1;
   out_83183435217321483[4] = 0;
   out_83183435217321483[5] = 0;
   out_83183435217321483[6] = 0;
   out_83183435217321483[7] = 0;
   out_83183435217321483[8] = 0;
   out_83183435217321483[9] = 0;
   out_83183435217321483[10] = 0;
   out_83183435217321483[11] = 0;
   out_83183435217321483[12] = 0;
   out_83183435217321483[13] = 0;
   out_83183435217321483[14] = 0;
   out_83183435217321483[15] = 0;
   out_83183435217321483[16] = 0;
   out_83183435217321483[17] = 0;
   out_83183435217321483[18] = 0;
   out_83183435217321483[19] = 0;
   out_83183435217321483[20] = 0;
   out_83183435217321483[21] = 0;
   out_83183435217321483[22] = 1;
   out_83183435217321483[23] = 0;
   out_83183435217321483[24] = 0;
   out_83183435217321483[25] = 0;
   out_83183435217321483[26] = 0;
   out_83183435217321483[27] = 0;
   out_83183435217321483[28] = 0;
   out_83183435217321483[29] = 0;
   out_83183435217321483[30] = 0;
   out_83183435217321483[31] = 0;
   out_83183435217321483[32] = 0;
   out_83183435217321483[33] = 0;
   out_83183435217321483[34] = 0;
   out_83183435217321483[35] = 0;
   out_83183435217321483[36] = 0;
   out_83183435217321483[37] = 0;
   out_83183435217321483[38] = 0;
   out_83183435217321483[39] = 0;
   out_83183435217321483[40] = 0;
   out_83183435217321483[41] = 1;
   out_83183435217321483[42] = 0;
   out_83183435217321483[43] = 0;
   out_83183435217321483[44] = 0;
   out_83183435217321483[45] = 0;
   out_83183435217321483[46] = 0;
   out_83183435217321483[47] = 0;
   out_83183435217321483[48] = 0;
   out_83183435217321483[49] = 0;
   out_83183435217321483[50] = 0;
   out_83183435217321483[51] = 0;
   out_83183435217321483[52] = 0;
   out_83183435217321483[53] = 0;
}
void h_14(double *state, double *unused, double *out_7153354114899869002) {
   out_7153354114899869002[0] = state[6];
   out_7153354114899869002[1] = state[7];
   out_7153354114899869002[2] = state[8];
}
void H_14(double *state, double *unused, double *out_667783595789830245) {
   out_667783595789830245[0] = 0;
   out_667783595789830245[1] = 0;
   out_667783595789830245[2] = 0;
   out_667783595789830245[3] = 0;
   out_667783595789830245[4] = 0;
   out_667783595789830245[5] = 0;
   out_667783595789830245[6] = 1;
   out_667783595789830245[7] = 0;
   out_667783595789830245[8] = 0;
   out_667783595789830245[9] = 0;
   out_667783595789830245[10] = 0;
   out_667783595789830245[11] = 0;
   out_667783595789830245[12] = 0;
   out_667783595789830245[13] = 0;
   out_667783595789830245[14] = 0;
   out_667783595789830245[15] = 0;
   out_667783595789830245[16] = 0;
   out_667783595789830245[17] = 0;
   out_667783595789830245[18] = 0;
   out_667783595789830245[19] = 0;
   out_667783595789830245[20] = 0;
   out_667783595789830245[21] = 0;
   out_667783595789830245[22] = 0;
   out_667783595789830245[23] = 0;
   out_667783595789830245[24] = 0;
   out_667783595789830245[25] = 1;
   out_667783595789830245[26] = 0;
   out_667783595789830245[27] = 0;
   out_667783595789830245[28] = 0;
   out_667783595789830245[29] = 0;
   out_667783595789830245[30] = 0;
   out_667783595789830245[31] = 0;
   out_667783595789830245[32] = 0;
   out_667783595789830245[33] = 0;
   out_667783595789830245[34] = 0;
   out_667783595789830245[35] = 0;
   out_667783595789830245[36] = 0;
   out_667783595789830245[37] = 0;
   out_667783595789830245[38] = 0;
   out_667783595789830245[39] = 0;
   out_667783595789830245[40] = 0;
   out_667783595789830245[41] = 0;
   out_667783595789830245[42] = 0;
   out_667783595789830245[43] = 0;
   out_667783595789830245[44] = 1;
   out_667783595789830245[45] = 0;
   out_667783595789830245[46] = 0;
   out_667783595789830245[47] = 0;
   out_667783595789830245[48] = 0;
   out_667783595789830245[49] = 0;
   out_667783595789830245[50] = 0;
   out_667783595789830245[51] = 0;
   out_667783595789830245[52] = 0;
   out_667783595789830245[53] = 0;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_4, H_4, NULL, in_z, in_R, in_ea, MAHA_THRESH_4);
}
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_10, H_10, NULL, in_z, in_R, in_ea, MAHA_THRESH_10);
}
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_13, H_13, NULL, in_z, in_R, in_ea, MAHA_THRESH_13);
}
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_14, H_14, NULL, in_z, in_R, in_ea, MAHA_THRESH_14);
}
void pose_err_fun(double *nom_x, double *delta_x, double *out_6279086065746722501) {
  err_fun(nom_x, delta_x, out_6279086065746722501);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_2166314779381952832) {
  inv_err_fun(nom_x, true_x, out_2166314779381952832);
}
void pose_H_mod_fun(double *state, double *out_5648904405508167058) {
  H_mod_fun(state, out_5648904405508167058);
}
void pose_f_fun(double *state, double dt, double *out_5535553793790539232) {
  f_fun(state,  dt, out_5535553793790539232);
}
void pose_F_fun(double *state, double dt, double *out_3418835783146762417) {
  F_fun(state,  dt, out_3418835783146762417);
}
void pose_h_4(double *state, double *unused, double *out_8681835026761677731) {
  h_4(state, unused, out_8681835026761677731);
}
void pose_H_4(double *state, double *unused, double *out_3295457260549654284) {
  H_4(state, unused, out_3295457260549654284);
}
void pose_h_10(double *state, double *unused, double *out_5359658747254490916) {
  h_10(state, unused, out_5359658747254490916);
}
void pose_H_10(double *state, double *unused, double *out_4562305178044036433) {
  H_10(state, unused, out_4562305178044036433);
}
void pose_h_13(double *state, double *unused, double *out_2317300498181279990) {
  h_13(state, unused, out_2317300498181279990);
}
void pose_H_13(double *state, double *unused, double *out_83183435217321483) {
  H_13(state, unused, out_83183435217321483);
}
void pose_h_14(double *state, double *unused, double *out_7153354114899869002) {
  h_14(state, unused, out_7153354114899869002);
}
void pose_H_14(double *state, double *unused, double *out_667783595789830245) {
  H_14(state, unused, out_667783595789830245);
}
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF pose = {
  .name = "pose",
  .kinds = { 4, 10, 13, 14 },
  .feature_kinds = {  },
  .f_fun = pose_f_fun,
  .F_fun = pose_F_fun,
  .err_fun = pose_err_fun,
  .inv_err_fun = pose_inv_err_fun,
  .H_mod_fun = pose_H_mod_fun,
  .predict = pose_predict,
  .hs = {
    { 4, pose_h_4 },
    { 10, pose_h_10 },
    { 13, pose_h_13 },
    { 14, pose_h_14 },
  },
  .Hs = {
    { 4, pose_H_4 },
    { 10, pose_H_10 },
    { 13, pose_H_13 },
    { 14, pose_H_14 },
  },
  .updates = {
    { 4, pose_update_4 },
    { 10, pose_update_10 },
    { 13, pose_update_13 },
    { 14, pose_update_14 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_lib_init(pose)
