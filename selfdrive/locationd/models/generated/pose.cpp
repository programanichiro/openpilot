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
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_3762798617585209229) {
   out_3762798617585209229[0] = delta_x[0] + nom_x[0];
   out_3762798617585209229[1] = delta_x[1] + nom_x[1];
   out_3762798617585209229[2] = delta_x[2] + nom_x[2];
   out_3762798617585209229[3] = delta_x[3] + nom_x[3];
   out_3762798617585209229[4] = delta_x[4] + nom_x[4];
   out_3762798617585209229[5] = delta_x[5] + nom_x[5];
   out_3762798617585209229[6] = delta_x[6] + nom_x[6];
   out_3762798617585209229[7] = delta_x[7] + nom_x[7];
   out_3762798617585209229[8] = delta_x[8] + nom_x[8];
   out_3762798617585209229[9] = delta_x[9] + nom_x[9];
   out_3762798617585209229[10] = delta_x[10] + nom_x[10];
   out_3762798617585209229[11] = delta_x[11] + nom_x[11];
   out_3762798617585209229[12] = delta_x[12] + nom_x[12];
   out_3762798617585209229[13] = delta_x[13] + nom_x[13];
   out_3762798617585209229[14] = delta_x[14] + nom_x[14];
   out_3762798617585209229[15] = delta_x[15] + nom_x[15];
   out_3762798617585209229[16] = delta_x[16] + nom_x[16];
   out_3762798617585209229[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_5516892092138461329) {
   out_5516892092138461329[0] = -nom_x[0] + true_x[0];
   out_5516892092138461329[1] = -nom_x[1] + true_x[1];
   out_5516892092138461329[2] = -nom_x[2] + true_x[2];
   out_5516892092138461329[3] = -nom_x[3] + true_x[3];
   out_5516892092138461329[4] = -nom_x[4] + true_x[4];
   out_5516892092138461329[5] = -nom_x[5] + true_x[5];
   out_5516892092138461329[6] = -nom_x[6] + true_x[6];
   out_5516892092138461329[7] = -nom_x[7] + true_x[7];
   out_5516892092138461329[8] = -nom_x[8] + true_x[8];
   out_5516892092138461329[9] = -nom_x[9] + true_x[9];
   out_5516892092138461329[10] = -nom_x[10] + true_x[10];
   out_5516892092138461329[11] = -nom_x[11] + true_x[11];
   out_5516892092138461329[12] = -nom_x[12] + true_x[12];
   out_5516892092138461329[13] = -nom_x[13] + true_x[13];
   out_5516892092138461329[14] = -nom_x[14] + true_x[14];
   out_5516892092138461329[15] = -nom_x[15] + true_x[15];
   out_5516892092138461329[16] = -nom_x[16] + true_x[16];
   out_5516892092138461329[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_7476032322802575233) {
   out_7476032322802575233[0] = 1.0;
   out_7476032322802575233[1] = 0.0;
   out_7476032322802575233[2] = 0.0;
   out_7476032322802575233[3] = 0.0;
   out_7476032322802575233[4] = 0.0;
   out_7476032322802575233[5] = 0.0;
   out_7476032322802575233[6] = 0.0;
   out_7476032322802575233[7] = 0.0;
   out_7476032322802575233[8] = 0.0;
   out_7476032322802575233[9] = 0.0;
   out_7476032322802575233[10] = 0.0;
   out_7476032322802575233[11] = 0.0;
   out_7476032322802575233[12] = 0.0;
   out_7476032322802575233[13] = 0.0;
   out_7476032322802575233[14] = 0.0;
   out_7476032322802575233[15] = 0.0;
   out_7476032322802575233[16] = 0.0;
   out_7476032322802575233[17] = 0.0;
   out_7476032322802575233[18] = 0.0;
   out_7476032322802575233[19] = 1.0;
   out_7476032322802575233[20] = 0.0;
   out_7476032322802575233[21] = 0.0;
   out_7476032322802575233[22] = 0.0;
   out_7476032322802575233[23] = 0.0;
   out_7476032322802575233[24] = 0.0;
   out_7476032322802575233[25] = 0.0;
   out_7476032322802575233[26] = 0.0;
   out_7476032322802575233[27] = 0.0;
   out_7476032322802575233[28] = 0.0;
   out_7476032322802575233[29] = 0.0;
   out_7476032322802575233[30] = 0.0;
   out_7476032322802575233[31] = 0.0;
   out_7476032322802575233[32] = 0.0;
   out_7476032322802575233[33] = 0.0;
   out_7476032322802575233[34] = 0.0;
   out_7476032322802575233[35] = 0.0;
   out_7476032322802575233[36] = 0.0;
   out_7476032322802575233[37] = 0.0;
   out_7476032322802575233[38] = 1.0;
   out_7476032322802575233[39] = 0.0;
   out_7476032322802575233[40] = 0.0;
   out_7476032322802575233[41] = 0.0;
   out_7476032322802575233[42] = 0.0;
   out_7476032322802575233[43] = 0.0;
   out_7476032322802575233[44] = 0.0;
   out_7476032322802575233[45] = 0.0;
   out_7476032322802575233[46] = 0.0;
   out_7476032322802575233[47] = 0.0;
   out_7476032322802575233[48] = 0.0;
   out_7476032322802575233[49] = 0.0;
   out_7476032322802575233[50] = 0.0;
   out_7476032322802575233[51] = 0.0;
   out_7476032322802575233[52] = 0.0;
   out_7476032322802575233[53] = 0.0;
   out_7476032322802575233[54] = 0.0;
   out_7476032322802575233[55] = 0.0;
   out_7476032322802575233[56] = 0.0;
   out_7476032322802575233[57] = 1.0;
   out_7476032322802575233[58] = 0.0;
   out_7476032322802575233[59] = 0.0;
   out_7476032322802575233[60] = 0.0;
   out_7476032322802575233[61] = 0.0;
   out_7476032322802575233[62] = 0.0;
   out_7476032322802575233[63] = 0.0;
   out_7476032322802575233[64] = 0.0;
   out_7476032322802575233[65] = 0.0;
   out_7476032322802575233[66] = 0.0;
   out_7476032322802575233[67] = 0.0;
   out_7476032322802575233[68] = 0.0;
   out_7476032322802575233[69] = 0.0;
   out_7476032322802575233[70] = 0.0;
   out_7476032322802575233[71] = 0.0;
   out_7476032322802575233[72] = 0.0;
   out_7476032322802575233[73] = 0.0;
   out_7476032322802575233[74] = 0.0;
   out_7476032322802575233[75] = 0.0;
   out_7476032322802575233[76] = 1.0;
   out_7476032322802575233[77] = 0.0;
   out_7476032322802575233[78] = 0.0;
   out_7476032322802575233[79] = 0.0;
   out_7476032322802575233[80] = 0.0;
   out_7476032322802575233[81] = 0.0;
   out_7476032322802575233[82] = 0.0;
   out_7476032322802575233[83] = 0.0;
   out_7476032322802575233[84] = 0.0;
   out_7476032322802575233[85] = 0.0;
   out_7476032322802575233[86] = 0.0;
   out_7476032322802575233[87] = 0.0;
   out_7476032322802575233[88] = 0.0;
   out_7476032322802575233[89] = 0.0;
   out_7476032322802575233[90] = 0.0;
   out_7476032322802575233[91] = 0.0;
   out_7476032322802575233[92] = 0.0;
   out_7476032322802575233[93] = 0.0;
   out_7476032322802575233[94] = 0.0;
   out_7476032322802575233[95] = 1.0;
   out_7476032322802575233[96] = 0.0;
   out_7476032322802575233[97] = 0.0;
   out_7476032322802575233[98] = 0.0;
   out_7476032322802575233[99] = 0.0;
   out_7476032322802575233[100] = 0.0;
   out_7476032322802575233[101] = 0.0;
   out_7476032322802575233[102] = 0.0;
   out_7476032322802575233[103] = 0.0;
   out_7476032322802575233[104] = 0.0;
   out_7476032322802575233[105] = 0.0;
   out_7476032322802575233[106] = 0.0;
   out_7476032322802575233[107] = 0.0;
   out_7476032322802575233[108] = 0.0;
   out_7476032322802575233[109] = 0.0;
   out_7476032322802575233[110] = 0.0;
   out_7476032322802575233[111] = 0.0;
   out_7476032322802575233[112] = 0.0;
   out_7476032322802575233[113] = 0.0;
   out_7476032322802575233[114] = 1.0;
   out_7476032322802575233[115] = 0.0;
   out_7476032322802575233[116] = 0.0;
   out_7476032322802575233[117] = 0.0;
   out_7476032322802575233[118] = 0.0;
   out_7476032322802575233[119] = 0.0;
   out_7476032322802575233[120] = 0.0;
   out_7476032322802575233[121] = 0.0;
   out_7476032322802575233[122] = 0.0;
   out_7476032322802575233[123] = 0.0;
   out_7476032322802575233[124] = 0.0;
   out_7476032322802575233[125] = 0.0;
   out_7476032322802575233[126] = 0.0;
   out_7476032322802575233[127] = 0.0;
   out_7476032322802575233[128] = 0.0;
   out_7476032322802575233[129] = 0.0;
   out_7476032322802575233[130] = 0.0;
   out_7476032322802575233[131] = 0.0;
   out_7476032322802575233[132] = 0.0;
   out_7476032322802575233[133] = 1.0;
   out_7476032322802575233[134] = 0.0;
   out_7476032322802575233[135] = 0.0;
   out_7476032322802575233[136] = 0.0;
   out_7476032322802575233[137] = 0.0;
   out_7476032322802575233[138] = 0.0;
   out_7476032322802575233[139] = 0.0;
   out_7476032322802575233[140] = 0.0;
   out_7476032322802575233[141] = 0.0;
   out_7476032322802575233[142] = 0.0;
   out_7476032322802575233[143] = 0.0;
   out_7476032322802575233[144] = 0.0;
   out_7476032322802575233[145] = 0.0;
   out_7476032322802575233[146] = 0.0;
   out_7476032322802575233[147] = 0.0;
   out_7476032322802575233[148] = 0.0;
   out_7476032322802575233[149] = 0.0;
   out_7476032322802575233[150] = 0.0;
   out_7476032322802575233[151] = 0.0;
   out_7476032322802575233[152] = 1.0;
   out_7476032322802575233[153] = 0.0;
   out_7476032322802575233[154] = 0.0;
   out_7476032322802575233[155] = 0.0;
   out_7476032322802575233[156] = 0.0;
   out_7476032322802575233[157] = 0.0;
   out_7476032322802575233[158] = 0.0;
   out_7476032322802575233[159] = 0.0;
   out_7476032322802575233[160] = 0.0;
   out_7476032322802575233[161] = 0.0;
   out_7476032322802575233[162] = 0.0;
   out_7476032322802575233[163] = 0.0;
   out_7476032322802575233[164] = 0.0;
   out_7476032322802575233[165] = 0.0;
   out_7476032322802575233[166] = 0.0;
   out_7476032322802575233[167] = 0.0;
   out_7476032322802575233[168] = 0.0;
   out_7476032322802575233[169] = 0.0;
   out_7476032322802575233[170] = 0.0;
   out_7476032322802575233[171] = 1.0;
   out_7476032322802575233[172] = 0.0;
   out_7476032322802575233[173] = 0.0;
   out_7476032322802575233[174] = 0.0;
   out_7476032322802575233[175] = 0.0;
   out_7476032322802575233[176] = 0.0;
   out_7476032322802575233[177] = 0.0;
   out_7476032322802575233[178] = 0.0;
   out_7476032322802575233[179] = 0.0;
   out_7476032322802575233[180] = 0.0;
   out_7476032322802575233[181] = 0.0;
   out_7476032322802575233[182] = 0.0;
   out_7476032322802575233[183] = 0.0;
   out_7476032322802575233[184] = 0.0;
   out_7476032322802575233[185] = 0.0;
   out_7476032322802575233[186] = 0.0;
   out_7476032322802575233[187] = 0.0;
   out_7476032322802575233[188] = 0.0;
   out_7476032322802575233[189] = 0.0;
   out_7476032322802575233[190] = 1.0;
   out_7476032322802575233[191] = 0.0;
   out_7476032322802575233[192] = 0.0;
   out_7476032322802575233[193] = 0.0;
   out_7476032322802575233[194] = 0.0;
   out_7476032322802575233[195] = 0.0;
   out_7476032322802575233[196] = 0.0;
   out_7476032322802575233[197] = 0.0;
   out_7476032322802575233[198] = 0.0;
   out_7476032322802575233[199] = 0.0;
   out_7476032322802575233[200] = 0.0;
   out_7476032322802575233[201] = 0.0;
   out_7476032322802575233[202] = 0.0;
   out_7476032322802575233[203] = 0.0;
   out_7476032322802575233[204] = 0.0;
   out_7476032322802575233[205] = 0.0;
   out_7476032322802575233[206] = 0.0;
   out_7476032322802575233[207] = 0.0;
   out_7476032322802575233[208] = 0.0;
   out_7476032322802575233[209] = 1.0;
   out_7476032322802575233[210] = 0.0;
   out_7476032322802575233[211] = 0.0;
   out_7476032322802575233[212] = 0.0;
   out_7476032322802575233[213] = 0.0;
   out_7476032322802575233[214] = 0.0;
   out_7476032322802575233[215] = 0.0;
   out_7476032322802575233[216] = 0.0;
   out_7476032322802575233[217] = 0.0;
   out_7476032322802575233[218] = 0.0;
   out_7476032322802575233[219] = 0.0;
   out_7476032322802575233[220] = 0.0;
   out_7476032322802575233[221] = 0.0;
   out_7476032322802575233[222] = 0.0;
   out_7476032322802575233[223] = 0.0;
   out_7476032322802575233[224] = 0.0;
   out_7476032322802575233[225] = 0.0;
   out_7476032322802575233[226] = 0.0;
   out_7476032322802575233[227] = 0.0;
   out_7476032322802575233[228] = 1.0;
   out_7476032322802575233[229] = 0.0;
   out_7476032322802575233[230] = 0.0;
   out_7476032322802575233[231] = 0.0;
   out_7476032322802575233[232] = 0.0;
   out_7476032322802575233[233] = 0.0;
   out_7476032322802575233[234] = 0.0;
   out_7476032322802575233[235] = 0.0;
   out_7476032322802575233[236] = 0.0;
   out_7476032322802575233[237] = 0.0;
   out_7476032322802575233[238] = 0.0;
   out_7476032322802575233[239] = 0.0;
   out_7476032322802575233[240] = 0.0;
   out_7476032322802575233[241] = 0.0;
   out_7476032322802575233[242] = 0.0;
   out_7476032322802575233[243] = 0.0;
   out_7476032322802575233[244] = 0.0;
   out_7476032322802575233[245] = 0.0;
   out_7476032322802575233[246] = 0.0;
   out_7476032322802575233[247] = 1.0;
   out_7476032322802575233[248] = 0.0;
   out_7476032322802575233[249] = 0.0;
   out_7476032322802575233[250] = 0.0;
   out_7476032322802575233[251] = 0.0;
   out_7476032322802575233[252] = 0.0;
   out_7476032322802575233[253] = 0.0;
   out_7476032322802575233[254] = 0.0;
   out_7476032322802575233[255] = 0.0;
   out_7476032322802575233[256] = 0.0;
   out_7476032322802575233[257] = 0.0;
   out_7476032322802575233[258] = 0.0;
   out_7476032322802575233[259] = 0.0;
   out_7476032322802575233[260] = 0.0;
   out_7476032322802575233[261] = 0.0;
   out_7476032322802575233[262] = 0.0;
   out_7476032322802575233[263] = 0.0;
   out_7476032322802575233[264] = 0.0;
   out_7476032322802575233[265] = 0.0;
   out_7476032322802575233[266] = 1.0;
   out_7476032322802575233[267] = 0.0;
   out_7476032322802575233[268] = 0.0;
   out_7476032322802575233[269] = 0.0;
   out_7476032322802575233[270] = 0.0;
   out_7476032322802575233[271] = 0.0;
   out_7476032322802575233[272] = 0.0;
   out_7476032322802575233[273] = 0.0;
   out_7476032322802575233[274] = 0.0;
   out_7476032322802575233[275] = 0.0;
   out_7476032322802575233[276] = 0.0;
   out_7476032322802575233[277] = 0.0;
   out_7476032322802575233[278] = 0.0;
   out_7476032322802575233[279] = 0.0;
   out_7476032322802575233[280] = 0.0;
   out_7476032322802575233[281] = 0.0;
   out_7476032322802575233[282] = 0.0;
   out_7476032322802575233[283] = 0.0;
   out_7476032322802575233[284] = 0.0;
   out_7476032322802575233[285] = 1.0;
   out_7476032322802575233[286] = 0.0;
   out_7476032322802575233[287] = 0.0;
   out_7476032322802575233[288] = 0.0;
   out_7476032322802575233[289] = 0.0;
   out_7476032322802575233[290] = 0.0;
   out_7476032322802575233[291] = 0.0;
   out_7476032322802575233[292] = 0.0;
   out_7476032322802575233[293] = 0.0;
   out_7476032322802575233[294] = 0.0;
   out_7476032322802575233[295] = 0.0;
   out_7476032322802575233[296] = 0.0;
   out_7476032322802575233[297] = 0.0;
   out_7476032322802575233[298] = 0.0;
   out_7476032322802575233[299] = 0.0;
   out_7476032322802575233[300] = 0.0;
   out_7476032322802575233[301] = 0.0;
   out_7476032322802575233[302] = 0.0;
   out_7476032322802575233[303] = 0.0;
   out_7476032322802575233[304] = 1.0;
   out_7476032322802575233[305] = 0.0;
   out_7476032322802575233[306] = 0.0;
   out_7476032322802575233[307] = 0.0;
   out_7476032322802575233[308] = 0.0;
   out_7476032322802575233[309] = 0.0;
   out_7476032322802575233[310] = 0.0;
   out_7476032322802575233[311] = 0.0;
   out_7476032322802575233[312] = 0.0;
   out_7476032322802575233[313] = 0.0;
   out_7476032322802575233[314] = 0.0;
   out_7476032322802575233[315] = 0.0;
   out_7476032322802575233[316] = 0.0;
   out_7476032322802575233[317] = 0.0;
   out_7476032322802575233[318] = 0.0;
   out_7476032322802575233[319] = 0.0;
   out_7476032322802575233[320] = 0.0;
   out_7476032322802575233[321] = 0.0;
   out_7476032322802575233[322] = 0.0;
   out_7476032322802575233[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_2337861120406759811) {
   out_2337861120406759811[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_2337861120406759811[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_2337861120406759811[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_2337861120406759811[3] = dt*state[12] + state[3];
   out_2337861120406759811[4] = dt*state[13] + state[4];
   out_2337861120406759811[5] = dt*state[14] + state[5];
   out_2337861120406759811[6] = state[6];
   out_2337861120406759811[7] = state[7];
   out_2337861120406759811[8] = state[8];
   out_2337861120406759811[9] = state[9];
   out_2337861120406759811[10] = state[10];
   out_2337861120406759811[11] = state[11];
   out_2337861120406759811[12] = state[12];
   out_2337861120406759811[13] = state[13];
   out_2337861120406759811[14] = state[14];
   out_2337861120406759811[15] = state[15];
   out_2337861120406759811[16] = state[16];
   out_2337861120406759811[17] = state[17];
}
void F_fun(double *state, double dt, double *out_7110526711015872718) {
   out_7110526711015872718[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7110526711015872718[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7110526711015872718[2] = 0;
   out_7110526711015872718[3] = 0;
   out_7110526711015872718[4] = 0;
   out_7110526711015872718[5] = 0;
   out_7110526711015872718[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7110526711015872718[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7110526711015872718[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7110526711015872718[9] = 0;
   out_7110526711015872718[10] = 0;
   out_7110526711015872718[11] = 0;
   out_7110526711015872718[12] = 0;
   out_7110526711015872718[13] = 0;
   out_7110526711015872718[14] = 0;
   out_7110526711015872718[15] = 0;
   out_7110526711015872718[16] = 0;
   out_7110526711015872718[17] = 0;
   out_7110526711015872718[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_7110526711015872718[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_7110526711015872718[20] = 0;
   out_7110526711015872718[21] = 0;
   out_7110526711015872718[22] = 0;
   out_7110526711015872718[23] = 0;
   out_7110526711015872718[24] = 0;
   out_7110526711015872718[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_7110526711015872718[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_7110526711015872718[27] = 0;
   out_7110526711015872718[28] = 0;
   out_7110526711015872718[29] = 0;
   out_7110526711015872718[30] = 0;
   out_7110526711015872718[31] = 0;
   out_7110526711015872718[32] = 0;
   out_7110526711015872718[33] = 0;
   out_7110526711015872718[34] = 0;
   out_7110526711015872718[35] = 0;
   out_7110526711015872718[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7110526711015872718[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7110526711015872718[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7110526711015872718[39] = 0;
   out_7110526711015872718[40] = 0;
   out_7110526711015872718[41] = 0;
   out_7110526711015872718[42] = 0;
   out_7110526711015872718[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7110526711015872718[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7110526711015872718[45] = 0;
   out_7110526711015872718[46] = 0;
   out_7110526711015872718[47] = 0;
   out_7110526711015872718[48] = 0;
   out_7110526711015872718[49] = 0;
   out_7110526711015872718[50] = 0;
   out_7110526711015872718[51] = 0;
   out_7110526711015872718[52] = 0;
   out_7110526711015872718[53] = 0;
   out_7110526711015872718[54] = 0;
   out_7110526711015872718[55] = 0;
   out_7110526711015872718[56] = 0;
   out_7110526711015872718[57] = 1;
   out_7110526711015872718[58] = 0;
   out_7110526711015872718[59] = 0;
   out_7110526711015872718[60] = 0;
   out_7110526711015872718[61] = 0;
   out_7110526711015872718[62] = 0;
   out_7110526711015872718[63] = 0;
   out_7110526711015872718[64] = 0;
   out_7110526711015872718[65] = 0;
   out_7110526711015872718[66] = dt;
   out_7110526711015872718[67] = 0;
   out_7110526711015872718[68] = 0;
   out_7110526711015872718[69] = 0;
   out_7110526711015872718[70] = 0;
   out_7110526711015872718[71] = 0;
   out_7110526711015872718[72] = 0;
   out_7110526711015872718[73] = 0;
   out_7110526711015872718[74] = 0;
   out_7110526711015872718[75] = 0;
   out_7110526711015872718[76] = 1;
   out_7110526711015872718[77] = 0;
   out_7110526711015872718[78] = 0;
   out_7110526711015872718[79] = 0;
   out_7110526711015872718[80] = 0;
   out_7110526711015872718[81] = 0;
   out_7110526711015872718[82] = 0;
   out_7110526711015872718[83] = 0;
   out_7110526711015872718[84] = 0;
   out_7110526711015872718[85] = dt;
   out_7110526711015872718[86] = 0;
   out_7110526711015872718[87] = 0;
   out_7110526711015872718[88] = 0;
   out_7110526711015872718[89] = 0;
   out_7110526711015872718[90] = 0;
   out_7110526711015872718[91] = 0;
   out_7110526711015872718[92] = 0;
   out_7110526711015872718[93] = 0;
   out_7110526711015872718[94] = 0;
   out_7110526711015872718[95] = 1;
   out_7110526711015872718[96] = 0;
   out_7110526711015872718[97] = 0;
   out_7110526711015872718[98] = 0;
   out_7110526711015872718[99] = 0;
   out_7110526711015872718[100] = 0;
   out_7110526711015872718[101] = 0;
   out_7110526711015872718[102] = 0;
   out_7110526711015872718[103] = 0;
   out_7110526711015872718[104] = dt;
   out_7110526711015872718[105] = 0;
   out_7110526711015872718[106] = 0;
   out_7110526711015872718[107] = 0;
   out_7110526711015872718[108] = 0;
   out_7110526711015872718[109] = 0;
   out_7110526711015872718[110] = 0;
   out_7110526711015872718[111] = 0;
   out_7110526711015872718[112] = 0;
   out_7110526711015872718[113] = 0;
   out_7110526711015872718[114] = 1;
   out_7110526711015872718[115] = 0;
   out_7110526711015872718[116] = 0;
   out_7110526711015872718[117] = 0;
   out_7110526711015872718[118] = 0;
   out_7110526711015872718[119] = 0;
   out_7110526711015872718[120] = 0;
   out_7110526711015872718[121] = 0;
   out_7110526711015872718[122] = 0;
   out_7110526711015872718[123] = 0;
   out_7110526711015872718[124] = 0;
   out_7110526711015872718[125] = 0;
   out_7110526711015872718[126] = 0;
   out_7110526711015872718[127] = 0;
   out_7110526711015872718[128] = 0;
   out_7110526711015872718[129] = 0;
   out_7110526711015872718[130] = 0;
   out_7110526711015872718[131] = 0;
   out_7110526711015872718[132] = 0;
   out_7110526711015872718[133] = 1;
   out_7110526711015872718[134] = 0;
   out_7110526711015872718[135] = 0;
   out_7110526711015872718[136] = 0;
   out_7110526711015872718[137] = 0;
   out_7110526711015872718[138] = 0;
   out_7110526711015872718[139] = 0;
   out_7110526711015872718[140] = 0;
   out_7110526711015872718[141] = 0;
   out_7110526711015872718[142] = 0;
   out_7110526711015872718[143] = 0;
   out_7110526711015872718[144] = 0;
   out_7110526711015872718[145] = 0;
   out_7110526711015872718[146] = 0;
   out_7110526711015872718[147] = 0;
   out_7110526711015872718[148] = 0;
   out_7110526711015872718[149] = 0;
   out_7110526711015872718[150] = 0;
   out_7110526711015872718[151] = 0;
   out_7110526711015872718[152] = 1;
   out_7110526711015872718[153] = 0;
   out_7110526711015872718[154] = 0;
   out_7110526711015872718[155] = 0;
   out_7110526711015872718[156] = 0;
   out_7110526711015872718[157] = 0;
   out_7110526711015872718[158] = 0;
   out_7110526711015872718[159] = 0;
   out_7110526711015872718[160] = 0;
   out_7110526711015872718[161] = 0;
   out_7110526711015872718[162] = 0;
   out_7110526711015872718[163] = 0;
   out_7110526711015872718[164] = 0;
   out_7110526711015872718[165] = 0;
   out_7110526711015872718[166] = 0;
   out_7110526711015872718[167] = 0;
   out_7110526711015872718[168] = 0;
   out_7110526711015872718[169] = 0;
   out_7110526711015872718[170] = 0;
   out_7110526711015872718[171] = 1;
   out_7110526711015872718[172] = 0;
   out_7110526711015872718[173] = 0;
   out_7110526711015872718[174] = 0;
   out_7110526711015872718[175] = 0;
   out_7110526711015872718[176] = 0;
   out_7110526711015872718[177] = 0;
   out_7110526711015872718[178] = 0;
   out_7110526711015872718[179] = 0;
   out_7110526711015872718[180] = 0;
   out_7110526711015872718[181] = 0;
   out_7110526711015872718[182] = 0;
   out_7110526711015872718[183] = 0;
   out_7110526711015872718[184] = 0;
   out_7110526711015872718[185] = 0;
   out_7110526711015872718[186] = 0;
   out_7110526711015872718[187] = 0;
   out_7110526711015872718[188] = 0;
   out_7110526711015872718[189] = 0;
   out_7110526711015872718[190] = 1;
   out_7110526711015872718[191] = 0;
   out_7110526711015872718[192] = 0;
   out_7110526711015872718[193] = 0;
   out_7110526711015872718[194] = 0;
   out_7110526711015872718[195] = 0;
   out_7110526711015872718[196] = 0;
   out_7110526711015872718[197] = 0;
   out_7110526711015872718[198] = 0;
   out_7110526711015872718[199] = 0;
   out_7110526711015872718[200] = 0;
   out_7110526711015872718[201] = 0;
   out_7110526711015872718[202] = 0;
   out_7110526711015872718[203] = 0;
   out_7110526711015872718[204] = 0;
   out_7110526711015872718[205] = 0;
   out_7110526711015872718[206] = 0;
   out_7110526711015872718[207] = 0;
   out_7110526711015872718[208] = 0;
   out_7110526711015872718[209] = 1;
   out_7110526711015872718[210] = 0;
   out_7110526711015872718[211] = 0;
   out_7110526711015872718[212] = 0;
   out_7110526711015872718[213] = 0;
   out_7110526711015872718[214] = 0;
   out_7110526711015872718[215] = 0;
   out_7110526711015872718[216] = 0;
   out_7110526711015872718[217] = 0;
   out_7110526711015872718[218] = 0;
   out_7110526711015872718[219] = 0;
   out_7110526711015872718[220] = 0;
   out_7110526711015872718[221] = 0;
   out_7110526711015872718[222] = 0;
   out_7110526711015872718[223] = 0;
   out_7110526711015872718[224] = 0;
   out_7110526711015872718[225] = 0;
   out_7110526711015872718[226] = 0;
   out_7110526711015872718[227] = 0;
   out_7110526711015872718[228] = 1;
   out_7110526711015872718[229] = 0;
   out_7110526711015872718[230] = 0;
   out_7110526711015872718[231] = 0;
   out_7110526711015872718[232] = 0;
   out_7110526711015872718[233] = 0;
   out_7110526711015872718[234] = 0;
   out_7110526711015872718[235] = 0;
   out_7110526711015872718[236] = 0;
   out_7110526711015872718[237] = 0;
   out_7110526711015872718[238] = 0;
   out_7110526711015872718[239] = 0;
   out_7110526711015872718[240] = 0;
   out_7110526711015872718[241] = 0;
   out_7110526711015872718[242] = 0;
   out_7110526711015872718[243] = 0;
   out_7110526711015872718[244] = 0;
   out_7110526711015872718[245] = 0;
   out_7110526711015872718[246] = 0;
   out_7110526711015872718[247] = 1;
   out_7110526711015872718[248] = 0;
   out_7110526711015872718[249] = 0;
   out_7110526711015872718[250] = 0;
   out_7110526711015872718[251] = 0;
   out_7110526711015872718[252] = 0;
   out_7110526711015872718[253] = 0;
   out_7110526711015872718[254] = 0;
   out_7110526711015872718[255] = 0;
   out_7110526711015872718[256] = 0;
   out_7110526711015872718[257] = 0;
   out_7110526711015872718[258] = 0;
   out_7110526711015872718[259] = 0;
   out_7110526711015872718[260] = 0;
   out_7110526711015872718[261] = 0;
   out_7110526711015872718[262] = 0;
   out_7110526711015872718[263] = 0;
   out_7110526711015872718[264] = 0;
   out_7110526711015872718[265] = 0;
   out_7110526711015872718[266] = 1;
   out_7110526711015872718[267] = 0;
   out_7110526711015872718[268] = 0;
   out_7110526711015872718[269] = 0;
   out_7110526711015872718[270] = 0;
   out_7110526711015872718[271] = 0;
   out_7110526711015872718[272] = 0;
   out_7110526711015872718[273] = 0;
   out_7110526711015872718[274] = 0;
   out_7110526711015872718[275] = 0;
   out_7110526711015872718[276] = 0;
   out_7110526711015872718[277] = 0;
   out_7110526711015872718[278] = 0;
   out_7110526711015872718[279] = 0;
   out_7110526711015872718[280] = 0;
   out_7110526711015872718[281] = 0;
   out_7110526711015872718[282] = 0;
   out_7110526711015872718[283] = 0;
   out_7110526711015872718[284] = 0;
   out_7110526711015872718[285] = 1;
   out_7110526711015872718[286] = 0;
   out_7110526711015872718[287] = 0;
   out_7110526711015872718[288] = 0;
   out_7110526711015872718[289] = 0;
   out_7110526711015872718[290] = 0;
   out_7110526711015872718[291] = 0;
   out_7110526711015872718[292] = 0;
   out_7110526711015872718[293] = 0;
   out_7110526711015872718[294] = 0;
   out_7110526711015872718[295] = 0;
   out_7110526711015872718[296] = 0;
   out_7110526711015872718[297] = 0;
   out_7110526711015872718[298] = 0;
   out_7110526711015872718[299] = 0;
   out_7110526711015872718[300] = 0;
   out_7110526711015872718[301] = 0;
   out_7110526711015872718[302] = 0;
   out_7110526711015872718[303] = 0;
   out_7110526711015872718[304] = 1;
   out_7110526711015872718[305] = 0;
   out_7110526711015872718[306] = 0;
   out_7110526711015872718[307] = 0;
   out_7110526711015872718[308] = 0;
   out_7110526711015872718[309] = 0;
   out_7110526711015872718[310] = 0;
   out_7110526711015872718[311] = 0;
   out_7110526711015872718[312] = 0;
   out_7110526711015872718[313] = 0;
   out_7110526711015872718[314] = 0;
   out_7110526711015872718[315] = 0;
   out_7110526711015872718[316] = 0;
   out_7110526711015872718[317] = 0;
   out_7110526711015872718[318] = 0;
   out_7110526711015872718[319] = 0;
   out_7110526711015872718[320] = 0;
   out_7110526711015872718[321] = 0;
   out_7110526711015872718[322] = 0;
   out_7110526711015872718[323] = 1;
}
void h_4(double *state, double *unused, double *out_5575517766908778607) {
   out_5575517766908778607[0] = state[6] + state[9];
   out_5575517766908778607[1] = state[7] + state[10];
   out_5575517766908778607[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_8230193324857287840) {
   out_8230193324857287840[0] = 0;
   out_8230193324857287840[1] = 0;
   out_8230193324857287840[2] = 0;
   out_8230193324857287840[3] = 0;
   out_8230193324857287840[4] = 0;
   out_8230193324857287840[5] = 0;
   out_8230193324857287840[6] = 1;
   out_8230193324857287840[7] = 0;
   out_8230193324857287840[8] = 0;
   out_8230193324857287840[9] = 1;
   out_8230193324857287840[10] = 0;
   out_8230193324857287840[11] = 0;
   out_8230193324857287840[12] = 0;
   out_8230193324857287840[13] = 0;
   out_8230193324857287840[14] = 0;
   out_8230193324857287840[15] = 0;
   out_8230193324857287840[16] = 0;
   out_8230193324857287840[17] = 0;
   out_8230193324857287840[18] = 0;
   out_8230193324857287840[19] = 0;
   out_8230193324857287840[20] = 0;
   out_8230193324857287840[21] = 0;
   out_8230193324857287840[22] = 0;
   out_8230193324857287840[23] = 0;
   out_8230193324857287840[24] = 0;
   out_8230193324857287840[25] = 1;
   out_8230193324857287840[26] = 0;
   out_8230193324857287840[27] = 0;
   out_8230193324857287840[28] = 1;
   out_8230193324857287840[29] = 0;
   out_8230193324857287840[30] = 0;
   out_8230193324857287840[31] = 0;
   out_8230193324857287840[32] = 0;
   out_8230193324857287840[33] = 0;
   out_8230193324857287840[34] = 0;
   out_8230193324857287840[35] = 0;
   out_8230193324857287840[36] = 0;
   out_8230193324857287840[37] = 0;
   out_8230193324857287840[38] = 0;
   out_8230193324857287840[39] = 0;
   out_8230193324857287840[40] = 0;
   out_8230193324857287840[41] = 0;
   out_8230193324857287840[42] = 0;
   out_8230193324857287840[43] = 0;
   out_8230193324857287840[44] = 1;
   out_8230193324857287840[45] = 0;
   out_8230193324857287840[46] = 0;
   out_8230193324857287840[47] = 1;
   out_8230193324857287840[48] = 0;
   out_8230193324857287840[49] = 0;
   out_8230193324857287840[50] = 0;
   out_8230193324857287840[51] = 0;
   out_8230193324857287840[52] = 0;
   out_8230193324857287840[53] = 0;
}
void h_10(double *state, double *unused, double *out_231064800032479191) {
   out_231064800032479191[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_231064800032479191[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_231064800032479191[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_809148575071622373) {
   out_809148575071622373[0] = 0;
   out_809148575071622373[1] = 9.8100000000000005*cos(state[1]);
   out_809148575071622373[2] = 0;
   out_809148575071622373[3] = 0;
   out_809148575071622373[4] = -state[8];
   out_809148575071622373[5] = state[7];
   out_809148575071622373[6] = 0;
   out_809148575071622373[7] = state[5];
   out_809148575071622373[8] = -state[4];
   out_809148575071622373[9] = 0;
   out_809148575071622373[10] = 0;
   out_809148575071622373[11] = 0;
   out_809148575071622373[12] = 1;
   out_809148575071622373[13] = 0;
   out_809148575071622373[14] = 0;
   out_809148575071622373[15] = 1;
   out_809148575071622373[16] = 0;
   out_809148575071622373[17] = 0;
   out_809148575071622373[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_809148575071622373[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_809148575071622373[20] = 0;
   out_809148575071622373[21] = state[8];
   out_809148575071622373[22] = 0;
   out_809148575071622373[23] = -state[6];
   out_809148575071622373[24] = -state[5];
   out_809148575071622373[25] = 0;
   out_809148575071622373[26] = state[3];
   out_809148575071622373[27] = 0;
   out_809148575071622373[28] = 0;
   out_809148575071622373[29] = 0;
   out_809148575071622373[30] = 0;
   out_809148575071622373[31] = 1;
   out_809148575071622373[32] = 0;
   out_809148575071622373[33] = 0;
   out_809148575071622373[34] = 1;
   out_809148575071622373[35] = 0;
   out_809148575071622373[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_809148575071622373[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_809148575071622373[38] = 0;
   out_809148575071622373[39] = -state[7];
   out_809148575071622373[40] = state[6];
   out_809148575071622373[41] = 0;
   out_809148575071622373[42] = state[4];
   out_809148575071622373[43] = -state[3];
   out_809148575071622373[44] = 0;
   out_809148575071622373[45] = 0;
   out_809148575071622373[46] = 0;
   out_809148575071622373[47] = 0;
   out_809148575071622373[48] = 0;
   out_809148575071622373[49] = 0;
   out_809148575071622373[50] = 1;
   out_809148575071622373[51] = 0;
   out_809148575071622373[52] = 0;
   out_809148575071622373[53] = 1;
}
void h_13(double *state, double *unused, double *out_6428852753307240167) {
   out_6428852753307240167[0] = state[3];
   out_6428852753307240167[1] = state[4];
   out_6428852753307240167[2] = state[5];
}
void H_13(double *state, double *unused, double *out_2605919540535562847) {
   out_2605919540535562847[0] = 0;
   out_2605919540535562847[1] = 0;
   out_2605919540535562847[2] = 0;
   out_2605919540535562847[3] = 1;
   out_2605919540535562847[4] = 0;
   out_2605919540535562847[5] = 0;
   out_2605919540535562847[6] = 0;
   out_2605919540535562847[7] = 0;
   out_2605919540535562847[8] = 0;
   out_2605919540535562847[9] = 0;
   out_2605919540535562847[10] = 0;
   out_2605919540535562847[11] = 0;
   out_2605919540535562847[12] = 0;
   out_2605919540535562847[13] = 0;
   out_2605919540535562847[14] = 0;
   out_2605919540535562847[15] = 0;
   out_2605919540535562847[16] = 0;
   out_2605919540535562847[17] = 0;
   out_2605919540535562847[18] = 0;
   out_2605919540535562847[19] = 0;
   out_2605919540535562847[20] = 0;
   out_2605919540535562847[21] = 0;
   out_2605919540535562847[22] = 1;
   out_2605919540535562847[23] = 0;
   out_2605919540535562847[24] = 0;
   out_2605919540535562847[25] = 0;
   out_2605919540535562847[26] = 0;
   out_2605919540535562847[27] = 0;
   out_2605919540535562847[28] = 0;
   out_2605919540535562847[29] = 0;
   out_2605919540535562847[30] = 0;
   out_2605919540535562847[31] = 0;
   out_2605919540535562847[32] = 0;
   out_2605919540535562847[33] = 0;
   out_2605919540535562847[34] = 0;
   out_2605919540535562847[35] = 0;
   out_2605919540535562847[36] = 0;
   out_2605919540535562847[37] = 0;
   out_2605919540535562847[38] = 0;
   out_2605919540535562847[39] = 0;
   out_2605919540535562847[40] = 0;
   out_2605919540535562847[41] = 1;
   out_2605919540535562847[42] = 0;
   out_2605919540535562847[43] = 0;
   out_2605919540535562847[44] = 0;
   out_2605919540535562847[45] = 0;
   out_2605919540535562847[46] = 0;
   out_2605919540535562847[47] = 0;
   out_2605919540535562847[48] = 0;
   out_2605919540535562847[49] = 0;
   out_2605919540535562847[50] = 0;
   out_2605919540535562847[51] = 0;
   out_2605919540535562847[52] = 0;
   out_2605919540535562847[53] = 0;
}
void h_14(double *state, double *unused, double *out_6226409763366102898) {
   out_6226409763366102898[0] = state[6];
   out_6226409763366102898[1] = state[7];
   out_6226409763366102898[2] = state[8];
}
void H_14(double *state, double *unused, double *out_6253309892512779247) {
   out_6253309892512779247[0] = 0;
   out_6253309892512779247[1] = 0;
   out_6253309892512779247[2] = 0;
   out_6253309892512779247[3] = 0;
   out_6253309892512779247[4] = 0;
   out_6253309892512779247[5] = 0;
   out_6253309892512779247[6] = 1;
   out_6253309892512779247[7] = 0;
   out_6253309892512779247[8] = 0;
   out_6253309892512779247[9] = 0;
   out_6253309892512779247[10] = 0;
   out_6253309892512779247[11] = 0;
   out_6253309892512779247[12] = 0;
   out_6253309892512779247[13] = 0;
   out_6253309892512779247[14] = 0;
   out_6253309892512779247[15] = 0;
   out_6253309892512779247[16] = 0;
   out_6253309892512779247[17] = 0;
   out_6253309892512779247[18] = 0;
   out_6253309892512779247[19] = 0;
   out_6253309892512779247[20] = 0;
   out_6253309892512779247[21] = 0;
   out_6253309892512779247[22] = 0;
   out_6253309892512779247[23] = 0;
   out_6253309892512779247[24] = 0;
   out_6253309892512779247[25] = 1;
   out_6253309892512779247[26] = 0;
   out_6253309892512779247[27] = 0;
   out_6253309892512779247[28] = 0;
   out_6253309892512779247[29] = 0;
   out_6253309892512779247[30] = 0;
   out_6253309892512779247[31] = 0;
   out_6253309892512779247[32] = 0;
   out_6253309892512779247[33] = 0;
   out_6253309892512779247[34] = 0;
   out_6253309892512779247[35] = 0;
   out_6253309892512779247[36] = 0;
   out_6253309892512779247[37] = 0;
   out_6253309892512779247[38] = 0;
   out_6253309892512779247[39] = 0;
   out_6253309892512779247[40] = 0;
   out_6253309892512779247[41] = 0;
   out_6253309892512779247[42] = 0;
   out_6253309892512779247[43] = 0;
   out_6253309892512779247[44] = 1;
   out_6253309892512779247[45] = 0;
   out_6253309892512779247[46] = 0;
   out_6253309892512779247[47] = 0;
   out_6253309892512779247[48] = 0;
   out_6253309892512779247[49] = 0;
   out_6253309892512779247[50] = 0;
   out_6253309892512779247[51] = 0;
   out_6253309892512779247[52] = 0;
   out_6253309892512779247[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_3762798617585209229) {
  err_fun(nom_x, delta_x, out_3762798617585209229);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_5516892092138461329) {
  inv_err_fun(nom_x, true_x, out_5516892092138461329);
}
void pose_H_mod_fun(double *state, double *out_7476032322802575233) {
  H_mod_fun(state, out_7476032322802575233);
}
void pose_f_fun(double *state, double dt, double *out_2337861120406759811) {
  f_fun(state,  dt, out_2337861120406759811);
}
void pose_F_fun(double *state, double dt, double *out_7110526711015872718) {
  F_fun(state,  dt, out_7110526711015872718);
}
void pose_h_4(double *state, double *unused, double *out_5575517766908778607) {
  h_4(state, unused, out_5575517766908778607);
}
void pose_H_4(double *state, double *unused, double *out_8230193324857287840) {
  H_4(state, unused, out_8230193324857287840);
}
void pose_h_10(double *state, double *unused, double *out_231064800032479191) {
  h_10(state, unused, out_231064800032479191);
}
void pose_H_10(double *state, double *unused, double *out_809148575071622373) {
  H_10(state, unused, out_809148575071622373);
}
void pose_h_13(double *state, double *unused, double *out_6428852753307240167) {
  h_13(state, unused, out_6428852753307240167);
}
void pose_H_13(double *state, double *unused, double *out_2605919540535562847) {
  H_13(state, unused, out_2605919540535562847);
}
void pose_h_14(double *state, double *unused, double *out_6226409763366102898) {
  h_14(state, unused, out_6226409763366102898);
}
void pose_H_14(double *state, double *unused, double *out_6253309892512779247) {
  H_14(state, unused, out_6253309892512779247);
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
