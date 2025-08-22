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
void err_fun(double *nom_x, double *delta_x, double *out_4197240932685800719) {
   out_4197240932685800719[0] = delta_x[0] + nom_x[0];
   out_4197240932685800719[1] = delta_x[1] + nom_x[1];
   out_4197240932685800719[2] = delta_x[2] + nom_x[2];
   out_4197240932685800719[3] = delta_x[3] + nom_x[3];
   out_4197240932685800719[4] = delta_x[4] + nom_x[4];
   out_4197240932685800719[5] = delta_x[5] + nom_x[5];
   out_4197240932685800719[6] = delta_x[6] + nom_x[6];
   out_4197240932685800719[7] = delta_x[7] + nom_x[7];
   out_4197240932685800719[8] = delta_x[8] + nom_x[8];
   out_4197240932685800719[9] = delta_x[9] + nom_x[9];
   out_4197240932685800719[10] = delta_x[10] + nom_x[10];
   out_4197240932685800719[11] = delta_x[11] + nom_x[11];
   out_4197240932685800719[12] = delta_x[12] + nom_x[12];
   out_4197240932685800719[13] = delta_x[13] + nom_x[13];
   out_4197240932685800719[14] = delta_x[14] + nom_x[14];
   out_4197240932685800719[15] = delta_x[15] + nom_x[15];
   out_4197240932685800719[16] = delta_x[16] + nom_x[16];
   out_4197240932685800719[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_8091385002608574816) {
   out_8091385002608574816[0] = -nom_x[0] + true_x[0];
   out_8091385002608574816[1] = -nom_x[1] + true_x[1];
   out_8091385002608574816[2] = -nom_x[2] + true_x[2];
   out_8091385002608574816[3] = -nom_x[3] + true_x[3];
   out_8091385002608574816[4] = -nom_x[4] + true_x[4];
   out_8091385002608574816[5] = -nom_x[5] + true_x[5];
   out_8091385002608574816[6] = -nom_x[6] + true_x[6];
   out_8091385002608574816[7] = -nom_x[7] + true_x[7];
   out_8091385002608574816[8] = -nom_x[8] + true_x[8];
   out_8091385002608574816[9] = -nom_x[9] + true_x[9];
   out_8091385002608574816[10] = -nom_x[10] + true_x[10];
   out_8091385002608574816[11] = -nom_x[11] + true_x[11];
   out_8091385002608574816[12] = -nom_x[12] + true_x[12];
   out_8091385002608574816[13] = -nom_x[13] + true_x[13];
   out_8091385002608574816[14] = -nom_x[14] + true_x[14];
   out_8091385002608574816[15] = -nom_x[15] + true_x[15];
   out_8091385002608574816[16] = -nom_x[16] + true_x[16];
   out_8091385002608574816[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_209432845494472972) {
   out_209432845494472972[0] = 1.0;
   out_209432845494472972[1] = 0.0;
   out_209432845494472972[2] = 0.0;
   out_209432845494472972[3] = 0.0;
   out_209432845494472972[4] = 0.0;
   out_209432845494472972[5] = 0.0;
   out_209432845494472972[6] = 0.0;
   out_209432845494472972[7] = 0.0;
   out_209432845494472972[8] = 0.0;
   out_209432845494472972[9] = 0.0;
   out_209432845494472972[10] = 0.0;
   out_209432845494472972[11] = 0.0;
   out_209432845494472972[12] = 0.0;
   out_209432845494472972[13] = 0.0;
   out_209432845494472972[14] = 0.0;
   out_209432845494472972[15] = 0.0;
   out_209432845494472972[16] = 0.0;
   out_209432845494472972[17] = 0.0;
   out_209432845494472972[18] = 0.0;
   out_209432845494472972[19] = 1.0;
   out_209432845494472972[20] = 0.0;
   out_209432845494472972[21] = 0.0;
   out_209432845494472972[22] = 0.0;
   out_209432845494472972[23] = 0.0;
   out_209432845494472972[24] = 0.0;
   out_209432845494472972[25] = 0.0;
   out_209432845494472972[26] = 0.0;
   out_209432845494472972[27] = 0.0;
   out_209432845494472972[28] = 0.0;
   out_209432845494472972[29] = 0.0;
   out_209432845494472972[30] = 0.0;
   out_209432845494472972[31] = 0.0;
   out_209432845494472972[32] = 0.0;
   out_209432845494472972[33] = 0.0;
   out_209432845494472972[34] = 0.0;
   out_209432845494472972[35] = 0.0;
   out_209432845494472972[36] = 0.0;
   out_209432845494472972[37] = 0.0;
   out_209432845494472972[38] = 1.0;
   out_209432845494472972[39] = 0.0;
   out_209432845494472972[40] = 0.0;
   out_209432845494472972[41] = 0.0;
   out_209432845494472972[42] = 0.0;
   out_209432845494472972[43] = 0.0;
   out_209432845494472972[44] = 0.0;
   out_209432845494472972[45] = 0.0;
   out_209432845494472972[46] = 0.0;
   out_209432845494472972[47] = 0.0;
   out_209432845494472972[48] = 0.0;
   out_209432845494472972[49] = 0.0;
   out_209432845494472972[50] = 0.0;
   out_209432845494472972[51] = 0.0;
   out_209432845494472972[52] = 0.0;
   out_209432845494472972[53] = 0.0;
   out_209432845494472972[54] = 0.0;
   out_209432845494472972[55] = 0.0;
   out_209432845494472972[56] = 0.0;
   out_209432845494472972[57] = 1.0;
   out_209432845494472972[58] = 0.0;
   out_209432845494472972[59] = 0.0;
   out_209432845494472972[60] = 0.0;
   out_209432845494472972[61] = 0.0;
   out_209432845494472972[62] = 0.0;
   out_209432845494472972[63] = 0.0;
   out_209432845494472972[64] = 0.0;
   out_209432845494472972[65] = 0.0;
   out_209432845494472972[66] = 0.0;
   out_209432845494472972[67] = 0.0;
   out_209432845494472972[68] = 0.0;
   out_209432845494472972[69] = 0.0;
   out_209432845494472972[70] = 0.0;
   out_209432845494472972[71] = 0.0;
   out_209432845494472972[72] = 0.0;
   out_209432845494472972[73] = 0.0;
   out_209432845494472972[74] = 0.0;
   out_209432845494472972[75] = 0.0;
   out_209432845494472972[76] = 1.0;
   out_209432845494472972[77] = 0.0;
   out_209432845494472972[78] = 0.0;
   out_209432845494472972[79] = 0.0;
   out_209432845494472972[80] = 0.0;
   out_209432845494472972[81] = 0.0;
   out_209432845494472972[82] = 0.0;
   out_209432845494472972[83] = 0.0;
   out_209432845494472972[84] = 0.0;
   out_209432845494472972[85] = 0.0;
   out_209432845494472972[86] = 0.0;
   out_209432845494472972[87] = 0.0;
   out_209432845494472972[88] = 0.0;
   out_209432845494472972[89] = 0.0;
   out_209432845494472972[90] = 0.0;
   out_209432845494472972[91] = 0.0;
   out_209432845494472972[92] = 0.0;
   out_209432845494472972[93] = 0.0;
   out_209432845494472972[94] = 0.0;
   out_209432845494472972[95] = 1.0;
   out_209432845494472972[96] = 0.0;
   out_209432845494472972[97] = 0.0;
   out_209432845494472972[98] = 0.0;
   out_209432845494472972[99] = 0.0;
   out_209432845494472972[100] = 0.0;
   out_209432845494472972[101] = 0.0;
   out_209432845494472972[102] = 0.0;
   out_209432845494472972[103] = 0.0;
   out_209432845494472972[104] = 0.0;
   out_209432845494472972[105] = 0.0;
   out_209432845494472972[106] = 0.0;
   out_209432845494472972[107] = 0.0;
   out_209432845494472972[108] = 0.0;
   out_209432845494472972[109] = 0.0;
   out_209432845494472972[110] = 0.0;
   out_209432845494472972[111] = 0.0;
   out_209432845494472972[112] = 0.0;
   out_209432845494472972[113] = 0.0;
   out_209432845494472972[114] = 1.0;
   out_209432845494472972[115] = 0.0;
   out_209432845494472972[116] = 0.0;
   out_209432845494472972[117] = 0.0;
   out_209432845494472972[118] = 0.0;
   out_209432845494472972[119] = 0.0;
   out_209432845494472972[120] = 0.0;
   out_209432845494472972[121] = 0.0;
   out_209432845494472972[122] = 0.0;
   out_209432845494472972[123] = 0.0;
   out_209432845494472972[124] = 0.0;
   out_209432845494472972[125] = 0.0;
   out_209432845494472972[126] = 0.0;
   out_209432845494472972[127] = 0.0;
   out_209432845494472972[128] = 0.0;
   out_209432845494472972[129] = 0.0;
   out_209432845494472972[130] = 0.0;
   out_209432845494472972[131] = 0.0;
   out_209432845494472972[132] = 0.0;
   out_209432845494472972[133] = 1.0;
   out_209432845494472972[134] = 0.0;
   out_209432845494472972[135] = 0.0;
   out_209432845494472972[136] = 0.0;
   out_209432845494472972[137] = 0.0;
   out_209432845494472972[138] = 0.0;
   out_209432845494472972[139] = 0.0;
   out_209432845494472972[140] = 0.0;
   out_209432845494472972[141] = 0.0;
   out_209432845494472972[142] = 0.0;
   out_209432845494472972[143] = 0.0;
   out_209432845494472972[144] = 0.0;
   out_209432845494472972[145] = 0.0;
   out_209432845494472972[146] = 0.0;
   out_209432845494472972[147] = 0.0;
   out_209432845494472972[148] = 0.0;
   out_209432845494472972[149] = 0.0;
   out_209432845494472972[150] = 0.0;
   out_209432845494472972[151] = 0.0;
   out_209432845494472972[152] = 1.0;
   out_209432845494472972[153] = 0.0;
   out_209432845494472972[154] = 0.0;
   out_209432845494472972[155] = 0.0;
   out_209432845494472972[156] = 0.0;
   out_209432845494472972[157] = 0.0;
   out_209432845494472972[158] = 0.0;
   out_209432845494472972[159] = 0.0;
   out_209432845494472972[160] = 0.0;
   out_209432845494472972[161] = 0.0;
   out_209432845494472972[162] = 0.0;
   out_209432845494472972[163] = 0.0;
   out_209432845494472972[164] = 0.0;
   out_209432845494472972[165] = 0.0;
   out_209432845494472972[166] = 0.0;
   out_209432845494472972[167] = 0.0;
   out_209432845494472972[168] = 0.0;
   out_209432845494472972[169] = 0.0;
   out_209432845494472972[170] = 0.0;
   out_209432845494472972[171] = 1.0;
   out_209432845494472972[172] = 0.0;
   out_209432845494472972[173] = 0.0;
   out_209432845494472972[174] = 0.0;
   out_209432845494472972[175] = 0.0;
   out_209432845494472972[176] = 0.0;
   out_209432845494472972[177] = 0.0;
   out_209432845494472972[178] = 0.0;
   out_209432845494472972[179] = 0.0;
   out_209432845494472972[180] = 0.0;
   out_209432845494472972[181] = 0.0;
   out_209432845494472972[182] = 0.0;
   out_209432845494472972[183] = 0.0;
   out_209432845494472972[184] = 0.0;
   out_209432845494472972[185] = 0.0;
   out_209432845494472972[186] = 0.0;
   out_209432845494472972[187] = 0.0;
   out_209432845494472972[188] = 0.0;
   out_209432845494472972[189] = 0.0;
   out_209432845494472972[190] = 1.0;
   out_209432845494472972[191] = 0.0;
   out_209432845494472972[192] = 0.0;
   out_209432845494472972[193] = 0.0;
   out_209432845494472972[194] = 0.0;
   out_209432845494472972[195] = 0.0;
   out_209432845494472972[196] = 0.0;
   out_209432845494472972[197] = 0.0;
   out_209432845494472972[198] = 0.0;
   out_209432845494472972[199] = 0.0;
   out_209432845494472972[200] = 0.0;
   out_209432845494472972[201] = 0.0;
   out_209432845494472972[202] = 0.0;
   out_209432845494472972[203] = 0.0;
   out_209432845494472972[204] = 0.0;
   out_209432845494472972[205] = 0.0;
   out_209432845494472972[206] = 0.0;
   out_209432845494472972[207] = 0.0;
   out_209432845494472972[208] = 0.0;
   out_209432845494472972[209] = 1.0;
   out_209432845494472972[210] = 0.0;
   out_209432845494472972[211] = 0.0;
   out_209432845494472972[212] = 0.0;
   out_209432845494472972[213] = 0.0;
   out_209432845494472972[214] = 0.0;
   out_209432845494472972[215] = 0.0;
   out_209432845494472972[216] = 0.0;
   out_209432845494472972[217] = 0.0;
   out_209432845494472972[218] = 0.0;
   out_209432845494472972[219] = 0.0;
   out_209432845494472972[220] = 0.0;
   out_209432845494472972[221] = 0.0;
   out_209432845494472972[222] = 0.0;
   out_209432845494472972[223] = 0.0;
   out_209432845494472972[224] = 0.0;
   out_209432845494472972[225] = 0.0;
   out_209432845494472972[226] = 0.0;
   out_209432845494472972[227] = 0.0;
   out_209432845494472972[228] = 1.0;
   out_209432845494472972[229] = 0.0;
   out_209432845494472972[230] = 0.0;
   out_209432845494472972[231] = 0.0;
   out_209432845494472972[232] = 0.0;
   out_209432845494472972[233] = 0.0;
   out_209432845494472972[234] = 0.0;
   out_209432845494472972[235] = 0.0;
   out_209432845494472972[236] = 0.0;
   out_209432845494472972[237] = 0.0;
   out_209432845494472972[238] = 0.0;
   out_209432845494472972[239] = 0.0;
   out_209432845494472972[240] = 0.0;
   out_209432845494472972[241] = 0.0;
   out_209432845494472972[242] = 0.0;
   out_209432845494472972[243] = 0.0;
   out_209432845494472972[244] = 0.0;
   out_209432845494472972[245] = 0.0;
   out_209432845494472972[246] = 0.0;
   out_209432845494472972[247] = 1.0;
   out_209432845494472972[248] = 0.0;
   out_209432845494472972[249] = 0.0;
   out_209432845494472972[250] = 0.0;
   out_209432845494472972[251] = 0.0;
   out_209432845494472972[252] = 0.0;
   out_209432845494472972[253] = 0.0;
   out_209432845494472972[254] = 0.0;
   out_209432845494472972[255] = 0.0;
   out_209432845494472972[256] = 0.0;
   out_209432845494472972[257] = 0.0;
   out_209432845494472972[258] = 0.0;
   out_209432845494472972[259] = 0.0;
   out_209432845494472972[260] = 0.0;
   out_209432845494472972[261] = 0.0;
   out_209432845494472972[262] = 0.0;
   out_209432845494472972[263] = 0.0;
   out_209432845494472972[264] = 0.0;
   out_209432845494472972[265] = 0.0;
   out_209432845494472972[266] = 1.0;
   out_209432845494472972[267] = 0.0;
   out_209432845494472972[268] = 0.0;
   out_209432845494472972[269] = 0.0;
   out_209432845494472972[270] = 0.0;
   out_209432845494472972[271] = 0.0;
   out_209432845494472972[272] = 0.0;
   out_209432845494472972[273] = 0.0;
   out_209432845494472972[274] = 0.0;
   out_209432845494472972[275] = 0.0;
   out_209432845494472972[276] = 0.0;
   out_209432845494472972[277] = 0.0;
   out_209432845494472972[278] = 0.0;
   out_209432845494472972[279] = 0.0;
   out_209432845494472972[280] = 0.0;
   out_209432845494472972[281] = 0.0;
   out_209432845494472972[282] = 0.0;
   out_209432845494472972[283] = 0.0;
   out_209432845494472972[284] = 0.0;
   out_209432845494472972[285] = 1.0;
   out_209432845494472972[286] = 0.0;
   out_209432845494472972[287] = 0.0;
   out_209432845494472972[288] = 0.0;
   out_209432845494472972[289] = 0.0;
   out_209432845494472972[290] = 0.0;
   out_209432845494472972[291] = 0.0;
   out_209432845494472972[292] = 0.0;
   out_209432845494472972[293] = 0.0;
   out_209432845494472972[294] = 0.0;
   out_209432845494472972[295] = 0.0;
   out_209432845494472972[296] = 0.0;
   out_209432845494472972[297] = 0.0;
   out_209432845494472972[298] = 0.0;
   out_209432845494472972[299] = 0.0;
   out_209432845494472972[300] = 0.0;
   out_209432845494472972[301] = 0.0;
   out_209432845494472972[302] = 0.0;
   out_209432845494472972[303] = 0.0;
   out_209432845494472972[304] = 1.0;
   out_209432845494472972[305] = 0.0;
   out_209432845494472972[306] = 0.0;
   out_209432845494472972[307] = 0.0;
   out_209432845494472972[308] = 0.0;
   out_209432845494472972[309] = 0.0;
   out_209432845494472972[310] = 0.0;
   out_209432845494472972[311] = 0.0;
   out_209432845494472972[312] = 0.0;
   out_209432845494472972[313] = 0.0;
   out_209432845494472972[314] = 0.0;
   out_209432845494472972[315] = 0.0;
   out_209432845494472972[316] = 0.0;
   out_209432845494472972[317] = 0.0;
   out_209432845494472972[318] = 0.0;
   out_209432845494472972[319] = 0.0;
   out_209432845494472972[320] = 0.0;
   out_209432845494472972[321] = 0.0;
   out_209432845494472972[322] = 0.0;
   out_209432845494472972[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_7558387581431867623) {
   out_7558387581431867623[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_7558387581431867623[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_7558387581431867623[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_7558387581431867623[3] = dt*state[12] + state[3];
   out_7558387581431867623[4] = dt*state[13] + state[4];
   out_7558387581431867623[5] = dt*state[14] + state[5];
   out_7558387581431867623[6] = state[6];
   out_7558387581431867623[7] = state[7];
   out_7558387581431867623[8] = state[8];
   out_7558387581431867623[9] = state[9];
   out_7558387581431867623[10] = state[10];
   out_7558387581431867623[11] = state[11];
   out_7558387581431867623[12] = state[12];
   out_7558387581431867623[13] = state[13];
   out_7558387581431867623[14] = state[14];
   out_7558387581431867623[15] = state[15];
   out_7558387581431867623[16] = state[16];
   out_7558387581431867623[17] = state[17];
}
void F_fun(double *state, double dt, double *out_5546184258405748350) {
   out_5546184258405748350[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5546184258405748350[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5546184258405748350[2] = 0;
   out_5546184258405748350[3] = 0;
   out_5546184258405748350[4] = 0;
   out_5546184258405748350[5] = 0;
   out_5546184258405748350[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5546184258405748350[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5546184258405748350[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5546184258405748350[9] = 0;
   out_5546184258405748350[10] = 0;
   out_5546184258405748350[11] = 0;
   out_5546184258405748350[12] = 0;
   out_5546184258405748350[13] = 0;
   out_5546184258405748350[14] = 0;
   out_5546184258405748350[15] = 0;
   out_5546184258405748350[16] = 0;
   out_5546184258405748350[17] = 0;
   out_5546184258405748350[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_5546184258405748350[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_5546184258405748350[20] = 0;
   out_5546184258405748350[21] = 0;
   out_5546184258405748350[22] = 0;
   out_5546184258405748350[23] = 0;
   out_5546184258405748350[24] = 0;
   out_5546184258405748350[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_5546184258405748350[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_5546184258405748350[27] = 0;
   out_5546184258405748350[28] = 0;
   out_5546184258405748350[29] = 0;
   out_5546184258405748350[30] = 0;
   out_5546184258405748350[31] = 0;
   out_5546184258405748350[32] = 0;
   out_5546184258405748350[33] = 0;
   out_5546184258405748350[34] = 0;
   out_5546184258405748350[35] = 0;
   out_5546184258405748350[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5546184258405748350[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5546184258405748350[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5546184258405748350[39] = 0;
   out_5546184258405748350[40] = 0;
   out_5546184258405748350[41] = 0;
   out_5546184258405748350[42] = 0;
   out_5546184258405748350[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5546184258405748350[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5546184258405748350[45] = 0;
   out_5546184258405748350[46] = 0;
   out_5546184258405748350[47] = 0;
   out_5546184258405748350[48] = 0;
   out_5546184258405748350[49] = 0;
   out_5546184258405748350[50] = 0;
   out_5546184258405748350[51] = 0;
   out_5546184258405748350[52] = 0;
   out_5546184258405748350[53] = 0;
   out_5546184258405748350[54] = 0;
   out_5546184258405748350[55] = 0;
   out_5546184258405748350[56] = 0;
   out_5546184258405748350[57] = 1;
   out_5546184258405748350[58] = 0;
   out_5546184258405748350[59] = 0;
   out_5546184258405748350[60] = 0;
   out_5546184258405748350[61] = 0;
   out_5546184258405748350[62] = 0;
   out_5546184258405748350[63] = 0;
   out_5546184258405748350[64] = 0;
   out_5546184258405748350[65] = 0;
   out_5546184258405748350[66] = dt;
   out_5546184258405748350[67] = 0;
   out_5546184258405748350[68] = 0;
   out_5546184258405748350[69] = 0;
   out_5546184258405748350[70] = 0;
   out_5546184258405748350[71] = 0;
   out_5546184258405748350[72] = 0;
   out_5546184258405748350[73] = 0;
   out_5546184258405748350[74] = 0;
   out_5546184258405748350[75] = 0;
   out_5546184258405748350[76] = 1;
   out_5546184258405748350[77] = 0;
   out_5546184258405748350[78] = 0;
   out_5546184258405748350[79] = 0;
   out_5546184258405748350[80] = 0;
   out_5546184258405748350[81] = 0;
   out_5546184258405748350[82] = 0;
   out_5546184258405748350[83] = 0;
   out_5546184258405748350[84] = 0;
   out_5546184258405748350[85] = dt;
   out_5546184258405748350[86] = 0;
   out_5546184258405748350[87] = 0;
   out_5546184258405748350[88] = 0;
   out_5546184258405748350[89] = 0;
   out_5546184258405748350[90] = 0;
   out_5546184258405748350[91] = 0;
   out_5546184258405748350[92] = 0;
   out_5546184258405748350[93] = 0;
   out_5546184258405748350[94] = 0;
   out_5546184258405748350[95] = 1;
   out_5546184258405748350[96] = 0;
   out_5546184258405748350[97] = 0;
   out_5546184258405748350[98] = 0;
   out_5546184258405748350[99] = 0;
   out_5546184258405748350[100] = 0;
   out_5546184258405748350[101] = 0;
   out_5546184258405748350[102] = 0;
   out_5546184258405748350[103] = 0;
   out_5546184258405748350[104] = dt;
   out_5546184258405748350[105] = 0;
   out_5546184258405748350[106] = 0;
   out_5546184258405748350[107] = 0;
   out_5546184258405748350[108] = 0;
   out_5546184258405748350[109] = 0;
   out_5546184258405748350[110] = 0;
   out_5546184258405748350[111] = 0;
   out_5546184258405748350[112] = 0;
   out_5546184258405748350[113] = 0;
   out_5546184258405748350[114] = 1;
   out_5546184258405748350[115] = 0;
   out_5546184258405748350[116] = 0;
   out_5546184258405748350[117] = 0;
   out_5546184258405748350[118] = 0;
   out_5546184258405748350[119] = 0;
   out_5546184258405748350[120] = 0;
   out_5546184258405748350[121] = 0;
   out_5546184258405748350[122] = 0;
   out_5546184258405748350[123] = 0;
   out_5546184258405748350[124] = 0;
   out_5546184258405748350[125] = 0;
   out_5546184258405748350[126] = 0;
   out_5546184258405748350[127] = 0;
   out_5546184258405748350[128] = 0;
   out_5546184258405748350[129] = 0;
   out_5546184258405748350[130] = 0;
   out_5546184258405748350[131] = 0;
   out_5546184258405748350[132] = 0;
   out_5546184258405748350[133] = 1;
   out_5546184258405748350[134] = 0;
   out_5546184258405748350[135] = 0;
   out_5546184258405748350[136] = 0;
   out_5546184258405748350[137] = 0;
   out_5546184258405748350[138] = 0;
   out_5546184258405748350[139] = 0;
   out_5546184258405748350[140] = 0;
   out_5546184258405748350[141] = 0;
   out_5546184258405748350[142] = 0;
   out_5546184258405748350[143] = 0;
   out_5546184258405748350[144] = 0;
   out_5546184258405748350[145] = 0;
   out_5546184258405748350[146] = 0;
   out_5546184258405748350[147] = 0;
   out_5546184258405748350[148] = 0;
   out_5546184258405748350[149] = 0;
   out_5546184258405748350[150] = 0;
   out_5546184258405748350[151] = 0;
   out_5546184258405748350[152] = 1;
   out_5546184258405748350[153] = 0;
   out_5546184258405748350[154] = 0;
   out_5546184258405748350[155] = 0;
   out_5546184258405748350[156] = 0;
   out_5546184258405748350[157] = 0;
   out_5546184258405748350[158] = 0;
   out_5546184258405748350[159] = 0;
   out_5546184258405748350[160] = 0;
   out_5546184258405748350[161] = 0;
   out_5546184258405748350[162] = 0;
   out_5546184258405748350[163] = 0;
   out_5546184258405748350[164] = 0;
   out_5546184258405748350[165] = 0;
   out_5546184258405748350[166] = 0;
   out_5546184258405748350[167] = 0;
   out_5546184258405748350[168] = 0;
   out_5546184258405748350[169] = 0;
   out_5546184258405748350[170] = 0;
   out_5546184258405748350[171] = 1;
   out_5546184258405748350[172] = 0;
   out_5546184258405748350[173] = 0;
   out_5546184258405748350[174] = 0;
   out_5546184258405748350[175] = 0;
   out_5546184258405748350[176] = 0;
   out_5546184258405748350[177] = 0;
   out_5546184258405748350[178] = 0;
   out_5546184258405748350[179] = 0;
   out_5546184258405748350[180] = 0;
   out_5546184258405748350[181] = 0;
   out_5546184258405748350[182] = 0;
   out_5546184258405748350[183] = 0;
   out_5546184258405748350[184] = 0;
   out_5546184258405748350[185] = 0;
   out_5546184258405748350[186] = 0;
   out_5546184258405748350[187] = 0;
   out_5546184258405748350[188] = 0;
   out_5546184258405748350[189] = 0;
   out_5546184258405748350[190] = 1;
   out_5546184258405748350[191] = 0;
   out_5546184258405748350[192] = 0;
   out_5546184258405748350[193] = 0;
   out_5546184258405748350[194] = 0;
   out_5546184258405748350[195] = 0;
   out_5546184258405748350[196] = 0;
   out_5546184258405748350[197] = 0;
   out_5546184258405748350[198] = 0;
   out_5546184258405748350[199] = 0;
   out_5546184258405748350[200] = 0;
   out_5546184258405748350[201] = 0;
   out_5546184258405748350[202] = 0;
   out_5546184258405748350[203] = 0;
   out_5546184258405748350[204] = 0;
   out_5546184258405748350[205] = 0;
   out_5546184258405748350[206] = 0;
   out_5546184258405748350[207] = 0;
   out_5546184258405748350[208] = 0;
   out_5546184258405748350[209] = 1;
   out_5546184258405748350[210] = 0;
   out_5546184258405748350[211] = 0;
   out_5546184258405748350[212] = 0;
   out_5546184258405748350[213] = 0;
   out_5546184258405748350[214] = 0;
   out_5546184258405748350[215] = 0;
   out_5546184258405748350[216] = 0;
   out_5546184258405748350[217] = 0;
   out_5546184258405748350[218] = 0;
   out_5546184258405748350[219] = 0;
   out_5546184258405748350[220] = 0;
   out_5546184258405748350[221] = 0;
   out_5546184258405748350[222] = 0;
   out_5546184258405748350[223] = 0;
   out_5546184258405748350[224] = 0;
   out_5546184258405748350[225] = 0;
   out_5546184258405748350[226] = 0;
   out_5546184258405748350[227] = 0;
   out_5546184258405748350[228] = 1;
   out_5546184258405748350[229] = 0;
   out_5546184258405748350[230] = 0;
   out_5546184258405748350[231] = 0;
   out_5546184258405748350[232] = 0;
   out_5546184258405748350[233] = 0;
   out_5546184258405748350[234] = 0;
   out_5546184258405748350[235] = 0;
   out_5546184258405748350[236] = 0;
   out_5546184258405748350[237] = 0;
   out_5546184258405748350[238] = 0;
   out_5546184258405748350[239] = 0;
   out_5546184258405748350[240] = 0;
   out_5546184258405748350[241] = 0;
   out_5546184258405748350[242] = 0;
   out_5546184258405748350[243] = 0;
   out_5546184258405748350[244] = 0;
   out_5546184258405748350[245] = 0;
   out_5546184258405748350[246] = 0;
   out_5546184258405748350[247] = 1;
   out_5546184258405748350[248] = 0;
   out_5546184258405748350[249] = 0;
   out_5546184258405748350[250] = 0;
   out_5546184258405748350[251] = 0;
   out_5546184258405748350[252] = 0;
   out_5546184258405748350[253] = 0;
   out_5546184258405748350[254] = 0;
   out_5546184258405748350[255] = 0;
   out_5546184258405748350[256] = 0;
   out_5546184258405748350[257] = 0;
   out_5546184258405748350[258] = 0;
   out_5546184258405748350[259] = 0;
   out_5546184258405748350[260] = 0;
   out_5546184258405748350[261] = 0;
   out_5546184258405748350[262] = 0;
   out_5546184258405748350[263] = 0;
   out_5546184258405748350[264] = 0;
   out_5546184258405748350[265] = 0;
   out_5546184258405748350[266] = 1;
   out_5546184258405748350[267] = 0;
   out_5546184258405748350[268] = 0;
   out_5546184258405748350[269] = 0;
   out_5546184258405748350[270] = 0;
   out_5546184258405748350[271] = 0;
   out_5546184258405748350[272] = 0;
   out_5546184258405748350[273] = 0;
   out_5546184258405748350[274] = 0;
   out_5546184258405748350[275] = 0;
   out_5546184258405748350[276] = 0;
   out_5546184258405748350[277] = 0;
   out_5546184258405748350[278] = 0;
   out_5546184258405748350[279] = 0;
   out_5546184258405748350[280] = 0;
   out_5546184258405748350[281] = 0;
   out_5546184258405748350[282] = 0;
   out_5546184258405748350[283] = 0;
   out_5546184258405748350[284] = 0;
   out_5546184258405748350[285] = 1;
   out_5546184258405748350[286] = 0;
   out_5546184258405748350[287] = 0;
   out_5546184258405748350[288] = 0;
   out_5546184258405748350[289] = 0;
   out_5546184258405748350[290] = 0;
   out_5546184258405748350[291] = 0;
   out_5546184258405748350[292] = 0;
   out_5546184258405748350[293] = 0;
   out_5546184258405748350[294] = 0;
   out_5546184258405748350[295] = 0;
   out_5546184258405748350[296] = 0;
   out_5546184258405748350[297] = 0;
   out_5546184258405748350[298] = 0;
   out_5546184258405748350[299] = 0;
   out_5546184258405748350[300] = 0;
   out_5546184258405748350[301] = 0;
   out_5546184258405748350[302] = 0;
   out_5546184258405748350[303] = 0;
   out_5546184258405748350[304] = 1;
   out_5546184258405748350[305] = 0;
   out_5546184258405748350[306] = 0;
   out_5546184258405748350[307] = 0;
   out_5546184258405748350[308] = 0;
   out_5546184258405748350[309] = 0;
   out_5546184258405748350[310] = 0;
   out_5546184258405748350[311] = 0;
   out_5546184258405748350[312] = 0;
   out_5546184258405748350[313] = 0;
   out_5546184258405748350[314] = 0;
   out_5546184258405748350[315] = 0;
   out_5546184258405748350[316] = 0;
   out_5546184258405748350[317] = 0;
   out_5546184258405748350[318] = 0;
   out_5546184258405748350[319] = 0;
   out_5546184258405748350[320] = 0;
   out_5546184258405748350[321] = 0;
   out_5546184258405748350[322] = 0;
   out_5546184258405748350[323] = 1;
}
void h_4(double *state, double *unused, double *out_1928798689699948695) {
   out_1928798689699948695[0] = state[6] + state[9];
   out_1928798689699948695[1] = state[7] + state[10];
   out_1928798689699948695[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_8528763386588516166) {
   out_8528763386588516166[0] = 0;
   out_8528763386588516166[1] = 0;
   out_8528763386588516166[2] = 0;
   out_8528763386588516166[3] = 0;
   out_8528763386588516166[4] = 0;
   out_8528763386588516166[5] = 0;
   out_8528763386588516166[6] = 1;
   out_8528763386588516166[7] = 0;
   out_8528763386588516166[8] = 0;
   out_8528763386588516166[9] = 1;
   out_8528763386588516166[10] = 0;
   out_8528763386588516166[11] = 0;
   out_8528763386588516166[12] = 0;
   out_8528763386588516166[13] = 0;
   out_8528763386588516166[14] = 0;
   out_8528763386588516166[15] = 0;
   out_8528763386588516166[16] = 0;
   out_8528763386588516166[17] = 0;
   out_8528763386588516166[18] = 0;
   out_8528763386588516166[19] = 0;
   out_8528763386588516166[20] = 0;
   out_8528763386588516166[21] = 0;
   out_8528763386588516166[22] = 0;
   out_8528763386588516166[23] = 0;
   out_8528763386588516166[24] = 0;
   out_8528763386588516166[25] = 1;
   out_8528763386588516166[26] = 0;
   out_8528763386588516166[27] = 0;
   out_8528763386588516166[28] = 1;
   out_8528763386588516166[29] = 0;
   out_8528763386588516166[30] = 0;
   out_8528763386588516166[31] = 0;
   out_8528763386588516166[32] = 0;
   out_8528763386588516166[33] = 0;
   out_8528763386588516166[34] = 0;
   out_8528763386588516166[35] = 0;
   out_8528763386588516166[36] = 0;
   out_8528763386588516166[37] = 0;
   out_8528763386588516166[38] = 0;
   out_8528763386588516166[39] = 0;
   out_8528763386588516166[40] = 0;
   out_8528763386588516166[41] = 0;
   out_8528763386588516166[42] = 0;
   out_8528763386588516166[43] = 0;
   out_8528763386588516166[44] = 1;
   out_8528763386588516166[45] = 0;
   out_8528763386588516166[46] = 0;
   out_8528763386588516166[47] = 1;
   out_8528763386588516166[48] = 0;
   out_8528763386588516166[49] = 0;
   out_8528763386588516166[50] = 0;
   out_8528763386588516166[51] = 0;
   out_8528763386588516166[52] = 0;
   out_8528763386588516166[53] = 0;
}
void h_10(double *state, double *unused, double *out_3989753312953530471) {
   out_3989753312953530471[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_3989753312953530471[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_3989753312953530471[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_6637515270266097066) {
   out_6637515270266097066[0] = 0;
   out_6637515270266097066[1] = 9.8100000000000005*cos(state[1]);
   out_6637515270266097066[2] = 0;
   out_6637515270266097066[3] = 0;
   out_6637515270266097066[4] = -state[8];
   out_6637515270266097066[5] = state[7];
   out_6637515270266097066[6] = 0;
   out_6637515270266097066[7] = state[5];
   out_6637515270266097066[8] = -state[4];
   out_6637515270266097066[9] = 0;
   out_6637515270266097066[10] = 0;
   out_6637515270266097066[11] = 0;
   out_6637515270266097066[12] = 1;
   out_6637515270266097066[13] = 0;
   out_6637515270266097066[14] = 0;
   out_6637515270266097066[15] = 1;
   out_6637515270266097066[16] = 0;
   out_6637515270266097066[17] = 0;
   out_6637515270266097066[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_6637515270266097066[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_6637515270266097066[20] = 0;
   out_6637515270266097066[21] = state[8];
   out_6637515270266097066[22] = 0;
   out_6637515270266097066[23] = -state[6];
   out_6637515270266097066[24] = -state[5];
   out_6637515270266097066[25] = 0;
   out_6637515270266097066[26] = state[3];
   out_6637515270266097066[27] = 0;
   out_6637515270266097066[28] = 0;
   out_6637515270266097066[29] = 0;
   out_6637515270266097066[30] = 0;
   out_6637515270266097066[31] = 1;
   out_6637515270266097066[32] = 0;
   out_6637515270266097066[33] = 0;
   out_6637515270266097066[34] = 1;
   out_6637515270266097066[35] = 0;
   out_6637515270266097066[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_6637515270266097066[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_6637515270266097066[38] = 0;
   out_6637515270266097066[39] = -state[7];
   out_6637515270266097066[40] = state[6];
   out_6637515270266097066[41] = 0;
   out_6637515270266097066[42] = state[4];
   out_6637515270266097066[43] = -state[3];
   out_6637515270266097066[44] = 0;
   out_6637515270266097066[45] = 0;
   out_6637515270266097066[46] = 0;
   out_6637515270266097066[47] = 0;
   out_6637515270266097066[48] = 0;
   out_6637515270266097066[49] = 0;
   out_6637515270266097066[50] = 1;
   out_6637515270266097066[51] = 0;
   out_6637515270266097066[52] = 0;
   out_6637515270266097066[53] = 1;
}
void h_13(double *state, double *unused, double *out_1637769040693296482) {
   out_1637769040693296482[0] = state[3];
   out_1637769040693296482[1] = state[4];
   out_1637769040693296482[2] = state[5];
}
void H_13(double *state, double *unused, double *out_2307349478804334521) {
   out_2307349478804334521[0] = 0;
   out_2307349478804334521[1] = 0;
   out_2307349478804334521[2] = 0;
   out_2307349478804334521[3] = 1;
   out_2307349478804334521[4] = 0;
   out_2307349478804334521[5] = 0;
   out_2307349478804334521[6] = 0;
   out_2307349478804334521[7] = 0;
   out_2307349478804334521[8] = 0;
   out_2307349478804334521[9] = 0;
   out_2307349478804334521[10] = 0;
   out_2307349478804334521[11] = 0;
   out_2307349478804334521[12] = 0;
   out_2307349478804334521[13] = 0;
   out_2307349478804334521[14] = 0;
   out_2307349478804334521[15] = 0;
   out_2307349478804334521[16] = 0;
   out_2307349478804334521[17] = 0;
   out_2307349478804334521[18] = 0;
   out_2307349478804334521[19] = 0;
   out_2307349478804334521[20] = 0;
   out_2307349478804334521[21] = 0;
   out_2307349478804334521[22] = 1;
   out_2307349478804334521[23] = 0;
   out_2307349478804334521[24] = 0;
   out_2307349478804334521[25] = 0;
   out_2307349478804334521[26] = 0;
   out_2307349478804334521[27] = 0;
   out_2307349478804334521[28] = 0;
   out_2307349478804334521[29] = 0;
   out_2307349478804334521[30] = 0;
   out_2307349478804334521[31] = 0;
   out_2307349478804334521[32] = 0;
   out_2307349478804334521[33] = 0;
   out_2307349478804334521[34] = 0;
   out_2307349478804334521[35] = 0;
   out_2307349478804334521[36] = 0;
   out_2307349478804334521[37] = 0;
   out_2307349478804334521[38] = 0;
   out_2307349478804334521[39] = 0;
   out_2307349478804334521[40] = 0;
   out_2307349478804334521[41] = 1;
   out_2307349478804334521[42] = 0;
   out_2307349478804334521[43] = 0;
   out_2307349478804334521[44] = 0;
   out_2307349478804334521[45] = 0;
   out_2307349478804334521[46] = 0;
   out_2307349478804334521[47] = 0;
   out_2307349478804334521[48] = 0;
   out_2307349478804334521[49] = 0;
   out_2307349478804334521[50] = 0;
   out_2307349478804334521[51] = 0;
   out_2307349478804334521[52] = 0;
   out_2307349478804334521[53] = 0;
}
void h_14(double *state, double *unused, double *out_2199265346921650482) {
   out_2199265346921650482[0] = state[6];
   out_2199265346921650482[1] = state[7];
   out_2199265346921650482[2] = state[8];
}
void H_14(double *state, double *unused, double *out_5445974954293143870) {
   out_5445974954293143870[0] = 0;
   out_5445974954293143870[1] = 0;
   out_5445974954293143870[2] = 0;
   out_5445974954293143870[3] = 0;
   out_5445974954293143870[4] = 0;
   out_5445974954293143870[5] = 0;
   out_5445974954293143870[6] = 1;
   out_5445974954293143870[7] = 0;
   out_5445974954293143870[8] = 0;
   out_5445974954293143870[9] = 0;
   out_5445974954293143870[10] = 0;
   out_5445974954293143870[11] = 0;
   out_5445974954293143870[12] = 0;
   out_5445974954293143870[13] = 0;
   out_5445974954293143870[14] = 0;
   out_5445974954293143870[15] = 0;
   out_5445974954293143870[16] = 0;
   out_5445974954293143870[17] = 0;
   out_5445974954293143870[18] = 0;
   out_5445974954293143870[19] = 0;
   out_5445974954293143870[20] = 0;
   out_5445974954293143870[21] = 0;
   out_5445974954293143870[22] = 0;
   out_5445974954293143870[23] = 0;
   out_5445974954293143870[24] = 0;
   out_5445974954293143870[25] = 1;
   out_5445974954293143870[26] = 0;
   out_5445974954293143870[27] = 0;
   out_5445974954293143870[28] = 0;
   out_5445974954293143870[29] = 0;
   out_5445974954293143870[30] = 0;
   out_5445974954293143870[31] = 0;
   out_5445974954293143870[32] = 0;
   out_5445974954293143870[33] = 0;
   out_5445974954293143870[34] = 0;
   out_5445974954293143870[35] = 0;
   out_5445974954293143870[36] = 0;
   out_5445974954293143870[37] = 0;
   out_5445974954293143870[38] = 0;
   out_5445974954293143870[39] = 0;
   out_5445974954293143870[40] = 0;
   out_5445974954293143870[41] = 0;
   out_5445974954293143870[42] = 0;
   out_5445974954293143870[43] = 0;
   out_5445974954293143870[44] = 1;
   out_5445974954293143870[45] = 0;
   out_5445974954293143870[46] = 0;
   out_5445974954293143870[47] = 0;
   out_5445974954293143870[48] = 0;
   out_5445974954293143870[49] = 0;
   out_5445974954293143870[50] = 0;
   out_5445974954293143870[51] = 0;
   out_5445974954293143870[52] = 0;
   out_5445974954293143870[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_4197240932685800719) {
  err_fun(nom_x, delta_x, out_4197240932685800719);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_8091385002608574816) {
  inv_err_fun(nom_x, true_x, out_8091385002608574816);
}
void pose_H_mod_fun(double *state, double *out_209432845494472972) {
  H_mod_fun(state, out_209432845494472972);
}
void pose_f_fun(double *state, double dt, double *out_7558387581431867623) {
  f_fun(state,  dt, out_7558387581431867623);
}
void pose_F_fun(double *state, double dt, double *out_5546184258405748350) {
  F_fun(state,  dt, out_5546184258405748350);
}
void pose_h_4(double *state, double *unused, double *out_1928798689699948695) {
  h_4(state, unused, out_1928798689699948695);
}
void pose_H_4(double *state, double *unused, double *out_8528763386588516166) {
  H_4(state, unused, out_8528763386588516166);
}
void pose_h_10(double *state, double *unused, double *out_3989753312953530471) {
  h_10(state, unused, out_3989753312953530471);
}
void pose_H_10(double *state, double *unused, double *out_6637515270266097066) {
  H_10(state, unused, out_6637515270266097066);
}
void pose_h_13(double *state, double *unused, double *out_1637769040693296482) {
  h_13(state, unused, out_1637769040693296482);
}
void pose_H_13(double *state, double *unused, double *out_2307349478804334521) {
  H_13(state, unused, out_2307349478804334521);
}
void pose_h_14(double *state, double *unused, double *out_2199265346921650482) {
  h_14(state, unused, out_2199265346921650482);
}
void pose_H_14(double *state, double *unused, double *out_5445974954293143870) {
  H_14(state, unused, out_5445974954293143870);
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
