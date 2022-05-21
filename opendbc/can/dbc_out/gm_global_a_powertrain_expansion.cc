#include "common_dbc.h"

namespace {

const Signal sigs_468[] = {
    {
      .name = "APM_Status",
      .b1 = 0,
      .b2 = 8,
      .bo = 56,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = true,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "APM_Voltage_Requested",
      .b1 = 8,
      .b2 = 8,
      .bo = 48,
      .is_signed = false,
      .factor = 0.0787402,
      .offset = 0,
      .is_little_endian = true,
      .type = SignalType::DEFAULT,
    },
};
const Signal sigs_470[] = {
    {
      .name = "APM_Status",
      .b1 = 0,
      .b2 = 8,
      .bo = 56,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "APM_High_Voltage_Input_Current",
      .b1 = 8,
      .b2 = 8,
      .bo = 48,
      .is_signed = true,
      .factor = 0.15,
      .offset = -7.0,
      .is_little_endian = true,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "APM_Low_Voltage_Sensed",
      .b1 = 16,
      .b2 = 8,
      .bo = 40,
      .is_signed = false,
      .factor = 0.0787402,
      .offset = 0,
      .is_little_endian = true,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "APM_Temperature_1",
      .b1 = 24,
      .b2 = 8,
      .bo = 32,
      .is_signed = false,
      .factor = 1,
      .offset = -40.0,
      .is_little_endian = true,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "APM_Temperature_2",
      .b1 = 32,
      .b2 = 8,
      .bo = 24,
      .is_signed = false,
      .factor = 1,
      .offset = -40.0,
      .is_little_endian = true,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "APM_Low_Voltage_Current_Output",
      .b1 = 40,
      .b2 = 8,
      .bo = 16,
      .is_signed = true,
      .factor = 1,
      .offset = 0,
      .is_little_endian = true,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "APM_Counter",
      .b1 = 48,
      .b2 = 8,
      .bo = 8,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = true,
      .type = SignalType::DEFAULT,
    },
};

const Msg msgs[] = {
  {
    .name = "APM_Command",
    .address = 0x1D4,
    .size = 2,
    .num_sigs = ARRAYSIZE(sigs_468),
    .sigs = sigs_468,
  },
  {
    .name = "APM_Stats",
    .address = 0x1D6,
    .size = 7,
    .num_sigs = ARRAYSIZE(sigs_470),
    .sigs = sigs_470,
  },
};

const Val vals[] = {
    {
      .name = "APM_Status",
      .address = 0x1D4,
      .def_val = "0 OFF 160 ON",
      .sigs = sigs_468,
    },
};

}

const DBC gm_global_a_powertrain_expansion = {
  .name = "gm_global_a_powertrain_expansion",
  .num_msgs = ARRAYSIZE(msgs),
  .msgs = msgs,
  .vals = vals,
  .num_vals = ARRAYSIZE(vals),
};

dbc_init(gm_global_a_powertrain_expansion)