#include "common_dbc.h"

namespace {

const Signal sigs_512[] = {
    {
      .name = "TORQUE_L",
      .b1 = 0,
      .b2 = 16,
      .bo = 48,
      .is_signed = false,
      .factor = 1,
      .offset = -32768.0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "TORQUE_R",
      .b1 = 16,
      .b2 = 16,
      .bo = 32,
      .is_signed = false,
      .factor = 1,
      .offset = -32768.0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
};
const Signal sigs_513[] = {
    {
      .name = "SPEED_L",
      .b1 = 0,
      .b2 = 16,
      .bo = 48,
      .is_signed = false,
      .factor = 1,
      .offset = -32768.0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "SPEED_R",
      .b1 = 16,
      .b2 = 16,
      .bo = 32,
      .is_signed = false,
      .factor = 1,
      .offset = -32768.0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "BAT_VOLTAGE",
      .b1 = 48,
      .b2 = 16,
      .bo = 0,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
};

const Msg msgs[] = {
  {
    .name = "BODY_COMMAND",
    .address = 0x200,
    .size = 4,
    .num_sigs = ARRAYSIZE(sigs_512),
    .sigs = sigs_512,
  },
  {
    .name = "BODY_SENSOR",
    .address = 0x201,
    .size = 8,
    .num_sigs = ARRAYSIZE(sigs_513),
    .sigs = sigs_513,
  },
};

const Val vals[] = {
};

}

const DBC comma_body = {
  .name = "comma_body",
  .num_msgs = ARRAYSIZE(msgs),
  .msgs = msgs,
  .vals = vals,
  .num_vals = ARRAYSIZE(vals),
};

dbc_init(comma_body)