#pragma once

#include "config.h"
#include "flatbuffers/rhd_common_builder.h"
#include <pico/stdio.h>

typedef struct rhd_packet_common {
  void *packet_buffer;
  size_t packet_size;
  RHDCommon_CommandType_enum_t packet_type;
} rhd_packet_common_t;

rhd_packet_common_t make_conv_all_packet(uint64_t timestamp_start_us,
                                         uint16_t conv_data[64],
                                         uint64_t timestamp_end_us);
