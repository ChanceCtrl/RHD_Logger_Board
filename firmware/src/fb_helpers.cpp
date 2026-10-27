#include "fb_helpers.h"
#include "flatbuffers/rhd_responses_builder.h"

rhd_packet_common_t make_conv_all_packet(uint64_t timestamp_start_us,
                                         uint16_t conv_data[64],
                                         uint64_t timestamp_end_us) {
  rhd_packet_common_t joe_packet;
  flatcc_builder_t B;

  flatcc_builder_init(&B);

  RHDResponses_ConversionResult_start_as_root(&B);

  // Timestamp start field
  RHDResponses_ConversionResult_time_conv_start_us_add(&B, timestamp_start_us);

  // The readings, man this is cancer
  RHDResponses_ConversionResult_channel_00_add(&B, conv_data[0]);
  RHDResponses_ConversionResult_channel_01_add(&B, conv_data[1]);
  RHDResponses_ConversionResult_channel_02_add(&B, conv_data[2]);
  RHDResponses_ConversionResult_channel_03_add(&B, conv_data[3]);
  RHDResponses_ConversionResult_channel_04_add(&B, conv_data[4]);
  RHDResponses_ConversionResult_channel_05_add(&B, conv_data[5]);
  RHDResponses_ConversionResult_channel_06_add(&B, conv_data[6]);
  RHDResponses_ConversionResult_channel_07_add(&B, conv_data[7]);
  RHDResponses_ConversionResult_channel_08_add(&B, conv_data[8]);
  RHDResponses_ConversionResult_channel_09_add(&B, conv_data[9]);
  RHDResponses_ConversionResult_channel_10_add(&B, conv_data[10]);
  RHDResponses_ConversionResult_channel_11_add(&B, conv_data[11]);
  RHDResponses_ConversionResult_channel_12_add(&B, conv_data[12]);
  RHDResponses_ConversionResult_channel_13_add(&B, conv_data[13]);
  RHDResponses_ConversionResult_channel_14_add(&B, conv_data[14]);
  RHDResponses_ConversionResult_channel_15_add(&B, conv_data[15]);
  RHDResponses_ConversionResult_channel_16_add(&B, conv_data[16]);
  RHDResponses_ConversionResult_channel_17_add(&B, conv_data[17]);
  RHDResponses_ConversionResult_channel_18_add(&B, conv_data[18]);
  RHDResponses_ConversionResult_channel_19_add(&B, conv_data[19]);
  RHDResponses_ConversionResult_channel_20_add(&B, conv_data[20]);
  RHDResponses_ConversionResult_channel_21_add(&B, conv_data[21]);
  RHDResponses_ConversionResult_channel_22_add(&B, conv_data[22]);
  RHDResponses_ConversionResult_channel_23_add(&B, conv_data[23]);
  RHDResponses_ConversionResult_channel_24_add(&B, conv_data[24]);
  RHDResponses_ConversionResult_channel_25_add(&B, conv_data[25]);
  RHDResponses_ConversionResult_channel_26_add(&B, conv_data[26]);
  RHDResponses_ConversionResult_channel_27_add(&B, conv_data[27]);
  RHDResponses_ConversionResult_channel_28_add(&B, conv_data[28]);
  RHDResponses_ConversionResult_channel_29_add(&B, conv_data[29]);
  RHDResponses_ConversionResult_channel_30_add(&B, conv_data[30]);
  RHDResponses_ConversionResult_channel_31_add(&B, conv_data[31]);
  RHDResponses_ConversionResult_channel_32_add(&B, conv_data[32]);
  RHDResponses_ConversionResult_channel_33_add(&B, conv_data[33]);
  RHDResponses_ConversionResult_channel_34_add(&B, conv_data[34]);
  RHDResponses_ConversionResult_channel_35_add(&B, conv_data[35]);
  RHDResponses_ConversionResult_channel_36_add(&B, conv_data[36]);
  RHDResponses_ConversionResult_channel_37_add(&B, conv_data[37]);
  RHDResponses_ConversionResult_channel_38_add(&B, conv_data[38]);
  RHDResponses_ConversionResult_channel_39_add(&B, conv_data[39]);
  RHDResponses_ConversionResult_channel_40_add(&B, conv_data[40]);
  RHDResponses_ConversionResult_channel_41_add(&B, conv_data[41]);
  RHDResponses_ConversionResult_channel_42_add(&B, conv_data[42]);
  RHDResponses_ConversionResult_channel_43_add(&B, conv_data[43]);
  RHDResponses_ConversionResult_channel_44_add(&B, conv_data[44]);
  RHDResponses_ConversionResult_channel_45_add(&B, conv_data[45]);
  RHDResponses_ConversionResult_channel_46_add(&B, conv_data[46]);
  RHDResponses_ConversionResult_channel_47_add(&B, conv_data[47]);
  RHDResponses_ConversionResult_channel_48_add(&B, conv_data[48]);
  RHDResponses_ConversionResult_channel_49_add(&B, conv_data[49]);
  RHDResponses_ConversionResult_channel_50_add(&B, conv_data[50]);
  RHDResponses_ConversionResult_channel_51_add(&B, conv_data[51]);
  RHDResponses_ConversionResult_channel_52_add(&B, conv_data[52]);
  RHDResponses_ConversionResult_channel_53_add(&B, conv_data[53]);
  RHDResponses_ConversionResult_channel_54_add(&B, conv_data[54]);
  RHDResponses_ConversionResult_channel_55_add(&B, conv_data[55]);
  RHDResponses_ConversionResult_channel_56_add(&B, conv_data[56]);
  RHDResponses_ConversionResult_channel_57_add(&B, conv_data[57]);
  RHDResponses_ConversionResult_channel_58_add(&B, conv_data[58]);
  RHDResponses_ConversionResult_channel_59_add(&B, conv_data[59]);
  RHDResponses_ConversionResult_channel_60_add(&B, conv_data[60]);
  RHDResponses_ConversionResult_channel_61_add(&B, conv_data[61]);
  RHDResponses_ConversionResult_channel_62_add(&B, conv_data[62]);
  RHDResponses_ConversionResult_channel_63_add(&B, conv_data[63]);

  // Close out the table
  RHDResponses_ConversionResult_end_as_root(&B);

  // Timestamp end field
  RHDResponses_ConversionResult_time_conv_end_us_add(&B, timestamp_end_us);

  joe_packet.packet_buffer =
      flatcc_builder_get_direct_buffer(&B, &joe_packet.packet_size);

  flatcc_builder_clear(&B);
}
