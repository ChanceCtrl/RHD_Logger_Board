#pragma once

#include "ddr_spi.h"
#include "flatbuffers/rhd_common_reader.h"

#define RHD_CALIBRATE_COMMAND_HEX 0x5500 // 0101 0101  0000 0000
#define RHD_CLEAR_CALIBRATION_HEX 0x6A00 // 0110 1010  0000 0000
#define RHD_CONVERT_COMMAND_HEX 0x0000   // 0000 0000  0000 0000
#define RHD_WRITE_COMMAND_HEX 0x8000     // 1000 0000  0000 0000
#define RHD_READ_COMMAND_HEX 0xC000      // 1100 0000  0000 0000
#define RHD_DO_SOMETHING_HEX 0xEB00      // 1110 1011  0000 0000
// ^ This just reads reg 40, which is just the intan I in ascii

typedef class rhd_2164_obj {
private:
  const ddr_spi_inst_t *ddr;

public:
  rhd_2164_obj(const ddr_spi_inst_t *ddr);

  void calibrate_rhd();

  void convert_all(uint16_t responses[64]);

  void convert_selected(uint8_t *channel_select, uint16_t *responses,
                        uint8_t channel_count);

  void read_reg(uint8_t reg, uint16_t *rhd_a_response,
                uint16_t *rhd_b_response);

  void write_reg(uint8_t reg, uint8_t data, uint16_t *rhd_a_response,
                 uint16_t *rhd_b_response);

  void raw_command(RHDCommon_CommandType_enum_t command, uint8_t reg,
                   uint8_t data, uint16_t *rhd_a_response,
                   uint16_t *rhd_b_response);
} rhd_2164_obj_t;
