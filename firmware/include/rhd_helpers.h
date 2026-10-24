#pragma once

#include "ddr_spi.h"

#define RHD_CALIBRATE_COMMAND_HEX 0x5500 // 0101 0101  0000 0000
#define RHD_CLEAR_CALIBRATION_HEX 0x6A00 // 0110 1010  0000 0000
#define RHD_CONVERT_COMMAND_HEX 0x0000   // 0000 0000  0000 0000
#define RHD_WRITE_COMMAND_HEX 0x8000     // 1000 0000  0000 0000
#define RHD_READ_COMMAND_HEX 0xC000      // 1100 0000  0000 0000
#define RHD_DO_SOMETHING_HEX 0xEB00      // 1110 1011  0000 0000
// ^ This just reads reg 40, which is just the intan I in ascii

typedef struct rhd_2164_conf {
  const ddr_spi_inst_t *ddr;
} rhd_2164_conf_t;

rhd_2164_conf_t init_rhd(const ddr_spi_inst_t *ddr);

void calibrate_rhd(const rhd_2164_conf_t *rhd);

void convert_all(const rhd_2164_conf_t *rhd, uint16_t responses[64]);

void convert_selected(const rhd_2164_conf_t *rhd, uint64_t channel_select,
                      uint16_t *responses);

void read_reg(const rhd_2164_conf_t *rhd, uint16_t *rhd_a_response,
              uint16_t *rhd_b_response);

void write_reg(const rhd_2164_conf_t *rhd, uint16_t *rhd_a_response,
               uint16_t *rhd_b_response);

void raw_command(const rhd_2164_conf_t *rhd, uint16_t *rhd_a_response,
                 uint16_t *rhd_b_response);
