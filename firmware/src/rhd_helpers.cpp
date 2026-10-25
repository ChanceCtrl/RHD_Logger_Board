#include "rhd_helpers.h"
#include "ddr_spi.h"

#define CALIBRATE_RHD_COMMAND_LEN 10
#define READ_WRITE_RHD_COMMAND_LEN 3
#define CONVERT_ALL_RHD_COMMAND_LEN 34

uint16_t extract_rhd_b_bits(uint32_t x) {
  x &= 0x55555555u;
  x = (x | (x >> 1)) & 0x33333333u;
  x = (x | (x >> 2)) & 0x0F0F0F0Fu;
  x = (x | (x >> 4)) & 0x00FF00FFu;
  x = (x | (x >> 8)) & 0x0000FFFFu;
  return (uint16_t)x;
}

uint16_t extract_rhd_a_bits(uint32_t x) { return extract_rhd_b_bits(x >> 1); }

rhd_2164_obj_t::rhd_2164_obj(const ddr_spi_inst_t *ddr) { this->ddr = ddr; };

void rhd_2164_obj::calibrate_rhd() {
  uint16_t command_list[CALIBRATE_RHD_COMMAND_LEN] = {
      RHD_CALIBRATE_COMMAND_HEX, RHD_DO_SOMETHING_HEX, RHD_DO_SOMETHING_HEX,
      RHD_DO_SOMETHING_HEX,      RHD_DO_SOMETHING_HEX, RHD_DO_SOMETHING_HEX,
      RHD_DO_SOMETHING_HEX,      RHD_DO_SOMETHING_HEX, RHD_DO_SOMETHING_HEX,
      RHD_DO_SOMETHING_HEX,
  };

  uint32_t rhd_response[CALIBRATE_RHD_COMMAND_LEN];

  ddr_transact(this->ddr, command_list, rhd_response,
               CALIBRATE_RHD_COMMAND_LEN);
}

void rhd_2164_obj::convert_all(uint16_t responses[64]) {
  uint16_t command_list[CONVERT_ALL_RHD_COMMAND_LEN];
  uint32_t response_list[CONVERT_ALL_RHD_COMMAND_LEN];

  // We get 64 channels from 32 commands due to the DDR nature, RHD A contains
  // channel x, and RHD B contains channel x + 32
  for (uint8_t x = 0; x < 32; x++) {
    command_list[x] = RHD_CONVERT_COMMAND_HEX;
    command_list[x] |= x << 8; // 00CC CCCC  0000 0000
  }

  // Two filler commands to get all 32 transactions in 1 array
  command_list[CONVERT_ALL_RHD_COMMAND_LEN - 2] = RHD_DO_SOMETHING_HEX;
  command_list[CONVERT_ALL_RHD_COMMAND_LEN - 1] = RHD_DO_SOMETHING_HEX;

  ddr_transact(this->ddr, command_list, response_list,
               CONVERT_ALL_RHD_COMMAND_LEN);

  // The first 2 commands the response list are going to be garbage due to the 2
  // command pipeline, so skip them
  for (uint8_t x = 2; x < CONVERT_ALL_RHD_COMMAND_LEN; x++) {
    // See above notes for these array offset equations
    responses[(x - 2)] = extract_rhd_a_bits(response_list[x]);
    responses[(x - 2) + 32] = extract_rhd_b_bits(response_list[x]);
  }
}

// void rhd_2164_obj::convert_selected(uint8_t *channel_select,
//                                     uint16_t *responses,
//                                     uint8_t channel_count) {
//   uint16_t command_list[channel_count] = {RHD_CONVERT_COMMAND_HEX};
// }

void rhd_2164_obj::read_reg(uint8_t reg, uint16_t *rhd_a_response,
                            uint16_t *rhd_b_response) {
  if (reg > 63) {
    return; // Break on passing a reg too high to prevent the bitshift bitwise
            // OR from doing anything funny
  }

  uint16_t command_list[READ_WRITE_RHD_COMMAND_LEN] = {
      RHD_READ_COMMAND_HEX, RHD_DO_SOMETHING_HEX, RHD_DO_SOMETHING_HEX};
  uint32_t rhd_response[READ_WRITE_RHD_COMMAND_LEN];

  command_list[0] |= reg << 8; // 11RR RRRR  0000 0000

  ddr_transact(this->ddr, command_list, rhd_response,
               CALIBRATE_RHD_COMMAND_LEN);

  *rhd_a_response =
      extract_rhd_a_bits(rhd_response[READ_WRITE_RHD_COMMAND_LEN - 1]);

  *rhd_b_response =
      extract_rhd_b_bits(rhd_response[READ_WRITE_RHD_COMMAND_LEN - 1]);
}

void rhd_2164_obj::write_reg(uint8_t reg, uint8_t data,
                             uint16_t *rhd_a_response,
                             uint16_t *rhd_b_response) {
  if (reg > 63) {
    return; // Break on passing a reg too high to prevent the bitshift bitwise
            // OR from doing anything funny
  }

  uint16_t command_list[READ_WRITE_RHD_COMMAND_LEN] = {
      RHD_WRITE_COMMAND_HEX, RHD_DO_SOMETHING_HEX, RHD_DO_SOMETHING_HEX};
  uint32_t rhd_response[READ_WRITE_RHD_COMMAND_LEN];

  command_list[0] |= reg << 8; // 10RR RRRR  0000 0000
  command_list[0] |= data;     // 10RR RRRR  DDDD DDDD

  ddr_transact(this->ddr, command_list, rhd_response,
               CALIBRATE_RHD_COMMAND_LEN);

  *rhd_a_response =
      extract_rhd_a_bits(rhd_response[READ_WRITE_RHD_COMMAND_LEN - 1]);

  *rhd_b_response =
      extract_rhd_b_bits(rhd_response[READ_WRITE_RHD_COMMAND_LEN - 1]);
}

void rhd_2164_obj::raw_command(RHDCommon_CommandType_enum_t command,
                               uint8_t reg, uint8_t data,
                               uint16_t *rhd_a_response,
                               uint16_t *rhd_b_response) {
  uint16_t command_half_word;
  switch (command) {
  case RHDCommon_CommandType_Convert:
    command_half_word = RHD_CONVERT_COMMAND_HEX;

    command_half_word |= reg << 8; // 00CC CCCC  0000 0000

    // Mask for the first bit here to prevent any wackyness
    command_half_word |= data & 0x1; // 00CC CCCC  0000 000H
    break;

  case RHDCommon_CommandType_Calibrate:
    command_half_word = RHD_CALIBRATE_COMMAND_HEX;
    break;

  case RHDCommon_CommandType_ClearCalibration:
    command_half_word = RHD_CLEAR_CALIBRATION_HEX;
    break;

  case RHDCommon_CommandType_WriteRegister:
    command_half_word = RHD_WRITE_COMMAND_HEX;

    command_half_word |= reg << 8; // 10RR RRRR  0000 0000
    command_half_word |= data;     // 10RR RRRR  DDDD DDDD
    break;

  case RHDCommon_CommandType_ReadRegister:
    command_half_word = RHD_READ_COMMAND_HEX;

    command_half_word |= reg << 8; // 11RR RRRR  0000 0000
    break;
  }

  uint32_t rhd_response;

  ddr_transact(this->ddr, &command_half_word, &rhd_response, 1);

  *rhd_a_response = extract_rhd_a_bits(rhd_response);

  *rhd_b_response = extract_rhd_b_bits(rhd_response);
}
