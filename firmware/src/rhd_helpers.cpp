#include "rhd_helpers.h"
#include "ddr_spi.h"

rhd_2164_conf_t init_rhd(const ddr_spi_inst_t *ddr);

#define CALIBRATE_RHD_COMMAND_LEN 10

void calibrate_rhd(const rhd_2164_conf_t *rhd) {
  uint16_t command_list[CALIBRATE_RHD_COMMAND_LEN] = {
      RHD_CALIBRATE_COMMAND_HEX, RHD_DO_SOMETHING_HEX, RHD_DO_SOMETHING_HEX,
      RHD_DO_SOMETHING_HEX,      RHD_DO_SOMETHING_HEX, RHD_DO_SOMETHING_HEX,
      RHD_DO_SOMETHING_HEX,      RHD_DO_SOMETHING_HEX, RHD_DO_SOMETHING_HEX,
      RHD_DO_SOMETHING_HEX,
  };

  uint32_t rhd_response[CALIBRATE_RHD_COMMAND_LEN];

  ddr_transact(rhd->ddr, command_list, rhd_response, CALIBRATE_RHD_COMMAND_LEN);
}

void convert_all(const rhd_2164_conf_t *rhd, uint16_t responses[64]);

void convert_selected(const rhd_2164_conf_t *rhd, uint64_t channel_select,
                      uint16_t *responses);

void read_reg(const rhd_2164_conf_t *rhd, uint16_t *rhd_a_response,
              uint16_t *rhd_b_response);

void write_reg(const rhd_2164_conf_t *rhd, uint16_t *rhd_a_response,
               uint16_t *rhd_b_response);

void raw_command(const rhd_2164_conf_t *rhd, uint16_t *rhd_a_response,
                 uint16_t *rhd_b_response);
