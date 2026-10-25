/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <pico/stdio_usb.h>
#include <pico/time.h>
#include <stdio.h>

#include "ddr_spi.h"
#include "ddr_spi.pio.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "rhd_helpers.h"

#define DDR_MOSI_GPIO 19
#define DDR_SCLK_GPIO 18
#define DDR_CS_GPIO 17
#define DDR_MISO_GPIO 16

int main() {
  stdio_usb_init();

  sleep_ms(1000);
  printf("Starting in 2 secs....\n");
  sleep_ms(2000);

  PIO tx_pio, rx_pio;
  uint tx_sm, rx_sm;
  uint tx_off, rx_off;

  // Find a free PIO and state machine and add the program
  bool rc = pio_claim_free_sm_and_add_program_for_gpio_range(
      &spi_ddr_tx_sclk_program, &tx_pio, &tx_sm, &tx_off, DDR_SCLK_GPIO, 2,
      true);
  hard_assert(rc);
  printf("Loaded TX program at %u on pio %u, sm:%u\n", tx_off, PIO_NUM(tx_pio),
         tx_sm);

  rc = pio_claim_free_sm_and_add_program_for_gpio_range(
      &spi_ddr_rx_cs_program, &rx_pio, &rx_sm, &rx_off, DDR_MISO_GPIO, 2, true);
  hard_assert(rc);
  printf("Loaded RX program at %u on pio %u, sm:%u\n", rx_off, PIO_NUM(rx_pio),
         rx_sm);

  // Start the SMs and get a fella to hold everything in one place
  ddr_spi_inst_t fella =
      ddr_spi_init(tx_pio, rx_pio, tx_sm, tx_off, rx_sm, rx_off, DDR_MOSI_GPIO,
                   DDR_MISO_GPIO, DDR_CS_GPIO, DDR_SCLK_GPIO);

  rhd_2164_obj_t rhd = rhd_2164_obj(&fella);
  rhd.calibrate_rhd();

  uint16_t in_all[64];

  while (true) {
    rhd.convert_all(in_all);
    printf("%u\n", in_all[46]);

    sleep_ms(1);
  }
}
