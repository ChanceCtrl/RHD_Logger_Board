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

#define DDR_MOSI_GPIO 19
#define DDR_SCLK_GPIO 18
#define DDR_CS_GPIO 17
#define DDR_MISO_GPIO 16

uint16_t extract_rhd_b_bits(uint32_t x) {
  x &= 0x55555555u;
  x = (x | (x >> 1)) & 0x33333333u;
  x = (x | (x >> 2)) & 0x0F0F0F0Fu;
  x = (x | (x >> 4)) & 0x00FF00FFu;
  x = (x | (x >> 8)) & 0x0000FFFFu;
  return (uint16_t)x;
}

uint16_t extract_rhd_a_bits(uint32_t x) { return extract_rhd_b_bits(x >> 1); }

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

  while (true) {
    uint16_t tx_buffer[5] = {0xEB00, 0xE900, 0xE800, 0xE900, 0xEA00};
    // uint16_t tx_buffer[5] = {0x00FB, 0xE900, 0xEA00, 0xEB00, 0xE900};
    uint32_t rx_buffer[5];
    uint16_t rhd_a, rhd_b;

    ddr_transact(&fella, tx_buffer, rx_buffer, 5);

    for (int i = 0; i < 5; i++) {
      rhd_a = extract_rhd_a_bits(rx_buffer[i]);
      rhd_b = extract_rhd_b_bits(rx_buffer[i]);
      printf("Sent %u, got rhd_a: %u\trhd_b: %u\n", tx_buffer[i], rhd_a, rhd_b);
    }

    sleep_ms(1000);
  }
}
