#include "config.h"

// For things like printf, sleep, etc.
#include <cstdio>
#include <hardware/timer.h>
#include <pico/assert.h>
#include <pico/cyw43_arch.h>
#include <pico/stdio.h>
#include <pico/stdio_usb.h>
#include <pico/stdlib.h>
#include <pico/time.h>

// Everything for the DDR spi stuff
#include "ddr_spi.h"
#include "ddr_spi.pio.h"
#include "rhd_helpers.h"
#include <hardware/pio.h>

// Our packet yeeting utiliti
#include "fb_helpers.h"
#include "server.h"

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

  printf("RHD started, opening AP....\n");
  int server_result = init_server();
  hard_assert(server_result == 0);

  uint16_t in_all[64];

  while (true) {
    // Get the data
    uint64_t time_start = time_us_64();
    rhd.convert_all(in_all);
    uint64_t time_end = time_us_64();

    // Package it all up neatly
    // rhd_packet_common_t packet =
    //     make_conv_all_packet(time_start, in_all, time_end);

    rhd_packet_common_t packet;
    packet.packet_buffer = in_all;
    packet.packet_size = 64;

    // Throw it out
    send_packet(packet);

    printf("SOL\n");

    // printf("mrow %u\n", (time_us_64() - time_end));
    sleep_ms(1000);
  }
}
