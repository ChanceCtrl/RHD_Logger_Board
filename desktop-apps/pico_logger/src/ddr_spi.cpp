#include "ddr_spi.h"
#include "ddr_spi.pio.h"
#include <hardware/pio.h>

ddr_spi_inst_t ddr_spi_init(PIO tx_pio, PIO rx_pio, uint tx_sm, uint tx_offset,
                            uint rx_sm, uint rx_offset, uint mosi_pin,
                            uint miso_pin, uint cs_pin, uint sclk_pin) {
  // Set up the TX state machine
  pio_gpio_init(tx_pio, mosi_pin);
  pio_gpio_init(tx_pio, sclk_pin);

  pio_sm_config tx_conf = spi_ddr_tx_sclk_program_get_default_config(tx_offset);

  sm_config_set_out_pins(&tx_conf, mosi_pin, 1);
  sm_config_set_sideset_pins(&tx_conf, sclk_pin);

  pio_sm_init(tx_pio, tx_sm, tx_offset, &tx_conf);

  pio_sm_set_consecutive_pindirs(tx_pio, tx_sm, mosi_pin, 1, true);
  pio_sm_set_consecutive_pindirs(tx_pio, tx_sm, sclk_pin, 1, true);

  // Set up the RX state machine
  pio_gpio_init(rx_pio, miso_pin);
  pio_gpio_init(rx_pio, cs_pin);

  pio_sm_config rx_conf = spi_ddr_rx_cs_program_get_default_config(rx_offset);

  sm_config_set_in_pins(&rx_conf, miso_pin);
  sm_config_set_sideset_pins(&rx_conf, cs_pin);
  // sm_config_set_set_pins(&rx_conf, cs_pin, 1);

  pio_sm_init(rx_pio, rx_sm, rx_offset, &rx_conf);

  pio_sm_set_consecutive_pindirs(rx_pio, rx_sm, cs_pin, 1, true);

  // Enable both
  pio_sm_set_enabled(tx_pio, tx_sm, true);
  pio_sm_set_enabled(rx_pio, rx_sm, true);

  return ddr_spi_inst_t{
      tx_pio = tx_pio, tx_sm = tx_sm, tx_offset = tx_offset,
      rx_pio = rx_pio, rx_sm = rx_sm, rx_offset = rx_offset,
  };
}

#include <pico/stdio_usb.h>
#include <pico/time.h>
#include <stdio.h>

void ddr_transact(const ddr_spi_inst_t *ddr, uint32_t *src, uint32_t *dst,
                  size_t len) {
  // Set the size to the len for both tx and rx for counters
  size_t tx_remain = len, rx_remain = len;

  // The tx buffer should only be 16 bits long
  io_rw_32 *txfifo = (io_rw_32 *)&ddr->tx_pio->txf[ddr->tx_sm];

  // The rx buffer should be 32 bits, 16 for RHD_A and 16 for RHD_B
  io_rw_32 *rxfifo = (io_rw_32 *)&ddr->rx_pio->rxf[ddr->rx_sm];

  // Loop until the tx and rx buffers are done
  while (tx_remain || rx_remain) {
    if (tx_remain && !pio_sm_is_tx_fifo_full(ddr->tx_pio, ddr->tx_sm)) {
      *txfifo = *src++;
      --tx_remain;
    }
    if (rx_remain && !pio_sm_is_rx_fifo_empty(ddr->rx_pio, ddr->rx_sm)) {
      *dst++ = *rxfifo;
      --rx_remain;
    }
    sleep_ms(1);
  }
}
