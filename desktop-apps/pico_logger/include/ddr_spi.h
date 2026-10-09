#pragma once

#include "ddr_spi.pio.h"
#include "hardware/pio.h"

typedef struct ddr_spi_inst {
  PIO pio;
  uint sm;
  uint cs_pin;
  uint offset;
} ddr_spi_inst_t;

void ddr_spi_write16_read32_blocking(const ddr_spi_inst_t *spi, uint32_t *src,
                                     uint32_t *dst, size_t len);

static inline void ddr_spi_init(PIO pio, uint sm, uint prog_offs, float clkdiv,
                                bool cpha, bool cpol, uint pin_sck,
                                uint pin_mosi, uint pin_miso) {
  pio_sm_config c = cpha ? ddr_spi_tx_program_get_default_config(prog_offs)
                         : ddr_spi_rx_program_get_default_config(prog_offs);
  sm_config_set_out_pins(&c, pin_mosi, 1);
  sm_config_set_in_pins(&c, pin_miso);
  sm_config_set_sideset_pins(&c, pin_sck);

  sm_config_set_out_shift(&c, false, true, 32);
  sm_config_set_in_shift(&c, false, true, 32);
  sm_config_set_clkdiv(&c, clkdiv);

  // MOSI, SCK output are low, MISO is input
  pio_sm_set_pins_with_mask64(pio, sm, 0,
                              (1ull << pin_sck) | (1ull << pin_mosi));
  pio_sm_set_pindirs_with_mask64(
      pio, sm, (1ull << pin_sck) | (1ull << pin_mosi),
      (1ull << pin_sck) | (1ull << pin_mosi) | (1ull << pin_miso));
  pio_gpio_init(pio, pin_mosi);
  pio_gpio_init(pio, pin_miso);
  pio_gpio_init(pio, pin_sck);

  // The pin muxes can be configured to invert the output (among other things
  // and this is a cheesy way to get CPOL=1
  gpio_set_outover(pin_sck, cpol ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);
  // SPI is synchronous, so bypass input synchroniser to reduce input delay.
  pio_set_input_sync_bypass_with_mask64(pio, 1ull << pin_miso,
                                        1ull << pin_miso);

  pio_sm_init(pio, sm, prog_offs, &c);
  pio_sm_set_enabled(pio, sm, true);
}
