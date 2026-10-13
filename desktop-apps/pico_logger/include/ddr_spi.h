#pragma once

#include "hardware/pio.h"

typedef struct ddr_spi_inst {
  PIO tx_pio;
  uint tx_sm;
  uint tx_offset;

  PIO rx_pio;
  uint rx_sm;
  uint rx_offset;
} ddr_spi_inst_t;

ddr_spi_inst_t ddr_spi_init(PIO tx_pio, PIO rx_pio, uint tx_sm, uint tx_offset,
                            uint rx_sm, uint rx_offset, uint mosi_pin,
                            uint miso_pin, uint cs_pin, uint sclk_pin);

void ddr_transact(const ddr_spi_inst_t *ddr, uint32_t *src, uint32_t *dst,
                  size_t len);
