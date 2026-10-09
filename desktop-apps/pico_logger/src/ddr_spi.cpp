#include "ddr_spi.h"
#include <pico/platform/sections.h>

void __time_critical_func(ddr_spi_write16_read32_blocking)(
    const ddr_spi_inst_t *spi, uint32_t *src, uint32_t *dst, size_t len) {
  size_t tx_remain = len, rx_remain = len;
  io_rw_32 *txfifo = (io_rw_32 *)&spi->pio->txf[spi->sm];
  io_rw_32 *rxfifo = (io_rw_32 *)&spi->pio->rxf[spi->sm];

  while (tx_remain || rx_remain) {
    // Check if we can put any more data into the txfifo, if we can, set the
    // fifo to the new value and move the pointer to the next value and loop
    if (tx_remain && !pio_sm_is_tx_fifo_full(spi->pio, spi->sm)) {
      *txfifo = *src++;
      --tx_remain;
    }

    // Check if there is data in the rxfifo, if there is, set the value at the
    // pointer to the new rxfifo data and then gg go next
    if (rx_remain && !pio_sm_is_rx_fifo_empty(spi->pio, spi->sm)) {
      *dst++ = *rxfifo;
      --rx_remain;
    }
  }
}
