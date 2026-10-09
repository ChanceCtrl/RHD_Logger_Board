#include <pico/stdlib.h>
#include <stdio.h>

#include "ddr_spi.h"

#define PIN_SCK 18
#define PIN_MOSI 19
#define PIN_MISO 16

#define BUF_SIZE 5

void get_intan(const ddr_spi_inst_t *spi) {
  static uint32_t txbuf[BUF_SIZE] = {0x40, 0x41, 0x42, 0x43, 0x41};
  static uint32_t rxbuf[BUF_SIZE];

  printf("TX: ");
  for (int i = 0; i < BUF_SIZE; ++i) {
    rxbuf[i] = 0;
    printf(" %02x", (int)txbuf[i]);
  }
  printf("\n");

  ddr_spi_write16_read32_blocking(spi, txbuf, rxbuf, BUF_SIZE);

  printf("RX:");
  for (int i = 0; i < BUF_SIZE; ++i) {
    printf(" %02x", (int)rxbuf[i]);
  }
  printf("\n");
}

int main() {
  stdio_init_all();

  printf("rhd intan start\n");
  ddr_spi_inst_t spi[2];

  float clkdiv = 15.625;

  // pio_claim_free_sm_and_add_program_for_gpio_range finds a free pio and state
  // machine for a program and sets the gpio base correctly
  const uint pin_base = MIN(PIN_SCK, MIN(PIN_MOSI, PIN_MISO));
  const uint pin_count = MAX(PIN_SCK, MAX(PIN_MOSI, PIN_MISO)) - pin_base + 1;
  hard_assert(pio_claim_free_sm_and_add_program_for_gpio_range(
      &ddr_spi_tx_program, &spi[0].pio, &spi[0].sm, &spi[0].offset, pin_base,
      pin_count, true));
  hard_assert(pio_claim_free_sm_and_add_program_for_gpio_range(
      &ddr_spi_rx_program, &spi[1].pio, &spi[1].sm, &spi[1].offset, pin_base,
      pin_count, true));

  while (true) {
    for (int cpha = 0; cpha <= 1; ++cpha) {
      for (int cpol = 0; cpol <= 1; ++cpol) {
        printf("CPHA = %d, CPOL = %d\n", cpha, cpol);
        ddr_spi_init(spi[cpha].pio, spi[cpha].sm, spi[cpha].offset, clkdiv,
                     cpha, cpol, PIN_SCK, PIN_MOSI, PIN_MISO);
        get_intan(&spi[cpha]);
        sleep_ms(10);
      }
    }

    sleep_ms(1000);
  }
}
