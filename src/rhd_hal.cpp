#include "rhd_hal.hpp"

#ifdef ARDUINO
#include <SPI.h>

void init_spi(void) {
  SPI.init();
  digitalWrite(SS, HIGH);
}

uint16_t spi_transact(uint16_t tx_data) {
  uint16_t rx_data = 0;

  SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE0));
  digitalWrite(SS, LOW);

  rx_data = SPI.transfer16(tx_data);

  digitalWrite(SS, HIGH);
  SPI.endTransaction();

  return rx_data;
}
#endif
