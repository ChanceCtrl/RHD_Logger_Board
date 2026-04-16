#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef ARDUINO
#include <Arduino.h>

#define SPI_SPEED 100000

static inline void wait_ms(int duration) { delay(duration); };

void init_spi(void);

void spi_transact(uint16_t tx_data);
#endif
