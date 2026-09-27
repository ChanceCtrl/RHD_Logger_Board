#include "main.h"

int main() {
  stdio_usb_init();

  int rc = cyw43_arch_init();
  hard_assert(rc == PICO_OK);

  bool sd_spi_good = init_sd();
  hard_assert(sd_spi_good);

  FATFS fs;
  FRESULT res = f_mount(&fs, "", 1);
  hard_assert(res == FR_OK);

  char buff[256];

  while (true) {
    scan_files(buff);
    sleep_ms(500);
  }
}
