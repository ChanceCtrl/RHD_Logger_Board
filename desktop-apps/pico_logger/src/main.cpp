#include "main.h"

int main() {
  stdio_usb_init();

  int rc = cyw43_arch_init();
  hard_assert(rc == PICO_OK);

  // FATFS fs;
  // FRESULT res = f_mount(&fs, "", 1);
  // hard_assert(res == FR_OK);

  char buff[256];

  while (true) {
    printf("mrow :3");
    sleep_ms(100);
  }
}
