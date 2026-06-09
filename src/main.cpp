#include <pico/stdio.h>
#include <pico/stdio_usb.h>

#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include <pico/time.h>

int main() {
  stdio_usb_init();

  int rc = cyw43_arch_init();
  hard_assert(rc == PICO_OK);

  while (true) {
    printf(":3\n");
    sleep_ms(500);
  }
}
