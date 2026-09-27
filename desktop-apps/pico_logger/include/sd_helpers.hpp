#pragma once

#include <pico/stdio.h>
#include <stdio.h>

#include "ff.h"
#include "tf_card.h"

pico_fatfs_spi_config_t config = {
    spi0,       // spi_inst (spi0, spi1 or NULL)
    (25 * KHZ), // clk_slow
    (10 * MHZ), // clk_fast
    16,         // pin_miso (SPIx_RX)
    22,         // pin_cs
    18,         // pin_sck  (SPIx_SCK)
    19,         // pin_mosi (SPIx_TX)
    true        // pullup
};

bool init_sd() { return pico_fatfs_set_config(&config); }

FRESULT scan_files(
    char *path /* Start node to be scanned (***also used as work area***) */
) {
  FRESULT res;
  DIR dir;
  UINT i;
  static FILINFO fno;

  res = f_opendir(&dir, path); /* Open the directory */
  if (res == FR_OK) {
    for (;;) {
      res = f_readdir(&dir, &fno); /* Read a directory item */
      if (fno.fname[0] == 0)
        break;                    /* Break on error or end of dir */
      if (fno.fattrib & AM_DIR) { /* The item is a directory */
        i = strlen(path);
        sprintf(&path[i], "/%s", fno.fname);
        res = scan_files(path); /* Enter the directory */
        if (res != FR_OK)
          break;
        path[i] = 0;
      } else { /* The item is a file. */
        printf("%s/%s\n", path, fno.fname);
      }
    }
    f_closedir(&dir);
  }

  return res;
}
