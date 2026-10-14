#pragma once

#include <Arduino.h>
#include <SD.h>

File start_sd_log(void) {
  File sd_file;

  // Set up SD card
  Serial.println("Initializing SD card...");

  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("SD card failed or not present");
  }

  // Go up till we find an original filename
  char filename[] = "data0000.csv";
  for (int i = 0; i < 10000; i++) {
    filename[4] = i / 1000 + '0';
    filename[5] = i / 100 % 10 + '0';
    filename[6] = i / 10 % 10 + '0';
    filename[7] = i % 10 + '0';

    if (!SD.exists(filename)) {
      sd_file = SD.open(filename, (uint8_t)O_WRITE | (uint8_t)O_CREAT);
      break;
    } else if (i == 9999) {
      Serial.println("Go clean your garbage.");
      break;
    }
  }

  sd_file = SD.open(filename, (uint8_t)O_WRITE | (uint8_t)O_CREAT);

  // Debug prints if it fails
  if (sd_file) { // Print on open
    Serial.print("Successfully opened SD file: ");
    Serial.println(filename);
  } else { // Print on fail
    Serial.println("Failed to open SD file");
  }

  return sd_file;
}
