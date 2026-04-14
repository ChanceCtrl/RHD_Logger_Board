#include "sd_helpers.hpp"
#include <Arduino.h>
#include <SD.h>

File logger;

void setup() {
  logger = start_sd_log();

  // Print CSV heading to the logfile
  logger.println("time,msg.id,msg.len,data,bus");
  logger.flush();

  // Do te ting
  Serial.println("Log start");
}

void loop() {}
