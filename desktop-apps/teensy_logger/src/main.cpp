#include "rhd2164.hpp"
#include "sd_helpers.hpp"
#include <Arduino.h>

RHD2164 rhd(Serial2);
File logger;

bool should_be_logging = true;

void setup() {
  Serial.println("You can press any key to pause the capture");

  Serial.println("Starting RHD...");
  rhd.init(5882353);
  delay(500);
  rhd.calibrate();
  delay(2000);

  logger = start_sd_log();
  logger.println("Timestamp,Command,RHD_A,RHD_B");

  Serial.println("Starting loop...");
}

void loop() {
  if (Serial.available() > 0) {
    if (should_be_logging) {
      Serial.println("Pausing...");
      should_be_logging = false;
    } else {
      Serial.println("Resuming...");
      should_be_logging = true;
    }
  }

  if (should_be_logging) {
    String conv_str = "";
    conv_str += String(Teensy3Clock.get()) + ",";

    for (int x = 0; x < 32; x++) {
      rhd.get_conversion(x, false);
      RHD2164Command res = rhd.get_result();
      conv_str += String(res.command) + "," + String(res.rhd_a_val) + "," +
                  String(res.rhd_b_val);
    }

    logger.println(conv_str);
    logger.flush();
  }
}
