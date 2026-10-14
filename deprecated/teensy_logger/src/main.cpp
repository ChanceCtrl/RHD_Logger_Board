#include "core_pins.h"
#include "rhd2164.hpp"
#include "sd_helpers.hpp"
#include <Arduino.h>

RHD2164 rhd(Serial2);
File logger;

bool should_be_logging = true;
IntervalTimer flush_timer;

void log_event() {
  RHD2164Command res = rhd.get_result();

  String conv_str = String(micros()) + ",";
  conv_str += String(res.command) + "," + String(res.reg) + "," +
              String(res.rhd_a_val) + "," + String(res.rhd_b_val);

  logger.println(conv_str);
  Serial.println(conv_str);
}

void log_flush() { logger.flush(); }

void setup() {
  Serial.println("Starting RHD...");
  rhd.init(5882353);
  delay(500);
  rhd.calibrate();

  // See https://intantech.com/files/Intan_RHD2000_series_datasheet.pdf page 17
  rhd.read_register(40); // 1
  rhd.read_register(40); // 2
  rhd.read_register(40); // 3
  rhd.read_register(40); // 4
  rhd.read_register(40); // 5
  rhd.read_register(40); // 6
  rhd.read_register(40); // 7
  rhd.read_register(40); // 8
  rhd.read_register(40); // 9
  delay(2000);

  logger = start_sd_log();
  logger.println("Timestamp,Command,Reg,RHD_A,RHD_B");
  flush_timer.begin(log_flush, 100000);

  rhd.read_register(40); // I
  log_event();

  rhd.read_register(41); // N
  log_event();

  rhd.read_register(42); // T
  log_event();

  rhd.read_register(43); // A
  log_event();

  rhd.read_register(41); // N
  log_event();

  Serial.println("Starting loop...");
}

void loop() {
  // for (int x = 0; x < 32; x++) {
  //   rhd.get_conversion(x, false);
  //   log_event();
  // }

  rhd.get_conversion(16, false);
  log_event();

  delay(1);
}
