#include "rhd2164.hpp"
#include <Arduino.h>

RHD2164 rhd(Serial2);

void setup() {
  rhd.init(5882353);
  delay(1000);

  Serial.println("Starting...");
}

void loop() {
  rhd.read_register(40);
  RHD2164Command res = rhd.get_result();

  rhd.read_register(41);
  res = rhd.get_result();

  rhd.read_register(42);
  res = rhd.get_result();

  rhd.read_register(43);
  res = rhd.get_result();

  rhd.read_register(41);
  res = rhd.get_result();
}
