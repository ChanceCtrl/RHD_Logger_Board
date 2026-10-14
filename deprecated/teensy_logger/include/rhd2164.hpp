#pragma once
#include "HardwareSerial.h"
#include <stdint.h>

struct RHD2164Command {
  enum {
    Convert,
    Calibrate,
    ClearCalibration,
    WriteRegister,
    ReadRegister,
  } command;

  uint8_t reg;
  uint8_t data;

  uint16_t rhd_a_val;
  uint16_t rhd_b_val;
};

class RHD2164 {
  HardwareSerial &serial_port;
  uint8_t uart_buf[4];

  RHD2164Command command_buf[3];
  uint8_t command_pointer = 0;

  void send_command(RHD2164Command command, uint16_t payload);

public:
  RHD2164(HardwareSerial &target_port) : serial_port(target_port) {};
  void init(uint32_t baudrate) { this->serial_port.begin(baudrate); }

  RHD2164Command get_result();

  void get_conversion(uint8_t channel, bool reset_highpass);
  void calibrate(void);
  void clear_calibration(void);
  void write_register(uint8_t reg, uint8_t data);
  void read_register(uint8_t reg);
};
