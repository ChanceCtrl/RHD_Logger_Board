#include "rhd2164.hpp"

void RHD2164::send_command(RHD2164Command command, uint16_t payload) {
  this->serial_port.write(payload >> 8);   // MSB
  this->serial_port.write(payload & 0xFF); // LSB

  this->serial_port.readBytes(this->uart_buf, 4);

  if (this->command_pointer == 3) {
    // Remove the oldest entry by shifting everything left.
    this->command_buf[0] = this->command_buf[1];
    this->command_buf[1] = this->command_buf[2];
    this->command_pointer = 2;
  }

  this->command_buf[command_pointer++] = command;
}

RHD2164Command RHD2164::get_result() {
  RHD2164Command command = this->command_buf[0];

  command.rhd_a_val = uint16_t(uart_buf[0]) | (uint16_t(uart_buf[1]) << 8);
  command.rhd_b_val = uint16_t(uart_buf[2]) | (uint16_t(uart_buf[3]) << 8);

  return command;
}

void RHD2164::get_conversion(uint8_t channel, bool reset_highpass) {
  if (channel > 63) {
    return;
  }

  uint16_t payload = 0;      // 0000 0000  0000 0000
  payload |= channel << 8;   // 00CC CCCC  0000 0000
  payload |= reset_highpass; // 00CC CCCC  0000 000H

  RHD2164Command cmd = {RHD2164Command::Convert, channel, reset_highpass, 0, 0};

  this->send_command(cmd, payload);
}

void RHD2164::calibrate(void) {
  uint16_t payload = 21760; // 0101 0101  0000 0000
  RHD2164Command cmd = {RHD2164Command::Calibrate, 0, 0, 0, 0};

  this->send_command(cmd, payload);
}

void RHD2164::clear_calibration(void) {
  uint16_t payload = 27136; // 0110 1010  0000 0000
  RHD2164Command cmd = {RHD2164Command::ClearCalibration, 0, 0, 0, 0};

  this->send_command(cmd, payload);
}

void RHD2164::write_register(uint8_t reg, uint8_t data) {
  if (reg > 63) {
    return;
  }

  uint16_t payload = 32768; // 1000 0000  0000 0000
  payload |= reg << 8;      // 10RR RRRR  0000 0000
  payload |= data;          // 10RR RRRR  DDDD DDDD

  RHD2164Command cmd = {RHD2164Command::WriteRegister, reg, data, 0, 0};

  this->send_command(cmd, payload);
}

void RHD2164::read_register(uint8_t reg) {
  if (reg > 63) {
    return;
  }

  uint16_t payload = 49152; // 1100 0000  0000 0000
  payload |= reg << 8;      // 11RR RRRR  0000 0000

  RHD2164Command command = {RHD2164Command::ReadRegister, reg, 0, 0, 0};

  this->send_command(command, payload);
}
