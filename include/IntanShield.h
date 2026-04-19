#pragma once

#include <SPI.h>

const int chipSelectPin = SS;
#define FIRSTCHANNEL 0
#define SECONDCHANNEL 15
#define DAC_SCALE_COEFFICIENT 12

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

void ISR_callback(void);
uint16_t SendReadCommand(uint8_t regnum);
uint16_t SendConvertCommand(uint8_t channelnum);
uint16_t SendConvertCommandH(uint8_t channelnum);
uint16_t SendWriteCommand(uint8_t regnum, uint8_t data);
uint8_t ScaleForDAC(int rawdata);
uint8_t ScaleForDAC_ACC(int rawdata);
void Calibrate();
void NotchFilter60();
void NotchFilter50();
void NotchFilterNone();
void SetAmpPwr(bool Ch1, bool Ch2);
bool LowGainMode();
