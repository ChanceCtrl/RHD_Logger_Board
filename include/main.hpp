#pragma once

#include <Metro.h>

#include "IntanShield.h"

/* Select which amplifier(s) should be turned on. FirstChannelPwr corresponds to
 * Phone Jack 1, and SecondChannelPwr corresponds to Phone Jack 2. Each
 * channel's power should only be defined once (i.e. true OR false, never true
 * AND false) */
bool FirstChannelPwr = true;
// bool FirstChannelPwr = false;

// bool SecondChannelPwr = true;
bool SecondChannelPwr = false;

uint8_t data; // variable that holds the 8-bit data to be sent to the DAC

long rawdata; // variable that holds the raw 16-bit data from the RHD2216 chip

Metro flush_timer(1000);

/* Select the lower cutoff of the bandwidth */
enum Bandselect { LowCutoff10Hz, LowCutoff1Hz, LowCutoff100mHz };
Bandselect band_setting; // band_setting has 3 possible values, corresponding to
                         // a low cutoff frequency of 10 Hz, 1 Hz, or 0.1 Hz
