#pragma once

#include "IntanShield.h"

/* Select which amplifier(s) should be turned on. FirstChannelPwr corresponds to
 * Phone Jack 1, and SecondChannelPwr corresponds to Phone Jack 2. Each
 * channel's power should only be defined once (i.e. true OR false, never true
 * AND false) */
bool FirstChannelPwr = true;
// bool FirstChannelPwr = false;

// bool SecondChannelPwr = true;
bool SecondChannelPwr = false;

String serialout = ""; // string to send through the Serial output

String serialconstants = ""; // constants to concatenate with String serialout
                             // to control auto-scaling of Serial Plotter

#define INTERRUPT_RATE                                                         \
  2000 // Interrupt rate in Hz. With the default 2 channel sample setting, each
       // channel is sampled at 1 kHz. Before changing, see notes in .cpp file
       // of library

/* Set configuration settings using the DIP switch. 5 settings to configure:
 *  1) audio_enable: enable/disable (Digital Pin 6) - determines if sounds are
 * played through speaker when a pulse has been detected from a channel. When
 * enabled, volume can be changed through rotary potentiomter. 2) low_gain_mode:
 * enable/disable (Digital Pin 7) - determines if channel data is scaled down by
 * a factor of 4. When enabled, signals appear weaker. Can be helpful for
 * viewing particularly strong signals, i.e. EKG, without clipping 3)
 * average_energy_mode: enable/disable (Analog Pin 1, aka Digital Pin 15) -
 * determines if DAC and Serial output display accumulated energy per 20 ms
 * period or raw data per 1 ms period (at amplifier sampling frequency) 4)
 * notch_setting: enable/disable (Analog Pin 2, aka Digital Pin 16) - determines
 * if the software notch filter is enabled or disabled (recommended to reduce
 * noise from power mains) 5) notch_setting: 60 Hz / 50 Hz (Analog Pin 3, aka
 * Digital Pin 17) - determines the frequency of the notch filter (60 Hz or 50
 * Hz depending on the power mains frequency of the country)
 */

bool average_energy_mode; // variable that determines if the DAC and Serial
                          // communication output the accumulated energy per 20
                          // ms, or raw channel data per 1 ms.

uint8_t data; // variable that holds the 8-bit data to be sent to the DAC

long rawdata; // variable that holds the raw 16-bit data from the RHD2216 chip

int serialdata1; // variable that holds the int data of the first channel to
                 // send over Serial

int serialdata2; // variable that holds the int data of the second channel to
                 // send over Serial

uint8_t d1; // variable that holds the first data segment to be sent to the DAC

uint8_t d2; // variable that holds the second data segment to be sent to the DAC

/* Select the lower cutoff of the bandwidth */
enum Bandselect { LowCutoff10Hz, LowCutoff1Hz, LowCutoff100mHz };
Bandselect band_setting; // band_setting has 3 possible values, corresponding to
                         // a low cutoff frequency of 10 Hz, 1 Hz, or 0.1 Hz
