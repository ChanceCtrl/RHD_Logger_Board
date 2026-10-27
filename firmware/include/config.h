#pragma once

#define AP_NAME "brace_test"
#define AP_PASSWORD "test_brace"

#define DATA_PORT 5005

#define DDR_MOSI_GPIO 19
#define DDR_SCLK_GPIO 18
#define DDR_CS_GPIO 17
#define DDR_MISO_GPIO 16

// NOTE: None of this section is real yet
// One of these has to be true, not both, XOR type beat to compile
// #define USE_MEMALLOC_BUFFERS // Not done yet
// #define USE_STATIC_BUFFERS
// #ifdef USE_STATIC_BUFFERS
// #define MAX_RHD_PACKET_SIZE 272 // ConversionResult packet * 2
// #endif
