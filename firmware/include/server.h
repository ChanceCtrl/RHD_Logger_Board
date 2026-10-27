#pragma once

#include "config.h"
#include <pico/stdio.h>

#include "fb_helpers.h"

int init_server(void);

int send_packet(rhd_packet_common_t rhd_packet);
