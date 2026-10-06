#include <cstdio>
#include <pico/cyw43_arch.h>
#include <pico/stdio.h>

#include "pico/status_led.h" // libary for simplifying onboard LED usage on Pico W

#include "lwip/ip4_addr.h"
#include "lwip/udp.h"

#include "rhd_responses_builder.h"

#define BEACON_MSG_LEN_MAX 127

void build_readwriterespons(flatcc_builder_t *B) {
  RHDResponses_RawResult_start_as_root(B);
  RHDResponses_RawResult_timestamp_us_add(B, 10);
  RHDResponses_RawResult_command_type_add(B,
                                          RHDCommon_CommandType_ReadRegister);
  RHDResponses_RawResult_register_add(B, 40);
  RHDResponses_RawResult_rhd_a_add(B, 63);
  RHDResponses_RawResult_rhd_b_add(B, 63);
  RHDResponses_RawResult_end_as_root(B);
}

int main() {
  stdio_init_all();

  if (cyw43_arch_init()) {
    printf("failed to initialise WiFi\n");
    return 1;
  }

  // Initialise led
  if (!status_led_init_with_context(cyw43_arch_async_context())) {
    printf("failed to initialise LED");
    return 1;
  }

  const char *ap_name = "brace_test";
  const char *password = "test_brace";

  cyw43_arch_enable_ap_mode(ap_name, password, CYW43_AUTH_WPA2_AES_PSK);

  ip4_addr_t target_ip;
  target_ip.addr = PP_HTONL(LWIP_MAKEU32(192, 168, 4, 2));
  struct udp_pcb *pcb = udp_new();

  flatcc_builder_t builder;
  void *buffer;
  size_t size;

  flatcc_builder_init(&builder);

  build_readwriterespons(&builder);
  buffer = flatcc_builder_get_direct_buffer(&builder, &size);
  printf("Size of joe, %i, ", size);

  flatcc_builder_clear(&builder);

  int counter = 0;
  while (true) {
    // Make the sandwich
    struct pbuf *p =
        pbuf_alloc(PBUF_TRANSPORT, BEACON_MSG_LEN_MAX + 1, PBUF_RAM);

    char *req = (char *)p->payload;

    memset(req, 0, BEACON_MSG_LEN_MAX + 1);

    memcpy(req, buffer, size);

    err_t er = udp_sendto(pcb, p, &target_ip, 5005);

    pbuf_free(p);

    if (er != ERR_OK) {
      printf("Failed to send UDP packet! error=%d", er);
    } else {
      printf("Sent packet %d\n", counter);
      counter++;
    }

    sleep_ms(1000);
  }
}
