#include "server.h"

#include "lwip/ip4_addr.h"
#include "lwip/udp.h"
#include <pico/cyw43_arch.h>
#include <pico/stdio.h>

struct udp_pcb *pcb;
ip4_addr_t target_ip;

int init_server() {
  if (cyw43_arch_init()) {
    printf("failed to initialise WiFi\n");
    return 1;
  }

  cyw43_arch_enable_ap_mode(AP_NAME, AP_PASSWORD, CYW43_AUTH_WPA2_AES_PSK);

  target_ip.addr = PP_HTONL(LWIP_MAKEU32(192, 168, 4, 2));
  pcb = udp_new();

  return 0;
}

int send_packet(rhd_packet_common_t rhd_packet) {
  // Grab the bread
  struct pbuf *p =
      pbuf_alloc(PBUF_TRANSPORT, rhd_packet.packet_size + 1, PBUF_RAM);

  // Spread the sauce
  char *req = (char *)p->payload;

  // Throw down the meat
  memset(req, 0, rhd_packet.packet_size + 1);

  // Add the flourishs (lettuce, jalapenos, pickle)
  memcpy(req, rhd_packet.packet_buffer, rhd_packet.packet_size);

  // Eat the sandwhich
  err_t er = udp_sendto(pcb, p, &target_ip, DATA_PORT);

  // Clean up the crumbs
  pbuf_free(p);

  // Im kinda hungry

  if (er != 0) {
    printf("Failed to send UDP packet! error=%d", er);
    return -1;
  } else {
    return 1;
  }
}
