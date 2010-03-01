#ifndef _ARTNET_H
#define _ARTNET_H

#define ARTNET_DMX 0x5000
#define ARTNET_SIGNATURE {'A','r','t','-','N','e','t',0"}
#define ARTNET_UNIVERSE_SIZE 512

typedef struct artnet_dmx_s {
  uint8_t  verH;
  uint8_t  ver;
  uint8_t  sequence;
  uint8_t  physical;
  uint16_t  universe;
  uint8_t  lengthHi;
  uint8_t  length;
  uint8_t  dmx_universe[ARTNET_UNIVERSE_SIZE];
} artnet_dmx_t;

typedef struct artnet_header_s {
  uint8_t  id[8];
  uint16_t op_code;
} artnet_header_t;

typedef struct artnet_packet_s {
  artnet_header_t artnet_header;
  union {
    artnet_dmx_t artnet_dmx;
  };
} artnet_packet_t;

artnet_packet_t artnet_packet;


#endif
