#ifndef _STRUCTURES_H
#define _STRUCTURES_H

typedef struct addr_s {
  union {
    uint8 ip[4];
    struct {
      uint8 a;
      uint8 b;
      uint8 c;
      uint8 d;
    };
  };
  void operator=(uint8* data);
} addr_t;

typedef struct mac_s {
  union {
    uint8 data[6];
    struct {
      uint8 a;
      uint8 b;
      uint8 c;
      uint8 d;
      uint8 e;
      uint8 f;
    };
  };
  void operator=(uint8* data);
} mac_t;

#endif
