extern "C" {
#include <utility/types.h>
#include <utility/w5100.h>
#include <utility/socket.h>
}

#include <Ethernet.h>
#include <EEPROM.h>
#include <HardwareSerial.h>

#define LOGLEVEL 3 // INFO

#define DECLARE_LOCK(x) volatile uint8 x
#define LOCK(x) do{while(!x);x=1;}while(0)
#define UNLOCK(x) do{while(x);x=0;}while(0)
#define UNLOCKED 0
#define LOCKED 1

#define REFERENCE_ADD(x,y) ((void*)((*uint8_t)(void *)&x)+y)
#define POINTER_ADD(x,y) ((void*)((*uint8_t)(void *)x)+y)

#include "logging.h"

#define ARTNET_DMX 0x5000
#define ARTNET_SIGNATURE {'A','r','t','-','N','e','t',0"}
#define my_universe() (1)
#define ARTNET_UNIVERSE_SIZE 512
#define FRAGMENT_SHIFT 4
#define FRAGMENT_SIZE (1<<FRAGMENT_SHIFT)

int sock = 0;
int artnet_port = 0x1936;
uint8_t fragment_buf[FRAGMENT_SIZE];
uint8_t ip[] = {10,10,10,20};
uint8_t gw[] = {10,10,10,1};
uint8_t subnet_mask[] = {255,255,255,0};
uint8_t mac[] = {0x00,0x08,0xDC,0x00,0x00,0x4F};

volatile uint8_t ready_for_dmx_data = 1;

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

typedef  struct node_config_s {
    addr_t addr;
    addr_t gw;
    addr_t subnet_mask;
    mac_t mac;
 } node_config_t;
typedef  struct artnet_config_s {
    uint16 port;
    uint8  universe;
    uint8  udp_socket;
    DECLARE_LOCK(processing);
  } artnet_config_t;

struct config_s {
  uint8 magic[4];
  node_config_t node;
  artnet_config_t artnet;
} config = {{'C','n','F','g'},
  {
    {10,10,10,20},
    {10,10,10,1},
    {255,255,255,0},
    {0x00,0x08,0xDC,0x00,0x00,0x4F}
  }, {
    0x1936,
    1,
    0,
    UNLOCKED
  }};


void addr_s::operator=(uint8* data) {
     this->a = data[0];
     this->b = data[1];
     this->c = data[2];
     this->d = data[3];
}

void mac_s::operator=(uint8* data) {
     this->a = data[0];
     this->b = data[1];
     this->c = data[2];
     this->d = data[3];
}

void read_config_from_eeprom() {
}

void setup() {
 Serial.begin(250000);    //Baudrate for DMX
 UCSR0C |= (1<< USBS0);  //Default is 8n1, just add 2 stop bits to the mix

 read_config_from_eeprom();
 
 iinchip_init();
 sysinit(0x55,0x55);
 
 node_config_t & node = config.node;
 setSIPR(node.addr.ip);
 setGAR(node.gw.ip);
 setSUBR(node.subnet_mask.ip);
 
 socket(sock,Sn_MR_UDP,artnet_port,0);
 DEBUG("WIZnet INIT OK");
 INFO("-- RUNNING --");
}

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

struct artnet_packet_s {
  artnet_header_t artnet_header;
  union {
    artnet_dmx_t artnet_dmx;
  };
} artnet_packet;


struct parse_env_t {
  uint16_t position;
  uint16_t opcode;
  void (*on_packet)(void);
} parse_env;

void callback_parse_artnet(uint8_t *data,uint8_t length,uint8_t fragmentNo,uint8_t *ip, uint16_t port) {
  if(fragmentNo == 0) {
    parse_env.position = 0;
    parse_env.opcode = 0;
  }
  DEBUG("PARSE ARTNET LENGTH: " ,length);
  DEBUG("PARSE ENV POSITION: " ,parse_env.position);
  DEBUG("PARSE FRAGMENT NO: ", fragmentNo);
  
  if(parse_env.opcode != -1) {
    memcpy(((uint8_t*)&artnet_packet)+parse_env.position,data,length);
    parse_env.position+=length;
  }
  if(parse_env.opcode == 0 && parse_env.position > sizeof(artnet_packet.artnet_header)) { 
      int artnet_length = parse_env.position - sizeof(artnet_packet.artnet_header);
      if(artnet_length >= 0) {
        parse_env.opcode = artnet_packet.artnet_header.op_code;
        if(artnet_packet.artnet_header.op_code == ARTNET_DMX) {
          parse_env.on_packet = &dmx_on_packet;
          parse_env.opcode = ARTNET_DMX;
        } else {
          parse_env.on_packet = &nop_on_packet;
          parse_env.opcode = -1;
          // Ignore all other packets
        }
      }
 }
}

void read_udp_packet(void (*callback)(uint8_t *data_buf,uint8_t len,uint8_t fragmentNo,uint8_t *ip, uint16_t port));

void loop() {
    while(ready_for_dmx_data == 0)
    ;
    parse_env.on_packet = &nop_on_packet;
    read_udp_packet(&callback_parse_artnet);
    parse_env.on_packet();
}

void waitAtLeastMicros(uint8_t us) {
  if(--us == 0)
    return;
  us <<= 2;
  us -= 2;
  
  // busy wait, we have interrupt enabled since only minimum time is important
  // if we're late we might loose framerate, but everything will still work
  __asm__ __volatile__ (
   "1: sbiw %0,1" "\n\t" // 2 cycles
   "brne 1b" : "=w" (us) : "0" (us) // 2 cycles
 );
}

void io_completion() {
  while(!(UCSR0A & (1<< TXC0)))
   ;
    // send a BREAK:
    UCSR0B &= ~(1<< TXEN0);	// disable UART transmitter
    PORTD &= ~(1<< PORTD1);	// pull TX pin low
    waitAtLeastMicros(88);
   // end of BREAK: send MARK AFTER BREAK
    PORTD |= (1<<PORTD1);	// pull TX pin high
    waitAtLeastMicros(8);
    ready_for_dmx_data = 1;
}

void dmx_on_packet() {
  DEBUG("DMX PACKET RCV");
  INFO_DUMP(artnet_packet.artnet_dmx.dmx_universe,artnet_packet.artnet_dmx.length);
  ready_for_dmx_data = 0;    // Ignore DMX until this is sent (we're unbuffered ft)

  UCSR0B |= (1<< TXEN0);	// enable UART transmitter
  Serial.write((uint8_t)0);		// send start byte
  Serial.write(artnet_packet.artnet_dmx.dmx_universe,artnet_packet.artnet_dmx.length,&io_completion);
}

void nop_on_packet() {
}


void read_udp_packet(void (*callback)(uint8_t *data_buf,uint8_t len,uint8_t fragmentNo,uint8_t *ip, uint16_t port)) {
  unsigned int available = getSn_RX_RSR(sock);
  uint8_t src_ip[4];
  uint8_t hdr_buf[8];
  uint16_t port;
  uint16_t ptr = 0;
  if(available > 0) {
      ptr = IINCHIP_READ(Sn_RX_RD0(sock));
      ptr = ((ptr & 0x00ff) << 8) + IINCHIP_READ(Sn_RX_RD0(sock) + 1);

      uint16_t recv_len = 0;
      read_data(sock,(uint8_t*)ptr,hdr_buf,8);
      ptr += 8;
  
      recv_len = hdr_buf[6];
      recv_len = (recv_len << 8) + hdr_buf[7];
      if(recv_len <= 0) goto SKIP;
      
      src_ip[0] = hdr_buf[0];
      src_ip[1] = hdr_buf[1];
      src_ip[2] = hdr_buf[2];
      src_ip[3] = hdr_buf[3];
      port = hdr_buf[4];  
      port = (port << 8) + hdr_buf[5];
      if(recv_len > 0) {
          uint8_t fragments = recv_len >> FRAGMENT_SHIFT;
          DEBUG("RECV: FRAGMENT COUNT: ",fragments);
          DEBUG("RECV: LENGTH: ",recv_len);

          for(uint8_t i=0;i <= fragments;i++) {
              uint8_t fragment_recv_len = (fragments != i) ? FRAGMENT_SIZE : recv_len & (FRAGMENT_SIZE-1);
              read_data(sock,(uint8_t*)ptr,fragment_buf,fragment_recv_len);
              ptr += fragment_recv_len;
              callback(fragment_buf,fragment_recv_len,i,src_ip,port);
          }
      }
      IINCHIP_WRITE(Sn_RX_RD0(sock),(uint8)((ptr & 0xff00) >> 8));
      IINCHIP_WRITE((Sn_RX_RD0(sock) + 1),(uint8)(ptr & 0x00ff));
      IINCHIP_WRITE(Sn_CR(sock),Sn_CR_RECV);
      while( IINCHIP_READ(Sn_CR(sock)) ); 


  }
  SKIP: return;
}  
  

