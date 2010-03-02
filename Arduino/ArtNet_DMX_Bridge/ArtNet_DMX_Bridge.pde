extern "C" {
#include <utility/types.h>
#include <utility/w5100.h>
#include <utility/socket.h>
}

#include <Ethernet.h>
#include <EEPROM.h>
#include <HardwareSerial.h>

#include "modules.h"
#include "structures.h"
#include "artnet.h"

#define REFERENCE_ADD(x,y) ((void*)((*uint8_t)(void *)&x)+y)
#define POINTER_ADD(x,y) ((void*)((*uint8_t)(void *)x)+y)
#define READ_WORD(x,y) *((uint32*)x) = *((uint32*)y)
#define READ_SHORT(x,y) *((uint16*)x) = *((uint16*)y)

#define FRAGMENT_SHIFT 4
#define FRAGMENT_SIZE (1<<FRAGMENT_SHIFT)

uint8_t fragment_buf[FRAGMENT_SIZE];

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
  char magic[4];
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

uint8_t read_config_from_eeprom() {
  char magic[] = {"CnFg"};
  for(int i=0;i<4;i++)
    config.magic[i] = EEPROM.read(i);
  if(strncmp(magic,config.magic,4) == 0) {
    for(int i=0;i<sizeof(config_s);i++) {
      ((uint8_t*)&config)[i] = EEPROM.read(i);
    }
    return 1;
  }
  return 0;  
}

void init_udp() {
 
 node_config_t & node = config.node;
 setSIPR(node.addr.ip);
 setGAR(node.gw.ip);
 setSUBR(node.subnet_mask.ip);
 
 socket(config.artnet.udp_socket,Sn_MR_UDP,config.artnet.port,0);
}

void setup() {
 iinchip_init();
 sysinit(0x55,0x55);

 init_udp();
 Serial.begin(250000);    //Baudrate for DMX
 UCSR0C |= (1<< USBS0);  //Default is 8n1, just add 2 stop bits to the mix
 uint16 socket = config.artnet.udp_socket;
 if(read_config_from_eeprom()) {
   close(socket);
   init_udp();
 }
 
 DEBUG("WIZnet INIT OK");
 INFO("-- RUNNING --");
}



typedef struct parse_env_s {
  uint16_t position;
  uint16_t opcode;
  void (*on_packet)(void);
} parse_env_t;

parse_env_t parse_env;

void send_dmx_universe(uint8_t *universe,uint16_t channel_count) {
  UCSR0B |= (1<< TXEN0);	  // enable UART transmitter
  Serial.write((uint8_t)0);	  // send start byte
  Serial.write(universe,channel_count,&dmx_buffer_sent);
  
  //Now we are event driven, waiting for the serial buffer to empty
}

/**
 * Namspace containing handlers for all handled ArtNet packet types
 */
namespace handlers {
  
void artnet_dmx() {
  artnet_dmx_t & dmx = artnet_packet.artnet_dmx;
  DEBUG("DMX PACKET RCV");
  INFO_DUMP(dmx.dmx_universe,dmx.length);
  LOCK(config.artnet.processing);    // Ignore DMX until this is sent (we're unbuffered ftm)
  send_dmx_universe(dmx.dmx_universe,dmx.length);
}

void ignore() {
}

};

/**
 * Parses a series of ArtNet packet fragments into a complete ArtNet packet. Unknown opcodes
 * are ignored after the header has been received.
 */
void parse_artnet_fragment(uint8_t *data,uint8_t length,uint8_t fragment_no,uint8_t *ip, uint16_t port) {
  parse_env_t &pe = parse_env;
  artnet_packet_t &pkt = artnet_packet;
  
  if(fragment_no == 0) {
    pe.position = 0;
    pe.opcode = 0;
  }
  
  if(parse_env.opcode != -1) {
    memcpy(((uint8_t*)&artnet_packet)+pe.position,data,length);
    pe.position+=length;
  }
  if(pe.opcode == 0 && pe.position > sizeof(artnet_header_t)) { 
      int artnet_length = pe.position - sizeof(artnet_header_t);
      if(artnet_length >= 0) {
        pe.opcode = pkt.artnet_header.op_code;
        if(pkt.artnet_header.op_code == ARTNET_DMX) {
          pe.on_packet = &handlers::artnet_dmx;
          pe.opcode = ARTNET_DMX;
        } else {
          pe.on_packet = &handlers::ignore;
          pe.opcode = -1;
          // Ignore all other packets
        }
      }
 }
}

/**
 * Reads an UDP datagram one fragment at a time. Fragment size must be quite small to allow a high-speed
 * serial send/receive in parallel since SPI need interrupts disabled while doing bulk IO
 */
void read_datagram(void (*fragment_callback)(uint8_t *data_buf,uint8_t len,uint8_t fragmentNo,uint8_t *ip, uint16_t port)) {
  int sock = config.artnet.udp_socket;
  unsigned int available = getSn_RX_RSR(sock);
  uint8_t src_ip[4];
  uint8_t hdr_buf[8];
  uint16_t port;
  uint16_t ptr = 0;
  if(available > 0) {
      ptr = IINCHIP_READ(Sn_RX_RD0(sock));
      ptr = ((ptr & 0x00ff) << 8) + IINCHIP_READ(Sn_RX_RD0(sock) + 1);

      uint16_t datagram_length = 0;
      read_data(sock,(uint8_t*)ptr,hdr_buf,8);
      ptr += 8;
  
      READ_SHORT(&datagram_length,hdr_buf+6);
      if(datagram_length <= 0) goto SKIP;    // Early exit

      READ_WORD(src_ip,hdr_buf);
      READ_SHORT(&port,hdr_buf+4);
      
      uint8_t fragments = datagram_length >> FRAGMENT_SHIFT;

      for(uint8_t i=0;i <= fragments;i++) {
          uint8_t fragment_length = (fragments != i) ? FRAGMENT_SIZE : datagram_length & (FRAGMENT_SIZE-1);
          read_data(sock,(uint8_t*)ptr,fragment_buf,fragment_length);
          ptr += fragment_length;
          fragment_callback(fragment_buf,fragment_length,i,src_ip,port);
      }
      IINCHIP_WRITE(Sn_RX_RD0(sock),(uint8)((ptr & 0xff00) >> 8));
      IINCHIP_WRITE((Sn_RX_RD0(sock) + 1),(uint8)(ptr & 0x00ff));
      IINCHIP_WRITE(Sn_CR(sock),Sn_CR_RECV);
      while( IINCHIP_READ(Sn_CR(sock)) ); 


  }
  SKIP: return;
}  

void loop() {
    WAIT(config.artnet.processing);    
    parse_env.on_packet = &handlers::ignore;
    read_datagram(&parse_artnet_fragment);
    parse_env.on_packet();
}


void dmx_buffer_sent() {
  while(!(UCSR0A & (1<< TXC0)))  // Wait for all bits to be sent (not only shifted out from buffer)
   ;
    // send a BREAK:
    UCSR0B &= ~(1<< TXEN0);	// disable UART transmitter
    PORTD &= ~(1<< PORTD1);	// pull TX pin low
    wait_at_least(88);
   // end of BREAK: send MARK AFTER BREAK
    PORTD |= (1<<PORTD1);	// pull TX pin high
    wait_at_least(8);
    UNLOCK(config.artnet.processing);
}


  

