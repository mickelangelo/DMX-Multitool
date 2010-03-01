extern "C" {
#include <utility/types.h>
#include <utility/w5100.h>
#include <utility/socket.h>
}
#include <stdio.h>
#include <string.h>
#include "logging.h"


#define UDP_LOGGING 1

#if defined(UDP_LOGGING)

#define LOG_BUF_SZ 256
int log_sock = 0;
int log_port = 4444;
uint8_t log_ip[] = {10,10,10,10};
uint8_t log_buf[LOG_BUF_SZ];

void log_send(uint8_t *buf,uint16_t len) {  
  if(len > 0 && len < 256)
    sendto(log_sock, buf,len,log_ip, log_port );
}

void log_debug(char *text,int line) {
  int len = snprintf((char*)log_buf,LOG_BUF_SZ,"%d: %s\n",line,text);
  log_send(log_buf,len);
}

void log_debug(char *text,int value,int line) {
  int len = snprintf((char*)log_buf,LOG_BUF_SZ,"%d: %s%d\n",line,text,value);
  log_send(log_buf,len);
}

void hexdump(uint8_t * data,uint8_t length,int line) {
  char buf[8];
  sprintf((char*)log_buf,"%d: ",line);
  for(int i=0;i<length;i++) {
    if(data[i]<32)
    sprintf(buf,"%x . ",data[i]);
    else
    sprintf(buf,"%x %s ",data[i]);
    strcat((char*)log_buf,buf);
    if(((i & 15) == 0 && i != 0) || i == length-1) {
      strcat((char*)log_buf,"\n");
      log_send(log_buf,strlen((char*)log_buf));
      sprintf((char*)log_buf,"%d: ",line);
    }
  }
}

#else  // ************************************************************


void log_debug(char *text,int line) {
  UCSR0B |= (1<< TXEN0);	// enable UART transmitter
  Serial.print(line);
  Serial.print(": ");
  Serial.println(text);
}

void log_debug(char *text,int value,int line) {
  UCSR0B |= (1<< TXEN0);	// enable UART transmitter
  Serial.print(line);
  Serial.print(": ");
  Serial.print(text);
  Serial.println(value);
}

void hexdump(uint8_t * data,uint8_t length,int line) {
  for(int i=0;i<length;i++) {
    Serial.print(data[i],16);
    Serial.print(" ");
    if(data[i]<32)
    Serial.print(".");
    else
    Serial.print(data[i]);
    Serial.print(" ");
    if((i & 15) == 0 && i != 0)
      Serial.println();
  }
  Serial.println();
}

#endif
