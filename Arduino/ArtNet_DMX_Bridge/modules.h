
// Module setup and configuration
#ifndef _MODULE_CONFIGED
#define _MODULE_CONFIGED

#define LOGLEVEL 3 // INFO
#define UDP_LOGGING 1  // Log to UDP

#define LOG_SOCKET 0
#define LOG_TO_IP {10,10,10,10}
#define LOG_TO_PORT 4444

#endif

// Util functions (util.cpp)
#ifndef _UTIL_H
#define _UTIL_H

#include <stdint.h>

extern void wait_at_least(uint8_t us);

#endif


// Flash macros
#ifndef _FLASH_H
#define _FLASH_H

#include <avr/pgmspace.h>
#define FSTR(q) __extension__({static char __c[] PROGMEM = q; &__c[0];})

#endif


// Logging module
#ifndef _LOGGING_H
#define _LOGGING_H

#define LEVEL_DEBUG 4
#define LEVEL_INFO 3

#if LOGLEVEL >= LEVEL_DEBUG
#define DEBUG(x ...) log_debug(x,(int)__LINE__)
#define DEBUG_DUMP(x,y) hexdump(x,y,(int)__LINE__)
#else
#define DEBUG(x ...) do {} while(0)
#define DEBUG_DUMP(x,y) do {} while(0)
#endif
#if LOGLEVEL >= LEVEL_INFO
#define INFO_DUMP(x,y) hexdump(x,y,(int)__LINE__)
#define INFO(x ...) log_debug(x,(int)__LINE__)
#else
#define INFO(x ...) do {} while(0)
#define INFO_DUMP(x,y) do {} while(0)
#endif

extern void log_debug(char *text,int line);
extern void log_debug(char *text,int value,int line);
extern void hexdump(uint8_t * data,uint8_t length,int line);

#endif

// Lock module
#ifndef _LOCKING_H
#define _LOCKING_H

#define DECLARE_LOCK(x) volatile uint8 x
#define LOCK(x) do{while(!x);x=1;}while(0)
#define UNLOCK(x) do{x=0;}while(0)
#define WAIT(x) while(x == LOCKED)
#define UNLOCKED 0
#define LOCKED 1

#endif
