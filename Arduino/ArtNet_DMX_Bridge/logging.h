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
