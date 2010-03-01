#include "modules.h"

void wait_at_least(uint8_t us) {
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

