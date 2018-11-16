#ifndef WATCHDOG_H
#define WATCHDOG_H

#include <Arduino.h>
#include "constants.h"

void KickDog() {
  noInterrupts();
  WDOG_REFRESH = 0xA602;
  WDOG_REFRESH = 0xB480;
  interrupts();
}

#ifdef __cplusplus
extern "C" {
#endif
void startup_early_hook() {
  WDOG_TOVALL = (WATCHDOG_TIMEOUT);  // The next 2 lines sets the time-out
                                     // value. This is the value that the
                                     // watchdog timer compare itself to.
  WDOG_TOVALH = 0;
  WDOG_STCTRLH = (WDOG_STCTRLH_ALLOWUPDATE | WDOG_STCTRLH_WDOGEN |
                  WDOG_STCTRLH_WAITEN | WDOG_STCTRLH_STOPEN);  // Enable WDG
  WDOG_PRESC = 0;  // prescaler, timer ticks at 1kHz
}
#ifdef __cplusplus
}
#endif

#endif