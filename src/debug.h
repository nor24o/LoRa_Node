#pragma once
#include <Arduino.h>
#include "node_config.h"

// ── Debug macros ───────────────────────────────────────────────────────────
// AVR HardwareSerial has no printf() — use snprintf+print instead.
// Float format in snprintf on AVR requires: -Wl,-u,vfprintf -lprintf_flt -lm
#if DEBUG_ENABLED
  #if defined(__AVR__)
    #define DBG(fmt, ...)  do { char _b[80]; snprintf(_b, sizeof(_b), fmt, ##__VA_ARGS__); DEBUG_SERIAL.print(_b); } while(0)
    #define DBGLN(s)       DEBUG_SERIAL.println(F(s))
    #define DBGHEX(b,n)    do { char _b[4]; for(uint8_t _i=0;_i<(n);_i++) { snprintf(_b,sizeof(_b),"%02X ",(b)[_i]); DEBUG_SERIAL.print(_b); } DEBUG_SERIAL.println(); } while(0)
  #else
    #define DBG(...)       DEBUG_SERIAL.printf(__VA_ARGS__)
    #define DBGLN(s)       DEBUG_SERIAL.println(F(s))
    #define DBGHEX(b,n)    do { for(uint8_t _i=0;_i<(n);_i++) DEBUG_SERIAL.printf("%02X ",(b)[_i]); DEBUG_SERIAL.println(); } while(0)
  #endif
#else
  #define DBG(...)
  #define DBGLN(s)
  #define DBGHEX(b,n)
#endif
