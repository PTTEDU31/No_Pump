#include <Arduino.h>
// ----------------- CONFIG DEBUG/DIAGNOSTIC -----------------
#define DEBUG true  // <--- Put false to silence the serial prints in the monitor
#define DIAG false  // <--- Put false to silence the detailed diagnostics

#if DEBUG
#define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
#define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#define DEBUG_PRINT_F(...) Serial.print(__VA_ARGS__)
#define DEBUG_PRINTLN_F(...) Serial.println(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#define DEBUG_PRINTLN(...)
#define DEBUG_PRINT_F(...)
#define DEBUG_PRINTLN_F(...)
#endif

#if DIAG
#define DIAG_PRINT(...) Serial.print(__VA_ARGS__)
#define DIAG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
#define DIAG_PRINT(...)
#define DIAG_PRINTLN(...)
#endif