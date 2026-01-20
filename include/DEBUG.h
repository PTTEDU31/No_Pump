#include <Arduino.h>
// ----------------- CONFIG DEBUG/DIAGNOSTIC -----------------
#define DEBUG true  // <--- Put false to silence the serial prints in the monitor
#define DIAG false  // <--- Put false to silence the detailed diagnostics

#if DEBUG
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#define DEBUG_PRINT_F(x, p) Serial.print((x), (p))
#define DEBUG_PRINTLN_F(x, p) Serial.println((x), (p))
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINT_F(x, p)
#define DEBUG_PRINTLN_F(x, p)
#endif

#if DIAG
#define DIAG_PRINT(x) Serial.print(x)
#define DIAG_PRINTLN(x) Serial.println(x)
#else
#define DIAG_PRINT(x)
#define DIAG_PRINTLN(x)
#endif