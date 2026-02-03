#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// =========================================================================
// NODE CREDENTIALS
// =========================================================================
extern const char* NODE_ID;
extern const char* USERNAME;
extern const char* PASSWORD;
extern const char* CRYPTOKEY;

// =========================================================================
// PIN DEFINITIONS
// =========================================================================
#define sim7070 Serial1
#define MODEM_PWR_PIN 12  // Pin to turn on/off the modem (also defined in Sim7070GDevice.h as MODEM_POWER_PIN)
#define CONTACT_PIN 20    // Pin to detect whether the pump is on or off
#define PWR_ON_PIN 4      // Pin to turn on pump (on relay)
#define PWR_OFF_PIN 3     // Pin to turn pump off (off relay)

#define ADC_PIN_1 A0       // AC Transducer input (line 1)
#define ADC_PIN_2 A1       // AC Transducer input (line 2)
#define ADC_PIN_3 A2       // AC Transducer input (line 3)
#define BATT_VOLTS A3      // Battery voltage input pin
#define Input_Supply_V 21  // Main power detection pin (detects whether there is power from the main line)

#define RS485_RX_PIN 5  // PIN RX for RS485 module
#define RS485_TX_PIN 6  // PIN TX for RS485 module

// =========================================================================
// MODBUS ADDRESSES
// =========================================================================
#define METER_CURRENT_ADDRESS 0x00000002      // Constant water flow address
#define METER_CUMULATIVE_ADDRESS 0x00380002   // Cumulative water flow address

// =========================================================================
// PDP CONTEXT (GPRS/NB-IoT)
// =========================================================================
#define PDP_CID 7
// #define PDP_APN "iot.1nce.net"
#define PDP_APN "m3-world"

// Network Mode Selection: 0 = GPRS (Cat-M1/2G), 1 = NB-IoT
// Change this to switch between GPRS and NB-IoT modes
#define NETWORK_MODE_NB 0  // 0 = GPRS, 1 = NB-IoT

// =========================================================================
// WATCHDOG
// =========================================================================
#define HW_WDT_TIMEOUT_SEC 16

// =========================================================================
// MQTT CONFIGURATION
// =========================================================================
extern const char mqtt_server[];
extern const int mqtt_port;

// =========================================================================
// BATTERY & PUBLISH INTERVALS
// =========================================================================
#define BAT_THRESH_PCT 90              // Battery percentage threshold that changes publication times
#define BAT_HYST_PCT 3                 // Hysteresis to change publication time (this avoids jumping configuration)
#define PUB_FAST_MS 15000UL            // Fast publication (when above battery threshold)
#define PUB_SLOW_MS 30000UL            // Slow publication (when below battery threshold)

// =========================================================================
// TIMING CONSTANTS
// =========================================================================
#define CHECK_INTERVAL_MS 15000UL      // Connection check interval
#define HEALTH_QUIET_MS 2000UL         // UART quiet period before health check
#define TEMP_CHECK_INTERVAL_MS 10000UL // CPU temperature check interval

// Modem restart timing
#define PWRKEY_MS 1600UL               // Valid pulse (1.5–2.0 s)
#define PRE_DRAIN_MS 40UL              // Clean UART backlogs before hitting PWRKEY
#define OFF_WAIT_MS 9000UL             // Give time if the first toggle was ON->OFF
#define BOOT_WAIT_MS 6000UL            // Boot time after the final on pulse

// Reconnection intervals
#define RECONN_INTERVAL_MS 10000UL     // 10 s

// Restart debounce
#define MIN_RESTART_GAP_MS 15000UL

// =========================================================================
// PUMP PROTECTION
// =========================================================================
#define PROTECT_ON_MS 120000UL         // 2 minutes protection period
#define PROTECT_TICK_MS 10000UL        // 10 second announcement interval

// =========================================================================
// MODEM HEALTH THRESHOLDS
// =========================================================================
#define AT_FAILS_BEFORE_RESTART 2      // More tolerant; initially 6
#define GPRS_FAILS_BEFORE_RESTART 3    // Initially 3
#define MQTT_FAILS_BEFORE_RESTART 3    // Initially 3
#define RECENT_OK_GRACE_MS 16000UL     // Initially 180000 (3 min) of "grace" if it recently was OK

// =========================================================================
// ADC & BATTERY VOLTAGE CALCULATION
// =========================================================================
#define ADC_VREF 3.3f                  // Calibrate if the real 3V3 differs
#define ADC_MAX 4095.0f
#define R_TOP 100000.0f                // 100k (BAT+ -> node)
#define R_BOTTOM 220000.0f             // 220k (node -> GND)

// =========================================================================
// MISC TIMING
// =========================================================================
#define READ_RESPONSE_WAIT_MS 300
#define SIM_WAIT_MS 1000
#define ON_PULSE_MS 5000

// =========================================================================
// AT COMMAND LIBRARY CONFIGURATION
// =========================================================================
#define AT_QUEUE_SIZE 8                    // Command queue size (circular buffer)
#define AT_MAX_URC_HANDLERS 16             // Maximum number of URC handlers
#define AT_RX_BUFFER_SIZE 512              // RX buffer size for line-by-line processing
#define AT_MAX_CMD_LENGTH 128              // Maximum AT command length
#define AT_MAX_RESPONSE_LENGTH 512         // Maximum response buffer length

// =========================================================================
// MODEM SERIAL CONFIGURATION
// =========================================================================
#define MODEM_BAUD_RATE 115200             // Modem serial baud rate
#define MODEM_POWER_PIN MODEM_PWR_PIN      // Alias for consistency

// =========================================================================
// MQTT KEEPALIVE
// =========================================================================
#define MQTT_KEEPALIVE_SEC 60              // MQTT keepalive interval in seconds

// =========================================================================
// MQTT BUFFER CONFIGURATION (for Sim7070GDevice)
// =========================================================================
#define MQTT_BUFFER_SIZE 4                 // Number of messages that can be buffered

#endif // CONFIG_H
