#include <Arduino.h>
#include <Arduino_LSM6DS3.h>
#include <TemperatureZero.h>
#include <Adafruit_SleepyDog.h>
#include <math.h>
#include "wiring_private.h"
#include "config.h"
#include "dev_WaterMeter/ModbusWaterMeter.h"
#include "device.h"
#include "DEBUG.h"
#include "dev_sim7070g/Sim7070GDevice.h"
#include "dev_pump/PumpDevice.h"

///////////////////////////////////////////////////////////////////////////
// CREDENTIALS - Change these values from the portal
///////////////////////////////////////////////////////////////////////////
const char* NODE_ID = "5a06bafb-e479-4dc3-87d9-d79734d71f13";
const char* USERNAME = "node_d935168a3a77";
const char* PASSWORD = "G9XqOmsUYuSgq3mmWd0ecTmP";
const char* CRYPTOKEY = "B262DF3DCFCAEB149785BFDB3E84CF1535EF0F849FCB702449CD9A5DC037545F";
// const char* NODE_ID = "5a06bafb-e479-4dc3-87d9-d79734d71f13";
// const char* NODE_ID = "001";
// const char* USERNAME = "";
// const char* PASSWORD = "";
///////////////////////////////////////////////////////////////////////////
// MQTT SERVER
///////////////////////////////////////////////////////////////////////////
const char mqtt_server[] = "mqtt.myvpn.id.vn";
const int mqtt_port = 1883;

///////////////////////////////////////////////////////////////////////////
// GLOBAL OBJECTS
///////////////////////////////////////////////////////////////////////////
TemperatureZero tempSAMD;

// Watchdog helpers
static inline void WDT_INIT() {
  Watchdog.enable(HW_WDT_TIMEOUT_SEC * 1000);
}
static inline void WDT_RESET() {
  Watchdog.reset();
}
///////////////////////////////////////////////////////////////////////////
// RUNTIME STATE VARIABLES
///////////////////////////////////////////////////////////////////////////
bool prevPwrOn = false;
bool serverOnCommand = false;

unsigned long lastRestartMs = 0;
unsigned long lastTempCheck = 0;
unsigned long lastSensorRead = 0;

int vibrationCount = 0;
int EventCountTotal = 0;

///////////////////////////////////////////////////////////////////////////
// SENSOR DATA STRUCTURE
///////////////////////////////////////////////////////////////////////////
struct SensorData {
  // AC Transducer readings (V1, V2, V3)
  int V1;
  int V2;
  int V3;
  
  // Battery voltage and percentage
  float batteryVolts;
  int batteryPercent;
  
  // Power status
  bool powerPresent;  // Main power line present
  
  // Pump status
  bool pumpOn;        // Pump contact status
  
  // CPU temperature
  float cpuTemp;
  
  // Water meter data (from ModbusWaterMeter device)
  float flowRate;     // Current flow (m³/h)
  float totalFlow;   // Cumulative flow (m³)
  
  // Timestamp
  unsigned long timestamp;
  
  // Validity flags
  bool batteryValid;
  bool cpuTempValid;
  bool waterMeterValid;
};

SensorData sensorData;

///////////////////////////////////////////////////////////////////////////
// DEVICE OBJECTS
///////////////////////////////////////////////////////////////////////////
// We instantiate a UART in SERCOM0
Uart RS485(&sercom0, RS485_RX_PIN, RS485_TX_PIN, SERCOM_RX_PAD_1, UART_TX_PAD_0);

// Modbus Water Meter Module
ModbusWaterMeter waterMeter(&RS485);

// SIM7070G Modem Device
Sim7070GDevice modem(&sim7070, NODE_ID);

// Pump Device (control, debounce, protection)
PumpDevice pump(CONTACT_PIN, PWR_ON_PIN, PWR_OFF_PIN);

// ----------------- PROTOTYPES -----------------
void turnOnPump();
void turnOffPump();
void latchRelaySafetySetup();
void latchRelaySafetyLoop();
void printCPUTemperature();

// Sensor reading functions
int readAverage(uint8_t pin);
float readBatteryVolts();
int calculateBatteryPercent(float volts);
bool readPowerStatus();
bool readPumpStatus();
float readCPUTemperature();
void readAllSensors();
void printSensorData();

// ========== STATIC INLINE FUNCTION IMPLEMENTATIONS ==========
static inline bool isPwrPresent() {
  byte highs = 0;
  highs += (digitalRead(Input_Supply_V) == HIGH);
  delay(20);
  WDT_RESET();
  highs += (digitalRead(Input_Supply_V) == HIGH);
  delay(20);
  WDT_RESET();
  highs += (digitalRead(Input_Supply_V) == HIGH);
  delay(20);
  WDT_RESET();
  highs += (digitalRead(Input_Supply_V) == HIGH);

  if (highs == 4) {
    prevPwrOn = true;
    return true;
  }
  if (highs == 0) {
    prevPwrOn = false;
    return false;
  }
  return prevPwrOn;  // ambiguous zone: keep state
}

// ========== END STATIC INLINE FUNCTIONS ==========

// Mandatory ISR to manage interruptions of SERCOM0
void SERCOM0_Handler() {
  RS485.IrqHandler();
}

// Device array for DeviceManager
Device* devices[] = {
    &waterMeter,
    &modem,
    &pump,
};

// ----------------- SETUP -----------------
void setup() {
  // WDT_INIT();
  delay(2000);

  pinMode(BATT_VOLTS, INPUT);
  pinMode(Input_Supply_V, INPUT);

  // Initialize Serial for debugging
  Serial.begin(115200);
  DEBUG_PRINTLN(F("Starting initialization..."));
  
  // Register devices with DeviceManager
  DeviceManager::getInstance().registerDevices(devices, sizeof(devices) / sizeof(devices[0]));
  DEBUG_PRINTLN(F("[DEVICE] Devices registered"));
  
  // Initialize all devices (PumpDevice configures PWR_ON, PWR_OFF, CONTACT pins)
  DeviceManager::getInstance().init();
  DEBUG_PRINTLN(F("[DEVICE] Devices initialized"));
  
  // Latch relay safety: if pump contact says OFF, pulse OFF to reset relay state
  latchRelaySafetySetup();
  
  // Start all devices
  DeviceManager::getInstance().start();
  DEBUG_PRINTLN(F("[DEVICE] Devices started"));
}

// ----------------- LOOP -----------------
void loop() {
  // WDT_RESET();
  
  // Update all devices via DeviceManager (handles timeout callbacks)
  DeviceManager::getInstance().update(millis());
}





// ----------------- EQUIPMENT CONTROL -----------------
void turnOffPump() {
  pump.requestTurnOff();
}

void turnOnPump() {
  pump.requestTurnOn();
}



// ============================================================================
// SENSOR READING FUNCTIONS
// ============================================================================

/**
 * Read analog pin with averaging (3 samples)
 * @param pin Analog pin number
 * @return Averaged ADC reading scaled by 0.1299
 */
int readAverage(uint8_t pin) {
  long sum = 0;
  for (byte i = 0; i < 3; i++) {
    sum += analogRead(pin);
    delay(10);
    WDT_RESET();
  }
  return (sum / 3) * 0.1299;  // Scale factor for Nano33IoT
}

/**
 * Read battery voltage with averaging (8 samples)
 * @return Battery voltage in volts
 */
float readBatteryVolts() {
  long sum = 0;
  for (int i = 0; i < 8; i++) {
    sum += analogRead(BATT_VOLTS);
    delay(2);
    WDT_RESET();
  }
  float raw = sum / 8.0f;
  float v_adc = raw * (ADC_VREF / ADC_MAX);               // volts on the pin
  float v_bat = v_adc * ((R_TOP + R_BOTTOM) / R_BOTTOM);  // voltage divider scale
  return v_bat;
}

/**
 * Calculate battery percentage from voltage
 * @param volts Battery voltage
 * @return Battery percentage (0-100)
 */
int calculateBatteryPercent(float volts) {
  const float V_MIN = 3.00f;
  const float V_MAX = 4.05f;
  float percent = constrain((volts - V_MIN) * (100.0f / (V_MAX - V_MIN)), 0.0f, 100.0f);
  return (int)lroundf(percent);
}

/**
 * Read main power supply status with debounce
 * @return true if main power is present
 */
bool readPowerStatus() {
  return isPwrPresent();
}

/**
 * Read pump contact status (from PumpDevice debounced state)
 * @return true if pump is on
 */
bool readPumpStatus() {
  return pump.isPumpOn();
}

/**
 * Read CPU internal temperature
 * @return CPU temperature in Celsius, or -127.0 if invalid
 */
float readCPUTemperature() {
  float cpuC = tempSAMD.readInternalTemperature();
  if (isnan(cpuC)) {
    return -127.0f;
  }
  return cpuC;
}

/**
 * Read all sensors and update sensorData structure
 */
void readAllSensors() {
  sensorData.timestamp = millis();
  
  // Read AC Transducer inputs (V1, V2, V3)
  sensorData.V1 = readAverage(ADC_PIN_1);
  sensorData.V2 = readAverage(ADC_PIN_2);
  sensorData.V3 = readAverage(ADC_PIN_3);
  
  // Read battery voltage and calculate percentage
  sensorData.batteryVolts = readBatteryVolts();
  sensorData.batteryPercent = calculateBatteryPercent(sensorData.batteryVolts);
  sensorData.batteryValid = (sensorData.batteryVolts > 2.5f && sensorData.batteryVolts < 5.0f);
  
  // Read power status
  sensorData.powerPresent = readPowerStatus();
  
  // Read pump status
  sensorData.pumpOn = readPumpStatus();
  
  // Read CPU temperature
  sensorData.cpuTemp = readCPUTemperature();
  sensorData.cpuTempValid = (sensorData.cpuTemp > -100.0f && sensorData.cpuTemp < 100.0f);
  
  // Read water meter data from device
  sensorData.flowRate = waterMeter.getCurrentFlow();
  sensorData.totalFlow = waterMeter.getCumulativeFlow();
  sensorData.waterMeterValid = waterMeter.isLastReadingValid();
  
  // Pump status from PumpDevice (debounce + edge detection inside device)
  sensorData.pumpOn = pump.isPumpOn();
}

/**
 * Print all sensor data for debugging
 */
void printSensorData() {
  DEBUG_PRINTLN(F("=== SENSOR DATA ==="));
  DEBUG_PRINT(F("V1: ")); DEBUG_PRINT(sensorData.V1);
  DEBUG_PRINT(F(" | V2: ")); DEBUG_PRINT(sensorData.V2);
  DEBUG_PRINT(F(" | V3: ")); DEBUG_PRINTLN(sensorData.V3);
  
  DEBUG_PRINT(F("Battery: ")); DEBUG_PRINT(sensorData.batteryVolts);
  DEBUG_PRINT(F("V (")); DEBUG_PRINT(sensorData.batteryPercent);
  DEBUG_PRINTLN(F("%)"));
  
  DEBUG_PRINT(F("Power: ")); DEBUG_PRINTLN(sensorData.powerPresent ? F("ON") : F("OFF"));
  DEBUG_PRINT(F("Pump: ")); DEBUG_PRINTLN(sensorData.pumpOn ? F("ON") : F("OFF"));
  
  DEBUG_PRINT(F("CPU Temp: ")); DEBUG_PRINT(sensorData.cpuTemp);
  DEBUG_PRINTLN(F(" °C"));
  
  DEBUG_PRINT(F("Flow: ")); DEBUG_PRINT(sensorData.flowRate);
  DEBUG_PRINT(F(" m³/h | Total: ")); DEBUG_PRINT(sensorData.totalFlow);
  DEBUG_PRINTLN(F(" m³"));
  DEBUG_PRINTLN(F("=================="));
}

void latchRelaySafetySetup() {
  if (!pump.isPumpOn()) {
    DEBUG_PRINTLN(F("Initial state: pump OFF. Reset Latch."));
    digitalWrite(PWR_OFF_PIN, HIGH);
    delay(1000);
    digitalWrite(PWR_OFF_PIN, LOW);
    delay(20);
  } else {
    DEBUG_PRINTLN(F("Initial state: pump ON."));
  }
}

void latchRelaySafetyLoop() {
  if (!pump.isPumpOn() && serverOnCommand) {
    DEBUG_PRINTLN(F("Turning pump OFF for safety (inside loop)."));
    pump.requestTurnOff();
    serverOnCommand = false;
  }
}

/**
 * Print CPU temperature periodically (every TEMP_CHECK_INTERVAL_MS)
 */
void printCPUTemperature() {
  if ((long)(millis() - lastTempCheck) <= (long)TEMP_CHECK_INTERVAL_MS) return;
  lastTempCheck = millis();

  float cpuC = readCPUTemperature();
  DEBUG_PRINT(F("CPU Temp: "));
  if (cpuC < -100.0f) {
    DEBUG_PRINTLN(F("N/A"));
  } else {
    DEBUG_PRINT(cpuC);
    DEBUG_PRINTLN(F(" °C"));
  }
}