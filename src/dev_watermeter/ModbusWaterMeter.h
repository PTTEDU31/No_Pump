
#ifndef MODBUS_WATER_METER_H
#define MODBUS_WATER_METER_H

#include <Arduino.h>
#include "device.h"

// Configuration
#define METER_ADDRESS 0x01
#define METER_FUNCTION_CODE 0x03
#define METER_NUM_REGISTERS 0x0002

// Register addresses (32-bit format used in original code)
#define METER_CURRENT_ADDRESS 0x00000002
#define METER_CUMULATIVE_ADDRESS 0x00380002

// History storage configuration
#define MAX_HISTORY_SIZE 60  // Store last 60 readings (15 min at 15s interval)

// Default read interval
#define WATER_METER_READ_INTERVAL_MS 15000  // 15 seconds

// Data structure for a single reading
struct WaterMeterReading {
  unsigned long timestamp;  // millis() when reading was taken
  float currentFlow;        // m³/h
  float cumulativeFlow;     // m³
  bool valid;               // Reading validity flag
};

// Statistics structure
struct WaterMeterStats {
  float currentFlowMin;
  float currentFlowMax;
  float currentFlowAvg;
  float cumulativeFlowStart;
  float cumulativeFlowEnd;
  float totalConsumed;      // cumulativeFlowEnd - cumulativeFlowStart
  uint16_t validReadings;
  uint16_t failedReadings;
};

/**
 * @brief Modbus Water Meter Device
 * Reads flow data from water meter via RS485 Modbus RTU
 */
class ModbusWaterMeter : public Device {
public:
  // Constructor
  ModbusWaterMeter(Uart* rs485Serial, uint32_t readIntervalMs = WATER_METER_READ_INTERVAL_MS);
  
  // Override Device methods
  bool initialize() override;
  int start() override;
  int timeout() override;
  int event() override;
  
  // Legacy method for backward compatibility
  void begin() { initialize(); }
  
  // Read all parameters (current + cumulative)
  bool readAllParameters();
  
  // Get latest readings
  float getCurrentFlow() const { return _latestReading.currentFlow; }
  float getCumulativeFlow() const { return _latestReading.cumulativeFlow; }
  bool isLastReadingValid() const { return _latestReading.valid; }
  
  // Get specific reading from history (0 = latest, 1 = previous, etc.)
  WaterMeterReading getHistoricalReading(uint8_t index) const;
  
  // Get statistics for stored history
  WaterMeterStats getStatistics() const;
  
  // Clear history
  void clearHistory();
  
  // Get history count
  uint16_t getHistoryCount() const { return _historyCount; }
  
  // Print debug info
  void printLastReading() const;
  void printStatistics() const;

private:
  Uart* _rs485;
  uint32_t _readIntervalMs;
  
  // Latest reading
  WaterMeterReading _latestReading;
  
  // Circular buffer for history
  WaterMeterReading _history[MAX_HISTORY_SIZE];
  uint16_t _historyIndex;
  uint16_t _historyCount;
  
  // Statistics counters
  uint32_t _totalValidReadings;
  uint32_t _totalFailedReadings;
  
  // Last read timestamp
  unsigned long _lastReadTime;
  
  // Reading state: 0 = read current flow, 1 = read cumulative flow
  uint8_t _readingState;
  
  // Internal methods
  bool readParameter(uint32_t regAddr, float& value);
  void sendModbusRequest(uint32_t regAddr);
  bool parseModbusResponse(uint32_t& rawValue);
  uint16_t calculateCRC(uint8_t* buffer, uint16_t length);
  float scaleValue(uint32_t rawValue, uint32_t regAddr);
  void addToHistory(const WaterMeterReading& reading);
  
  // Modbus frame buffer
  uint8_t _txBuffer[8];
  uint8_t _rxBuffer[9];
};

#endif // MODBUS_WATER_METER_H
