
#ifndef MODBUS_WATER_METER_H
#define MODBUS_WATER_METER_H

#include <Arduino.h>
#include "device.h"
#include "config.h"


// Data structure for a single reading
struct WaterMeterReading {
  unsigned long timestamp;  // millis() when reading was taken
  float currentFlow;        // m³/s (Instantaneous flow rate)
  float cumulativeFlow;     // m³ (Cumulative flow)
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
  
  // Reading state: 0 = read flow unit, 1 = read flow rate, 2 = read cumulative flow
  uint8_t _readingState;
  uint8_t _flowUnit;  // Flow unit (0-8) from register 0x0006
  
  // Internal methods
  bool readParameter(uint32_t regAddr, float& value);
  bool readUint16Parameter(uint32_t regAddr, uint16_t& value);
  void sendModbusRequest(uint32_t regAddr, uint16_t numRegisters = METER_NUM_REGISTERS);
  bool parseModbusResponse(float& floatValue, bool isFloatInverse);
  bool parseModbusResponseUint16(uint16_t& value);
  uint16_t calculateCRC(uint8_t* buffer, uint16_t length);
  float parseFloatInverse(uint8_t* data);  // Parse IEEE754 float with inverse byte order
  float convertToM3PerSecond(float value, uint8_t flowUnit);  // Convert to m³/s based on flow unit
  void addToHistory(const WaterMeterReading& reading);
  
  // Modbus frame buffer
  uint8_t _txBuffer[8];
  uint8_t _rxBuffer[9];
};

#endif // MODBUS_WATER_METER_H
