#include "ModbusWaterMeter.h"
#include <Arduino.h>
#include "wiring_private.h"  // For pinPeripheral()
#include "DEBUG.h"

// Constructor
ModbusWaterMeter::ModbusWaterMeter(Uart* rs485Serial, uint32_t readIntervalMs) 
  : Device(EVENT_NONE, 1),  // No event subscription, run on loop core
    _rs485(rs485Serial),
    _readIntervalMs(readIntervalMs),
    _historyIndex(0),
    _historyCount(0),
    _totalValidReadings(0),
    _totalFailedReadings(0),
    _lastReadTime(0),
    _readingState(0) {
  
  // Initialize latest reading
  _latestReading.timestamp = 0;
  _latestReading.currentFlow = 0.0f;
  _latestReading.cumulativeFlow = 0.0f;
  _latestReading.valid = false;
  
  // Clear history
  clearHistory();
}

// Initialize - Called at the beginning of setup()
bool ModbusWaterMeter::initialize() {
  DEBUG_PRINTLN(F("[MODBUS] Initializing RS485..."));
  
  // Configure SERCOM0 pins for RS485
  pinPeripheral(5, PIO_SERCOM_ALT);  // RS485_RX_PIN
  pinPeripheral(6, PIO_SERCOM_ALT);  // RS485_TX_PIN
  
  // Initialize RS485 UART at 9600 baud
  _rs485->begin(9600);
  delay(100);
  
  DEBUG_PRINTLN(F("[MODBUS] RS485 initialized"));
  
  // Clear any pending data in RS485 buffer
  while (_rs485->available()) {
    _rs485->read();
  }
  
  DEBUG_PRINTLN(F("[MODBUS] Water meter module initialized"));
  return true;
}

// Start - Called at the end of setup(), returns timeout duration
int ModbusWaterMeter::start() {
  DEBUG_PRINTLN(F("[MODBUS] Water meter started"));
  _lastReadTime = millis();
  _readingState = 0;  // Start with current flow
  
  // Return immediately to start first reading
  return DURATION_IMMEDIATELY;
}

// Timeout - Called when timeout expires, returns new duration
int ModbusWaterMeter::timeout() {
  if (_readingState == 0) {
    // Step 1: Read current flow
    DEBUG_PRINTLN(F("[MODBUS] Reading current flow..."));
    
    float currentFlow = 0.0f;
    if (readParameter(METER_CURRENT_ADDRESS, currentFlow)) {
      _latestReading.currentFlow = currentFlow;
      _latestReading.timestamp = millis();
      DEBUG_PRINT(F("[MODBUS] Current flow: "));
      DEBUG_PRINT(currentFlow);
      DEBUG_PRINTLN(F(" m³/h"));
    } else {
      DEBUG_PRINTLN(F("[MODBUS] Failed to read current flow"));
    }
    
    _readingState = 1;  // Next: read cumulative
    return 50;  // Wait 50ms before reading cumulative
    
  } else {
    // Step 2: Read cumulative flow
    DEBUG_PRINTLN(F("[MODBUS] Reading cumulative flow..."));
    
    float cumulativeFlow = 0.0f;
    bool success = readParameter(METER_CUMULATIVE_ADDRESS, cumulativeFlow);
    
    if (success) {
      _latestReading.cumulativeFlow = cumulativeFlow;
      _latestReading.valid = true;
      _totalValidReadings++;
      
      DEBUG_PRINT(F("[MODBUS] Cumulative flow: "));
      DEBUG_PRINT(cumulativeFlow);
      DEBUG_PRINTLN(F(" m³"));
    } else {
      _latestReading.valid = false;
      _totalFailedReadings++;
      DEBUG_PRINTLN(F("[MODBUS] Failed to read cumulative flow"));
    }
    
    // Store reading in history
    addToHistory(_latestReading);
    
    _readingState = 0;  // Next cycle: start with current flow again
    return _readIntervalMs;  // Wait 5 seconds before next reading cycle
  }
}

// Event - Handle events (not subscribed to any, so just ignore)
int ModbusWaterMeter::event() {
  return DURATION_IGNORE;
}

// Read all parameters
bool ModbusWaterMeter::readAllParameters() {
  WaterMeterReading newReading;
  newReading.timestamp = millis();
  newReading.valid = true;
  
  // Read current flow
  if (!readParameter(METER_CURRENT_ADDRESS, newReading.currentFlow)) {
    newReading.currentFlow = 0.0f;
    newReading.valid = false;
  }
  
  delay(50);  // Small delay between readings
  
  // Read cumulative flow
  if (!readParameter(METER_CUMULATIVE_ADDRESS, newReading.cumulativeFlow)) {
    newReading.cumulativeFlow = 0.0f;
    newReading.valid = false;
  }
  
  // Update statistics
  if (newReading.valid) {
    _totalValidReadings++;
  } else {
    _totalFailedReadings++;
  }
  
  // Store reading
  _latestReading = newReading;
  addToHistory(newReading);
  
  return newReading.valid;
}

// Read a single parameter
bool ModbusWaterMeter::readParameter(uint32_t regAddr, float& value) {
  // Send request
  sendModbusRequest(regAddr);
  delay(50);
  
  // Parse response
  uint32_t rawValue = 0;
  if (!parseModbusResponse(rawValue)) {
    Serial.println(F("[MODBUS] Failed to parse response"));
    return false;
  }
  
  // Scale value
  value = scaleValue(rawValue, regAddr);
  
  return true;
}

// Send Modbus request
void ModbusWaterMeter::sendModbusRequest(uint32_t regAddr) {
  // Extract register address from 32-bit format
  uint16_t startReg = (regAddr >> 16) & 0xFFFF;
  
  // Build request frame
  _txBuffer[0] = METER_ADDRESS;
  _txBuffer[1] = METER_FUNCTION_CODE;
  _txBuffer[2] = (startReg >> 8) & 0xFF;   // Register high byte
  _txBuffer[3] = startReg & 0xFF;          // Register low byte
  _txBuffer[4] = (METER_NUM_REGISTERS >> 8) & 0xFF;  // Num registers high
  _txBuffer[5] = METER_NUM_REGISTERS & 0xFF;         // Num registers low
  
  // Calculate CRC
  uint16_t crc = calculateCRC(_txBuffer, 6);
  _txBuffer[6] = crc >> 8;    // CRC high byte
  _txBuffer[7] = crc & 0xFF;  // CRC low byte
  
  // Send request
  _rs485->write(_txBuffer, 8);
  _rs485->flush();
  delayMicroseconds(800);
  
  // Debug print
  Serial.print(F("[MODBUS] TX: "));
  for (int i = 0; i < 8; i++) {
    if (_txBuffer[i] < 0x10) Serial.print('0');
    Serial.print(_txBuffer[i], HEX);
    Serial.print(' ');
  }
  Serial.println();
}

// Parse Modbus response
bool ModbusWaterMeter::parseModbusResponse(uint32_t& rawValue) {
  const uint8_t EXPECT_BYTES = 9;
  const uint16_t FIRST_BYTE_TIMEOUT = 200;  // ms
  const uint16_t INTER_BYTE_TIMEOUT = 30;   // ms
  
  uint8_t bytesReceived = 0;
  
  // Wait for first byte
  unsigned long t0 = millis();
  while (!_rs485->available() && (millis() - t0) < FIRST_BYTE_TIMEOUT) {
    // Wait
  }
  
  if (!_rs485->available()) {
    Serial.println(F("[MODBUS] Timeout waiting for first byte"));
    return false;
  }
  
  // Read bytes with inter-byte timeout
  unsigned long lastByteTime = millis();
  while (bytesReceived < EXPECT_BYTES) {
    while (_rs485->available() && bytesReceived < EXPECT_BYTES) {
      _rxBuffer[bytesReceived++] = _rs485->read();
      lastByteTime = millis();
    }
    
    if ((millis() - lastByteTime) > INTER_BYTE_TIMEOUT) {
      break;  // Frame stalled
    }
  }
  
  // Debug print
  Serial.print(F("[MODBUS] RX("));
  Serial.print(bytesReceived);
  Serial.print(F("): "));
  for (int i = 0; i < bytesReceived; i++) {
    if (_rxBuffer[i] < 0x10) Serial.print('0');
    Serial.print(_rxBuffer[i], HEX);
    Serial.print(' ');
  }
  Serial.println();
  
  // Verify frame length
  if (bytesReceived != EXPECT_BYTES) {
    Serial.println(F("[MODBUS] Incomplete frame"));
    return false;
  }
  
  // Verify CRC
  uint16_t crcCalc = calculateCRC(_rxBuffer, EXPECT_BYTES - 2);
  uint16_t crcResp = ((uint16_t)_rxBuffer[EXPECT_BYTES - 2] << 8) | _rxBuffer[EXPECT_BYTES - 1];
  
  if (crcCalc != crcResp) {
    Serial.println(F("[MODBUS] CRC error"));
    return false;
  }
  
  // Verify header
  if (_rxBuffer[0] != METER_ADDRESS || 
      _rxBuffer[1] != METER_FUNCTION_CODE || 
      _rxBuffer[2] != 4) {
    Serial.println(F("[MODBUS] Invalid header"));
    return false;
  }
  
  // Extract 4-byte data (big-endian)
  rawValue = ((uint32_t)_rxBuffer[3] << 24) |
             ((uint32_t)_rxBuffer[4] << 16) |
             ((uint32_t)_rxBuffer[5] << 8) |
             ((uint32_t)_rxBuffer[6]);
  
  return true;
}

// Calculate Modbus CRC-16
uint16_t ModbusWaterMeter::calculateCRC(uint8_t* buffer, uint16_t length) {
  uint16_t crc = 0xFFFF;
  
  for (uint16_t i = 0; i < length; i++) {
    crc ^= buffer[i];
    
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 0x0001) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  
  // Swap bytes (Modbus convention)
  return ((crc >> 8) | (crc << 8));
}

// Scale raw value based on register type
float ModbusWaterMeter::scaleValue(uint32_t rawValue, uint32_t regAddr) {
  switch (regAddr) {
    case METER_CURRENT_ADDRESS:
      // Current flow: divide by 1000 to get m³/h
      return rawValue / 1000.0f;
      
    case METER_CUMULATIVE_ADDRESS:
      // Cumulative: no scaling, return as m³
      return (float)rawValue;
      
    default:
      return (float)rawValue;
  }
}

// Add reading to circular history buffer
void ModbusWaterMeter::addToHistory(const WaterMeterReading& reading) {
  _history[_historyIndex] = reading;
  _historyIndex = (_historyIndex + 1) % MAX_HISTORY_SIZE;
  
  if (_historyCount < MAX_HISTORY_SIZE) {
    _historyCount++;
  }
}

// Get historical reading
WaterMeterReading ModbusWaterMeter::getHistoricalReading(uint8_t index) const {
  if (index >= _historyCount) {
    WaterMeterReading invalid;
    invalid.valid = false;
    return invalid;
  }
  
  // Calculate actual index in circular buffer
  int actualIndex = (_historyIndex - 1 - index + MAX_HISTORY_SIZE) % MAX_HISTORY_SIZE;
  return _history[actualIndex];
}

// Calculate statistics
WaterMeterStats ModbusWaterMeter::getStatistics() const {
  WaterMeterStats stats;
  stats.currentFlowMin = 999999.0f;
  stats.currentFlowMax = -999999.0f;
  stats.currentFlowAvg = 0.0f;
  stats.cumulativeFlowStart = 0.0f;
  stats.cumulativeFlowEnd = 0.0f;
  stats.totalConsumed = 0.0f;
  stats.validReadings = 0;
  stats.failedReadings = _totalFailedReadings;
  
  if (_historyCount == 0) {
    return stats;
  }
  
  float sumCurrentFlow = 0.0f;
  
  // Iterate through history
  for (uint16_t i = 0; i < _historyCount; i++) {
    const WaterMeterReading& reading = _history[i];
    
    if (reading.valid) {
      // Current flow statistics
      if (reading.currentFlow < stats.currentFlowMin) {
        stats.currentFlowMin = reading.currentFlow;
      }
      if (reading.currentFlow > stats.currentFlowMax) {
        stats.currentFlowMax = reading.currentFlow;
      }
      sumCurrentFlow += reading.currentFlow;
      
      // Cumulative flow
      if (stats.validReadings == 0) {
        stats.cumulativeFlowStart = reading.cumulativeFlow;
      }
      stats.cumulativeFlowEnd = reading.cumulativeFlow;
      
      stats.validReadings++;
    }
  }
  
  // Calculate average
  if (stats.validReadings > 0) {
    stats.currentFlowAvg = sumCurrentFlow / stats.validReadings;
    stats.totalConsumed = stats.cumulativeFlowEnd - stats.cumulativeFlowStart;
  }
  
  return stats;
}

// Clear history
void ModbusWaterMeter::clearHistory() {
  for (int i = 0; i < MAX_HISTORY_SIZE; i++) {
    _history[i].timestamp = 0;
    _history[i].currentFlow = 0.0f;
    _history[i].cumulativeFlow = 0.0f;
    _history[i].valid = false;
  }
  _historyIndex = 0;
  _historyCount = 0;
}

// Print last reading
void ModbusWaterMeter::printLastReading() const {
  Serial.println(F("========== WATER METER READING =========="));
  Serial.print(F("Timestamp: "));
  Serial.print(_latestReading.timestamp);
  Serial.println(F(" ms"));
  
  Serial.print(F("Current Flow: "));
  Serial.print(_latestReading.currentFlow, 3);
  Serial.println(F(" m³/h"));
  
  Serial.print(F("Cumulative Flow: "));
  Serial.print(_latestReading.cumulativeFlow, 1);
  Serial.println(F(" m³"));
  
  Serial.print(F("Valid: "));
  Serial.println(_latestReading.valid ? F("YES") : F("NO"));
  
  Serial.println(F("========================================="));
}

// Print statistics
void ModbusWaterMeter::printStatistics() const {
  WaterMeterStats stats = getStatistics();
  
  Serial.println(F("========== WATER METER STATISTICS =========="));
  Serial.print(F("History Count: "));
  Serial.print(_historyCount);
  Serial.print(F(" / "));
  Serial.println(MAX_HISTORY_SIZE);
  
  Serial.print(F("Valid Readings: "));
  Serial.println(stats.validReadings);
  
  Serial.print(F("Failed Readings: "));
  Serial.println(stats.failedReadings);
  
  if (stats.validReadings > 0) {
    Serial.print(F("Current Flow - Min: "));
    Serial.print(stats.currentFlowMin, 3);
    Serial.println(F(" m³/h"));
    
    Serial.print(F("Current Flow - Max: "));
    Serial.print(stats.currentFlowMax, 3);
    Serial.println(F(" m³/h"));
    
    Serial.print(F("Current Flow - Avg: "));
    Serial.print(stats.currentFlowAvg, 3);
    Serial.println(F(" m³/h"));
    
    Serial.print(F("Cumulative - Start: "));
    Serial.print(stats.cumulativeFlowStart, 1);
    Serial.println(F(" m³"));
    
    Serial.print(F("Cumulative - End: "));
    Serial.print(stats.cumulativeFlowEnd, 1);
    Serial.println(F(" m³"));
    
    Serial.print(F("Total Consumed (period): "));
    Serial.print(stats.totalConsumed, 3);
    Serial.println(F(" m³"));
  }
  
  Serial.println(F("=========================================="));
}
