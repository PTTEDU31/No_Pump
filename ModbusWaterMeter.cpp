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
    _readingState(0),
    _flowUnit(1) {  // Default to m³/s
  
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
  _readingState = 0;  // Start with flow unit
  _flowUnit = 1;  // Default to m³/s if not read yet
  
  // Return immediately to start first reading
  return DURATION_IMMEDIATELY;
}

// Timeout - Called when timeout expires, returns new duration
int ModbusWaterMeter::timeout() {
  if (_readingState == 0) {
    // Step 1: Read flow unit
    DEBUG_PRINTLN(F("[MODBUS] Reading flow unit..."));
    
    uint16_t flowUnit = 1;  // Default to m³/s
    if (readUint16Parameter(METER_FLOW_UNIT_ADDRESS, flowUnit)) {
      _flowUnit = (uint8_t)flowUnit;
      if (_flowUnit > 8) _flowUnit = 1;  // Sanity check
      DEBUG_PRINT(F("[MODBUS] Flow unit: "));
      DEBUG_PRINTLN(_flowUnit);
    } else {
      DEBUG_PRINTLN(F("[MODBUS] Failed to read flow unit, using default (m³/s)"));
    }
    
    _readingState = 1;  // Next: read flow rate
    return 50;  // Wait 50ms before reading flow rate
    
  } else if (_readingState == 1) {
    // Step 2: Read flow rate and convert to m³/s
    DEBUG_PRINTLN(F("[MODBUS] Reading flow rate..."));
    
    float flowRate = 0.0f;
    if (readParameter(METER_FLOW_RATE_ADDRESS, flowRate)) {
      // Convert to m³/s based on flow unit
      _latestReading.currentFlow = convertToM3PerSecond(flowRate, _flowUnit);
      _latestReading.timestamp = millis();
      DEBUG_PRINT(F("[MODBUS] Flow rate (raw): "));
      DEBUG_PRINT(flowRate);
      DEBUG_PRINT(F(" | Converted: "));
      DEBUG_PRINT(_latestReading.currentFlow);
      DEBUG_PRINTLN(F(" m³/s"));
    } else {
      DEBUG_PRINTLN(F("[MODBUS] Failed to read flow rate"));
    }
    
    _readingState = 2;  // Next: read cumulative
    return 50;  // Wait 50ms before reading cumulative
    
  } else {
    // Step 3: Read cumulative flow
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
    
    _readingState = 0;  // Next cycle: start with flow unit again
    return _readIntervalMs;  // Wait before next reading cycle
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
  
  // Read flow unit first
  uint16_t flowUnit = 1;
  readUint16Parameter(METER_FLOW_UNIT_ADDRESS, flowUnit);
  uint8_t unit = (uint8_t)flowUnit;
  if (unit > 8) unit = 1;
  
  // Read flow rate and convert to m³/s
  float flowRate = 0.0f;
  if (readParameter(METER_FLOW_RATE_ADDRESS, flowRate)) {
    newReading.currentFlow = convertToM3PerSecond(flowRate, unit);
  } else {
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

// Read a single parameter (float)
bool ModbusWaterMeter::readParameter(uint32_t regAddr, float& value) {
  // Send request
  sendModbusRequest(regAddr);
  delay(50);
  
  // Parse response - all variable data uses Float Inverse format
  bool isFloatInverse = true;  // All float parameters use inverse format
  if (!parseModbusResponse(value, isFloatInverse)) {
    Serial.println(F("[MODBUS] Failed to parse response"));
    return false;
  }
  
  return true;
}

// Read a single parameter (uint16_t)
bool ModbusWaterMeter::readUint16Parameter(uint32_t regAddr, uint16_t& value) {
  // Send request for 1 register (2 bytes)
  sendModbusRequest(regAddr, 1);
  delay(50);
  
  // Parse response as uint16_t
  if (!parseModbusResponseUint16(value)) {
    Serial.println(F("[MODBUS] Failed to parse uint16 response"));
    return false;
  }
  
  return true;
}

// Send Modbus request
void ModbusWaterMeter::sendModbusRequest(uint32_t regAddr, uint16_t numRegisters) {
  // Extract register address (16-bit)
  uint16_t startReg = regAddr & 0xFFFF;
  
  // Build request frame
  _txBuffer[0] = METER_ADDRESS;
  _txBuffer[1] = METER_FUNCTION_CODE;  // 0x04 for Read Input Registers
  _txBuffer[2] = (startReg >> 8) & 0xFF;   // Register high byte
  _txBuffer[3] = startReg & 0xFF;          // Register low byte
  _txBuffer[4] = (numRegisters >> 8) & 0xFF;  // Num registers high
  _txBuffer[5] = numRegisters & 0xFF;         // Num registers low
  
  // Calculate CRC
  uint16_t crc = calculateCRC(_txBuffer, 6);
  _txBuffer[6] = crc & 0xFF;   // CRC low byte
  _txBuffer[7] = crc >> 8;     // CRC high byte
  
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

// Parse Modbus response (float)
bool ModbusWaterMeter::parseModbusResponse(float& floatValue, bool isFloatInverse) {
  const uint8_t EXPECT_BYTES = 9;  // Address(1) + Function(1) + Length(1) + Data(4) + CRC(2)
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
  
  // Verify CRC (Modbus RTU: CRC low byte first, then high byte)
  uint16_t crcCalc = calculateCRC(_rxBuffer, EXPECT_BYTES - 2);
  uint16_t crcResp = ((uint16_t)_rxBuffer[EXPECT_BYTES - 1] << 8) | _rxBuffer[EXPECT_BYTES - 2];
  
  if (crcCalc != crcResp) {
    Serial.print(F("[MODBUS] CRC error - Calc: "));
    Serial.print(crcCalc, HEX);
    Serial.print(F(" Resp: "));
    Serial.println(crcResp, HEX);
    return false;
  }
  
  // Verify header
  if (_rxBuffer[0] != METER_ADDRESS || 
      _rxBuffer[1] != METER_FUNCTION_CODE || 
      _rxBuffer[2] != 4) {
    Serial.println(F("[MODBUS] Invalid header"));
    return false;
  }
  
  // Parse float value from bytes 3-6 (Float Inverse format: byte order is reversed)
  if (isFloatInverse) {
    floatValue = parseFloatInverse(&_rxBuffer[3]);
  } else {
    // For cumulative, also use float inverse format
    floatValue = parseFloatInverse(&_rxBuffer[3]);
  }
  
  return true;
}

// Parse Modbus response (uint16_t)
bool ModbusWaterMeter::parseModbusResponseUint16(uint16_t& value) {
  const uint8_t EXPECT_BYTES = 7;  // Address(1) + Function(1) + Length(1) + Data(2) + CRC(2)
  const uint16_t FIRST_BYTE_TIMEOUT = 200;  // ms
  const uint16_t INTER_BYTE_TIMEOUT = 30;   // ms
  
  uint8_t bytesReceived = 0;
  
  // Wait for first byte
  unsigned long t0 = millis();
  while (!_rs485->available() && (millis() - t0) < FIRST_BYTE_TIMEOUT) {
    // Wait
  }
  
  if (!_rs485->available()) {
    Serial.println(F("[MODBUS] Timeout waiting for first byte (uint16)"));
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
  Serial.print(F("[MODBUS] RX(uint16)("));
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
    Serial.println(F("[MODBUS] Incomplete frame (uint16)"));
    return false;
  }
  
  // Verify CRC
  uint16_t crcCalc = calculateCRC(_rxBuffer, EXPECT_BYTES - 2);
  uint16_t crcResp = ((uint16_t)_rxBuffer[EXPECT_BYTES - 1] << 8) | _rxBuffer[EXPECT_BYTES - 2];
  
  if (crcCalc != crcResp) {
    Serial.print(F("[MODBUS] CRC error (uint16) - Calc: "));
    Serial.print(crcCalc, HEX);
    Serial.print(F(" Resp: "));
    Serial.println(crcResp, HEX);
    return false;
  }
  
  // Verify header
  if (_rxBuffer[0] != METER_ADDRESS || 
      _rxBuffer[1] != METER_FUNCTION_CODE || 
      _rxBuffer[2] != 2) {  // 2 bytes for uint16_t
    Serial.println(F("[MODBUS] Invalid header (uint16)"));
    return false;
  }
  
  // Parse uint16_t from bytes 3-4 (high byte first, then low byte)
  value = ((uint16_t)_rxBuffer[3] << 8) | _rxBuffer[4];
  
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
  
  return crc;  // CRC is already in correct byte order for Modbus RTU
}

// Parse IEEE754 float from Float Inverse format (byte order reversed)
// According to L-MAG-BM protocol: bytes are in order [byte4, byte3, byte2, byte1]
// We need to reverse them to [byte1, byte2, byte3, byte4] for IEEE754
float ModbusWaterMeter::parseFloatInverse(uint8_t* data) {
  // Float Inverse format: data[0]=byte4, data[1]=byte3, data[2]=byte2, data[3]=byte1
  // We need to reverse to: byte1, byte2, byte3, byte4 for IEEE754
  union {
    uint32_t u32;
    float f32;
  } converter;
  
  // Reverse byte order: [byte4, byte3, byte2, byte1] -> [byte1, byte2, byte3, byte4]
  converter.u32 = ((uint32_t)data[3] << 24) |
                  ((uint32_t)data[2] << 16) |
                  ((uint32_t)data[1] << 8) |
                  ((uint32_t)data[0]);
  
  return converter.f32;
}

// Convert flow rate to m³/s based on flow unit
// Flow unit: 0=none, 1=m³/s, 2=m³/min, 3=m³/h, 4=m³/d, 5=m³/h, 6=L/s, 7=L/min, 8=L/h
float ModbusWaterMeter::convertToM3PerSecond(float value, uint8_t flowUnit) {
  switch (flowUnit) {
    case 0:  // none - return as is (assume already in m³/s)
      return value;
    case 1:  // m³/s - no conversion
      return value;
    case 2:  // m³/min - divide by 60
      return value / 60.0f;
    case 3:  // m³/h - divide by 3600
      return value / 3600.0f;
    case 4:  // m³/d - divide by 86400
      return value / 86400.0f;
    case 5:  // m³/h (confirm) - divide by 3600
      return value / 3600.0f;
    case 6:  // L/s - divide by 1000
      return value / 1000.0f;
    case 7:  // L/min - divide by 60000
      return value / 60000.0f;
    case 8:  // L/h - divide by 3600000
      return value / 3600000.0f;
    default:
      // Unknown unit, return as is
      return value;
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
  Serial.print(_latestReading.currentFlow, 6);
  Serial.println(F(" m³/s"));
  
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
    Serial.print(stats.currentFlowMin, 6);
    Serial.println(F(" m³/s"));
    
    Serial.print(F("Current Flow - Max: "));
    Serial.print(stats.currentFlowMax, 6);
    Serial.println(F(" m³/s"));
    
    Serial.print(F("Current Flow - Avg: "));
    Serial.print(stats.currentFlowAvg, 6);
    Serial.println(F(" m³/s"));
    
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
