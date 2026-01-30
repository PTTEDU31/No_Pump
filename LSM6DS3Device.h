#ifndef LSM6DS3_DEVICE_H
#define LSM6DS3_DEVICE_H

#include <Arduino.h>
#include <Arduino_LSM6DS3.h>
#include "device.h"

// Read interval configuration
#define LSM6DS3_READ_INTERVAL_MS 15000  // Read sensor every 15 seconds

/**
 * @brief LSM6DS3 Device
 * Manages accelerometer and gyroscope reading from Arduino_LSM6DS3 sensor
 */
class LSM6DS3Device : public Device {
public:
  // Constructor
  LSM6DS3Device(uint32_t readIntervalMs = LSM6DS3_READ_INTERVAL_MS);
  
  // Override Device methods
  bool initialize() override;
  int start() override;
  int timeout() override;
  int event() override;
  
  // Public API - Get latest sensor readings
  bool isLastReadingValid() const { return _lastReadingValid; }
  
  // Accelerometer data (m/s²) - converted from g's
  float getAccelX() const { return _accelX; }
  float getAccelY() const { return _accelY; }
  float getAccelZ() const { return _accelZ; }
  
  // Gyroscope data (rad/s) - converted from degrees per second
  float getGyroX() const { return _gyroX; }
  float getGyroY() const { return _gyroY; }
  float getGyroZ() const { return _gyroZ; }
  
  // Timestamp of last reading
  unsigned long getLastReadingTimestamp() const { return _lastReadingTimestamp; }

private:
  uint32_t _readIntervalMs;
  
  // Sensor data
  float _accelX, _accelY, _accelZ;
  float _gyroX, _gyroY, _gyroZ;
  bool _lastReadingValid;
  unsigned long _lastReadingTimestamp;
  
  // Internal methods
  void readSensor();
};

#endif // LSM6DS3_DEVICE_H
