#include "LSM6DS3Device.h"
#include "DEBUG.h"
#include <Adafruit_SleepyDog.h>

// Watchdog helper
static inline void WDT_RESET() {
  Watchdog.reset();
}

// Constructor
LSM6DS3Device::LSM6DS3Device(uint32_t readIntervalMs)
  : Device(EVENT_NONE, 1),  // No event subscription, run on loop core
    _readIntervalMs(readIntervalMs),
    _accelX(0.0f),
    _accelY(0.0f),
    _accelZ(0.0f),
    _gyroX(0.0f),
    _gyroY(0.0f),
    _gyroZ(0.0f),
    _lastReadingValid(false),
    _lastReadingTimestamp(0) {
}

// Initialize - Called at the beginning of setup()
bool LSM6DS3Device::initialize() {
  DEBUG_PRINTLN(F("[LSM6DS3] Initializing LSM6DS3 device..."));
  
  // Initialize the LSM6DS3 sensor
  if (!IMU.begin()) {
    DEBUG_PRINTLN(F("[LSM6DS3] Failed to initialize LSM6DS3 sensor!"));
    return false;
  }
  
  DEBUG_PRINTLN(F("[LSM6DS3] LSM6DS3 sensor initialized successfully"));
  DEBUG_PRINT(F("[LSM6DS3] Accelerometer sample rate: "));
  DEBUG_PRINT(IMU.accelerationSampleRate());
  DEBUG_PRINTLN(F(" Hz"));
  DEBUG_PRINT(F("[LSM6DS3] Gyroscope sample rate: "));
  DEBUG_PRINT(IMU.gyroscopeSampleRate());
  DEBUG_PRINTLN(F(" Hz"));
  
  return true;
}

// Start - Called at the end of setup()
int LSM6DS3Device::start() {
  DEBUG_PRINTLN(F("[LSM6DS3] LSM6DS3 device started"));
  DEBUG_PRINT(F("[LSM6DS3] Read interval: "));
  DEBUG_PRINT(_readIntervalMs);
  DEBUG_PRINTLN(F(" ms"));
  
  // Start reading immediately
  return DURATION_IMMEDIATELY;
}

// Timeout - Called when timeout expires
int LSM6DS3Device::timeout() {
  // Read sensor data
  readSensor();
  
  // Return interval for next reading
  return _readIntervalMs;
}

// Event - Called when event is fired
int LSM6DS3Device::event() {
  return DURATION_IGNORE;
}

// Read sensor data
void LSM6DS3Device::readSensor() {
  WDT_RESET();
  
  float accelX_g, accelY_g, accelZ_g;
  float gyroX_dps, gyroY_dps, gyroZ_dps;
  
  // Read accelerometer (returns values in g's)
  if (IMU.accelerationAvailable()) {
    if (IMU.readAcceleration(accelX_g, accelY_g, accelZ_g)) {
      // Convert from g's to m/s² (1g = 9.80665 m/s²)
      const float G_TO_MS2 = 9.80665f;
      _accelX = accelX_g * G_TO_MS2;
      _accelY = accelY_g * G_TO_MS2;
      _accelZ = accelZ_g * G_TO_MS2;
    } else {
      DEBUG_PRINTLN(F("[LSM6DS3] Failed to read accelerometer"));
      _lastReadingValid = false;
      return;
    }
  } else {
    DEBUG_PRINTLN(F("[LSM6DS3] Accelerometer data not available"));
    _lastReadingValid = false;
    return;
  }
  
  WDT_RESET();
  
  // Read gyroscope (returns values in degrees per second)
  if (IMU.gyroscopeAvailable()) {
    if (IMU.readGyroscope(gyroX_dps, gyroY_dps, gyroZ_dps)) {
      // Convert from degrees per second to rad/s (1 dps = π/180 rad/s)
      const float DPS_TO_RADPS = 3.14159265359f / 180.0f;
      _gyroX = gyroX_dps * DPS_TO_RADPS;
      _gyroY = gyroY_dps * DPS_TO_RADPS;
      _gyroZ = gyroZ_dps * DPS_TO_RADPS;
      
      _lastReadingValid = true;
      _lastReadingTimestamp = millis();
      
      // Debug output
      DEBUG_PRINT(F("[LSM6DS3] Accel: X="));
      DEBUG_PRINT(_accelX);
      DEBUG_PRINT(F(" Y="));
      DEBUG_PRINT(_accelY);
      DEBUG_PRINT(F(" Z="));
      DEBUG_PRINT(_accelZ);
      DEBUG_PRINT(F(" m/s² | Gyro: X="));
      DEBUG_PRINT(_gyroX);
      DEBUG_PRINT(F(" Y="));
      DEBUG_PRINT(_gyroY);
      DEBUG_PRINT(F(" Z="));
      DEBUG_PRINT(_gyroZ);
      DEBUG_PRINTLN(F(" rad/s"));
    } else {
      DEBUG_PRINTLN(F("[LSM6DS3] Failed to read gyroscope"));
      _lastReadingValid = false;
    }
  } else {
    DEBUG_PRINTLN(F("[LSM6DS3] Gyroscope data not available"));
    _lastReadingValid = false;
  }
}
