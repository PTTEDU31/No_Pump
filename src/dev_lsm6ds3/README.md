# LSM6DS3Device

Device class for reading accelerometer and gyroscope data from the Arduino LSM6DS3 sensor on Nano 33 IoT.

## Overview

The `LSM6DS3Device` reads accelerometer (m/s²) and gyroscope (rad/s) data from the built-in LSM6DS3 sensor every 15 seconds (configurable).

## Features

- **Non-blocking operation**: Uses DeviceManager timeout mechanism
- **Automatic initialization**: Initializes sensor in `initialize()`
- **Periodic reading**: Reads sensor data at configured interval (default: 15 seconds)
- **Data validation**: Tracks reading validity status
- **Unit conversion**: Automatically converts accelerometer from g's to m/s² and gyroscope from dps to rad/s
- **Watchdog support**: Resets watchdog during sensor operations

## Usage

### Basic Setup

```cpp
#include "dev_lsm6ds3/LSM6DS3Device.h"

// Create device instance (reads every 15 seconds by default)
LSM6DS3Device lsm6ds3;

// Or with custom interval (e.g., 10 seconds)
LSM6DS3Device lsm6ds3(10000);

// Register with DeviceManager
Device *devices[] = {
    &lsm6ds3,
    // ... other devices
};
```

### Reading Data

```cpp
// Check if last reading was valid
if (lsm6ds3.isLastReadingValid()) {
    // Get accelerometer data (m/s²) - automatically converted from g's
    float accelX = lsm6ds3.getAccelX();
    float accelY = lsm6ds3.getAccelY();
    float accelZ = lsm6ds3.getAccelZ();
    
    // Get gyroscope data (rad/s) - automatically converted from degrees per second
    float gyroX = lsm6ds3.getGyroX();
    float gyroY = lsm6ds3.getGyroY();
    float gyroZ = lsm6ds3.getGyroZ();
    
    // Get timestamp
    unsigned long timestamp = lsm6ds3.getLastReadingTimestamp();
}
```

## API Reference

### Constructor

```cpp
LSM6DS3Device(uint32_t readIntervalMs = LSM6DS3_READ_INTERVAL_MS)
```

- `readIntervalMs`: Interval between sensor readings in milliseconds (default: 15000 = 15 seconds)

### Public Methods

- `bool initialize()`: Initializes the LSM6DS3 sensor. Returns `true` on success.
- `int start()`: Starts the device. Returns `DURATION_IMMEDIATELY` to trigger first reading.
- `int timeout()`: Called when reading interval expires. Reads sensor and returns next interval.
- `bool isLastReadingValid()`: Returns `true` if the last sensor reading was successful.
- `float getAccelX/Y/Z()`: Returns accelerometer data in m/s².
- `float getGyroX/Y/Z()`: Returns gyroscope data in rad/s.
- `unsigned long getLastReadingTimestamp()`: Returns timestamp (millis()) of last reading.

## Configuration

Default read interval is defined in `LSM6DS3Device.h`:

```cpp
#define LSM6DS3_READ_INTERVAL_MS 15000  // 15 seconds
```

## Device Lifecycle

1. **initialize()**: Called during `DeviceManager::init()`. Initializes LSM6DS3 sensor via `IMU.begin()`.
2. **start()**: Called during `DeviceManager::start()`. Returns `DURATION_IMMEDIATELY` to trigger first reading.
3. **timeout()**: Called every `readIntervalMs` milliseconds. Reads sensor data and returns next interval.

## Error Handling

- If sensor initialization fails, `initialize()` returns `false` and device is disabled.
- If sensor reading fails, `isLastReadingValid()` returns `false` and previous values are retained.
- Debug messages are printed for initialization and reading failures.

## Dependencies

- `Arduino_LSM6DS3` library (included in `platformio.ini`)
- `device.h` (base Device class)
- `DEBUG.h` (for debug output)
- `Adafruit_SleepyDog` (for watchdog support)
