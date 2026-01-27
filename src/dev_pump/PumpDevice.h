#ifndef PUMP_DEVICE_H
#define PUMP_DEVICE_H

#include <Arduino.h>
#include "device.h"
#include "config.h"

// Debounce configuration
#define PUMP_DEBOUNCE_SAMPLES 4
#define PUMP_DEBOUNCE_DELAY_MS 20
#define PUMP_READ_INTERVAL_MS 1000  // Read pump status every 1 second

// Control timing
#define PUMP_OFF_PULSE_MS 1000
#define PUMP_ON_PULSE1_MS 5000
#define PUMP_ON_PULSE2_MS 5000

/**
 * @brief Pump Device
 * Manages pump control and status reading with non-blocking state machines
 */
class PumpDevice : public Device {
public:
  // Constructor
  PumpDevice(uint8_t contactPin, uint8_t pwrOnPin, uint8_t pwrOffPin, 
             uint32_t readIntervalMs = PUMP_READ_INTERVAL_MS);
  
  // Override Device methods
  bool initialize() override;
  int start() override;
  int timeout() override;
  int event() override;
  
  // Public API
  bool isPumpOn() const { return _pumpState; }
  bool getPreviousPumpState() const { return _prevPumpState; }
  bool isProtectionActive() const { return _protectOnStart; }
  unsigned long getProtectionRemainingMs() const;
  
  // Control methods (non-blocking)
  void requestTurnOn();
  void requestTurnOff();
  void setLastOffByHexFlag(bool flag) { _lastOffByHexFlag = flag; }
  
  // Get pump status byte (for compatibility)
  byte getPumpStatusByte() const { return _pumpState ? 1 : 0; }

private:
  // Pin definitions
  uint8_t _contactPin;
  uint8_t _pwrOnPin;
  uint8_t _pwrOffPin;
  uint32_t _readIntervalMs;
  
  // Debounce state machine
  enum DebounceState {
    DEBOUNCE_IDLE,
    DEBOUNCE_READING_SAMPLE_0,
    DEBOUNCE_READING_SAMPLE_1,
    DEBOUNCE_READING_SAMPLE_2,
    DEBOUNCE_READING_SAMPLE_3,
    DEBOUNCE_DONE
  };
  DebounceState _debounceState;
  byte _debounceHighs;
  
  // Control state machine
  enum ControlState {
    CONTROL_IDLE,
    CONTROL_TURNING_ON_PULSE1,
    CONTROL_TURNING_ON_WAIT,
    CONTROL_TURNING_ON_PULSE2,
    CONTROL_TURNING_OFF_PULSE,
    CONTROL_TURNING_OFF_WAIT
  };
  ControlState _controlState;
  bool _controlRequested;
  bool _controlRequestOn;  // true = turn on, false = turn off
  
  // Pump state
  bool _pumpState;
  bool _prevPumpState;
  
  // Protection timer
  bool _protectOnStart;
  unsigned long _protectStartMs;
  unsigned long _protectLastAnnounce;
  
  // Control flags
  bool _lastOffByHexFlag;
  
  // Internal methods
  void processDebounce();
  int processControl();  // returns next timeout ms, or DURATION_IGNORE
  void processProtection();
  void updatePumpState(bool newState);
  void armStartProtection(const char* motivo);
};

#endif // PUMP_DEVICE_H