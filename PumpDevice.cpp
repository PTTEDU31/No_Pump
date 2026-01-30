#include "PumpDevice.h"
#include "DEBUG.h"
#include <Adafruit_SleepyDog.h>

// Watchdog helper
static inline void WDT_RESET() {
  Watchdog.reset();
}

// Constructor
PumpDevice::PumpDevice(uint8_t contactPin, uint8_t pwrOnPin, uint8_t pwrOffPin, 
                       uint32_t readIntervalMs)
  : Device(EVENT_NONE, 1),  // No event subscription, run on loop core
    _contactPin(contactPin),
    _pwrOnPin(pwrOnPin),
    _pwrOffPin(pwrOffPin),
    _readIntervalMs(readIntervalMs),
    _debounceState(DEBOUNCE_IDLE),
    _debounceHighs(0),
    _controlState(CONTROL_IDLE),
    _controlRequested(false),
    _controlRequestOn(false),
    _pumpState(false),
    _prevPumpState(false),
    _protectOnStart(false),
    _protectStartMs(0),
    _protectLastAnnounce(0),
    _lastOffByHexFlag(false) {
}

// Initialize - Called at the beginning of setup()
bool PumpDevice::initialize() {
  DEBUG_PRINTLN(F("[PUMP] Initializing pump device..."));
  
  pinMode(_contactPin, INPUT);
  pinMode(_pwrOnPin, OUTPUT);
  pinMode(_pwrOffPin, OUTPUT);
  
  // Initialize pins to LOW
  digitalWrite(_pwrOnPin, LOW);
  digitalWrite(_pwrOffPin, LOW);
  
  // Read initial state
  _pumpState = (digitalRead(_contactPin) == HIGH);
  _prevPumpState = _pumpState;
  
  DEBUG_PRINT(F("[PUMP] Initial pump state: "));
  DEBUG_PRINTLN(_pumpState ? F("ON") : F("OFF"));
  
  DEBUG_PRINTLN(F("[PUMP] Pump device initialized"));
  return true;
}

// Start - Called at the end of setup()
int PumpDevice::start() {
  DEBUG_PRINTLN(F("[PUMP] Pump device started"));
  _debounceState = DEBOUNCE_IDLE;
  _controlState = CONTROL_IDLE;
  
  // Start debounce reading immediately
  return DURATION_IMMEDIATELY;
}

// Timeout - Called when timeout expires
int PumpDevice::timeout() {
  // Process control state machine first (higher priority)
  if (_controlState != CONTROL_IDLE || _controlRequested) {
    int d = processControl();
    if (d != DURATION_IGNORE) {
      return d;
    }
  }
  
  // Process debounce state machine
  if (_debounceState != DEBOUNCE_IDLE && _debounceState != DEBOUNCE_DONE) {
    processDebounce();
  }
  
  // Process protection timer
  processProtection();
  
  // Determine next timeout
  if (_controlState != CONTROL_IDLE) {
    return 10;  // Fallback: check every 10ms during control
  }
  
  if (_debounceState == DEBOUNCE_IDLE) {
    // Start new debounce cycle
    _debounceState = DEBOUNCE_READING_SAMPLE_0;
    _debounceHighs = 0;
    return DURATION_IMMEDIATELY;
  }
  
  if (_debounceState == DEBOUNCE_DONE) {
    // Debounce complete, wait for next read interval
    _debounceState = DEBOUNCE_IDLE;
    return _readIntervalMs;
  }
  
  // During debounce, wait for delay
  return PUMP_DEBOUNCE_DELAY_MS;
}

// Event - Called when event is fired
int PumpDevice::event() {
  return DURATION_IGNORE;
}

// Process debounce state machine
void PumpDevice::processDebounce() {
  switch (_debounceState) {
    case DEBOUNCE_READING_SAMPLE_0:
      _debounceHighs += (digitalRead(_contactPin) == HIGH);
      _debounceState = DEBOUNCE_READING_SAMPLE_1;
      WDT_RESET();
      break;
      
    case DEBOUNCE_READING_SAMPLE_1:
      _debounceHighs += (digitalRead(_contactPin) == HIGH);
      _debounceState = DEBOUNCE_READING_SAMPLE_2;
      WDT_RESET();
      break;
      
    case DEBOUNCE_READING_SAMPLE_2:
      _debounceHighs += (digitalRead(_contactPin) == HIGH);
      _debounceState = DEBOUNCE_READING_SAMPLE_3;
      WDT_RESET();
      break;
      
    case DEBOUNCE_READING_SAMPLE_3:
      _debounceHighs += (digitalRead(_contactPin) == HIGH);
      _debounceState = DEBOUNCE_DONE;
      WDT_RESET();
      
      // Evaluate debounced result
      if (_debounceHighs == PUMP_DEBOUNCE_SAMPLES) {
        updatePumpState(true);
      } else if (_debounceHighs == 0) {
        updatePumpState(false);
      }
      // If ambiguous (1-3 highs), keep previous state
      break;
      
    default:
      break;
  }
}

// Process control state machine
int PumpDevice::processControl() {
  // Check for new control request
  if (_controlRequested && _controlState == CONTROL_IDLE) {
    if (_controlRequestOn) {
      // Request to turn on
      if (_protectOnStart) {
        unsigned long elapsed = millis() - _protectStartMs;
        DEBUG_PRINT(F("[PUMP] Turn ON request ignored: currently in protection. ("));
        DEBUG_PRINT(elapsed / 1000UL);
        DEBUG_PRINTLN(F(" s)."));
        _controlRequested = false;
        return DURATION_IGNORE;
      }
      
      if (_pumpState) {
        DEBUG_PRINTLN(F("[PUMP] Pump is already ON."));
        _controlRequested = false;
        return DURATION_IGNORE;
      }
      
      DEBUG_PRINTLN(F("[PUMP] Starting turn ON sequence..."));
      _controlState = CONTROL_TURNING_ON_PULSE1;
      digitalWrite(_pwrOnPin, HIGH);
      WDT_RESET();
    } else {
      // Request to turn off
      if (!_pumpState) {
        DEBUG_PRINTLN(F("[PUMP] Pump is already OFF."));
        _controlRequested = false;
        return DURATION_IGNORE;
      }
      
      DEBUG_PRINTLN(F("[PUMP] Starting turn OFF sequence..."));
      _controlState = CONTROL_TURNING_OFF_PULSE;
      digitalWrite(_pwrOffPin, HIGH);
      WDT_RESET();
    }
    _controlRequested = false;
    return DURATION_IMMEDIATELY;  // Run next step on next loop
  }
  
  // Process current control state
  switch (_controlState) {
    case CONTROL_TURNING_ON_PULSE1:
      _controlState = CONTROL_TURNING_ON_WAIT;
      return (int)PUMP_ON_PULSE1_MS;
      
    case CONTROL_TURNING_ON_WAIT:
      WDT_RESET();
      _controlState = CONTROL_TURNING_ON_PULSE2;
      return (int)PUMP_ON_PULSE2_MS;
      
    case CONTROL_TURNING_ON_PULSE2:
      digitalWrite(_pwrOnPin, LOW);
      delay(10);
      _controlState = CONTROL_IDLE;
      DEBUG_PRINTLN(F("[PUMP] Turn ON sequence complete"));
      return DURATION_IMMEDIATELY;
      
    case CONTROL_TURNING_OFF_PULSE:
      _controlState = CONTROL_TURNING_OFF_WAIT;
      return (int)PUMP_OFF_PULSE_MS;
      
    case CONTROL_TURNING_OFF_WAIT:
      digitalWrite(_pwrOffPin, LOW);
      delay(10);
      _controlState = CONTROL_IDLE;
      DEBUG_PRINTLN(F("[PUMP] Turn OFF sequence complete"));
      return DURATION_IMMEDIATELY;
      
    default:
      return DURATION_IGNORE;
  }
}

// Process protection timer
void PumpDevice::processProtection() {
  if (!_protectOnStart) return;
  
  unsigned long elapsed = millis() - _protectStartMs;
  
  if (elapsed >= PROTECT_ON_MS) {
    _protectOnStart = false;
    DEBUG_PRINTLN(F("[PUMP] Protection period ended"));
  } else {
    if ((millis() - _protectLastAnnounce) >= PROTECT_TICK_MS) {
      _protectLastAnnounce = millis();
      unsigned long remain = (PROTECT_ON_MS - elapsed) / 1000;
      DEBUG_PRINT(F("[PUMP] Protection active. Remain: "));
      DEBUG_PRINT(remain);
      DEBUG_PRINTLN(F(" s."));
    }
  }
}

// Update pump state and detect edges
void PumpDevice::updatePumpState(bool newState) {
  if (_pumpState != newState) {
    _prevPumpState = _pumpState;
    _pumpState = newState;
    
    // Detection of edge: ON -> OFF (turned off by contact, without HEX)
    if (_prevPumpState && !_pumpState) {
      if (!_lastOffByHexFlag) {
        armStartProtection("Turned OFF detected by contact (without HEX)");
      } else {
        _lastOffByHexFlag = false;
      }
    }
  }
}

// Arm start protection
void PumpDevice::armStartProtection(const char* motivo) {
  _protectOnStart = true;
  _protectStartMs = millis();
  _protectLastAnnounce = 0;
  DEBUG_PRINT(F("[PUMP] Protection started: "));
  DEBUG_PRINTLN(motivo);
}

// Get protection remaining time
unsigned long PumpDevice::getProtectionRemainingMs() const {
  if (!_protectOnStart) return 0;
  unsigned long elapsed = millis() - _protectStartMs;
  if (elapsed >= PROTECT_ON_MS) return 0;
  return PROTECT_ON_MS - elapsed;
}

// Request turn on (non-blocking)
void PumpDevice::requestTurnOn() {
  _controlRequested = true;
  _controlRequestOn = true;
}

// Request turn off (non-blocking)
void PumpDevice::requestTurnOff() {
  _controlRequested = true;
  _controlRequestOn = false;
}