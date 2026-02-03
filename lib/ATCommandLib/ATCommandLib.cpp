/**
 * @file ATCommandLib.cpp
 * @brief AT Command Manager Implementation
 */

#include "ATCommandLib.h"

// Constructor
ATCommandManager::ATCommandManager(Stream* serial)
  : _serial(serial),
    _debug(false),
    _rxPos(0),
    _queueHead(0),
    _queueTail(0),
    _queueCount(0),
    _currentCommand(nullptr),
    _urcCount(0),
    _useCustomTerminator(false)
{
  memset(&_stats, 0, sizeof(_stats));
  memset(_commandQueue, 0, sizeof(_commandQueue));
  memset(_urcHandlers, 0, sizeof(_urcHandlers));
  memset(_rxBuffer, 0, sizeof(_rxBuffer));
  memset(_customTerminator, 0, sizeof(_customTerminator));
  memset(&_syncState, 0, sizeof(_syncState));
  _syncState.active = false;
  _syncState.completed = false;
}

// Destructor
ATCommandManager::~ATCommandManager() {
  // Nothing to clean up
}

// Initialize
bool ATCommandManager::begin() {
  if (!_serial) {
    return false;
  }
  
  _rxPos = 0;
  _queueHead = 0;
  _queueTail = 0;
  _queueCount = 0;
  _currentCommand = nullptr;
  _urcCount = 0;
  
  resetStats();
  
  if (_debug) {
    debugPrint("AT", "Initialized");
  }
  
  return true;
}

// Main loop - MUST be called frequently
void ATCommandManager::loop() {
  // 1. Process incoming serial data
  processSerial();
  
  // 2. Check timeout for current command
  checkTimeout();
  
  // 3. Send next command if no active command and queue not empty
  if (!_currentCommand && _queueCount > 0) {
    dequeueCommand();
  }
}

// Process serial data
void ATCommandManager::processSerial() {
  while (_serial && _serial->available()) {
    char c = _serial->read();
    _stats.bytesReceived++;
    
    // Handle line endings (both \r and \n)
    if (c == '\r' || c == '\n') {
      if (_rxPos > 0) {
        _rxBuffer[_rxPos] = '\0';
        processLine(_rxBuffer);
        _rxPos = 0;
      }
    }
    // Add character to buffer
    else if (_rxPos < sizeof(_rxBuffer) - 1) {
      _rxBuffer[_rxPos++] = c;
    }
    // Buffer overflow - process what we have
    else {
      _rxBuffer[_rxPos] = '\0';
      processLine(_rxBuffer);
      _rxPos = 0;
      _rxBuffer[_rxPos++] = c;
    }
  }
}

// Process a complete line
void ATCommandManager::processLine(const char* line) {
  // Skip empty lines
  if (line[0] == '\0' || strcmp(line, "") == 0) {
    return;
  }
  
  if (_debug) {
    debugPrint("RX", line);
  }
  
  // Check for "OK" response
  if (strcmp(line, "OK") == 0) {
    completeCommand(ATResponseType::OK);
    return;
  }
  
  // Check for "ERROR" response
  if (strstr(line, "ERROR") != nullptr) {
    completeCommand(ATResponseType::ERROR);
    return;
  }
  
  // Check for custom terminator
  if (_useCustomTerminator && _currentCommand && 
      strstr(line, _customTerminator) != nullptr) {
    completeCommand(ATResponseType::CUSTOM);
    return;
  }
  
  // Check if this is a URC (Unsolicited Result Code)
  if (matchURC(line)) {
    return;
  }
  
  // Otherwise, accumulate response for current command
  if (_currentCommand && _currentCommand->active) {
    size_t lineLen = strlen(line);
    size_t remaining = sizeof(_currentCommand->response) - _currentCommand->responsePos - 1;
    
    if (remaining > 0) {
      // Add line to response buffer
      if (remaining > lineLen) {
        strcpy(_currentCommand->response + _currentCommand->responsePos, line);
        _currentCommand->responsePos += lineLen;
        
        // Add newline if there's space
        if (_currentCommand->responsePos < sizeof(_currentCommand->response) - 1) {
          _currentCommand->response[_currentCommand->responsePos++] = '\n';
          _currentCommand->response[_currentCommand->responsePos] = '\0';
        }
      }
    }
  }
}

// Match line against registered URC handlers
bool ATCommandManager::matchURC(const char* line) {
  for (uint8_t i = 0; i < _urcCount; i++) {
    if (!_urcHandlers[i].active) continue;
    
    size_t prefixLen = strlen(_urcHandlers[i].prefix);
    if (strncmp(line, _urcHandlers[i].prefix, prefixLen) == 0) {
      // Found matching URC
      _stats.urcsReceived++;
      
      if (_debug) {
        debugPrint("URC", _urcHandlers[i].prefix);
      }
      
      // Extract data after prefix
      const char* data = line + prefixLen;
      
      // Skip whitespace and colon
      while (*data == ' ' || *data == ':' || *data == '\t') {
        data++;
      }
      
      // Call handler
      if (_urcHandlers[i].handler) {
        _urcHandlers[i].handler(_urcHandlers[i].prefix, data, _urcHandlers[i].userData);
      }
      
      return true;
    }
  }
  
  return false;
}

// Check command timeout
void ATCommandManager::checkTimeout() {
  if (_currentCommand && _currentCommand->active) {
    unsigned long elapsed = millis() - _currentCommand->sentTime;
    if (elapsed >= _currentCommand->timeout) {
      if (_debug) {
        debugPrint("TIMEOUT", _currentCommand->cmd);
      }
      
      completeCommand(ATResponseType::TIMEOUT);
    }
  }
}

// Send command asynchronously
bool ATCommandManager::sendCommandAsync(const char* cmd, unsigned long timeout,
                                        ATCallback callback, void* userData) {
  if (!cmd) return false;
  
  // Check if queue is full
  if (_queueCount >= AT_QUEUE_SIZE) {
    if (_debug) {
      debugPrint("QUEUE_FULL", cmd);
    }
    return false;
  }
  
  // Create command
  ATCommand newCmd;
  strncpy(newCmd.cmd, cmd, sizeof(newCmd.cmd) - 1);
  newCmd.cmd[sizeof(newCmd.cmd) - 1] = '\0';
  newCmd.timeout = timeout;
  newCmd.callback = callback;
  newCmd.userData = userData;
  newCmd.active = false;
  newCmd.sentTime = 0;
  newCmd.response[0] = '\0';
  newCmd.responsePos = 0;
  
  return enqueueCommand(newCmd);
}

// Static callback for sync commands
void ATCommandManager::syncCommandCallback(ATResponseType type, const char* response, void* userData) {
  if (!userData) return;
  
  ATCommandManager* manager = (ATCommandManager*)userData;
  
  // Store result
  manager->_syncState.result = type;
  manager->_syncState.completed = true;
  
  // Copy response if available
  if (response && response[0] != '\0') {
    strncpy(manager->_syncState.response, response, sizeof(manager->_syncState.response) - 1);
    manager->_syncState.response[sizeof(manager->_syncState.response) - 1] = '\0';
  }
}

// IMPROVED: Send command synchronously using async path + loop()
bool ATCommandManager::sendCommandSync(const char* cmd, const char* expectedResponse,
                                       unsigned long timeout,
                                       char* response, size_t responseSize) {
  if (!cmd || !_serial) return false;
  
  if (_debug) {
    debugPrint("TX_SYNC", cmd);
  }
  
  // Initialize sync state
  _syncState.active = true;
  _syncState.completed = false;
  _syncState.result = ATResponseType::NONE;
  _syncState.response[0] = '\0';
  _syncState.expectedResponse = expectedResponse;
  
  // Send command using async path (reuses all logic!)
  if (!sendCommandAsync(cmd, timeout, syncCommandCallback, this)) {
    _syncState.active = false;
    if (_debug) {
      debugPrint("SYNC_FAIL", "Queue full");
    }
    return false;
  }
  
  // Wait for completion by calling loop() repeatedly
  // This ensures URCs are still processed!
  unsigned long start = millis();
  
  while (!_syncState.completed && (millis() - start) < timeout) {
    loop();  // ✅ Reuse async processing path
    
    delay(1);  // Small delay to avoid tight loop
  }
  
  // Check if timed out
  if (!_syncState.completed) {
    if (_debug) {
      debugPrint("SYNC_TIMEOUT", cmd);
    }
    _syncState.active = false;
    return false;
  }
  
  // Copy response to user buffer if provided
  if (response && responseSize > 0 && _syncState.response[0] != '\0') {
    strncpy(response, _syncState.response, responseSize - 1);
    response[responseSize - 1] = '\0';
  }
  
  // Check if expected response was found
  bool success = false;
  if (_syncState.result == ATResponseType::OK) {
    // If expectedResponse specified, verify it's in the response
    if (expectedResponse) {
      success = (strstr(_syncState.response, expectedResponse) != nullptr);
    } else {
      success = true;  // No specific response expected, OK is enough
    }
  }
  
  _syncState.active = false;
  
  if (_debug) {
    debugPrint("SYNC_RESULT", success ? "OK" : "FAIL");
  }
  
  return success;
}

// Register URC handler
bool ATCommandManager::registerURCHandler(const char* prefix, URCHandler handler, void* userData) {
  if (!prefix || !handler) return false;
  
  // Check if already registered
  for (uint8_t i = 0; i < _urcCount; i++) {
    if (strcmp(_urcHandlers[i].prefix, prefix) == 0) {
      // Update existing handler
      _urcHandlers[i].handler = handler;
      _urcHandlers[i].userData = userData;
      _urcHandlers[i].active = true;
      return true;
    }
  }
  
  // Check if we have space
  if (_urcCount >= AT_MAX_URC_HANDLERS) {
    return false;
  }
  
  // Add new handler
  strncpy(_urcHandlers[_urcCount].prefix, prefix, sizeof(_urcHandlers[_urcCount].prefix) - 1);
  _urcHandlers[_urcCount].prefix[sizeof(_urcHandlers[_urcCount].prefix) - 1] = '\0';
  _urcHandlers[_urcCount].handler = handler;
  _urcHandlers[_urcCount].userData = userData;
  _urcHandlers[_urcCount].active = true;
  _urcCount++;
  
  if (_debug) {
    debugPrint("URC_REG", prefix);
  }
  
  return true;
}

// Unregister URC handler
void ATCommandManager::unregisterURCHandler(const char* prefix) {
  if (!prefix) return;
  
  for (uint8_t i = 0; i < _urcCount; i++) {
    if (strcmp(_urcHandlers[i].prefix, prefix) == 0) {
      _urcHandlers[i].active = false;
      
      if (_debug) {
        debugPrint("URC_UNREG", prefix);
      }
      
      break;
    }
  }
}

// Check if modem is alive
bool ATCommandManager::isAlive(unsigned long timeout) {
  return sendCommandSync("AT", "OK", timeout);
}

// Clear command queue
void ATCommandManager::clearQueue() {
  _queueHead = 0;
  _queueTail = 0;
  _queueCount = 0;
  _currentCommand = nullptr;
  
  memset(_commandQueue, 0, sizeof(_commandQueue));
  
  if (_debug) {
    debugPrint("QUEUE", "Cleared");
  }
}

// Reset statistics
void ATCommandManager::resetStats() {
  memset(&_stats, 0, sizeof(_stats));
}

// Enqueue command to circular buffer
bool ATCommandManager::enqueueCommand(const ATCommand& cmd) {
  if (_queueCount >= AT_QUEUE_SIZE) {
    return false;
  }
  
  _commandQueue[_queueHead] = cmd;
  _queueHead = (_queueHead + 1) % AT_QUEUE_SIZE;
  _queueCount++;
  
  if (_debug) {
    char buf[64];
    snprintf(buf, sizeof(buf), "Queued (%d/%d)", _queueCount, AT_QUEUE_SIZE);
    debugPrint("ENQUEUE", buf);
  }
  
  return true;
}

// Dequeue and send next command
bool ATCommandManager::dequeueCommand() {
  if (_queueCount == 0 || _currentCommand) {
    return false;
  }
  
  // Get next command from queue
  _currentCommand = &_commandQueue[_queueTail];
  _currentCommand->active = true;
  _currentCommand->sentTime = millis();
  _currentCommand->response[0] = '\0';
  _currentCommand->responsePos = 0;
  
  // Send command
  if (_serial) {
    _serial->println(_currentCommand->cmd);
    _stats.bytesSent += strlen(_currentCommand->cmd) + 2; // +2 for \r\n
    _stats.commandsSent++;
    
    if (_debug) {
      debugPrint("TX", _currentCommand->cmd);
    }
  }
  
  return true;
}

// Complete current command
void ATCommandManager::completeCommand(ATResponseType type) {
  if (!_currentCommand || !_currentCommand->active) {
    return;
  }
  
  // Update statistics
  switch (type) {
    case ATResponseType::OK:
    case ATResponseType::CUSTOM:
      _stats.commandsOK++;
      break;
    case ATResponseType::ERROR:
      _stats.commandsError++;
      break;
    case ATResponseType::TIMEOUT:
      _stats.commandsTimeout++;
      break;
    default:
      break;
  }
  
  // Call callback if provided
  if (_currentCommand->callback) {
    _currentCommand->callback(type, _currentCommand->response, _currentCommand->userData);
  }
  
  if (_debug) {
    const char* typeStr = "UNKNOWN";
    switch (type) {
      case ATResponseType::OK: typeStr = "OK"; break;
      case ATResponseType::ERROR: typeStr = "ERROR"; break;
      case ATResponseType::TIMEOUT: typeStr = "TIMEOUT"; break;
      case ATResponseType::CUSTOM: typeStr = "CUSTOM"; break;
      default: break;
    }
    debugPrint("COMPLETE", typeStr);
  }
  
  // Clear current command
  _currentCommand->active = false;
  _currentCommand = nullptr;
  
  // Move to next command
  _queueTail = (_queueTail + 1) % AT_QUEUE_SIZE;
  _queueCount--;
}

// Debug print helper
void ATCommandManager::debugPrint(const char* prefix, const char* msg) {
  if (!_debug) return;
  
  Serial.print("[AT:");
  Serial.print(prefix);
  Serial.print("] ");
  Serial.println(msg);
}
