#include "Sim7070G_AT.h"
#include "DEBUG.h"
#include "Adafruit_SleepyDog.h"

Sim7070G_AT::Sim7070G_AT(HardwareSerial* serial, size_t rxBufferSize)
  : _serial(serial),
    _rxBuffer(nullptr),
    _rxBufferSize(rxBufferSize),
    _rxHead(0),
    _rxTail(0),
    _rxCount(0),
    _queueHead(0),
    _queueTail(0),
    _queueCount(0),
    _currentCommand(nullptr),
    _urcHandlerCount(0),
    _responsePos(0)
{
  _rxBuffer = (uint8_t*)malloc(rxBufferSize);
  if (!_rxBuffer) {
    _rxBufferSize = 0;
  }
  
  memset(_commandQueue, 0, sizeof(_commandQueue));
  memset(_urcHandlers, 0, sizeof(_urcHandlers));
  memset(_lastError, 0, sizeof(_lastError));
  memset(_responseBuffer, 0, sizeof(_responseBuffer));
}

Sim7070G_AT::~Sim7070G_AT() {
  if (_rxBuffer) {
    free(_rxBuffer);
    _rxBuffer = nullptr;
  }
}

bool Sim7070G_AT::begin(uint32_t baudRate) {
  if (!_serial || !_rxBuffer) {
    setError("Invalid serial or buffer");
    return false;
  }
  
  _serial->begin(baudRate);
  delay(50);
  
  // Clear any pending data
  while (_serial->available()) {
    _serial->read();
  }
  
  return true;
}

void Sim7070G_AT::loop() {
  // Read incoming data from serial
  processRxData();
  
  // Process responses
  processResponse();
  
  // Check for command timeout
  if (_currentCommand && _currentCommand->waiting) {
    unsigned long now = millis();
    if ((now - _currentCommand->sentTime) >= _currentCommand->timeout) {
      DEBUG_PRINTLN(F("[AT] Command timeout"));
      completeCommand(ATResponseType::TIMEOUT, nullptr);
    }
  }
  
  // Send next command if queue is not empty and no command is waiting
  if (_queueCount > 0 && (!_currentCommand || !_currentCommand->waiting)) {
    sendNextCommand();
  }
}

int Sim7070G_AT::sendCommand(const char* command, unsigned long timeout, 
                             ATCallback callback, void* userData) {
  if (!command || strlen(command) == 0) {
    setError("Invalid command");
    return -1;
  }
  
  if (_queueCount >= MAX_QUEUE_SIZE) {
    setError("Command queue full");
    return -1;
  }
  
  ATCommand* cmd = &_commandQueue[_queueHead];
  strncpy(cmd->command, command, sizeof(cmd->command) - 1);
  cmd->command[sizeof(cmd->command) - 1] = '\0';
  cmd->timeout = timeout;
  cmd->callback = callback;
  cmd->userData = userData;
  cmd->sentTime = 0;
  cmd->waiting = false;
  cmd->responseType = ATResponseType::NONE;
  cmd->response[0] = '\0';
  
  _queueHead = (_queueHead + 1) % MAX_QUEUE_SIZE;
  _queueCount++;
  
  // Try to send immediately if no command is waiting
  if (!_currentCommand || !_currentCommand->waiting) {
    sendNextCommand();
  }
  
  return _queueHead - 1;
}

bool Sim7070G_AT::sendCommandSync(const char* command, unsigned long timeout, 
                                   char* response, size_t responseSize) {
  if (!command) {
    return false;
  }
  
  bool result = false;
  unsigned long startTime = millis();
  
  // Send command - get the command that was just queued
  // The command is at _queueHead - 1 (before increment)
  uint8_t cmdIndex = _queueHead;
  if (sendCommand(command, timeout, nullptr, nullptr) < 0) {
    return false;
  }
  
  ATCommand* sentCmd = &_commandQueue[cmdIndex];
  
  // Wait for response
  while ((millis() - startTime) < timeout) {
    loop();
    Watchdog.reset();

    // Check if command completed (not waiting and has response type)
    if (!sentCmd->waiting && sentCmd->responseType != ATResponseType::NONE) {
      // Command completed
      if (sentCmd->responseType == ATResponseType::OK) {
        result = true;
        if (response && responseSize > 0 && sentCmd->response[0] != '\0') {
          strncpy(response, sentCmd->response, responseSize - 1);
          response[responseSize - 1] = '\0';
        }
      }
      break;
    }
    
    delay(10);
  }
  
  return result;
}

void Sim7070G_AT::registerURCHandler(const char* urcPrefix, URCHandler handler) {
  if (!urcPrefix || !handler || _urcHandlerCount >= MAX_URC_HANDLERS) {
    return;
  }
  
  URCHandlerEntry* entry = &_urcHandlers[_urcHandlerCount++];
  strncpy(entry->prefix, urcPrefix, sizeof(entry->prefix) - 1);
  entry->prefix[sizeof(entry->prefix) - 1] = '\0';
  entry->handler = handler;
}

bool Sim7070G_AT::isAlive() {
  return sendCommandSync("AT", 2000);
}

void Sim7070G_AT::clearQueue() {
  _queueCount = 0;
  _queueHead = 0;
  _queueTail = 0;
  _currentCommand = nullptr;
  _responsePos = 0;
  memset(_responseBuffer, 0, sizeof(_responseBuffer));
}

void Sim7070G_AT::processRxData() {
  while (_serial->available() && !rxBufferFull()) {
    rxBufferPut(_serial->read());
  }
}

void Sim7070G_AT::processResponse() {
  while (rxBufferAvailable() > 0) {
    uint8_t byte = rxBufferGet();
    
    if (byte == '\r') {
      continue; // Skip CR
    }
    
    if (byte == '\n') {
      // End of line
      if (_responsePos > 0) {
        _responseBuffer[_responsePos] = '\0';
        
        // Format: AT command (echo) -> DATA -> OK/ERROR
        // Step 1: Skip echo of command if it matches the sent command
        if (_currentCommand && _currentCommand->waiting) {
          if (strcmp(_responseBuffer, _currentCommand->command) == 0) {
            // This is echo of command, skip it
            _responsePos = 0;
            memset(_responseBuffer, 0, sizeof(_responseBuffer));
            continue;
          }
        }
        
        // Check if current command is a query command (ends with ?)
        bool isQueryCommand = false;
        if (_currentCommand && _currentCommand->waiting) {
          const char* cmd = _currentCommand->command;
          size_t cmdLen = strlen(cmd);
          if (cmdLen > 0 && cmd[cmdLen - 1] == '?') {
            isQueryCommand = true;
          }
        }
        
        // Step 2: Handle DATA lines (start with +)
        if (_responseBuffer[0] == '+') {
          // If there's a command waiting, treat + lines as DATA response, not URC
          if (_currentCommand && _currentCommand->waiting) {
            // This is a data response line - append to command's response buffer
            char* resp = _currentCommand->response;
            size_t respLen = strlen(resp);
            size_t remaining = sizeof(_currentCommand->response) - respLen - 1;
            
            if (remaining > 0) {
              if (respLen > 0) {
                // Add newline separator if not first line
                resp[respLen++] = '\n';
                remaining--;
              }
              strncpy(resp + respLen, _responseBuffer, remaining);
              resp[sizeof(_currentCommand->response) - 1] = '\0';
            }
          } else {
            // No command waiting, treat as URC
            handleURC(_responseBuffer);
          }
        } else {
          // Step 3: Handle OK/ERROR or other response lines
          ATResponseType type = parseResponse(_responseBuffer);
          if (type == ATResponseType::OK || type == ATResponseType::ERROR) {
            if (_currentCommand && _currentCommand->waiting) {
              // Use accumulated response if available, otherwise use current line
              const char* finalResponse = _responseBuffer;
              if (_currentCommand->response[0] != '\0') {
                finalResponse = _currentCommand->response;
              }
              completeCommand(type, finalResponse);
            }
          } else if (type == ATResponseType::DATA) {
            // Other data lines (not starting with +) - append to response buffer
            if (_currentCommand && _currentCommand->waiting) {
              char* resp = _currentCommand->response;
              size_t respLen = strlen(resp);
              size_t remaining = sizeof(_currentCommand->response) - respLen - 1;
              
              if (remaining > 0) {
                if (respLen > 0) {
                  resp[respLen++] = '\n';
                  remaining--;
                }
                strncpy(resp + respLen, _responseBuffer, remaining);
                resp[sizeof(_currentCommand->response) - 1] = '\0';
              }
            }
          }
        }
        
        _responsePos = 0;
        memset(_responseBuffer, 0, sizeof(_responseBuffer));
      }
    } else {
      // Add to response buffer
      if (_responsePos < (sizeof(_responseBuffer) - 1)) {
        _responseBuffer[_responsePos++] = byte;
      }
    }
  }
}

ATResponseType Sim7070G_AT::parseResponse(const char* line) {
  if (!line || strlen(line) == 0) {
    return ATResponseType::NONE;
  }
  
  // Check for OK
  if (strcmp(line, "OK") == 0) {
    return ATResponseType::OK;
  }
  
  // Check for ERROR
  if (strcmp(line, "ERROR") == 0 || strncmp(line, "+CME ERROR:", 11) == 0 || 
      strncmp(line, "+CMS ERROR:", 11) == 0) {
    return ATResponseType::ERROR;
  }
  
  // Check for URC (starts with +)
  if (line[0] == '+') {
    return ATResponseType::URC;
  }
  
  // Data response
  return ATResponseType::DATA;
}

void Sim7070G_AT::handleURC(const char* line) {
  if (!line || line[0] != '+') {
    return;
  }
  
  // Find URC handler
  URCHandler handler = nullptr;
  if (findURCHandler(line, &handler)) {
    if (handler) {
      // Extract URC prefix and data
      const char* colon = strchr(line, ':');
      if (colon) {
        char urc[32];
        size_t len = colon - line;
        if (len < sizeof(urc)) {
          strncpy(urc, line, len);
          urc[len] = '\0';
          handler(urc, colon + 1);
        }
      } else {
        handler(line, nullptr);
      }
    }
  }
  
  // If there's a waiting command, URC might be part of response
  // Don't complete command yet, wait for OK/ERROR
}

bool Sim7070G_AT::findURCHandler(const char* urc, URCHandler* handler) {
  if (!urc || !handler) {
    return false;
  }
  
  // Extract prefix (up to first colon or comma)
  char prefix[16];
  const char* sep = strchr(urc, ':');
  if (!sep) {
    sep = strchr(urc, ',');
  }
  
  size_t len = sep ? (sep - urc) : strlen(urc);
  if (len >= sizeof(prefix)) {
    len = sizeof(prefix) - 1;
  }
  strncpy(prefix, urc, len);
  prefix[len] = '\0';
  
  // Find matching handler
  for (uint8_t i = 0; i < _urcHandlerCount; i++) {
    if (strncmp(_urcHandlers[i].prefix, prefix, sizeof(prefix)) == 0) {
      *handler = _urcHandlers[i].handler;
      return true;
    }
  }
  
  return false;
}

void Sim7070G_AT::sendNextCommand() {
  if (_queueCount == 0) {
    return;
  }
  
  ATCommand* cmd = &_commandQueue[_queueTail];
  _currentCommand = cmd;
  
  // Send command
  _serial->print(cmd->command);
  _serial->print("\r\n");
  _serial->flush();
  
  cmd->sentTime = millis();
  cmd->waiting = true;
  
  DEBUG_PRINT(F("[AT] TX: "));
  DEBUG_PRINTLN(cmd->command);
  
  // Remove from queue
  _queueTail = (_queueTail + 1) % MAX_QUEUE_SIZE;
  _queueCount--;
  
  // Clear response buffer
  _responsePos = 0;
  memset(_responseBuffer, 0, sizeof(_responseBuffer));
}

void Sim7070G_AT::completeCommand(ATResponseType type, const char* response) {
  if (!_currentCommand) {
    return;
  }
  
  ATCommand* cmd = _currentCommand;
  cmd->waiting = false;
  cmd->responseType = type;
  
  // Save response data
  if (response) {
    strncpy(cmd->response, response, sizeof(cmd->response) - 1);
    cmd->response[sizeof(cmd->response) - 1] = '\0';
  } else {
    cmd->response[0] = '\0';
  }
  
  DEBUG_PRINT(F("[AT] RX: "));
  if (response) {
    DEBUG_PRINTLN(response);
  } else {
    DEBUG_PRINTLN(F("(timeout)"));
  }
  
  // Call callback if set
  if (cmd->callback) {
    cmd->callback(type, response, cmd->userData);
  }
  
  _currentCommand = nullptr;
}

void Sim7070G_AT::setError(const char* error) {
  if (error) {
    strncpy(_lastError, error, sizeof(_lastError) - 1);
    _lastError[sizeof(_lastError) - 1] = '\0';
  } else {
    _lastError[0] = '\0';
  }
}

// Ring buffer helpers
bool Sim7070G_AT::rxBufferFull() const {
  return _rxCount >= _rxBufferSize;
}

bool Sim7070G_AT::rxBufferEmpty() const {
  return _rxCount == 0;
}

void Sim7070G_AT::rxBufferPut(uint8_t byte) {
  if (rxBufferFull()) {
    return; // Buffer full, drop byte
  }
  
  _rxBuffer[_rxHead] = byte;
  _rxHead = (_rxHead + 1) % _rxBufferSize;
  _rxCount++;
}

uint8_t Sim7070G_AT::rxBufferGet() {
  if (rxBufferEmpty()) {
    return 0;
  }
  
  uint8_t byte = _rxBuffer[_rxTail];
  _rxTail = (_rxTail + 1) % _rxBufferSize;
  _rxCount--;
  
  return byte;
}

size_t Sim7070G_AT::rxBufferAvailable() const {
  return _rxCount;
}
