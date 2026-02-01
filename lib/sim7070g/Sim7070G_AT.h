#ifndef SIM7070G_AT_H
#define SIM7070G_AT_H

#include <Arduino.h>
#include "config.h"

// AT Response types
enum class ATResponseType {
  NONE,
  OK,
  ERROR,
  URC,           // Unsolicited Result Code
  DATA,          // Data response (e.g., +CSQ: 20,5)
  TIMEOUT
};

// AT Command callback function type
typedef void (*ATCallback)(ATResponseType type, const char* response, void* userData);

// AT Command structure
struct ATCommand {
  char command[128];        // AT command string
  unsigned long timeout;    // Timeout in milliseconds
  ATCallback callback;     // Callback function
  void* userData;          // User data for callback
  unsigned long sentTime;  // When command was sent
  bool waiting;            // Waiting for response
  ATResponseType responseType;  // Response type (OK, ERROR, TIMEOUT)
  char response[256];       // Response data
};

// URC (Unsolicited Result Code) handler
typedef void (*URCHandler)(const char* urc, const char* data);

class Sim7070G_AT {
public:
  Sim7070G_AT(HardwareSerial* serial, size_t rxBufferSize = 512);
  ~Sim7070G_AT();

  // Initialize
  bool begin(uint32_t baudRate = MODEM_BAUD_RATE);

  // Main loop - call this frequently (non-blocking)
  void loop();

  // Send AT command (async)
  // Returns command ID (>=0) on success, -1 on failure
  int sendCommand(const char* command, unsigned long timeout = 5000, 
                  ATCallback callback = nullptr, void* userData = nullptr);

  // Send AT command and wait for response (semi-blocking, with timeout)
  // Returns true if OK received, false otherwise
  bool sendCommandSync(const char* command, unsigned long timeout = 5000, 
                       char* response = nullptr, size_t responseSize = 0);

  // Register URC handler
  void registerURCHandler(const char* urcPrefix, URCHandler handler);

  // Get pending command count
  uint8_t getPendingCommandCount() const { return _queueCount; }

  // Check if modem is responding
  bool isAlive();

  // Clear all pending commands
  void clearQueue();

  // Get last error
  const char* getLastError() const { return _lastError; }

  // Callback when OK/ERROR received with no command waiting (e.g. MQTT publish response)
  void setUnsolicitedResponseCallback(ATCallback callback, void* userData);

private:
  HardwareSerial* _serial;
  uint8_t* _rxBuffer;
  size_t _rxBufferSize;
  size_t _rxHead;
  size_t _rxTail;
  size_t _rxCount;

  // Command queue
  static const uint8_t MAX_QUEUE_SIZE = 8;
  ATCommand _commandQueue[MAX_QUEUE_SIZE];
  uint8_t _queueHead;
  uint8_t _queueTail;
  uint8_t _queueCount;
  ATCommand* _currentCommand;

  // Unsolicited OK/ERROR callback (e.g. after MQTT publish payload)
  ATCallback _unsolicitedResponseCallback;
  void* _unsolicitedResponseUserData;

  // URC handlers
  static const uint8_t MAX_URC_HANDLERS = 16;
  struct URCHandlerEntry {
    char prefix[16];
    URCHandler handler;
  };
  URCHandlerEntry _urcHandlers[MAX_URC_HANDLERS];
  uint8_t _urcHandlerCount;

  char _lastError[64];
  char _responseBuffer[256];
  size_t _responsePos;

  // Internal methods
  void processRxData();
  void processResponse();
  ATResponseType parseResponse(const char* line);
  void handleURC(const char* line);
  bool findURCHandler(const char* urc, URCHandler* handler);
  void sendNextCommand();
  void completeCommand(ATResponseType type, const char* response);
  void setError(const char* error);
  
  // Ring buffer helpers
  bool rxBufferFull() const;
  bool rxBufferEmpty() const;
  void rxBufferPut(uint8_t byte);
  uint8_t rxBufferGet();
  size_t rxBufferAvailable() const;
};

#endif // SIM7070G_AT_H
