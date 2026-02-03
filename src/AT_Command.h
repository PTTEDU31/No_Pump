#ifndef SIM7070G_AT_H
#define SIM7070G_AT_H

#include <Arduino.h>
#include "config.h"

enum class ATResponseType {
  NONE,
  OK,
  ERROR,
  URC,
  DATA,
  TIMEOUT
};

typedef void (*ATCallback)(ATResponseType type, const char* response, void* userData);

struct ATCommand {
  char command[128];
  unsigned long timeout;
  ATCallback callback;
  void* userData;
  unsigned long sentTime;
  bool waiting;
  ATResponseType responseType;
  char response[256];
};

typedef void (*URCHandler)(const char* urc, const char* data);

class Sim7070G_AT {
public:
  Sim7070G_AT(HardwareSerial* serial, size_t rxBufferSize = 512);
  ~Sim7070G_AT();

  bool begin(uint32_t baudRate = MODEM_BAUD_RATE);
  void loop();
  int sendCommand(const char* command, unsigned long timeout = 5000, 
                  ATCallback callback = nullptr, void* userData = nullptr);
  bool sendCommandSync(const char* command, unsigned long timeout = 5000, 
                       char* response = nullptr, size_t responseSize = 0);
  void registerURCHandler(const char* urcPrefix, URCHandler handler);
  uint8_t getPendingCommandCount() const { return _queueCount; }
  bool isAlive();
  void clearQueue();
  const char* getLastError() const { return _lastError; }
  void setUnsolicitedResponseCallback(ATCallback callback, void* userData);

private:
  HardwareSerial* _serial;
  uint8_t* _rxBuffer;
  size_t _rxBufferSize;
  size_t _rxHead;
  size_t _rxTail;
  size_t _rxCount;

  static const uint8_t MAX_QUEUE_SIZE = 8;
  ATCommand _commandQueue[MAX_QUEUE_SIZE];
  uint8_t _queueHead;
  uint8_t _queueTail;
  uint8_t _queueCount;
  ATCommand* _currentCommand;
  ATCallback _unsolicitedResponseCallback;
  void* _unsolicitedResponseUserData;
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

  void processRxData();
  void processResponse();
  ATResponseType parseResponse(const char* line);
  void handleURC(const char* line);
  bool findURCHandler(const char* urc, URCHandler* handler);
  void sendNextCommand();
  void completeCommand(ATResponseType type, const char* response);
  void setError(const char* error);
  bool rxBufferFull() const;
  bool rxBufferEmpty() const;
  void rxBufferPut(uint8_t byte);
  uint8_t rxBufferGet();
  size_t rxBufferAvailable() const;
};

#endif // SIM7070G_AT_H
