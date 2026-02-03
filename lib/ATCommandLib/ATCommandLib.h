/**
 * @file ATCommandLib.h
 * @brief AT Command Manager with Callback and URC Support
 * @version 1.0
 * @date 2026-02-03
 * 
 * Features:
 * - Non-blocking async command execution
 * - Command queue management
 * - Timeout handling per command
 * - URC (Unsolicited Result Code) handlers
 * - Callback mechanism for responses
 * - Statistics tracking
 */

#ifndef AT_COMMAND_LIB_H
#define AT_COMMAND_LIB_H

#include <Arduino.h>

// Import configuration from config.h if available
// Otherwise use default values
#ifdef CONFIG_H
  // Configuration already defined in config.h
#else
  // Default configuration if config.h not included
  #ifndef AT_QUEUE_SIZE
  #define AT_QUEUE_SIZE 8
  #endif

  #ifndef AT_MAX_URC_HANDLERS
  #define AT_MAX_URC_HANDLERS 16
  #endif

  #ifndef AT_RX_BUFFER_SIZE
  #define AT_RX_BUFFER_SIZE 512
  #endif

  #ifndef AT_MAX_CMD_LENGTH
  #define AT_MAX_CMD_LENGTH 128
  #endif

  #ifndef AT_MAX_RESPONSE_LENGTH
  #define AT_MAX_RESPONSE_LENGTH 512
  #endif
#endif

// Response types
enum class ATResponseType {
  NONE,           // No response yet
  OK,             // "OK" received
  ERROR,          // "ERROR" or "+CME ERROR" received
  TIMEOUT,        // Command timed out
  CUSTOM          // Custom response (for specific commands)
};

// Callback function types
typedef void (*ATCallback)(ATResponseType type, const char* response, void* userData);
typedef void (*URCHandler)(const char* urcPrefix, const char* data, void* userData);

// Command structure
struct ATCommand {
  char cmd[AT_MAX_CMD_LENGTH];                    // AT command string
  unsigned long timeout;                          // Timeout in milliseconds
  unsigned long sentTime;                         // When command was sent
  ATCallback callback;                            // Callback function
  void* userData;                                 // User data for callback
  char response[AT_MAX_RESPONSE_LENGTH];          // Accumulated response
  bool active;                                    // Currently executing
  uint16_t responsePos;                           // Current position in response buffer
};

// URC handler entry
struct URCEntry {
  char prefix[32];                                // URC prefix (e.g., "+SMSUB:", "+APP PDP:")
  URCHandler handler;                             // Handler function
  void* userData;                                 // User data for handler
  bool active;                                    // Active flag
};

// Statistics
struct ATStats {
  uint32_t commandsSent;
  uint32_t commandsOK;
  uint32_t commandsError;
  uint32_t commandsTimeout;
  uint32_t urcsReceived;
  uint32_t bytesReceived;
  uint32_t bytesSent;
};

/**
 * @class ATCommandManager
 * @brief Manages AT commands with non-blocking execution and URC handling
 */
class ATCommandManager {
public:
  /**
   * @brief Constructor
   * @param serial Pointer to HardwareSerial (e.g., &Serial1)
   */
  ATCommandManager(Stream* serial);
  
  /**
   * @brief Destructor
   */
  ~ATCommandManager();
  
  /**
   * @brief Initialize the AT command manager
   * @return true if successful
   */
  bool begin();
  
  /**
   * @brief Main loop - MUST be called frequently from loop()
   * This processes serial data, checks timeouts, and sends queued commands
   */
  void loop();
  
  /**
   * @brief Send AT command asynchronously
   * @param cmd AT command string (without \r\n)
   * @param timeout Timeout in milliseconds
   * @param callback Callback function (nullptr if not needed)
   * @param userData User data passed to callback
   * @return true if command was queued successfully
   */
  bool sendCommandAsync(const char* cmd, unsigned long timeout = 5000,
                        ATCallback callback = nullptr, void* userData = nullptr);
  
  /**
   * @brief Send AT command synchronously (blocking)
   * @param cmd AT command string
   * @param expectedResponse Expected response string (e.g., "OK", "+CSQ:")
   * @param timeout Timeout in milliseconds
   * @param response Buffer to store response (optional)
   * @param responseSize Size of response buffer
   * @return true if expected response received
   */
  bool sendCommandSync(const char* cmd, const char* expectedResponse,
                      unsigned long timeout = 5000,
                      char* response = nullptr, size_t responseSize = 0);
  
  /**
   * @brief Register URC (Unsolicited Result Code) handler
   * @param prefix URC prefix to match (e.g., "+SMSUB:", "+APP PDP:")
   * @param handler Handler function
   * @param userData User data passed to handler
   * @return true if registered successfully
   */
  bool registerURCHandler(const char* prefix, URCHandler handler, void* userData = nullptr);
  
  /**
   * @brief Unregister URC handler
   * @param prefix URC prefix to unregister
   */
  void unregisterURCHandler(const char* prefix);
  
  /**
   * @brief Check if modem is alive (responds to AT)
   * @param timeout Timeout in milliseconds
   * @return true if modem responds with OK
   */
  bool isAlive(unsigned long timeout = 1000);
  
  /**
   * @brief Clear all pending commands in queue
   */
  void clearQueue();
  
  /**
   * @brief Get number of pending commands
   * @return Number of commands in queue
   */
  uint8_t getPendingCommandCount() const { return _queueCount; }
  
  /**
   * @brief Check if a command is currently executing
   * @return true if command is active
   */
  bool isCommandActive() const { return _currentCommand != nullptr && _currentCommand->active; }
  
  /**
   * @brief Get statistics
   * @return Const reference to stats structure
   */
  const ATStats& getStats() const { return _stats; }
  
  /**
   * @brief Reset statistics
   */
  void resetStats();
  
  /**
   * @brief Enable/disable debug output
   * @param enable true to enable debug
   */
  void setDebug(bool enable) { _debug = enable; }
  
  /**
   * @brief Set custom line terminator (default is "OK")
   * Useful for commands that don't return OK (e.g., AT+CGDCONT?)
   */
  void setCustomTerminator(const char* terminator) {
    if (terminator && strlen(terminator) < 16) {
      strncpy(_customTerminator, terminator, sizeof(_customTerminator) - 1);
      _customTerminator[sizeof(_customTerminator) - 1] = '\0';
      _useCustomTerminator = true;
    } else {
      _useCustomTerminator = false;
    }
  }
  
  void clearCustomTerminator() {
    _useCustomTerminator = false;
  }

private:
  Stream* _serial;
  bool _debug;
  
  // RX Buffer for line-by-line processing
  char _rxBuffer[AT_RX_BUFFER_SIZE];
  uint16_t _rxPos;
  
  // Command queue (circular buffer)
  ATCommand _commandQueue[AT_QUEUE_SIZE];
  uint8_t _queueHead;
  uint8_t _queueTail;
  uint8_t _queueCount;
  ATCommand* _currentCommand;
  
  // URC handlers
  URCEntry _urcHandlers[AT_MAX_URC_HANDLERS];
  uint8_t _urcCount;
  
  // Statistics
  ATStats _stats;
  
  // Custom terminator
  char _customTerminator[16];
  bool _useCustomTerminator;
  
  // Sync command state tracking
  struct SyncCommandState {
    bool active;
    bool completed;
    ATResponseType result;
    char response[AT_MAX_RESPONSE_LENGTH];
    const char* expectedResponse;
  };
  SyncCommandState _syncState;
  
  // Internal methods
  void processSerial();
  void processLine(const char* line);
  void checkTimeout();
  bool matchURC(const char* line);
  bool enqueueCommand(const ATCommand& cmd);
  bool dequeueCommand();
  void completeCommand(ATResponseType type);
  void debugPrint(const char* prefix, const char* msg);
  
  // Sync helper: Callback for sync commands
  static void syncCommandCallback(ATResponseType type, const char* response, void* userData);
};

#endif // AT_COMMAND_LIB_H
