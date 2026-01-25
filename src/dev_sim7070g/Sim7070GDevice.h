#ifndef SIM7070G_DEVICE_H
#define SIM7070G_DEVICE_H

#include <Arduino.h>
#include "device.h"
#include <Sim7070G.h>

// SIM7070G Configuration
#define MODEM_BAUD_RATE 19200
#define MODEM_POWER_PIN 12

// Network configuration
#define PDP_CONTEXT_ID 0 // 0 is the default context
#define APN_NAME "iot.1nce.net"

// MQTT Configuration
#define MQTT_MAX_RECONNECT_ATTEMPTS 3
#define MQTT_KEEPALIVE_SEC 60
#define MQTT_BUFFER_SIZE 10  // Maximum number of messages to buffer


// Connection states
enum ModemState {
  MODEM_OFF = 0,
  MODEM_INITIALIZING,
  MODEM_RESET,
  MODEM_READY,
  MODEM_CHECK_SIM,
  MODEM_GPRS_CONNECTING_SET_BAUD,
  MODEM_GPRS_CONNECTING_SET_MODE,
  MODEM_GPRS_CONNECTING_WAIT_MODE,
  MODEM_GPRS_CONNECTING_CHECK_SIGNAL,
  MODEM_GPRS_CONNECTING_ATTACH,
  MODEM_GPRS_CONNECTED,
  MODEM_GPRS_WAIT_CONNECTION,
  MODEM_GPRS_IP_CHECK,
  MODEM_QUERY_NETWORK_INFO,  // Query network info async
  MODEM_MQTT_CONNECTING_HANDSHAKE,
  MODEM_MQTT_CONNECTING,
  MODEM_MQTT_CONNECTED_FIRST,
  MODEM_GPRS_CONNECTING_PDP_CONTEXT,
  MODEM_MQTT_CONNECTED,
  MODEM_ERROR
};

// Message buffer structure (renamed to avoid conflict with Sim7070G::MQTTMessage)
struct MQTTBufferedMessage {
  unsigned long timestamp;  // millis() when message was queued
  char topic[96];          // MQTT topic
  char payload[256];       // Message payload
  uint8_t qos;            // Quality of Service
};

// Statistics structure
struct ModemStats {
  uint32_t modemRestarts;
  uint32_t gprsConnects;
  uint32_t mqttConnects;
  uint32_t mqttDisconnects;
  uint32_t atCommandsSent;
  uint32_t atCommandsFailed;
  int8_t lastRSSI;
  unsigned long lastGPRSConnectTime;
  unsigned long lastMQTTConnectTime;
};

/**
 * @brief SIM7070G Modem Device
 * Manages cellular connectivity and MQTT connection
 */
class Sim7070GDevice : public Device {
public:
  // Constructor
  Sim7070GDevice(HardwareSerial* modemSerial, const char* nodeId);
  
  // Destructor
  ~Sim7070GDevice();
  
  // Override Device methods
  bool initialize() override;
  int start() override;
  int timeout() override;
  int event() override;
  
private:
  static Sim7070GDevice* s_instance;
  static void mqttMessageThunk(const char* topic, const uint8_t* payload, uint32_t len);

  HardwareSerial* _modemSerial;
  Sim7070G* _modem;
  bool _mqttBegun = false;
  
  const char* _nodeId;
  char _pubTopic[96];
  char _subTopic[96];
  
  ModemState _state;
  ModemStats _stats;
  
  // MQTT credentials
  char _mqttBroker[64];
  uint16_t _mqttPort;
  char _mqttClientId[64];
  char _mqttUsername[64];
  char _mqttPassword[64];
  
  // GPRS credentials
  char _apn[64];
  char _gprsUser[64];
  char _gprsPass[64];
  // NetworkMode _networkMode;
  
  // Timing
  unsigned long _lastHealthCheck;
  unsigned long _lastNetworkCheck;
  unsigned long _lastStateChange;
  unsigned long _stateStartTime;  // Timestamp when entering current state
  uint8_t _retryCount;  // Retry counter for state operations
  unsigned long _lastHeartbeatMs = 0;
  
  // Message buffer
  MQTTBufferedMessage _messageBuffer[MQTT_BUFFER_SIZE];
  uint8_t _bufferHead;   // Write position
  uint8_t _bufferTail;   // Read position
  uint8_t _bufferCount;  // Number of messages in buffer
  
  // Internal methods
  bool applyBootConfig();
  bool checkModemAlive();
  void updateState(ModemState newState);
  void handleIncomingData();
  void onMqttMessage(const char* topic, const uint8_t* payload, uint32_t len);
  void powerPulse();
  bool ensureNetworkReady();
  bool ensureMqttReady();
  
  // Async command callback handlers
  static void onQueryNetworkInfoCallback(ATResponseType type, const char* response, void* userData);
  static void onMqttConnectCallback(ATResponseType type, const char* response, void* userData);
  bool _networkInfoQueryPending;  // Flag to track if query is pending
  bool _networkInfoQueryCompleted; // Flag to track if query completed
  bool _mqttConnectPending;  // Flag to track if MQTT connect is pending
  bool _mqttConnectCompleted; // Flag to track if MQTT connect completed
  
  // Buffer management methods
  bool enqueueMessage(const char* topic, const char* payload, uint8_t qos);
  bool dequeueMessage(MQTTBufferedMessage& msg);
  void clearMessageBuffer();
  bool flushMessageBuffer();
};

#endif // SIM7070G_DEVICE_H
