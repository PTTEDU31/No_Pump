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
  MODEM_POWER_ON,                    // Bật nguồn
  MODEM_ACTIVATE_CNACT_FIRST,         // Kích hoạt CNACT=1,1 lần đầu
  MODEM_WAIT_FOR_IP_FIRST,            // Đợi 10s sau khi kích hoạt CNACT lần đầu
  MODEM_CHECK_IP_FIRST,               // Kiểm tra IP lần đầu
  MODEM_APPLY_BOOT_CONFIG,            // Áp dụng boot config (nếu IP check fail)
  MODEM_CHECK_SIM_AFTER_CONFIG,       // Kiểm tra SIM sau config
  MODEM_CHECK_SIGNAL_AFTER_CONFIG,     // Kiểm tra tín hiệu sau config
  MODEM_SET_APN,                      // Set APN
  MODEM_WAIT_FOR_IP_AFTER_APN,        // Đợi 10s sau khi set APN
  MODEM_CHECK_IP_AFTER_APN,           // Kiểm tra IP sau khi set APN
  MODEM_SYNC_NTP_TIME,                // Sync NTP time after GPRS connected
  MODEM_MQTT_CONNECTING_HANDSHAKE,     // Cấu hình MQTT
  MODEM_MQTT_CONNECTING,               // Kết nối MQTT
  MODEM_MQTT_CONNECTED_FIRST,         // MQTT connected lần đầu
  MODEM_MQTT_CONNECTED,               // MQTT connected
  MODEM_WAITING_SUCCESS,              // Đợi cờ waiting done rồi chuyển sang state chỉ định
  MODEM_ERROR
};

// Message buffer structure (renamed to avoid conflict with Sim7070G::MQTTMessage)
#define MQTT_BUFFER_PAYLOAD_MAX 1024  // Telemetry hex payload can be ~900 bytes
struct MQTTBufferedMessage {
  unsigned long timestamp;  // millis() when message was queued
  char topic[96];          // MQTT topic
  char payload[MQTT_BUFFER_PAYLOAD_MAX];  // Message payload (telemetry hex)
  uint8_t qos;             // Quality of Service
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
  
  // Telemetry / MQTT helpers
  Sim7070G* getModem() { return _modem; }
  bool isMqttConnected() const;
  bool publishTelemetryNow(const char* payload);
  /** Enqueue telemetry to MQTT buffer; sent automatically when connected */
  bool enqueueTelemetry(const char* payload);
  int getCSQ();  // 0-31, -1 on fail
  uint32_t getModemRestarts() const { return _stats.modemRestarts; }
  uint32_t getGprsConnects() const { return _stats.gprsConnects; }
  uint32_t getMqttConnects() const { return _stats.mqttConnects; }

  /** Chuyển sang trạng thái WAITING_SUCCESS; khi setWaitingDone() được gọi sẽ chuyển sang \p nextState */
  void gotoWaitingSuccess(ModemState nextState);
  /** Set cờ "waiting done" – khi đang ở MODEM_WAITING_SUCCESS, lần timeout() tiếp theo sẽ chuyển sang state đã chỉ định trong gotoWaitingSuccess(). */
  void setWaitingDone();

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

  // Waiting success state: đợi cờ done rồi chuyển sang state chỉ định
  bool _waitingDone = false;
  ModemState _waitingSuccessNextState = MODEM_OFF;

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
  void handleMqttCommandJson(const char* plain);
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
