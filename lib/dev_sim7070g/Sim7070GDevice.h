#ifndef SIM7070G_DEVICE_H
#define SIM7070G_DEVICE_H

#include <Arduino.h>
#include "device.h"
#include <DFRobot_SIM7070G.h>

// SIM7070G Configuration
#define MODEM_BAUD_RATE 115200
#define MODEM_POWER_PIN 12

// Network configuration
#define PDP_CONTEXT_ID 1
#define APN_NAME "iot.1nce.net"

// MQTT Configuration
#define MQTT_MAX_RECONNECT_ATTEMPTS 3
#define MQTT_KEEPALIVE_SEC 60

// Network mode enumeration
enum NetworkMode {
  NET_MODE_GPRS = 0,
  NET_MODE_NB = 1
};

// Connection states
enum ModemState {
  MODEM_OFF = 0,
  MODEM_INITIALIZING,
  MODEM_READY,
  MODEM_GPRS_CONNECTING,
  MODEM_GPRS_CONNECTING_SET_MODE,
  MODEM_GPRS_CONNECTING_WAIT_MODE,
  MODEM_GPRS_CONNECTING_ATTACH,
  MODEM_GPRS_CONNECTING_WAIT_ATTACH,
  MODEM_GPRS_CONNECTED,
  MODEM_MQTT_CONNECTING,
  MODEM_MQTT_CONNECTING_OPEN_NET,
  MODEM_MQTT_CONNECTING_WAIT_OPEN,
  MODEM_MQTT_CONNECTING_HANDSHAKE,
  MODEM_MQTT_CONNECTING_WAIT_HANDSHAKE,
  MODEM_MQTT_CONNECTED,
  MODEM_ERROR
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
  
  // Override Device methods
  bool initialize() override;
  int start() override;
  int timeout() override;
  int event() override;
  
  // Modem control
  bool powerOn();
  bool powerOff();
  bool restart();
  
  // Network management
  bool connectGPRS();
  bool disconnectGPRS();
  bool isGPRSConnected();
  
  bool connectNB();
  bool disconnectNB();
  bool isNBConnected();
  
  // MQTT management
  bool connectMQTT(const char* broker, uint16_t port, 
                   const char* clientId, const char* username, const char* password);
  bool disconnectMQTT();
  bool isMQTTConnected();
  bool publish(const char* topic, const char* payload, uint8_t qos = 0);
  bool subscribe(const char* topic, uint8_t qos = 0);
  bool unsubscribe(const char* topic);
  bool receiveMQTT(char* topic, char* payload, int maxLen);
  
  // AT commands
  bool sendAT(const char* cmd, unsigned long timeout = 1000);
  bool sendATwaitOK(const char* cmd, char* response, size_t responseSize, unsigned long timeout = 2500);
  
  // Network info
  int8_t getRSSI();
  bool getNetworkTime(char* isoTime, size_t size);
  
  // State management
  ModemState getState() const { return _state; }
  const char* getStateString() const;
  
  // Statistics
  ModemStats getStats() const { return _stats; }
  void resetStats();
  
  // Configuration
  void setTopics(const char* pubTopic, const char* subTopic);
  void setMQTTCredentials(const char* broker, uint16_t port, 
                          const char* clientId, const char* username, const char* password);
  void setNetworkCredentials(const char* apn, const char* user, const char* pass, NetworkMode mode = NET_MODE_GPRS);
  void setGPRSCredentials(const char* apn, const char* user, const char* pass) { 
    setNetworkCredentials(apn, user, pass, NET_MODE_GPRS); 
  }
  
private:
  HardwareSerial* _modemSerial;
  DFRobot_SIM7070G* _modem;
  
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
  NetworkMode _networkMode;
  
  // Timing
  unsigned long _lastHealthCheck;
  unsigned long _lastNetworkCheck;
  unsigned long _lastStateChange;
  unsigned long _stateStartTime;  // Timestamp when entering current state
  
  // Internal methods
  bool applyBootConfig();
  bool checkModemAlive();
  void updateState(ModemState newState);
  bool waitForResponse(const char* expected, unsigned long timeout);
  void handleIncomingData();
};

#endif // SIM7070G_DEVICE_H
