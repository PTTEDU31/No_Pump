#ifndef SIM7070G_H
#define SIM7070G_H

#include <Arduino.h>
#include "Sim7070G_AT.h"
#include "config.h"

// Network states
enum class Sim7070GState {
  IDLE,
  INITIALIZING,
  CHECKING_SIM,
  CONFIGURING_NETWORK,
  CONNECTING_NETWORK,
  NETWORK_CONNECTED,
  NETWORK_DISCONNECTED,
  MQTT_CONNECTING,
  MQTT_CONNECTED,
  MQTT_DISCONNECTED,
  ERROR
};

// Network registration status
enum class NetworkRegStatus {
  NOT_REGISTERED,
  REGISTERED_HOME,
  SEARCHING,
  REGISTRATION_DENIED,
  UNKNOWN,
  REGISTERED_ROAMING
};

// MQTT states
enum class MQTTState {
  DISCONNECTED,
  CONNECTING,
  CONNECTED,
  DISCONNECTING,
  ERROR
};

// Callback types
typedef void (*NetworkStateCallback)(Sim7070GState state);
typedef void (*MQTTMessageCallback)(const char* topic, const uint8_t* payload, uint32_t len);
typedef void (*MQTTStateCallback)(MQTTState state);
typedef void (*HTTPResponseCallback)(int statusCode, const char* response, size_t len, void* userData);
typedef void (*SocketDataCallback)(int socketId, const uint8_t* data, size_t len, void* userData);

// Network info structure
struct NetworkInfo {
  int8_t rssi;                    // Signal strength (0-31, 99 = unknown)
  uint8_t ber;                    // Bit error rate
  NetworkRegStatus regStatus;     // Registration status
  char operatorName[32];          // Operator name
  char apn[64];                   // APN name
  char ipAddress[16];             // IP address
};

// MQTT message structure
struct MQTTMessage {
  char topic[128];
  uint8_t payload[512];
  size_t payloadLen;
  uint8_t qos;
  bool retain;
};

class Sim7070G {
public:
  Sim7070G(HardwareSerial* serial, uint8_t powerPin = 12, uint32_t baudRate = MODEM_BAUD_RATE);
  ~Sim7070G();

  // Initialization
  bool begin();
  void loop(); // Must be called frequently in main loop

  // Power management
  void powerOn();
  void powerOff();
  bool isPoweredOn();
  
  // Check if modem is alive with custom timeout (for boot checks)
  bool isAlive(unsigned long timeout = 5000);

  // Network management
  bool checkSIM();
  bool setPreferredRAT(uint8_t rat); // 1=CAT-M, 2=NB-IoT, 3=Both (AT+CMNB)
  bool setNetworkMode(uint8_t mode); // 2=Automatic, 13=GSM only, 38=LTE only, 51=GSM and LTE (AT+CNMP)
  bool getSignalQuality(int8_t* rssi, uint8_t* ber);
  bool getNetworkRegistration(NetworkRegStatus* status);
  bool getAPN(char* apn, size_t len);
  bool setAPN(uint8_t cid, const char* apn, const char* username = nullptr, const char* password = nullptr);
  bool getSystemInfo(char* info, size_t len);
  Sim7070GState getState();
  // PDP Context activation - split into 3 parts
  bool attacthService(void);  // Step 1-3: Check SIM, RF signal, PS service
  bool queryNetworkInfo();           // Step 4: Query network information
  bool PDPContextActivation(uint8_t cid, const char* apn, 
                                    const char* username = nullptr, const char* password = nullptr);  // Step 5-8: APN, activate, get IP
  
  bool activatePDPContext(uint8_t cid = 0, const char* apn = nullptr, 
                          const char* username = nullptr, const char* password = nullptr);
  bool deactivatePDPContext(uint8_t cid = 0);
  bool getIPAddress(char* ip, size_t len);
  NetworkInfo getNetworkInfo();
  bool isNetworkConnected();

  // Network state callbacks
  void setNetworkStateCallback(NetworkStateCallback callback) { _networkStateCallback = callback; }

  // MQTT Client
  bool mqttBegin();
  bool mqttSetConfig(const char* clientId, const char* server, uint16_t port,
                    const char* username = nullptr, const char* password = nullptr,
                    uint16_t keepalive = 60,const char *topic = nullptr, const char *message = nullptr, uint8_t qos = 0, bool subhex = false, bool retain = false, bool asyncmode = false, bool cleanSession = true);
  bool mqttSetSSL(uint8_t sslIndex = 0);
  bool mqttConnect();
  bool mqttDisconnect();
  bool mqttPublish(const char* topic, const char* payload, uint8_t qos = 0, bool retain = false);
  bool mqttPublishHex(const char* topic, const uint8_t* payload, size_t len, uint8_t qos = 0, bool retain = false);
  /** Consume publish response (OK/ERROR received after payload). Returns true if response was received; outOk = true for OK, false for ERROR. */
  bool consumePublishResponse(bool* outOk = nullptr);
  bool mqttSubscribe(const char* topic, uint8_t qos = 0);
  bool mqttUnsubscribe(const char* topic);
  bool mqttGetState(MQTTState* state);
  bool isMQTTConnected();

  // MQTT callbacks
  void setMQTTMessageCallback(MQTTMessageCallback callback) { _mqttMessageCallback = callback; }
  void setMQTTStateCallback(MQTTStateCallback callback) { _mqttStateCallback = callback; }

  // HTTP Client
  bool httpBegin();
  bool httpSetSSL(uint8_t sslIndex = 0);
  bool httpSetURL(const char* url);
  bool httpAddHeader(const char* header);
  bool httpClearHeaders();
  bool httpSetBody(const char* body);
  bool httpSetBody(const uint8_t* body, size_t len);
  bool httpGET(HTTPResponseCallback callback = nullptr, void* userData = nullptr);
  bool httpPOST(HTTPResponseCallback callback = nullptr, void* userData = nullptr);
  bool httpPUT(HTTPResponseCallback callback = nullptr, void* userData = nullptr);
  bool httpDELETE(HTTPResponseCallback callback = nullptr, void* userData = nullptr);
  bool httpReadResponse(char* buffer, size_t bufferSize, size_t* bytesRead);
  bool httpGetState(int* state);
  bool httpDisconnect();

  // TCP/UDP Socket
  bool socketSetSSL(int socketId, uint8_t sslIndex = 0);
  bool socketOpen(int socketId, const char* host, uint16_t port, bool tcp = true);
  bool socketSend(int socketId, const uint8_t* data, size_t len);
  bool socketReceive(int socketId, uint8_t* buffer, size_t bufferSize, size_t* bytesRead);
  bool socketClose(int socketId);
  bool socketSetTransparentMode(int socketId, bool enable);
  bool socketSwitchTransparent(int socketId, bool enable);

  // NTP Time Synchronization
  /**
   * Synchronize time with NTP server via CNTP command
   * @param cid PDP context ID (default: 1)
   * @param server NTP server hostname (default: "time.google.com")
   * @param waitTotalMs Total timeout in milliseconds (default: 12000)
   * @return true if synchronization successful
   */
  bool syncTimeUTC_viaCNTP(uint8_t cid = 1, const char* server = "time.google.com", uint32_t waitTotalMs = 12000);
  
  /**
   * Try multiple NTP servers until one succeeds
   * @param cid PDP context ID (default: 1)
   * @param waitTotalMs Total timeout per server in milliseconds (default: 12000)
   * @return true if synchronization successful with any server
   */
  bool syncTimeUTC_any(uint8_t cid = 1, uint32_t waitTotalMs = 12000);
  
  /**
   * Get network time in ISO-8601 format (YYYY-MM-DDTHH:MM:SSZ)
   * @param isoOut Output buffer (must be at least 21 bytes)
   * @param outCap Buffer capacity
   * @return true if time retrieved successfully
   */
  bool getNetworkTimeISO8601(char* isoOut, size_t outCap);
  
  /**
   * Parse CCLK response to ISO-8601 format
   * @param cclkResp CCLK response string
   * @param isoOut Output buffer (must be at least 21 bytes)
   * @param outCap Buffer capacity
   * @return true if parsing successful
   */
  bool parseCCLKToISO(const char* cclkResp, char* isoOut, size_t outCap);
  
  /**
   * Get network time in HH:MM:SS format
   * @param hms Output buffer (must be at least 9 bytes)
   * @param cap Buffer capacity
   * @return true if time retrieved successfully
   */
  bool getNetworkHHMMSS(char* hms, size_t cap);
  
  /**
   * Set timezone (AT+CTZR command)
   * @param timezone Timezone offset in quarter-hours (e.g., 28 for GMT+7, -8 for GMT-2)
   * @return true if timezone set successfully
   */
  bool setTimezone(int8_t timezone);

  // Statistics
  uint32_t getATCommandCount() const { return _atCommandCount; }
  uint32_t getATErrorCount() const { return _atErrorCount; }
  const char* getLastError() const { return _at.getLastError(); }

  // Command wrappers - expose Sim7070G_AT methods
  bool sendCommandSync(const char* command, unsigned long timeout = 5000, 
                      char* response = nullptr, size_t responseSize = 0);
  int sendCommand(const char* command, unsigned long timeout = 5000, 
                  ATCallback callback = nullptr, void* userData = nullptr);
  
  // Check command response against expected result
  bool checkSendCommandSync(const char* command, const char* expectedResult, 
                           unsigned long timeout = 5000);

private:
  HardwareSerial* _serial;
  uint8_t _powerPin;
  uint32_t _baudRate;
  Sim7070G_AT _at;
  
  Sim7070GState _state;
  NetworkInfo _networkInfo;
  MQTTState _mqttState;
  
  // Network credentials
  char _apn[64];
  char _apnUsername[32];
  char _apnPassword[32];
  uint8_t _pdpContextId;
  
  // MQTT credentials
  char _mqttClientId[64];
  char _mqttServer[128];
  uint16_t _mqttPort;
  char _mqttUsername[64];
  char _mqttPassword[64];
  uint16_t _mqttKeepalive;
  bool _mqttCleanSession;
  uint8_t _mqttSSLIndex;
  
  // HTTP state
  int _httpState;
  char _httpURL[256];
  uint8_t _httpSSLIndex;
  
  // Callbacks
  NetworkStateCallback _networkStateCallback;
  MQTTMessageCallback _mqttMessageCallback;
  MQTTStateCallback _mqttStateCallback;
  
  // Statistics
  uint32_t _atCommandCount;
  uint32_t _atErrorCount;

  // Publish response (OK/ERROR after payload) - async
  volatile bool _waitingPublishResponse;
  volatile bool _publishResponseReceived;
  volatile bool _publishResponseOk;
  
  // Internal methods
  void updateState(Sim7070GState newState);
  void updateMQTTState(MQTTState newState);
  void processURCs();
  
  // URC handlers
  static void onURC_SMSTATE(const char* urc, const char* data);
  static void onURC_SMPUB(const char* urc, const char* data);
  static void onURC_SMSUB(const char* urc, const char* data);
  static void onURC_CGREG(const char* urc, const char* data);
  static void onURC_CGNAPN(const char* urc, const char* data);
  static void onURC_CNACT(const char* urc, const char* data);
  static void onURC_APP_PDP(const char* urc, const char* data);
  static void onURC_SMCONN(const char* urc, const char* data);
  static void onUnsolicitedResponse(ATResponseType type, const char* response, void* userData);
  
  // Static instance for URC callbacks
  static Sim7070G* _instance;
};

#endif // SIM7070G_H
