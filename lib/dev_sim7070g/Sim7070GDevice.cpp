#include "Sim7070GDevice.h"
#include "DEBUG.h"
#include "config.h"

// Constructor
Sim7070GDevice::Sim7070GDevice(HardwareSerial* modemSerial, const char* nodeId)
  : Device(EVENT_NONE, 0),  // No event subscription, run on core 0
    _modemSerial(modemSerial),
    _modem(nullptr),
    _nodeId(nodeId),
    _state(MODEM_OFF),
    _mqttPort(1883),
    _lastHealthCheck(0),
    _lastNetworkCheck(0),
    _lastStateChange(0),
    _stateStartTime(0) {
  
  // Clear topics
  _pubTopic[0] = '\0';
  _subTopic[0] = '\0';
  
  // Clear MQTT credentials
  _mqttBroker[0] = '\0';
  _mqttClientId[0] = '\0';
  _mqttUsername[0] = '\0';
  _mqttPassword[0] = '\0';
  
  // Clear GPRS credentials
  _apn[0] = '\0';
  _gprsUser[0] = '\0';
  _gprsPass[0] = '\0';
  _networkMode = NET_MODE_NB;  // Default to NB
  
  // Reset statistics
  resetStats();
}

// Initialize - Called at the beginning of setup()
bool Sim7070GDevice::initialize() {
  DEBUG_PRINTLN(F("[SIM7070G] Initializing modem..."));
  
  // Configure power pin
  pinMode(MODEM_POWER_PIN, OUTPUT);
  digitalWrite(MODEM_POWER_PIN, HIGH);  // Ensure modem is powered
  
  // Initialize serial communication
  _modemSerial->begin(MODEM_BAUD_RATE);
  delay(100);
  
  // Create DFRobot modem instance
  _modem = new DFRobot_SIM7070G(_modemSerial);
  
  if (!_modem) {
    DEBUG_PRINTLN(F("[SIM7070G] Failed to create modem instance"));
    return false;
  }
  
  DEBUG_PRINTLN(F("[SIM7070G] Modem initialized"));
  updateState(MODEM_INITIALIZING);
  
  return true;
}

// Start - Called at the end of setup()
int Sim7070GDevice::start() {
  DEBUG_PRINTLN(F("[SIM7070G] Starting modem..."));
  
  // Configure MQTT topics and credentials
  char topic_pub[96];
  char topic_sub[96];
  char clientID[64];
  char mqtt_username[64];
  char mqtt_password[64];
  
  snprintf(topic_pub, sizeof(topic_pub), "nono/%s/pub", _nodeId);
  snprintf(topic_sub, sizeof(topic_sub), "nono/%s/sub", _nodeId);
  snprintf(clientID, sizeof(clientID), "nono_%s", _nodeId);
  snprintf(mqtt_username, sizeof(mqtt_username), "%s", USERNAME);
  snprintf(mqtt_password, sizeof(mqtt_password), "%s", PASSWORD);
  
  this->setMQTTCredentials(mqtt_server, mqtt_port, clientID, mqtt_username, mqtt_password);
  this->setTopics(topic_pub, topic_sub);
  this->setNetworkCredentials(PDP_APN, "", "", NETWORK_MODE_NB ? NET_MODE_NB : NET_MODE_GPRS);
  DEBUG_PRINTLN(F("[SIM7070G] MQTT configured"));
  
  // Check if modem is responding
  if (checkModemAlive()) {
    DEBUG_PRINTLN(F("[SIM7070G] Modem is alive"));
    updateState(MODEM_READY);
  } else {
    DEBUG_PRINTLN(F("[SIM7070G] Modem not responding, attempting power on..."));
    if (!powerOn()) {
      DEBUG_PRINTLN(F("[SIM7070G] Failed to power on modem"));
      updateState(MODEM_ERROR);
      return 30000;  // Retry in 30 seconds
    }
  }
  
  // Apply boot configuration
  if (!applyBootConfig()) {
    DEBUG_PRINTLN(F("[SIM7070G] Failed to apply boot config"));
  }
  
  // Schedule first GPRS connection attempt
  return 5000;  // Try to connect in 5 seconds
}

// Timeout - Called periodically
int Sim7070GDevice::timeout() {
  unsigned long now = millis();
  
  // Handle incoming data
  handleIncomingData();
  
  switch (_state) {
    case MODEM_READY:
      // Start network connection (GPRS or NB-IoT based on config)
      if (_networkMode == NET_MODE_NB) {
        DEBUG_PRINTLN(F("[SIM7070G] Starting NB-IoT connection..."));
        updateState(MODEM_GPRS_CONNECTING_SET_MODE);
      } else {
        DEBUG_PRINTLN(F("[SIM7070G] Starting GPRS connection..."));
        updateState(MODEM_GPRS_CONNECTING_SET_MODE);
      }
      return 100;  // Continue immediately
      
    case MODEM_GPRS_CONNECTING_SET_MODE:
      // Step 1: Set network mode
      if (_networkMode == NET_MODE_NB) {
        DEBUG_PRINTLN(F("[SIM7070G] Setting NB-IoT mode..."));
        if (_modem && _modem->setNetMode(DFRobot_SIM7070G::eNB)) {
          updateState(MODEM_GPRS_CONNECTING_WAIT_MODE);
          return 2000;  // NB-IoT needs more time
        }
      } else {
        DEBUG_PRINTLN(F("[SIM7070G] Setting GPRS mode..."));
        if (_modem && _modem->setNetMode(DFRobot_SIM7070G::eGPRS)) {
          updateState(MODEM_GPRS_CONNECTING_WAIT_MODE);
          return 1000;  // GPRS needs less time
        }
      }
      DEBUG_PRINTLN(F("[SIM7070G] Failed to set network mode"));
      updateState(MODEM_READY);
      return 30000;  // Retry in 30 seconds
      
    case MODEM_GPRS_CONNECTING_WAIT_MODE:
      // Step 2: Wait for mode to be set, then attach service
      updateState(MODEM_GPRS_CONNECTING_ATTACH);
      return 100;  // Continue immediately
      
    case MODEM_GPRS_CONNECTING_ATTACH:
      // Step 3: Attach to network service
      DEBUG_PRINTLN(F("[SIM7070G] Attaching to network service..."));
      if (_modem && _modem->attacthService()) {
        updateState(MODEM_GPRS_CONNECTING_WAIT_ATTACH);
        return (_networkMode == NET_MODE_NB) ? 2000 : 1000;
      }
      DEBUG_PRINTLN(F("[SIM7070G] Failed to attach service"));
      updateState(MODEM_READY);
      return 30000;  // Retry in 30 seconds
      
    case MODEM_GPRS_CONNECTING_WAIT_ATTACH:
      // Step 4: Verify connection
      {
        bool connected = (_networkMode == NET_MODE_NB) ? isNBConnected() : isGPRSConnected();
        if (connected) {
          DEBUG_PRINTLN(F("[SIM7070G] Network connected"));
          _stats.gprsConnects++;
          _stats.lastGPRSConnectTime = now;
          updateState(MODEM_GPRS_CONNECTED);
          return 2000;  // Check MQTT in 2 seconds
        } else {
          // Check timeout (max 30 seconds for connection)
          if ((now - _stateStartTime) > 30000) {
            DEBUG_PRINTLN(F("[SIM7070G] Network connection timeout"));
            updateState(MODEM_READY);
            return 30000;  // Retry in 30 seconds
          }
          return 2000;  // Check again in 2 seconds
        }
      }
      break;
      
    case MODEM_GPRS_CONNECTED:
      // Try to connect MQTT
      if (_mqttBroker[0] != '\0') {  // MQTT configured
        DEBUG_PRINTLN(F("[SIM7070G] Starting MQTT connection..."));
        updateState(MODEM_MQTT_CONNECTING_OPEN_NET);
        return 100;  // Continue immediately
      }
      return 30000;  // Wait for MQTT config
      
    case MODEM_MQTT_CONNECTING_OPEN_NET:
      // Step 1: Open TCP network connection
      DEBUG_PRINT(F("[SIM7070G] Opening TCP connection to "));
      DEBUG_PRINT(_mqttBroker);
      DEBUG_PRINT(F(":"));
      DEBUG_PRINTLN(_mqttPort);
      
      if (_modem && _modem->openNetwork(DFRobot_SIM7070G::eTCP, _mqttBroker, _mqttPort)) {
        updateState(MODEM_MQTT_CONNECTING_WAIT_OPEN);
        return 500;  // Wait for TCP connection to establish
      }
      DEBUG_PRINTLN(F("[SIM7070G] Failed to open TCP connection"));
      updateState(MODEM_GPRS_CONNECTED);
      return 10000;  // Retry in 10 seconds
      
    case MODEM_MQTT_CONNECTING_WAIT_OPEN:
      // Step 2: Send MQTT connect
      updateState(MODEM_MQTT_CONNECTING_HANDSHAKE);
      return 100;  // Continue immediately
      
    case MODEM_MQTT_CONNECTING_HANDSHAKE:
      // Step 3: MQTT handshake
      DEBUG_PRINTLN(F("[SIM7070G] Sending MQTT connect..."));
      
      if (_modem && _modem->mqttConnect(_mqttClientId, _mqttUsername, _mqttPassword)) {
        updateState(MODEM_MQTT_CONNECTING_WAIT_HANDSHAKE);
        return 1000;  // Wait for MQTT handshake
      }
      DEBUG_PRINTLN(F("[SIM7070G] MQTT handshake failed"));
      if (_modem) _modem->closeNetwork();
      updateState(MODEM_GPRS_CONNECTED);
      return 10000;  // Retry in 10 seconds
      
    case MODEM_MQTT_CONNECTING_WAIT_HANDSHAKE:
      // Step 4: Verify MQTT connection
      if (isMQTTConnected()) {
        DEBUG_PRINTLN(F("[SIM7070G] MQTT connected"));
        _stats.mqttConnects++;
        _stats.lastMQTTConnectTime = now;
        
        // Subscribe to topic if configured
        if (_subTopic[0] != '\0') {
          subscribe(_subTopic, 0);
        }
        
        updateState(MODEM_MQTT_CONNECTED);
        return 15000;  // Check status every 15 seconds
      } else {
        // Check timeout (max 10 seconds for MQTT handshake)
        if ((now - _stateStartTime) > 10000) {
          DEBUG_PRINTLN(F("[SIM7070G] MQTT connection timeout"));
          if (_modem) _modem->closeNetwork();
          updateState(MODEM_GPRS_CONNECTED);
          return 10000;  // Retry in 10 seconds
        }
        return 1000;  // Check again in 1 second
      }
      break;
      
    case MODEM_MQTT_CONNECTED:
      // Periodic health check
      if ((now - _lastHealthCheck) >= 15000) {
        _lastHealthCheck = now;
        
        if (!isMQTTConnected()) {
          DEBUG_PRINTLN(F("[SIM7070G] MQTT disconnected"));
          _stats.mqttDisconnects++;
          updateState(MODEM_GPRS_CONNECTED);
          return 5000;  // Try to reconnect
        }
        
        // Check network connection based on mode
        bool networkOk = (_networkMode == NET_MODE_NB) ? isNBConnected() : isGPRSConnected();
        if (!networkOk) {
          DEBUG_PRINTLN(F("[SIM7070G] Network disconnected"));
          updateState(MODEM_READY);
          return 5000;  // Try to reconnect
        }
      }
      return 15000;  // Check again in 15 seconds
      break;
      
    case MODEM_ERROR:
      // Try to restart modem
      DEBUG_PRINTLN(F("[SIM7070G] Error state, restarting..."));
      if (restart()) {
        updateState(MODEM_READY);
        return 5000;
      }
      return 60000;  // Wait 1 minute before retry
      break;
      
    default:
      return 10000;
  }
}

// Event - Handle events
int Sim7070GDevice::event() {
  return DURATION_IGNORE;
}

// Power on modem
bool Sim7070GDevice::powerOn() {
  DEBUG_PRINTLN(F("[SIM7070G] Powering on..."));
  
  pinMode(MODEM_POWER_PIN, OUTPUT);
  if (!_modem) return false;
  
  // Check if modem is already on
  if (_modem->checkSendCmd("AT\r\n", "OK", 100)) {
    DEBUG_PRINTLN(F("[SIM7070G] Modem already on, powering down first..."));
    _modem->checkSendCmd("AT+CPOWD=1\r\n", "NORMAL POWER DOWN", 2000);
    delay(4000);
  }
  
  // Power on sequence: HIGH -> LOW pulse
  DEBUG_PRINTLN(F("[SIM7070G] Sending power pulse..."));
  digitalWrite(MODEM_POWER_PIN, HIGH);
  delay(1000);
  digitalWrite(MODEM_POWER_PIN, LOW);
  delay(7000);  // Wait for modem to boot
  
  // Check if modem responds
  if (_modem->checkSendCmd("AT\r\n", "OK", 2000)) {
    DEBUG_PRINTLN(F("[SIM7070G] Modem powered on successfully"));
    return checkModemAlive();
  } else {
    // Retry once
    DEBUG_PRINTLN(F("[SIM7070G] First attempt failed, retrying..."));
    delay(100);
    digitalWrite(MODEM_POWER_PIN, HIGH);
    delay(1000);
    digitalWrite(MODEM_POWER_PIN, LOW);
    delay(7000);
    
    if (_modem->checkSendCmd("AT\r\n", "OK", 2000)) {
      DEBUG_PRINTLN(F("[SIM7070G] Modem powered on after retry"));
      return checkModemAlive();
    }
  }
  
  DEBUG_PRINTLN(F("[SIM7070G] Failed to power on modem"));
  return false;
}

// Power off modem
bool Sim7070GDevice::powerOff() {
  DEBUG_PRINTLN(F("[SIM7070G] Powering off..."));
  
  if (!_modem) {
    updateState(MODEM_OFF);
    return true;
  }
  
  // Send power down command
  if (_modem->checkSendCmd("AT+CPOWD=1\r\n", "NORMAL POWER DOWN", 2000)) {
    DEBUG_PRINTLN(F("[SIM7070G] Power down command sent"));
    delay(4000);  // Wait for graceful shutdown
  }
  
  // Verify modem is off
  delay(1000);
  if (!_modem->checkSendCmd("AT\r\n", "OK", 500)) {
    DEBUG_PRINTLN(F("[SIM7070G] Modem powered off"));
    updateState(MODEM_OFF);
    return true;
  }
  
  DEBUG_PRINTLN(F("[SIM7070G] Modem still responding after power off"));
  updateState(MODEM_OFF);
  return false;
}

// Restart modem
bool Sim7070GDevice::restart() {
  DEBUG_PRINTLN(F("[SIM7070G] Restarting modem..."));
  
  _stats.modemRestarts++;
  
  powerOff();
  delay(2000);
  
  if (powerOn()) {
    applyBootConfig();
    return true;
  }
  
  return false;
}

// Connect GPRS (deprecated - use state machine in timeout())
bool Sim7070GDevice::connectGPRS() {
  // This function is kept for backward compatibility but should not be used
  // The async state machine in timeout() handles connection automatically
  return isGPRSConnected();
}

// Connect NB-IoT (deprecated - use state machine in timeout())
bool Sim7070GDevice::connectNB() {
  // This function is kept for backward compatibility but should not be used
  // The async state machine in timeout() handles connection automatically
  return isNBConnected();
}

// Disconnect GPRS
bool Sim7070GDevice::disconnectGPRS() {
  char cmd[64];
  
  snprintf(cmd, sizeof(cmd), "AT+CNACT=%d,0", PDP_CONTEXT_ID);
  if (!_modem) return false;
  String command = String(cmd) + "\r\n";
  return _modem->checkSendCmd(command.c_str(), "OK", 10000);
}

// Disconnect NB-IoT
bool Sim7070GDevice::disconnectNB() {
  // Same command for NB-IoT
  return disconnectGPRS();
}

// Check GPRS connection
bool Sim7070GDevice::isGPRSConnected() {
  if (!_modem) return false;
  return _modem->checkSendCmd("AT+CNACT?\r\n", "+CNACT: 1,1", 2000);
}

// Check NB-IoT connection
bool Sim7070GDevice::isNBConnected() {
  if (!_modem) return false;
  // Check EPS registration status (for LTE networks)
  if (_modem->checkSendCmd("AT+CEREG?\r\n", "+CEREG: 0,1", 2000)) {
    return isGPRSConnected();
  }
  if (_modem->checkSendCmd("AT+CEREG?\r\n", "+CEREG: 0,5", 2000)) {
    return isGPRSConnected();
  }
  return false;
}

// Connect MQTT (deprecated - use state machine in timeout())
bool Sim7070GDevice::connectMQTT(const char* broker, uint16_t port,
                                  const char* clientId, const char* username, const char* password) {
  // Store credentials for state machine to use
  strncpy(_mqttBroker, broker, sizeof(_mqttBroker) - 1);
  _mqttPort = port;
  strncpy(_mqttClientId, clientId, sizeof(_mqttClientId) - 1);
  strncpy(_mqttUsername, username, sizeof(_mqttUsername) - 1);
  strncpy(_mqttPassword, password, sizeof(_mqttPassword) - 1);
  
  // This function is kept for backward compatibility but should not be used
  // The async state machine in timeout() handles connection automatically
  return isMQTTConnected();
}

// Disconnect MQTT
bool Sim7070GDevice::disconnectMQTT() {
  if (_modem) {
    _modem->mqttDisconnect();
    delay(200);
    return _modem->closeNetwork();  // Close TCP connection after MQTT disconnect
  }
  return false;
}

// Check MQTT connection
bool Sim7070GDevice::isMQTTConnected() {
  if (!_modem) return false;
  return _modem->checkSendCmd("AT+SMSTATE?\r\n", "+SMSTATE: 1", 2000);
}

// Publish MQTT message
bool Sim7070GDevice::publish(const char* topic, const char* payload, uint8_t qos) {
  if (_state != MODEM_MQTT_CONNECTED) {
    DEBUG_PRINTLN(F("[SIM7070G] Cannot publish: MQTT not connected"));
    return false;
  }
  
  if (!_modem) return false;
  
  String data = String(payload);
  return _modem->mqttPublish(topic, data);
}

// Subscribe to MQTT topic
bool Sim7070GDevice::subscribe(const char* topic, uint8_t qos) {
  if (!_modem) return false;
  
  return _modem->mqttSubscribe(topic);
}

// Unsubscribe from MQTT topic
bool Sim7070GDevice::unsubscribe(const char* topic) {
  if (!_modem) return false;
  
  return _modem->mqttUnsubscribe(topic);
}

// Receive MQTT message
bool Sim7070GDevice::receiveMQTT(char* topic, char* payload, int maxLen) {
  if (!_modem) return false;
  
  return _modem->mqttRecv(topic, payload, maxLen);
}

// Send AT command
bool Sim7070GDevice::sendAT(const char* cmd, unsigned long timeout) {
  _stats.atCommandsSent++;
  
  _modemSerial->println(cmd);
  
  unsigned long start = millis();
  while (millis() - start < timeout) {
    if (_modemSerial->available()) {
      return true;
    }
    delay(10);
  }
  
  _stats.atCommandsFailed++;
  return false;
}

// Send AT and wait for OK
bool Sim7070GDevice::sendATwaitOK(const char* cmd, char* response, size_t responseSize, unsigned long timeout) {
  _stats.atCommandsSent++;
  
  // Clear buffer
  while (_modemSerial->available()) {
    _modemSerial->read();
  }
  
  // Send command
  _modemSerial->println(cmd);
  
  // Read response
  unsigned long start = millis();
  size_t index = 0;
  response[0] = '\0';
  
  while (millis() - start < timeout && index < responseSize - 1) {
    if (_modemSerial->available()) {
      char c = _modemSerial->read();
      response[index++] = c;
      response[index] = '\0';
      
      // Check for OK or ERROR
      if (strstr(response, "OK")) {
        return true;
      }
      if (strstr(response, "ERROR")) {
        _stats.atCommandsFailed++;
        return false;
      }
    }
  }
  
  _stats.atCommandsFailed++;
  return false;
}

// Get RSSI
int8_t Sim7070GDevice::getRSSI() {
  if (!_modem) return -1;
  
  int rssi = _modem->checkSignalQuality();
  
  if (rssi >= 0 && rssi <= 31) {
    _stats.lastRSSI = rssi;
    return rssi;
  }
  
  return -1;
}

// Get network time
bool Sim7070GDevice::getNetworkTime(char* isoTime, size_t size) {
  char resp[256];
  
  if (!sendATwaitOK("AT+CCLK?", resp, sizeof(resp), 2000)) {
    return false;
  }
  
  // Parse response: +CCLK: "YY/MM/DD,HH:MM:SS±TZ"
  char* p = strchr(resp, '"');
  if (!p) return false;
  
  p++;
  char* end = strchr(p, '"');
  if (!end) return false;
  
  *end = '\0';
  
  // Convert to ISO format
  int yy, mm, dd, hh, mi, ss;
  if (sscanf(p, "%2d/%2d/%2d,%2d:%2d:%2d", &yy, &mm, &dd, &hh, &mi, &ss) == 6) {
    snprintf(isoTime, size, "20%02d-%02d-%02dT%02d:%02d:%02dZ", yy, mm, dd, hh, mi, ss);
    return true;
  }
  
  return false;
}

// Get state string
const char* Sim7070GDevice::getStateString() const {
  switch (_state) {
    case MODEM_OFF: return "OFF";
    case MODEM_INITIALIZING: return "INITIALIZING";
    case MODEM_READY: return "READY";
    case MODEM_GPRS_CONNECTING: return "GPRS_CONNECTING";
    case MODEM_GPRS_CONNECTING_SET_MODE: return "GPRS_SET_MODE";
    case MODEM_GPRS_CONNECTING_WAIT_MODE: return "GPRS_WAIT_MODE";
    case MODEM_GPRS_CONNECTING_ATTACH: return "GPRS_ATTACH";
    case MODEM_GPRS_CONNECTING_WAIT_ATTACH: return "GPRS_WAIT_ATTACH";
    case MODEM_GPRS_CONNECTED: return "GPRS_CONNECTED";
    case MODEM_MQTT_CONNECTING: return "MQTT_CONNECTING";
    case MODEM_MQTT_CONNECTING_OPEN_NET: return "MQTT_OPEN_NET";
    case MODEM_MQTT_CONNECTING_WAIT_OPEN: return "MQTT_WAIT_OPEN";
    case MODEM_MQTT_CONNECTING_HANDSHAKE: return "MQTT_HANDSHAKE";
    case MODEM_MQTT_CONNECTING_WAIT_HANDSHAKE: return "MQTT_WAIT_HANDSHAKE";
    case MODEM_MQTT_CONNECTED: return "MQTT_CONNECTED";
    case MODEM_ERROR: return "ERROR";
    default: return "UNKNOWN";
  }
}

// Reset statistics
void Sim7070GDevice::resetStats() {
  _stats.modemRestarts = 0;
  _stats.gprsConnects = 0;
  _stats.mqttConnects = 0;
  _stats.mqttDisconnects = 0;
  _stats.atCommandsSent = 0;
  _stats.atCommandsFailed = 0;
  _stats.lastRSSI = -1;
  _stats.lastGPRSConnectTime = 0;
  _stats.lastMQTTConnectTime = 0;
}

// Apply boot configuration
bool Sim7070GDevice::applyBootConfig() {
  DEBUG_PRINTLN(F("[SIM7070G] Applying boot config..."));
  
  if (!_modem) return false;
  
  _modem->checkSendCmd("ATE0\r\n", "OK", 500);
  _modem->checkSendCmd("AT+CFUN=1\r\n", "OK", 1000);
  _modem->checkSendCmd("AT+CMEE=2\r\n", "OK", 500);
  _modem->checkSendCmd("AT+CPIN?\r\n", "OK", 500);
  _modem->checkSendCmd("AT+CNMP=38\r\n", "OK", 500);
  _modem->checkSendCmd("AT+CMNB=2\r\n", "OK", 500);
  _modem->checkSendCmd("AT+CBANDCFG=\"CAT-M\",2\r\n", "OK", 500);
  
  delay(500);
  
  return true;
}

// Check modem alive
bool Sim7070GDevice::checkModemAlive() {
  if (!_modem) return false;
  
  // Try AT command first for quick check
  if (_modem->checkSendCmd("AT\r\n", "OK", 1000)) {
    // Also verify SIM status
    return _modem->checkSIMStatus();
  }
  
  return false;
}

// Update state
void Sim7070GDevice::updateState(ModemState newState) {
  if (_state != newState) {
    DEBUG_PRINT(F("[SIM7070G] State: "));
    DEBUG_PRINT(getStateString());
    DEBUG_PRINT(F(" -> "));
    _state = newState;
    DEBUG_PRINTLN(getStateString());
    _lastStateChange = millis();
    _stateStartTime = millis();  // Reset timer for new state
  }
}

// Wait for response
bool Sim7070GDevice::waitForResponse(const char* expected, unsigned long timeout) {
  char buffer[256];
  size_t index = 0;
  unsigned long start = millis();
  
  while (millis() - start < timeout && index < sizeof(buffer) - 1) {
    if (_modemSerial->available()) {
      char c = _modemSerial->read();
      buffer[index++] = c;
      buffer[index] = '\0';
      
      if (strstr(buffer, expected)) {
        return true;
      }
    }
  }
  
  return false;
}

// Handle incoming data
void Sim7070GDevice::handleIncomingData() {
  // Process URC messages like +SMSUB for MQTT subscriptions
  // Use library function if available
  
  while (_modemSerial->available()) {
    _modemSerial->read();  // Consume to prevent buffer overflow
  }
}

// Configuration methods
void Sim7070GDevice::setTopics(const char* pubTopic, const char* subTopic) {
  strncpy(_pubTopic, pubTopic, sizeof(_pubTopic) - 1);
  _pubTopic[sizeof(_pubTopic) - 1] = '\0';
  
  strncpy(_subTopic, subTopic, sizeof(_subTopic) - 1);
  _subTopic[sizeof(_subTopic) - 1] = '\0';
  
  DEBUG_PRINT(F("[SIM7070G] Topics set - Pub: "));
  DEBUG_PRINT(_pubTopic);
  DEBUG_PRINT(F(", Sub: "));
  DEBUG_PRINTLN(_subTopic);
}

void Sim7070GDevice::setMQTTCredentials(const char* broker, uint16_t port, 
                                         const char* clientId, const char* username, const char* password) {
  strncpy(_mqttBroker, broker, sizeof(_mqttBroker) - 1);
  _mqttBroker[sizeof(_mqttBroker) - 1] = '\0';
  
  _mqttPort = port;
  
  strncpy(_mqttClientId, clientId, sizeof(_mqttClientId) - 1);
  _mqttClientId[sizeof(_mqttClientId) - 1] = '\0';
  
  strncpy(_mqttUsername, username, sizeof(_mqttUsername) - 1);
  _mqttUsername[sizeof(_mqttUsername) - 1] = '\0';
  
  strncpy(_mqttPassword, password, sizeof(_mqttPassword) - 1);
  _mqttPassword[sizeof(_mqttPassword) - 1] = '\0';
  
  DEBUG_PRINT(F("[SIM7070G] MQTT credentials set - Broker: "));
  DEBUG_PRINT(_mqttBroker);
  DEBUG_PRINT(F(":"));
  DEBUG_PRINTLN(_mqttPort);
}

void Sim7070GDevice::setNetworkCredentials(const char* apn, const char* user, const char* pass, NetworkMode mode) {
  strncpy(_apn, apn, sizeof(_apn) - 1);
  _apn[sizeof(_apn) - 1] = '\0';
  
  strncpy(_gprsUser, user, sizeof(_gprsUser) - 1);
  _gprsUser[sizeof(_gprsUser) - 1] = '\0';
  
  strncpy(_gprsPass, pass, sizeof(_gprsPass) - 1);
  _gprsPass[sizeof(_gprsPass) - 1] = '\0';
  
  _networkMode = mode;
  
  DEBUG_PRINT(F("[SIM7070G] Network credentials set - APN: "));
  DEBUG_PRINT(_apn);
  DEBUG_PRINT(F(", Mode: "));
  DEBUG_PRINTLN(mode == NET_MODE_NB ? "NB-IoT" : "GPRS");
}

