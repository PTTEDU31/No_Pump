#include "Sim7070GDevice.h"
#include "DEBUG.h"
#include "config.h"

// Constructor
Sim7070GDevice::Sim7070GDevice(HardwareSerial *modemSerial, const char *nodeId)
    : Device(EVENT_NONE, 0), // No event subscription, run on core 0
      _modemSerial(modemSerial),
      _modem(nullptr),
      _nodeId(nodeId),
      _state(MODEM_OFF),
      _mqttPort(1883),
      _lastHealthCheck(0),
      _lastNetworkCheck(0),
      _lastStateChange(0),
      _stateStartTime(0),
      _retryCount(0),
      _bufferHead(0),
      _bufferTail(0),
      _bufferCount(0)
{

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
  _networkMode = NET_MODE_NB; // Default to NB

  // Reset statistics
  resetStats();
}

// Initialize - Called at the beginning of setup()
bool Sim7070GDevice::initialize()
{
  DEBUG_PRINTLN(F("[SIM7070G] Initializing modem..."));

  // Configure power pin
  pinMode(MODEM_POWER_PIN, OUTPUT);
  digitalWrite(MODEM_POWER_PIN, HIGH); // Ensure modem is powered

  // Initialize serial communication
  _modemSerial->begin(MODEM_BAUD_RATE);
  delay(100);

  // Create DFRobot modem instance
  _modem = new DFRobot_SIM7070G(_modemSerial);

  if (!_modem)
  {
    DEBUG_PRINTLN(F("[SIM7070G] Failed to create modem instance"));
    return false;
  }

  DEBUG_PRINTLN(F("[SIM7070G] Modem initialized"));
  updateState(MODEM_INITIALIZING);

  return true;
}

// Start - Called at the end of setup()
int Sim7070GDevice::start()
{
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

  return DURATION_IMMEDIATELY; // Start immediately
}
// Timeout - Called periodically
int Sim7070GDevice::timeout()
{
  unsigned long now = millis();

  // Handle incoming data
  handleIncomingData();

  switch (_state)
  {
  case MODEM_INITIALIZING:
    // Power on the modem
    pinMode(MODEM_POWER_PIN, OUTPUT);
    DEBUG_PRINTLN(F("[SIM7070G] Powering on modem..."));
    if (_modem->checkSendCmd("AT\r\n", "OK", 100))
    {
      _modem->checkSendCmd("AT+CPOWD=1\r\n", "NORMAL POWER DOWN");
    }
    updateState(MODEM_RESET);
    return (4000);
    break;
  case MODEM_RESET:
    delay(100);
    digitalWrite(MODEM_POWER_PIN, HIGH);
    delay(1000);
    digitalWrite(MODEM_POWER_PIN, LOW);
    updateState(MODEM_READY);
    return (7000);
    break;
  case MODEM_READY:
    if (_modem->checkSendCmd("AT\r\n", "OK"))
    {
      updateState(MODEM_CHECK_SIM);
      _retryCount = 0;
      return DURATION_IMMEDIATELY;
    }
    else
    {
      delay(100);
      digitalWrite(MODEM_POWER_PIN, HIGH);
      delay(1000);
      digitalWrite(MODEM_POWER_PIN, LOW);
      DEBUG_PRINTLN(F("[SIM7070G] Modem not responding, retrying power on..."));
      // updateState(MODEM_OFF);
      return 5000; // Retry in 5s
    }
  case MODEM_CHECK_SIM:
    // Check SIM card with timeout
    DEBUG_PRINTLN(F("[SIM7070G] Checking SIM card..."));
    if (_modem && _modem->checkSIMStatus())
    {
      DEBUG_PRINTLN(F("[SIM7070G] SIM card READY"));
      updateState(MODEM_GPRS_CONNECTING_SET_BAUD);
      _retryCount = 0;
      return DURATION_IMMEDIATELY;
    }
    else
    {
      _retryCount++;
      if (_retryCount >= 3)
      {
        DEBUG_PRINTLN(F("[SIM7070G] SIM card ERROR after 3 attempts"));
        updateState(MODEM_ERROR);
        _retryCount = 0;
        return 30000; // Retry after 30s
      }
      DEBUG_PRINT(F("[SIM7070G] SIM check failed, retry "));
      DEBUG_PRINT(_retryCount);
      DEBUG_PRINTLN(F("/3"));
      return 2000; // Retry after 2s
    }
    break;

  case MODEM_GPRS_CONNECTING_SET_BAUD:
    // Step 0: Set baud rate
    DEBUG_PRINTLN(F("[SIM7070G] Setting baud rate..."));
    if (_modem && _modem->setBaudRate(MODEM_BAUD_RATE))
    {
      updateState(MODEM_GPRS_CONNECTING_SET_MODE);
      return DURATION_IMMEDIATELY; // Wait for baud rate to take effect
    }
    DEBUG_PRINTLN(F("[SIM7070G] Failed to set baud rate"));
    updateState(MODEM_READY);
    return 1000; // Retry in 1 seconds
  case MODEM_GPRS_CONNECTING_SET_MODE:
    // Step 1: Set network mode
    if (_networkMode == NET_MODE_NB)
    {
      DEBUG_PRINTLN(F("[SIM7070G] Setting NB-IoT mode..."));
      if (_modem && _modem->setNetMode(DFRobot_SIM7070G::eNB))
      {
        updateState(MODEM_GPRS_CONNECTING_WAIT_MODE);
        return 2000; // NB-IoT needs more time
      }
    }
    else
    {
      DEBUG_PRINTLN(F("[SIM7070G] Setting GPRS mode..."));
      if (_modem && _modem->setNetMode(DFRobot_SIM7070G::eGPRS))
      {
        updateState(MODEM_GPRS_CONNECTING_WAIT_MODE);
        return 1000; // GPRS needs less time
      }
    }
    DEBUG_PRINTLN(F("[SIM7070G] Failed to set network mode"));
    updateState(MODEM_READY);
    return 30000; // Retry in 30 seconds

  case MODEM_GPRS_CONNECTING_WAIT_MODE:
    // Step 2: Wait for mode to be set, then check signal
    updateState(MODEM_GPRS_CONNECTING_CHECK_SIGNAL);
    _retryCount = 0;
    return 1500; // Wait 1.5s as in example

  case MODEM_GPRS_CONNECTING_CHECK_SIGNAL:
    // Step 3: Check signal quality
    DEBUG_PRINTLN(F("[SIM7070G] Checking signal quality..."));
    {
      int signalStrength = _modem ? _modem->checkSignalQuality() : 0;
      DEBUG_PRINT(F("[SIM7070G] Signal strength: "));
      DEBUG_PRINTLN(signalStrength);
      _stats.lastRSSI = signalStrength;

      if (signalStrength > 0 && signalStrength < 99)
      {
        DEBUG_PRINTLN(F("[SIM7070G] Signal OK, proceeding to attach"));
        updateState(MODEM_GPRS_CONNECTING_ATTACH);
        _retryCount = 0;
        return 500; // Wait 500ms before attach
      }
      else
      {
        _retryCount++;
        if (_retryCount >= 5)
        {
          DEBUG_PRINTLN(F("[SIM7070G] No signal after 5 attempts"));
          updateState(MODEM_READY);
          _retryCount = 0;
          return 30000; // Retry entire process after 30s
        }
        DEBUG_PRINT(F("[SIM7070G] Weak/no signal, retry "));
        DEBUG_PRINT(_retryCount);
        DEBUG_PRINTLN(F("/5"));
        return 3000; // Check again after 3s
      }
    }
    break;

  case MODEM_GPRS_CONNECTING_ATTACH:
    // Step 4: Attach to network service
    DEBUG_PRINTLN(F("[SIM7070G] Attaching to network service..."));
    if (_modem && _modem->attacthService())
    {
      DEBUG_PRINTLN(F("[SIM7070G] Attach service successful"));
      updateState(MODEM_GPRS_CONNECTED);
      _retryCount = 0;
      return (_networkMode == NET_MODE_NB) ? 2000 : 1000;
    }
    else
    {
      _retryCount++;
      if (_retryCount >= 5)
      {
        DEBUG_PRINTLN(F("[SIM7070G] Failed to attach service after 5 attempts"));
        updateState(MODEM_READY);
        _retryCount = 0;
        return 30000; // Retry entire process after 30s
      }
      DEBUG_PRINT(F("[SIM7070G] Failed to attach service, retry "));
      DEBUG_PRINT(_retryCount);
      DEBUG_PRINTLN(F("/5"));
      return 3000; // Retry after 3s
    }
    break;
  case MODEM_GPRS_CONNECTED:
    // Try to connect MQTT using AT commands
    if (_mqttBroker[0] != '\0')
    { // MQTT configured
      DEBUG_PRINTLN(F("[SIM7070G] Configuring MQTT with AT commands..."));
      
      // Configure MQTT parameters
      if (_modem && _modem->mqttConfigAT(_mqttBroker, _mqttPort, _mqttClientId, 
                                          _mqttUsername, _mqttPassword)) {
        DEBUG_PRINTLN(F("[SIM7070G] MQTT config OK, connecting..."));
        updateState(MODEM_MQTT_CONNECTING_HANDSHAKE);
        return 1000; // Wait before connecting
      } else {
        DEBUG_PRINTLN(F("[SIM7070G] MQTT config failed"));
        return 10000; // Retry in 10 seconds
      }
    }
    return DURATION_NEVER; // Wait for MQTT config

  case MODEM_MQTT_CONNECTING_HANDSHAKE:
    // Connect to MQTT broker using AT+SMCONN
    DEBUG_PRINTLN(F("[SIM7070G] Connecting to MQTT broker..."));
    if (_modem && _modem->mqttConnectAT())
    {
      DEBUG_PRINTLN(F("[SIM7070G] MQTT connected successfully"));
      updateState(MODEM_MQTT_CONNECTED_FIRST);
      return 500;
    }
    DEBUG_PRINTLN(F("[SIM7070G] MQTT connection failed"));
    updateState(MODEM_GPRS_CONNECTED);
    return 10000; // Retry in 10 seconds

  case MODEM_MQTT_CONNECTED_FIRST:
    // Publish device info once on first connection
    DEBUG_PRINTLN(F("[SIM7070G] Publishing device info..."));
    {
      // Build device info JSON - keep it short
      char deviceInfo[128];
      char deviceTopic[96];
      snprintf(deviceTopic, sizeof(deviceTopic), "devices/%s", _mqttClientId);
      
      snprintf(deviceInfo, sizeof(deviceInfo),
               "{\"rssi\":%d,\"uptime\":%lu,\"connects\":%lu}",
               _stats.lastRSSI,
               millis() / 1000,
               _stats.mqttConnects);
      
      DEBUG_PRINT(F("[SIM7070G] Topic: "));
      DEBUG_PRINTLN(deviceTopic);
      DEBUG_PRINT(F("[SIM7070G] JSON: "));
      DEBUG_PRINTLN(deviceInfo);
      
      if (_modem->mqttPublishAT(deviceTopic, deviceInfo, 1, 0))
      {
        DEBUG_PRINTLN(F("[SIM7070G] Device info published OK"));
      }
      else
      {
        DEBUG_PRINTLN(F("[SIM7070G] Device info publish FAILED"));
      }
    }
    updateState(MODEM_MQTT_CONNECTED);
    return DURATION_IMMEDIATELY;
    break;

  case MODEM_MQTT_CONNECTED:
    // Flush any buffered messages
    if (_bufferCount > 0)
    {
      DEBUG_PRINT(F("[SIM7070G] Flushing "));
      DEBUG_PRINT(_bufferCount);
      DEBUG_PRINTLN(F(" buffered messages..."));
      flushMessageBuffer();
    }
    return 15000; // Check status every 15 seconds
    break;

  case MODEM_ERROR:
    // Try to restart modem
    DEBUG_PRINTLN(F("[SIM7070G] Error state, restarting..."));
    if (restart())
    {
      updateState(MODEM_READY);
      return 5000;
    }
    return 60000; // Wait 1 minute before retry
    break;

  default:
    return 10000;
  }
}

// Event - Handle events
int Sim7070GDevice::event()
{
  return DURATION_IGNORE;
}

// Power on modem
bool Sim7070GDevice::powerOn()
{
  DEBUG_PRINTLN(F("[SIM7070G] Powering on..."));

  pinMode(MODEM_POWER_PIN, OUTPUT);
  if (!_modem)
    return false;

  // Check if modem is already on
  if (_modem->checkSendCmd("AT\r\n", "OK", 100))
  {
    DEBUG_PRINTLN(F("[SIM7070G] Modem already on, powering down first..."));
    _modem->checkSendCmd("AT+CPOWD=1\r\n", "NORMAL POWER DOWN", 2000);
    delay(4000);
  }

  // Power on sequence: HIGH -> LOW pulse
  DEBUG_PRINTLN(F("[SIM7070G] Sending power pulse..."));
  digitalWrite(MODEM_POWER_PIN, HIGH);
  delay(1000);
  digitalWrite(MODEM_POWER_PIN, LOW);
  delay(7000); // Wait for modem to boot

  // Check if modem responds
  if (_modem->checkSendCmd("AT\r\n", "OK", 2000))
  {
    DEBUG_PRINTLN(F("[SIM7070G] Modem powered on successfully"));
    return checkModemAlive();
  }
  else
  {
    // Retry once
    DEBUG_PRINTLN(F("[SIM7070G] First attempt failed, retrying..."));
    delay(100);
    digitalWrite(MODEM_POWER_PIN, HIGH);
    delay(1000);
    digitalWrite(MODEM_POWER_PIN, LOW);
    delay(7000);

    if (_modem->checkSendCmd("AT\r\n", "OK", 2000))
    {
      DEBUG_PRINTLN(F("[SIM7070G] Modem powered on after retry"));
      return checkModemAlive();
    }
  }

  DEBUG_PRINTLN(F("[SIM7070G] Failed to power on modem"));
  return false;
}

// Power off modem
bool Sim7070GDevice::powerOff()
{
  DEBUG_PRINTLN(F("[SIM7070G] Powering off..."));

  if (!_modem)
  {
    updateState(MODEM_OFF);
    return true;
  }

  // Send power down command
  if (_modem->checkSendCmd("AT+CPOWD=1\r\n", "NORMAL POWER DOWN", 2000))
  {
    DEBUG_PRINTLN(F("[SIM7070G] Power down command sent"));
    delay(4000); // Wait for graceful shutdown
  }

  // Verify modem is off
  delay(1000);
  if (!_modem->checkSendCmd("AT\r\n", "OK", 500))
  {
    DEBUG_PRINTLN(F("[SIM7070G] Modem powered off"));
    updateState(MODEM_OFF);
    return true;
  }

  DEBUG_PRINTLN(F("[SIM7070G] Modem still responding after power off"));
  updateState(MODEM_OFF);
  return false;
}

// Restart modem
bool Sim7070GDevice::restart()
{
  DEBUG_PRINTLN(F("[SIM7070G] Restarting modem..."));

  _stats.modemRestarts++;

  powerOff();
  delay(2000);

  if (powerOn())
  {
    applyBootConfig();
    return true;
  }

  return false;
}

// Check GPRS connection
bool Sim7070GDevice::isGPRSConnected()
{
  if (!_modem)
    return false;
  return _modem->checkSendCmd("AT+CNACT?\r\n", "+CNACT: 1,1", 2000);
}

// Check NB-IoT connection
bool Sim7070GDevice::isNBConnected()
{
  if (!_modem)
    return false;
  // Check EPS registration status (for LTE networks)
  if (_modem->checkSendCmd("AT+CEREG?\r\n", "+CEREG: 0,1", 2000))
  {
    return isGPRSConnected();
  }
  if (_modem->checkSendCmd("AT+CEREG?\r\n", "+CEREG: 0,5", 2000))
  {
    return isGPRSConnected();
  }
  return false;
}

// Disconnect MQTT
bool Sim7070GDevice::disconnectMQTT()
{
  if (_modem)
  {
    return _modem->mqttDisconnectAT();
  }
  return false;
}

// Check MQTT connection
bool Sim7070GDevice::isMQTTConnected()
{
  if (!_modem)
    return false;
  return (_modem->mqttStateAT() == 1);
}

// Publish MQTT message
bool Sim7070GDevice::publish(const char *topic, const char *payload, uint8_t qos)
{
  if (_state != MODEM_MQTT_CONNECTED && _state != MODEM_MQTT_CONNECTED_FIRST)
  {
    DEBUG_PRINTLN(F("[SIM7070G] MQTT not connected, buffering message..."));
    return enqueueMessage(topic, payload, qos);
  }

  if (!_modem)
    return false;

  // Use AT MQTT command instead of manual packet building
  return _modem->mqttPublishAT(topic, payload, qos, 0);
}

// Subscribe to MQTT topic
bool Sim7070GDevice::subscribe(const char *topic, uint8_t qos)
{
  if (!_modem)
    return false;

  return _modem->mqttSubscribeAT(topic, qos);
}

// Unsubscribe from MQTT topic
bool Sim7070GDevice::unsubscribe(const char *topic)
{
  if (!_modem)
    return false;

  return _modem->mqttUnsubscribeAT(topic);
}

// Receive MQTT message
bool Sim7070GDevice::receiveMQTT(char *topic, char *payload, int maxLen)
{
  if (!_modem)
    return false;

  return _modem->mqttRecv(topic, payload, maxLen);
}

// Send AT command
bool Sim7070GDevice::sendAT(const char *cmd, unsigned long timeout)
{
  _stats.atCommandsSent++;

  _modemSerial->println(cmd);

  unsigned long start = millis();
  while (millis() - start < timeout)
  {
    if (_modemSerial->available())
    {
      return true;
    }
    delay(10);
  }

  _stats.atCommandsFailed++;
  return false;
}

// Send AT and wait for OK
bool Sim7070GDevice::sendATwaitOK(const char *cmd, char *response, size_t responseSize, unsigned long timeout)
{
  _stats.atCommandsSent++;

  // Clear buffer
  while (_modemSerial->available())
  {
    _modemSerial->read();
  }

  // Send command
  _modemSerial->println(cmd);

  // Read response
  unsigned long start = millis();
  size_t index = 0;
  response[0] = '\0';

  while (millis() - start < timeout && index < responseSize - 1)
  {
    if (_modemSerial->available())
    {
      char c = _modemSerial->read();
      response[index++] = c;
      response[index] = '\0';

      // Check for OK or ERROR
      if (strstr(response, "OK"))
      {
        return true;
      }
      if (strstr(response, "ERROR"))
      {
        _stats.atCommandsFailed++;
        return false;
      }
    }
  }

  _stats.atCommandsFailed++;
  return false;
}

// Get RSSI
int8_t Sim7070GDevice::getRSSI()
{
  if (!_modem)
    return -1;

  int rssi = _modem->checkSignalQuality();

  if (rssi >= 0 && rssi <= 31)
  {
    _stats.lastRSSI = rssi;
    return rssi;
  }

  return -1;
}

// Get network time
bool Sim7070GDevice::getNetworkTime(char *isoTime, size_t size)
{
  char resp[256];

  if (!sendATwaitOK("AT+CCLK?", resp, sizeof(resp), 2000))
  {
    return false;
  }

  // Parse response: +CCLK: "YY/MM/DD,HH:MM:SS±TZ"
  char *p = strchr(resp, '"');
  if (!p)
    return false;

  p++;
  char *end = strchr(p, '"');
  if (!end)
    return false;

  *end = '\0';

  // Convert to ISO format
  int yy, mm, dd, hh, mi, ss;
  if (sscanf(p, "%2d/%2d/%2d,%2d:%2d:%2d", &yy, &mm, &dd, &hh, &mi, &ss) == 6)
  {
    snprintf(isoTime, size, "20%02d-%02d-%02dT%02d:%02d:%02dZ", yy, mm, dd, hh, mi, ss);
    return true;
  }

  return false;
}

// Get state string
const char *Sim7070GDevice::getStateString() const
{
  switch (_state)
  {
  case MODEM_OFF:
    return "OFF";
  case MODEM_INITIALIZING:
    return "INITIALIZING";
  case MODEM_RESET:
    return "RESET";
  case MODEM_READY:
    return "READY";
  case MODEM_CHECK_SIM:
    return "CHECK_SIM";
  case MODEM_GPRS_CONNECTING_SET_BAUD:
    return "GPRS_SET_BAUD";
  case MODEM_GPRS_CONNECTING_SET_MODE:
    return "GPRS_SET_MODE";
  case MODEM_GPRS_CONNECTING_WAIT_MODE:
    return "GPRS_WAIT_MODE";
  case MODEM_GPRS_CONNECTING_CHECK_SIGNAL:
    return "GPRS_CHECK_SIGNAL";
  case MODEM_GPRS_CONNECTING_ATTACH:
    return "GPRS_ATTACH";
  case MODEM_GPRS_CONNECTED:
    return "GPRS_CONNECTED";
  case MODEM_MQTT_CONNECTING_HANDSHAKE:
    return "MQTT_HANDSHAKE";
  case MODEM_MQTT_CONNECTED_FIRST:
    return "MQTT_CONNECTED_FIRST";
  case MODEM_MQTT_CONNECTED:
    return "MQTT_CONNECTED";
  case MODEM_ERROR:
    return "ERROR";
  default:
    return "UNKNOWN";
  }
}

// Reset statistics
void Sim7070GDevice::resetStats()
{
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
bool Sim7070GDevice::applyBootConfig()
{
  DEBUG_PRINTLN(F("[SIM7070G] Applying boot config..."));

  if (!_modem)
    return false;

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
bool Sim7070GDevice::checkModemAlive()
{
  if (!_modem)
    return false;

  // Try AT command first for quick check
  if (_modem->checkSendCmd("AT\r\n", "OK", 1000))
  {
    // Also verify SIM status
    return _modem->checkSIMStatus();
  }

  return false;
}

// Update state
void Sim7070GDevice::updateState(ModemState newState)
{
  if (_state != newState)
  {
    DEBUG_PRINT(F("[SIM7070G] State: "));
    DEBUG_PRINT(getStateString());
    DEBUG_PRINT(F(" -> "));
    _state = newState;
    DEBUG_PRINTLN(getStateString());
    _lastStateChange = millis();
    _stateStartTime = millis(); // Reset timer for new state
  }
}

// Handle incoming data
void Sim7070GDevice::handleIncomingData()
{
  // Process URC messages like +SMSUB for MQTT subscriptions
  // Use library function if available

  while (_modemSerial->available())
  {
    _modemSerial->read(); // Consume to prevent buffer overflow
  }
}

// Configuration methods
void Sim7070GDevice::setTopics(const char *pubTopic, const char *subTopic)
{
  strncpy(_pubTopic, pubTopic, sizeof(_pubTopic) - 1);
  _pubTopic[sizeof(_pubTopic) - 1] = '\0';

  strncpy(_subTopic, subTopic, sizeof(_subTopic) - 1);
  _subTopic[sizeof(_subTopic) - 1] = '\0';

  DEBUG_PRINT(F("[SIM7070G] Topics set - Pub: "));
  DEBUG_PRINT(_pubTopic);
  DEBUG_PRINT(F(", Sub: "));
  DEBUG_PRINTLN(_subTopic);
}

void Sim7070GDevice::setMQTTCredentials(const char *broker, uint16_t port,
                                        const char *clientId, const char *username, const char *password)
{
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

void Sim7070GDevice::setNetworkCredentials(const char *apn, const char *user, const char *pass, NetworkMode mode)
{
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

// Buffer management methods

// Enqueue message to buffer
bool Sim7070GDevice::enqueueMessage(const char *topic, const char *payload, uint8_t qos)
{
  // Check if buffer is full
  if (_bufferCount >= MQTT_BUFFER_SIZE)
  {
    DEBUG_PRINTLN(F("[SIM7070G] Message buffer full, dropping oldest message"));
    // Drop oldest message to make room
    MQTTMessage dummy;
    dequeueMessage(dummy);
  }

  // Store message
  MQTTMessage &msg = _messageBuffer[_bufferHead];
  msg.timestamp = millis();
  strncpy(msg.topic, topic, sizeof(msg.topic) - 1);
  msg.topic[sizeof(msg.topic) - 1] = '\0';
  strncpy(msg.payload, payload, sizeof(msg.payload) - 1);
  msg.payload[sizeof(msg.payload) - 1] = '\0';
  msg.qos = qos;

  // Update head pointer (circular buffer)
  _bufferHead = (_bufferHead + 1) % MQTT_BUFFER_SIZE;
  _bufferCount++;

  DEBUG_PRINT(F("[SIM7070G] Message buffered ("));
  DEBUG_PRINT(_bufferCount);
  DEBUG_PRINT(F("/"));
  DEBUG_PRINT(MQTT_BUFFER_SIZE);
  DEBUG_PRINTLN(F(")"));

  return true;
}

// Dequeue message from buffer
bool Sim7070GDevice::dequeueMessage(MQTTMessage &msg)
{
  // Check if buffer is empty
  if (_bufferCount == 0)
  {
    return false;
  }

  // Copy message
  msg = _messageBuffer[_bufferTail];

  // Update tail pointer (circular buffer)
  _bufferTail = (_bufferTail + 1) % MQTT_BUFFER_SIZE;
  _bufferCount--;

  return true;
}

// Clear message buffer
void Sim7070GDevice::clearMessageBuffer()
{
  _bufferHead = 0;
  _bufferTail = 0;
  _bufferCount = 0;
  DEBUG_PRINTLN(F("[SIM7070G] Message buffer cleared"));
}

// Flush all buffered messages
bool Sim7070GDevice::flushMessageBuffer()
{
  if (_bufferCount == 0)
  {
    return true;
  }

  DEBUG_PRINT(F("[SIM7070G] Flushing "));
  DEBUG_PRINT(_bufferCount);
  DEBUG_PRINTLN(F(" buffered messages..."));

  uint8_t successCount = 0;
  uint8_t failCount = 0;
  uint8_t messagesToSend = _bufferCount;

  for (uint8_t i = 0; i < messagesToSend; i++)
  {
    MQTTMessage msg;
    if (!dequeueMessage(msg))
    {
      break;
    }

    DEBUG_PRINT(F("[SIM7070G] Sending buffered message: "));
    DEBUG_PRINTLN(msg.topic);

    // Try to publish the message
    if (_modem && _modem->mqttPublishAT(msg.topic, msg.payload, msg.qos, 0))
    {
      successCount++;
      DEBUG_PRINTLN(F("[SIM7070G] Buffered message sent successfully"));
    }
    else
    {
      failCount++;
      DEBUG_PRINTLN(F("[SIM7070G] Failed to send buffered message, re-queuing"));
      // Re-queue the message at the front
      enqueueMessage(msg.topic, msg.payload, msg.qos);
      // Stop trying to send more messages
      break;
    }

    // Small delay between messages
    delay(100);
  }

  DEBUG_PRINT(F("[SIM7070G] Buffer flush complete: "));
  DEBUG_PRINT(successCount);
  DEBUG_PRINT(F(" sent, "));
  DEBUG_PRINT(failCount);
  DEBUG_PRINT(F(" failed, "));
  DEBUG_PRINT(_bufferCount);
  DEBUG_PRINTLN(F(" remaining"));

  return (failCount == 0);
}
