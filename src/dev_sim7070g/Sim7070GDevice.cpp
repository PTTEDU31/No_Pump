#include "Sim7070GDevice.h"
#include "DEBUG.h"
#include "config.h"

Sim7070GDevice* Sim7070GDevice::s_instance = nullptr;

void Sim7070GDevice::mqttMessageThunk(const char* topic, const uint8_t* payload, uint32_t len)
{
  if (s_instance)
  {
    s_instance->onMqttMessage(topic, payload, len);
  }
}

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
      _bufferCount(0),
      _networkInfoQueryPending(false),
      _networkInfoQueryCompleted(false),
      _mqttConnectPending(false),
      _mqttConnectCompleted(false)
{
  s_instance = this;
  memset(&_stats, 0, sizeof(_stats));

  // Create Sim7070G instance
  _modem = new Sim7070G(modemSerial, MODEM_POWER_PIN, MODEM_BAUD_RATE);

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
  // _networkMode = NET_MODE_NB; // Default to NB

  // Reset statistics
  // resetStats();
}

// Destructor
Sim7070GDevice::~Sim7070GDevice()
{
  if (_modem) {
    delete _modem;
    _modem = nullptr;
  }
}

// Initialize - Called at the beginning of setup()
bool Sim7070GDevice::initialize()
{
  DEBUG_PRINTLN(F("[SIM7070G] Initializing modem device..."));

  if (!_modem) {
    DEBUG_PRINTLN(F("[SIM7070G] ERROR: Modem instance not created!"));
    return false;
  }

  // Initialize Sim7070G library
  if (!_modem->begin()) {
    DEBUG_PRINTLN(F("[SIM7070G] Failed to initialize Sim7070G library"));
    return false;
  }

  // Set MQTT message callback
  _modem->setMQTTMessageCallback(&Sim7070GDevice::mqttMessageThunk);

  updateState(MODEM_OFF);
  return true;
}

// Start - Called at the end of setup()
int Sim7070GDevice::start()
{
  DEBUG_PRINTLN(F("[SIM7070G] Starting modem..."));

  // MQTT topics
  snprintf(_pubTopic, sizeof(_pubTopic), "nono/%s/pub", _nodeId);
  snprintf(_subTopic, sizeof(_subTopic), "nono/%s/sub", _nodeId);

  // MQTT credentials
  snprintf(_mqttBroker, sizeof(_mqttBroker), "%s", mqtt_server);
  _mqttPort = (uint16_t)mqtt_port;
  snprintf(_mqttClientId, sizeof(_mqttClientId), "nono_%s", _nodeId);
  snprintf(_mqttUsername, sizeof(_mqttUsername), "%s", USERNAME ? USERNAME : "");
  snprintf(_mqttPassword, sizeof(_mqttPassword), "%s", PASSWORD ? PASSWORD : "");

  // APN
  snprintf(_apn, sizeof(_apn), "%s", PDP_APN);
  _gprsUser[0] = '\0';
  _gprsPass[0] = '\0';

  _mqttBegun = false;
  _lastHeartbeatMs = 0;

  updateState(MODEM_INITIALIZING);
  return DURATION_IMMEDIATELY;
}

void Sim7070GDevice::powerPulse()
{
  // Best-effort drain UART before power toggling
  unsigned long t0 = millis();
  while (_modemSerial->available() && (millis() - t0) < PRE_DRAIN_MS)
  {
    _modemSerial->read();
  }

  DEBUG_PRINTLN(F("[SIM7070G] PWRKEY pulse"));
  if (_modem) {
    _modem->powerOn();
  } else {
    // Fallback to direct pin control
    digitalWrite(MODEM_POWER_PIN, LOW);
    delay(50);
    digitalWrite(MODEM_POWER_PIN, HIGH);
  }
}

bool Sim7070GDevice::applyBootConfig()
{
  if (!_modem) return false;

  // Step 1: Select preferred RAT (CAT-M vs NB-IoT)
  // AT+CMNB: 1=CAT-M, 2=NB-IoT, 3=CAT-M and NB-IoT
  uint8_t rat;
  if (NETWORK_MODE_NB)
  {
    rat = 2; // NB-IoT
  }
  else
  {
    rat = 1; // CAT-M
  }
  
  if (!_modem->setPreferredRAT(rat)) {
    DEBUG_PRINTLN(F("[SIM7070G] Failed to set preferred RAT"));
    return false;
  }
  
  // Step 2: Set network mode to LTE only
  // AT+CNMP: 2=Automatic, 13=GSM only, 38=LTE only, 51=GSM and LTE only
  if (!_modem->setNetworkMode(2)) { //Automatic
    DEBUG_PRINTLN(F("[SIM7070G] Failed to set network mode"));
    return false;
  }
  
  // Step 3: Set timezone to GMT+7 (Vietnam timezone)
  // GMT+7 = 7 * 4 = 28 quarter-hours
  if (!_modem->setTimezone(28)) {
    DEBUG_PRINTLN(F("[SIM7070G] Failed to set timezone, continuing anyway..."));
    // Don't fail boot config if timezone setting fails
  }
  
  // _modem->sendCommandSync("AT+CFUN=1,1",10);
  // DEBUG_PRINTLN(F("[SIM7070G] Modem soft reset to apply boot config"));
  return true;
}
  
bool Sim7070GDevice::ensureNetworkReady()
{
  if (!_modem) return false;


  // // Wait for network registration (with timeout)
  // unsigned long startTime = millis();
  // NetworkRegStatus regStatus;
  // while ((millis() - startTime) < 60000) {
  //   _modem->loop(); // Process async operations
    
  //   if (_modem->getNetworkRegistration(&regStatus)) {
  //     if (regStatus == NetworkRegStatus::REGISTERED_HOME || 
  //         regStatus == NetworkRegStatus::REGISTERED_ROAMING) {
  //       break; // Registered, continue to activate PDP
  //     }
  //   }
  //   delay(1000);
  // }
  // Activate PDP context
  // const char* username = _gprsUser[0] ? _gprsUser : nullptr;
  // const char* password = _gprsPass[0] ? _gprsPass : nullptr;
  
  // bool ok = _modem->activatePDPContext(PDP_CONTEXT_ID, _apn, username, password);
  // if (ok)
  // {
  //   _stats.gprsConnects++;
  //   _stats.lastGPRSConnectTime = millis();
  // }
  // bool ok = _modem->attacthService();
  return true;
}

bool Sim7070GDevice::ensureMqttReady()
{
  if (!_modem) return false;

  if (!_mqttBegun)
  {
    if (!_modem->mqttBegin())
    {
      return false;
    }
    
    // Set MQTT configuration
    const char* user = _mqttUsername[0] ? _mqttUsername : nullptr;
    const char* pass = _mqttPassword[0] ? _mqttPassword : nullptr;
    
    if (!_modem->mqttSetConfig(_mqttClientId, _mqttBroker, _mqttPort, 
                               user, pass, MQTT_KEEPALIVE_SEC, true))
    {
      return false;
    }
    
    _mqttBegun = true;
    _modem->setMQTTMessageCallback(&Sim7070GDevice::mqttMessageThunk);
  }

  if (_modem->isMQTTConnected())
  {
    return true;
  }

  bool ok = _modem->mqttConnect();
  if (ok)
  {
    _stats.mqttConnects++;
    _stats.lastMQTTConnectTime = millis();
  }
  return ok;
}

void Sim7070GDevice::handleIncomingData()
{
  // Sim7070G library handles incoming data automatically in loop()
  // This method is kept for compatibility but does nothing
  if (_modem) {
    _modem->loop();
  }
}

void Sim7070GDevice::onMqttMessage(const char* topic, const uint8_t* payload, uint32_t len)
{
  DEBUG_PRINT(F("[SIM7070G] MQTT RX topic="));
  DEBUG_PRINT(topic ? topic : "(null)");
  DEBUG_PRINT(F(" len="));
  DEBUG_PRINTLN(len);

  // TODO: Map message to actions; for now just print payload (truncated)
  if (payload && len)
  {
    const uint32_t maxPrint = 200;
    uint32_t n = (len > maxPrint) ? maxPrint : len;
    for (uint32_t i = 0; i < n; i++)
    {
      Serial.write((char)payload[i]);
    }
    if (len > maxPrint) { Serial.print(F("...")); }
    Serial.println();
  }
}

// Timeout - Called periodically
int Sim7070GDevice::timeout()
{
  switch (_state)
  {
  case MODEM_INITIALIZING:
    // Process async operations
    if (_modem) {
      _modem->loop();
    }
    if (_modem->checkSendCommandSync("AT", "OK", 1000))
    {
      _modem->checkSendCommandSync("AT+CPOWD=1", "NORMAL POWER DOWN", 1000);
        
    }
    updateState(MODEM_RESET);
    return 3000;

  case MODEM_RESET:
  {
    // Process async operations
    if (_modem) {
      _modem->loop();
    }
    powerPulse();
    updateState(MODEM_READY);
    return (7000);
  }
  case MODEM_READY:
  {
    // Process async operations
    if (_modem) {
      _modem->loop();
    }
    
    if (_modem->checkSendCommandSync("AT", "OK",100))
    {
      updateState(MODEM_CHECK_SIM);
      _retryCount = 0;
      return DURATION_IMMEDIATELY;
    }else{
      powerPulse();
      DEBUG_PRINTLN(F("[SIM7070G] Modem not responding, retrying power on..."));
      return 5000;
    }
  }
  case MODEM_CHECK_SIM:
  {
    // Process async operations
    if (_modem) {
      _modem->loop();
    }
    if(_modem->checkSIM()){
      DEBUG_PRINTLN(F("[SIM7070G] SIM card READY"));
      updateState(MODEM_GPRS_CONNECTING_SET_BAUD);
      _retryCount = 0;
      return DURATION_IMMEDIATELY;
    }
    _retryCount++;
    if (_retryCount >= 6)
    {
      updateState(MODEM_ERROR);
      return 15000;
    }
    return 2000;
  }
  case MODEM_GPRS_CONNECTING_SET_BAUD:
  {
    updateState(MODEM_GPRS_CONNECTING_SET_MODE);
    return DURATION_IMMEDIATELY;
  }
  case MODEM_GPRS_CONNECTING_SET_MODE:
  {
    if(!applyBootConfig()){
      updateState(MODEM_ERROR);
      DEBUG_PRINTLN(F("[SIM7070G] Failed to apply boot config"));
      return 10000;
    }
    updateState(MODEM_QUERY_NETWORK_INFO);
    return 5000;
  }
  case MODEM_GPRS_CONNECTING_ATTACH:
    // Process async operations
    if (_modem) {
      _modem->loop();
    }
    
    if(_modem->attacthService())
    {
      
      // Move to query network info state
      updateState(MODEM_GPRS_WAIT_CONNECTION);
      return 5000;
    }

    _retryCount++;
    if (_retryCount >= MQTT_MAX_RECONNECT_ATTEMPTS)
    {
      updateState(MODEM_GPRS_CONNECTING_ATTACH);
      return 5000;
    }


  case MODEM_QUERY_NETWORK_INFO:
  {
    // Process async operations
    if (_modem) {
      _modem->loop();
    }
    
    // Check CREG (CS network registration)
    char cregResponse[128];
    bool cregOk = false;
    if (_modem->sendCommandSync("AT+CREG?", 2000, cregResponse, sizeof(cregResponse))) {
      // Parse +CREG: <n>,<stat>
      // n: 0=disable, 1=enable
      // stat: 0=not registered, 1=registered home, 2=searching, 3=denied, 5=registered roaming
      int n, stat;
      if (sscanf(cregResponse, "+CREG: %d,%d", &n, &stat) == 2) {
        if (stat == 1) { // Registered home network
          cregOk = true;
          DEBUG_PRINTLN(F("[SIM7070G] CREG: Registered home network"));
        } else {
          DEBUG_PRINT(F("[SIM7070G] CREG: Not registered (stat="));
          DEBUG_PRINT(stat);
          DEBUG_PRINTLN(F(")"));
        }
      }
    }
    
    // Check CGREG (PS network registration)
    char cgregResponse[128];
    bool cgregOk = false;
    if (_modem->sendCommandSync("AT+CGREG?", 2000, cgregResponse, sizeof(cgregResponse))) {
      // Parse +CGREG: <n>,<stat>
      // n: 0=disable, 1=enable
      // stat: 0=not registered, 1=registered home, 2=searching, 3=denied, 5=registered roaming
      int n, stat;
      if (sscanf(cgregResponse, "+CGREG: %d,%d", &n, &stat) == 2) {
        if (stat == 1 || stat == 5) { // Registered home (1) or roaming (5)
          cgregOk = true;
          DEBUG_PRINT(F("[SIM7070G] CGREG: Registered (stat="));
          DEBUG_PRINT(stat);
          DEBUG_PRINTLN(stat == 1 ? F(", home)") : F(", roaming)"));
        } else {
          DEBUG_PRINT(F("[SIM7070G] CGREG: Not registered (stat="));
          DEBUG_PRINT(stat);
          DEBUG_PRINTLN(F(")"));
        }
      }
    }
    
    // Only proceed if both CREG and CGREG are registered
    if (cregOk && cgregOk) {
      DEBUG_PRINTLN(F("[SIM7070G] Network registration OK, proceeding to attach"));
      _modem->sendCommandSync("AT+COPS?", 10000);
      updateState(MODEM_GPRS_CONNECTING_ATTACH);
      return DURATION_IMMEDIATELY;
    } else {
      DEBUG_PRINTLN(F("[SIM7070G] Network not registered yet, waiting..."));
      _retryCount++;
      if (_retryCount >= 10) { // Retry up to 10 times (50 seconds total)
        DEBUG_PRINTLN(F("[SIM7070G] Network registration timeout"));
        updateState(MODEM_ERROR);
        return 10000;
      }
      return 5000; // Retry after 5 seconds
    }
  }
  case MODEM_GPRS_CONNECTING_PDP_CONTEXT:
  {
    // Process async operations
    if (_modem) {
      _modem->loop();
    }
    bool ok = _modem->PDPContextActivation(PDP_CONTEXT_ID, _apn, _gprsUser, _gprsPass);
    if (ok)
    {
      updateState(MODEM_GPRS_CONNECTED);
      return DURATION_IMMEDIATELY;
    }

    _retryCount++;
    if (_retryCount >= MQTT_MAX_RECONNECT_ATTEMPTS)
    {
      updateState(MODEM_GPRS_CONNECTING_ATTACH);
      return 10000;
    }

    return 5000;
  }
  case MODEM_GPRS_WAIT_CONNECTION:
  {
    // Process async operations
    if (_modem) {
      _modem->loop();
    }
    if(_modem->getState() == Sim7070GState::NETWORK_CONNECTED)
    {
      updateState(MODEM_GPRS_IP_CHECK);
      return 10000;
    }

    return 500;
  }
  case MODEM_GPRS_IP_CHECK:
  {
    // Process async operations
    if (_modem) {
      _modem->loop();
    }
    char ip[16];

    if(_modem->getIPAddress(ip, sizeof(ip)))
    {
      DEBUG_PRINT(F("[SIM7070G] Got IP address: "));
      DEBUG_PRINTLN(ip);
      updateState(MODEM_SYNC_NTP_TIME);
      _retryCount = 0;
      return DURATION_IMMEDIATELY;
    }
    return 10000;
  }
  case MODEM_SYNC_NTP_TIME:
  {
    // Process async operations
    if (_modem) {
      _modem->loop();
    }
    
    DEBUG_PRINTLN(F("[SIM7070G] Synchronizing NTP time..."));
    
    // Try to sync NTP time with multiple servers
    bool ntpSuccess = _modem->syncTimeUTC_any(PDP_CONTEXT_ID, 12000);
    
    if (ntpSuccess)
    {
      // Get and print the synchronized time
      char isoTime[21];
      if (_modem->getNetworkTimeISO8601(isoTime, sizeof(isoTime)))
      {
        DEBUG_PRINT(F("[SIM7070G] NTP sync successful. Current time: "));
        DEBUG_PRINTLN(isoTime);
      }
      else
      {
        DEBUG_PRINTLN(F("[SIM7070G] NTP sync successful but failed to get ISO time"));
      }
      
      // Move to MQTT connection
      updateState(MODEM_MQTT_CONNECTING_HANDSHAKE);
      _retryCount = 0;
      return DURATION_IMMEDIATELY;
    }
    else
    {
      // NTP sync failed, but continue anyway (don't block MQTT connection)
      DEBUG_PRINTLN(F("[SIM7070G] NTP sync failed, continuing to MQTT connection..."));
      _retryCount++;
      
      // Retry NTP sync up to 2 times, then continue anyway
      if (_retryCount >= 2)
      {
        DEBUG_PRINTLN(F("[SIM7070G] NTP sync failed after retries, proceeding to MQTT"));
        updateState(MODEM_MQTT_CONNECTING_HANDSHAKE);
        _retryCount = 0;
        return DURATION_IMMEDIATELY;
      }
      
      return 5000; // Retry NTP sync after 5 seconds
    }
  }
  case MODEM_MQTT_CONNECTING_HANDSHAKE:
  {
    // Process async operations
    if (_modem) {
      _modem->loop();
    }
    DEBUG_PRINTLN(F("[SIM7070G] Setting MQTT config"));
    DEBUG_PRINTLN(F("[SIM7070G] Client ID: "));
    DEBUG_PRINTLN(_mqttClientId);
    DEBUG_PRINTLN(F("[SIM7070G] Broker: "));
    DEBUG_PRINTLN(_mqttBroker);
    DEBUG_PRINTLN(F("[SIM7070G] Port: "));
    DEBUG_PRINTLN(_mqttPort);
    DEBUG_PRINTLN(F("[SIM7070G] Username: "));
    DEBUG_PRINTLN(_mqttUsername);
    DEBUG_PRINTLN(F("[SIM7070G] Password: "));
    DEBUG_PRINTLN(_mqttPassword);
    DEBUG_PRINTLN(F("[SIM7070G] Keepalive: "));
    DEBUG_PRINTLN(MQTT_KEEPALIVE_SEC);
    DEBUG_PRINTLN(F("[SIM7070G] Clean session: "));
    DEBUG_PRINTLN(true);

    if(!_modem->mqttSetConfig(_mqttClientId, _mqttBroker, _mqttPort, _mqttUsername, _mqttPassword, MQTT_KEEPALIVE_SEC, true))
    {
      DEBUG_PRINTLN(F("[SIM7070G] Failed to set MQTT config"));
      // updateState(MODEM_ERROR);
      return 20000;
    }
    DEBUG_PRINTLN(F("[SIM7070G] Connecting to MQTT"));
    _modem->sendCommandSync("AT+SMCONF?",5000);
    updateState(MODEM_MQTT_CONNECTING);
    return 2000;
  }
  case MODEM_MQTT_CONNECTING:
  {
    // Process async operations
    if (_modem) {
      _modem->loop();
    }
    
    // Check if this is first entry to state - send async command
    if (!_mqttConnectPending && !_mqttConnectCompleted) {
      DEBUG_PRINTLN(F("[SIM7070G] Sending async AT+SMCONN command..."));
      int cmdId = _modem->sendCommand("AT+SMCONN", 60000, onMqttConnectCallback, nullptr);
      if (cmdId >= 0) {
        _mqttConnectPending = true;
        DEBUG_PRINTLN(F("[SIM7070G] MQTT connect command queued, waiting for response..."));
      } else {
        DEBUG_PRINTLN(F("[SIM7070G] Failed to queue MQTT connect command"));
        _retryCount++;
        if (_retryCount >= 3) {
          updateState(MODEM_ERROR);
          DEBUG_PRINTLN(F("[SIM7070G] Failed to connect to MQTT after retries"));
          return 10000;
        }
        return 2000; // Retry after 2s
      }
    }
    
    // Check if connect completed
    if (_mqttConnectCompleted) {
      DEBUG_PRINTLN(F("[SIM7070G] MQTT connect completed successfully"));
      _mqttConnectPending = false;
      _mqttConnectCompleted = false;
      updateState(MODEM_MQTT_CONNECTED_FIRST);
      return DURATION_IMMEDIATELY;
    }
    
    // Check timeout (60 seconds)
    if ((millis() - _stateStartTime) >= 60000) {
      DEBUG_PRINTLN(F("[SIM7070G] MQTT connect timeout after 60s"));
      _mqttConnectPending = false;
      _mqttConnectCompleted = false;
      _retryCount++;
      if (_retryCount >= 3) {
        updateState(MODEM_ERROR);
        DEBUG_PRINTLN(F("[SIM7070G] MQTT connect failed after max retries"));
        return 10000;
      }
      // Reset and retry
      return DURATION_IMMEDIATELY;
    }
    
    // Still waiting for response
    return 500; // Check frequently
  }
  case MODEM_MQTT_CONNECTED_FIRST:
  {
    // Process async operations
    if (_modem) {
      _modem->loop();
      
      // Subscribe to topic
      _modem->mqttSubscribe(_subTopic, 0);

      // Publish online status
      char msg[160];
      snprintf(msg, sizeof(msg), "{\"id\":\"%s\",\"status\":\"online\"}", _nodeId);
      _modem->mqttPublish(_pubTopic, msg, 0, false);
    }

    _lastHeartbeatMs = millis();
    updateState(MODEM_MQTT_CONNECTED);
    return 200;
  }

  case MODEM_MQTT_CONNECTED:
    // CRITICAL: Must call loop() to process async operations
    if (_modem) {
      _modem->loop();
    }
    handleIncomingData();

    if (!_modem || !_modem->isMQTTConnected())
    {
      _stats.mqttDisconnects++;
      updateState(MODEM_GPRS_CONNECTED);
      return 2000;
    }

    if (!_modem->isNetworkConnected())
    {
      updateState(MODEM_GPRS_CONNECTING_ATTACH);
      return 5000;
    }

    if (_lastHeartbeatMs == 0 || (millis() - _lastHeartbeatMs) >= 30000UL)
    {
      char hb[200];
      int8_t rssi;
      if (_modem && _modem->getSignalQuality(&rssi, nullptr)) {
        snprintf(hb, sizeof(hb), "{\"id\":\"%s\",\"csq\":%d,\"uptime\":%lu}",
                 _nodeId, (int)rssi, (unsigned long)(millis() / 1000UL));
        _modem->mqttPublish(_pubTopic, hb, 0, false);
        _lastHeartbeatMs = millis();
      }
    }

    if (_bufferCount > 0)
    {
      flushMessageBuffer();
    }

    return 200;

  case MODEM_ERROR:
    DEBUG_PRINTLN(F("[SIM7070G] Error state, retrying full init..."));
    _stats.modemRestarts++;
    _mqttBegun = false;
    updateState(MODEM_INITIALIZING);
    return 15000;

  default:
    return 10000;
  }
}

// Event - Handle events
int Sim7070GDevice::event()
{
  return DURATION_IGNORE;
}

static const __FlashStringHelper* modemStateName(ModemState s)
{
  switch (s)
  {
    case MODEM_OFF: return F("MODEM_OFF");
    case MODEM_INITIALIZING: return F("MODEM_INITIALIZING");
    case MODEM_RESET: return F("MODEM_RESET");
    case MODEM_READY: return F("MODEM_READY");
    case MODEM_CHECK_SIM: return F("MODEM_CHECK_SIM");
    case MODEM_GPRS_CONNECTING_SET_BAUD: return F("MODEM_GPRS_CONNECTING_SET_BAUD");
    case MODEM_GPRS_CONNECTING_SET_MODE: return F("MODEM_GPRS_CONNECTING_SET_MODE");
    case MODEM_GPRS_CONNECTING_WAIT_MODE: return F("MODEM_GPRS_CONNECTING_WAIT_MODE");
    case MODEM_GPRS_CONNECTING_CHECK_SIGNAL: return F("MODEM_GPRS_CONNECTING_CHECK_SIGNAL");
    case MODEM_GPRS_CONNECTING_ATTACH: return F("MODEM_GPRS_CONNECTING_ATTACH");
    case MODEM_GPRS_CONNECTED: return F("MODEM_GPRS_CONNECTED");
    case MODEM_QUERY_NETWORK_INFO: return F("MODEM_QUERY_NETWORK_INFO");
    case MODEM_MQTT_CONNECTING_HANDSHAKE: return F("MODEM_MQTT_CONNECTING_HANDSHAKE");
    case MODEM_MQTT_CONNECTED_FIRST: return F("MODEM_MQTT_CONNECTED_FIRST");
    case MODEM_MQTT_CONNECTED: return F("MODEM_MQTT_CONNECTED");
    case MODEM_ERROR: return F("MODEM_ERROR");
    default: return F("MODEM_UNKNOWN");
  }
}

void Sim7070GDevice::updateState(ModemState newState)
{
  if (_state == newState)
  {
    return;
  }

  _state = newState;
  _lastStateChange = millis();
  _stateStartTime = _lastStateChange;
  _retryCount = 0;
  
  // Reset async query flags when entering query state
  if (newState == MODEM_QUERY_NETWORK_INFO) {
    _networkInfoQueryPending = false;
    _networkInfoQueryCompleted = false;
  }
  
  // Reset async MQTT connect flags when entering MQTT connecting state
  if (newState == MODEM_MQTT_CONNECTING) {
    _mqttConnectPending = false;
    _mqttConnectCompleted = false;
  }

  DEBUG_PRINT(F("[SIM7070G] State -> "));
  DEBUG_PRINT(modemStateName(newState));
  DEBUG_PRINT(F("("));
  DEBUG_PRINT((int)newState);
  DEBUG_PRINTLN(F(")"));
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
    MQTTBufferedMessage dummy;
    dequeueMessage(dummy);
  }

  // Store message
  MQTTBufferedMessage &msg = _messageBuffer[_bufferHead];
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
bool Sim7070GDevice::dequeueMessage(MQTTBufferedMessage &msg)
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
    MQTTBufferedMessage msg;
    if (!dequeueMessage(msg))
    {
      break;
    }

    DEBUG_PRINT(F("[SIM7070G] Sending buffered message: "));
    DEBUG_PRINTLN(msg.topic);

    if (_state == MODEM_MQTT_CONNECTED && _modem && _modem->isMQTTConnected())
    {
      // msg.payload is char[], so it's compatible with mqttPublish
      if (_modem->mqttPublish(msg.topic, msg.payload, msg.qos, false))
      {
        successCount++;
      }
      else
      {
        failCount++;
        enqueueMessage(msg.topic, msg.payload, msg.qos);
        break;
      }
    }
    else
    {
      failCount++;
      enqueueMessage(msg.topic, msg.payload, msg.qos);
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

// Async command callback handlers
void Sim7070GDevice::onQueryNetworkInfoCallback(ATResponseType type, const char* response, void* userData) {
  if (!s_instance) {
    return;
  }
  
  Sim7070GDevice* device = s_instance;
  device->_networkInfoQueryPending = false;
  
  if (type == ATResponseType::OK && response) {
    DEBUG_PRINT(F("[SIM7070G] Network info query OK: "));
    DEBUG_PRINTLN(response);
    device->_networkInfoQueryCompleted = true;
  } else {
    DEBUG_PRINT(F("[SIM7070G] Network info query failed: "));
    if (type == ATResponseType::TIMEOUT) {
      DEBUG_PRINTLN(F("TIMEOUT"));
    } else if (type == ATResponseType::ERROR) {
      DEBUG_PRINTLN(F("ERROR"));
    } else {
      DEBUG_PRINTLN(F("UNKNOWN"));
    }
    device->_networkInfoQueryCompleted = false;
  }
}

// Async MQTT connect callback handler
void Sim7070GDevice::onMqttConnectCallback(ATResponseType type, const char* response, void* userData) {
  if (!s_instance) {
    return;
  }
  
  Sim7070GDevice* device = s_instance;
  device->_mqttConnectPending = false;
  
  if (type == ATResponseType::OK && response) {
    DEBUG_PRINT(F("[SIM7070G] MQTT connect OK: "));
    DEBUG_PRINTLN(response);
    device->_mqttConnectCompleted = true;
  } else {
    DEBUG_PRINT(F("[SIM7070G] MQTT connect failed: "));
    if (type == ATResponseType::TIMEOUT) {
      DEBUG_PRINTLN(F("TIMEOUT"));
    } else if (type == ATResponseType::ERROR) {
      DEBUG_PRINTLN(F("ERROR"));
    } else {
      DEBUG_PRINTLN(F("UNKNOWN"));
    }
    device->_mqttConnectCompleted = false;
  }
}
