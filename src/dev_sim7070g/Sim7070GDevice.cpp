#include "Sim7070GDevice.h"
#include "DEBUG.h"
#include "config.h"
#include "crypto_json_utils.h"
#include "dev_pump/PumpDevice.h"

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
  snprintf(_pubTopic, sizeof(_pubTopic), "%s", _nodeId);
  snprintf(_subTopic, sizeof(_subTopic), "%s", _nodeId);

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

  // Start by powering on modem
  updateState(MODEM_POWER_ON);
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
    digitalWrite(MODEM_POWER_PIN, HIGH);
    delay(1000);
    digitalWrite(MODEM_POWER_PIN, LOW);
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

  if (!payload || len == 0) {
    return;
  }

  // Convert payload to null-terminated string (assuming it's HEX string)
  char hexPayload[600];
  size_t hexLen = (len > sizeof(hexPayload) - 1) ? sizeof(hexPayload) - 1 : len;
  memcpy(hexPayload, payload, hexLen);
  hexPayload[hexLen] = '\0';

  DEBUG_PRINT(F("[RX] HEX payload (raw), len="));
  DEBUG_PRINTLN(strlen(hexPayload));
  DEBUG_PRINTLN(hexPayload);

  const char* hexStr = hexPayload;
  size_t hexStrLen = strlen(hexStr);

  // Encrypted path: valid HEX length and successful decode
  if ((hexStrLen % 2) == 0 && hexStrLen >= (12 + 16) * 2) {
    uint8_t buf[480];
    size_t bytes = 0;
    if (hexDecode(hexStr, buf, sizeof(buf), bytes) && bytes >= (12 + 16)) {
      DEBUG_PRINT(F("[RX] HEX payload (raw), len="));
      DEBUG_PRINTLN(hexStrLen);
      DEBUG_PRINTLN(hexPayload);

  // Extract nonce, ciphertext, and tag
  uint8_t* nonce = buf;
  uint8_t* tag = buf + (bytes - 16);
  uint8_t* ct = buf + 12;
  size_t ctLen = bytes - 12 - 16;

  // Extract time from nonce
  int nH = nonce[4];
  int nM = nonce[5];
  int nS = nonce[6];
  int secMsg = nH * 3600 + nM * 60 + nS;

  // Get network time (try up to 5 times)
  char isoNow[21] = { 0 };
  int rH = 0, rM = 0, rS = 0;
  bool timeOk = false;

  for (uint8_t attempt = 1; attempt <= 5; attempt++) {
    DEBUG_PRINT(F("[TIME] Try number "));
    DEBUG_PRINT(attempt);
    DEBUG_PRINTLN(F(" to obtain ISO time..."));

    memset(isoNow, 0, sizeof(isoNow));
    if (_modem && _modem->getNetworkTimeISO8601(isoNow, sizeof(isoNow))) {
      DEBUG_PRINT(F("[TIME] Response CCLK ISO: "));
      DEBUG_PRINTLN(isoNow);

      if (sscanf(isoNow + 11, "%2d:%2d:%2d", &rH, &rM, &rS) == 3) {
        DEBUG_PRINT(F("[TIME] Successful parsing of time on try number: "));
        DEBUG_PRINTLN(attempt);
        timeOk = true;
        break;
      } else {
        DEBUG_PRINTLN(F("[TIME] Failure to parse format HH:MM:SS."));
      }
    } else {
      DEBUG_PRINTLN(F("[TIME] getNetworkTimeISO8601() failed (without response)."));
    }

    delay(200 * attempt);
  }

  if (!timeOk) {
    DEBUG_PRINTLN(F("[TIME] Unable to obtain hour from network after 5 tries. Command discarded."));
    return;
  }

  // Calculate time difference (circular difference for 24-hour clock)
  const int secNow = rH * 3600 + rM * 60 + rS;
  const int DAY = 86400;
  int d = abs(secMsg - secNow);
  if (d > DAY) d %= DAY;
  int dsec = min(d, DAY - d);

  DEBUG_PRINT(F(" | NET HMS="));
  DEBUG_PRINT(isoNow + 11);
  DEBUG_PRINT(F(" | Δs="));
  DEBUG_PRINTLN(dsec);

  // Check time window (30 seconds)
  const int WINDOW_SEC = 30;
  if (dsec > WINDOW_SEC) {
    DEBUG_PRINTLN(F("[TIME] Command outside of time window (>30 s). Ignored."));
    return;
  }

  // Decrypt payload
  char plain[420];
  bool ok = decryptPayload(ct, ctLen, tag, nonce, plain, sizeof(plain));
  DEBUG_PRINT(F("[DECRYPT] Result: "));
  DEBUG_PRINTLN(ok ? F("OK") : F("FAIL"));
  if (!ok) {
    return;
  }

  DEBUG_PRINT(F("[RX] clear JSON: "));
  DEBUG_PRINTLN(plain);

  handleMqttCommandJson(plain);
  return;
  }
  }  // end if valid hex length

  // Plain text path: payload is not encrypted HEX (e.g. "hello" or {"X":1})
  DEBUG_PRINTLN(F("[RX] Plain text message"));
  char plainBuf[512];
  size_t plainLen = (len < sizeof(plainBuf) - 1) ? len : sizeof(plainBuf) - 1;
  memcpy(plainBuf, payload, plainLen);
  plainBuf[plainLen] = '\0';
  DEBUG_PRINTLN(plainBuf);
  handleMqttCommandJson(plainBuf);
}

void Sim7070GDevice::handleMqttCommandJson(const char* plain)
{
  if (!plain) return;

  long xVal = -1;
  if (jsonGetInt(plain, "X", xVal)) {
    extern PumpDevice* g_pumpDevice;
    if (!g_pumpDevice) {
      DEBUG_PRINTLN(F("[ACT] PumpDevice not available"));
      return;
    }
    if (xVal == 1) {
      if (g_pumpDevice->isProtectionActive()) {
        unsigned long remain = g_pumpDevice->getProtectionRemainingMs();
        DEBUG_PRINT(F("[ACT] ON command arrived, ignored for protection ("));
        DEBUG_PRINT(remain / 1000UL);
        DEBUG_PRINTLN(F(" s remaining)."));
      } else {
        DEBUG_PRINTLN(F("[ACT] Turning pump ON"));
        g_pumpDevice->requestTurnOn();
      }
    } else if (xVal == 0) {
      DEBUG_PRINTLN(F("[ACT] Turning pump OFF"));
      g_pumpDevice->setLastOffByHexFlag(true);
      g_pumpDevice->requestTurnOff();
      g_pumpDevice->armProtection("Turned off by HEX command");
    } else {
      DEBUG_PRINT(F("[ACT] Invalid X Value: "));
      DEBUG_PRINTLN(xVal);
    }
  } else {
    DEBUG_PRINTLN(F("[ACT] JSON without X field."));
  }

  long newPulse;
  if (jsonGetInt(plain, "onPulse", newPulse)) {
    int onPulse = constrain((int)newPulse, 500, 10000);
    DEBUG_PRINT(F("[ACT] onPulse updated to "));
    DEBUG_PRINTLN(onPulse);
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
      _modem->checkSendCommandSync("AT+CPOWD=1", "", 1000);

    }
    updateState(MODEM_POWER_ON);
    return DURATION_IMMEDIATELY;

  case MODEM_POWER_ON:
  {
    // Process async operations
    if (_modem) {
      _modem->loop();
    }
    DEBUG_PRINTLN(F("[SIM7070G] Powering on modem..."));
    powerPulse();
    updateState(MODEM_ACTIVATE_CNACT_FIRST);
    
    return 15000; // Wait for modem to boot
  }

  case MODEM_ACTIVATE_CNACT_FIRST:
  {
    // Process async operations
    if (_modem) {
      _modem->loop();
    }
    if (!_modem->checkSendCommandSync("AT", "OK", 1000)) {
      DEBUG_PRINTLN(F("[SIM7070G] Modem not responding, retrying..."));
      updateState(MODEM_POWER_ON);
      return 200;
    }
    char ip[16];
    if (_modem->getIPAddress(ip, sizeof(ip)))
    {
      DEBUG_PRINTLN(F("[SIM7070G] IP already assigned, skipping CNACT..."));
      DEBUG_PRINT(F("[SIM7070G] IP: "));
      DEBUG_PRINTLN(ip);
      updateState(MODEM_MQTT_CONNECTING_HANDSHAKE);
      _retryCount = 0;
      return 500;
    }
   

    DEBUG_PRINTLN(F("[SIM7070G] Initiating AT+CNACT=1,1..."));
    if (_modem->checkSendCommandSync("AT+CNACT=0,1", "OK", 10000)) {
      DEBUG_PRINTLN(F("[SIM7070G] CNACT=1,1 success"));
      updateState(MODEM_WAIT_FOR_IP_FIRST);
      _retryCount = 0;
      return 7000; // Wait 7 seconds
    } else {
      DEBUG_PRINTLN(F("[SIM7070G] CNACT=1,1 failed, retrying..."));
      _retryCount++;
      if (_retryCount >= 3) {
        // After 3 failures, switch to apply boot config
        DEBUG_PRINTLN(F("[SIM7070G] CNACT failed after 3 attempts, switching to boot config"));
        _modem->checkSendCommandSync("AT+CNACT=0,0", "OK", 10000);
        updateState(MODEM_APPLY_BOOT_CONFIG);
        _retryCount = 0;
        return DURATION_IMMEDIATELY;
      }
      return 5000; // Retry after 5 seconds
    }
  }

  case MODEM_WAIT_FOR_IP_FIRST:
  {
    // Process async operations
    if (_modem) {
      _modem->loop();
    }
    DEBUG_PRINTLN(F("[SIM7070G] 10s wait done, checking IP..."));
    updateState(MODEM_CHECK_IP_FIRST);
    return DURATION_IMMEDIATELY;
  }

  case MODEM_CHECK_IP_FIRST:
  {
    // Process async operations
    if (_modem) {
      _modem->loop();
    }
    char ip[16];
    if (_modem->getIPAddress(ip, sizeof(ip))) {
      DEBUG_PRINT(F("[SIM7070G] IP check success: "));
      DEBUG_PRINTLN(ip);
      // Success, proceed to MQTT
      updateState(MODEM_MQTT_CONNECTING_HANDSHAKE);
      _retryCount = 0;
      return DURATION_IMMEDIATELY;
    } else {
      DEBUG_PRINTLN(F("[SIM7070G] IP check failed, switching to boot config..."));
      // Failed, switch to apply boot config
      updateState(MODEM_APPLY_BOOT_CONFIG);
      _retryCount = 0;
      return DURATION_IMMEDIATELY;
    }
  }

  case MODEM_APPLY_BOOT_CONFIG:
  {
    // Process async operations
    if (_modem) {
      _modem->loop();
    }
    DEBUG_PRINTLN(F("[SIM7070G] Applying boot config..."));
    if (!applyBootConfig()) {
      DEBUG_PRINTLN(F("[SIM7070G] Applying boot config failed"));
      updateState(MODEM_ERROR);
      return 10000;
    }
    DEBUG_PRINTLN(F("[SIM7070G] Boot config success"));
    updateState(MODEM_CHECK_SIM_AFTER_CONFIG);
    _retryCount = 0;
    return DURATION_IMMEDIATELY;
  }

  case MODEM_CHECK_SIM_AFTER_CONFIG:
  {
    // Process async operations
    if (_modem) {
      _modem->loop();
    }
    if (_modem->checkSIM()) {
      DEBUG_PRINTLN(F("[SIM7070G] SIM card READY"));
      updateState(MODEM_CHECK_SIGNAL_AFTER_CONFIG);
      _retryCount = 0;
      return DURATION_IMMEDIATELY;
    }
    _retryCount++;
    if (_retryCount >= 6) {
      updateState(MODEM_ERROR);
      return 15000;
    }
    return 2000;
  }

  case MODEM_CHECK_SIGNAL_AFTER_CONFIG:
  {
    // Process async operations
    if (_modem) {
      _modem->loop();
    }
    int8_t rssi;
    uint8_t ber;
    if (_modem->getSignalQuality(&rssi, &ber)) {
      DEBUG_PRINT(F("[SIM7070G] Signal - RSSI: "));
      DEBUG_PRINT(rssi);
      DEBUG_PRINT(F(", BER: "));
      DEBUG_PRINTLN(ber);
      updateState(MODEM_SET_APN);
      _retryCount = 0;
      return DURATION_IMMEDIATELY;
    } else {
      DEBUG_PRINTLN(F("[SIM7070G] Unable to get signal quality, retrying..."));
      _retryCount++;
      if (_retryCount >= 5) {
        updateState(MODEM_ERROR);
        return 10000;
      }
      return 2000;
    }
  }

  case MODEM_SET_APN:
  {
    // Process async operations
    if (_modem) {
      _modem->loop();
    }
    DEBUG_PRINT(F("[SIM7070G] Set APN: "));
    DEBUG_PRINTLN(_apn);
    if (_modem->setAPN(1, _apn, _gprsUser[0] ? _gprsUser : nullptr, _gprsPass[0] ? _gprsPass : nullptr)) {
      DEBUG_PRINTLN(F("[SIM7070G] Set APN success"));
      // After set APN, activate CNACT again
      if (_modem->checkSendCommandSync("AT+CNACT=1,1", "OK", 10000)) {
        DEBUG_PRINTLN(F("[SIM7070G] CNACT=1,1 after set APN success"));
        updateState(MODEM_WAIT_FOR_IP_AFTER_APN);
        _retryCount = 0;
        return 10000; // Wait 10 seconds
      } else {
        DEBUG_PRINTLN(F("[SIM7070G] CNACT=1,1 after set APN failed"));
        _retryCount++;
        if (_retryCount >= 3) {
          updateState(MODEM_ERROR);
          return 10000;
        }
        return 5000;
      }
    } else {
      DEBUG_PRINTLN(F("[SIM7070G] Set APN failed"));
      _retryCount++;
      if (_retryCount >= 3) {
        updateState(MODEM_ERROR);
        return 10000;
      }
      return 5000;
    }
  }

  case MODEM_WAIT_FOR_IP_AFTER_APN:
  {
    // Process async operations
    if (_modem) {
      _modem->loop();
    }
    DEBUG_PRINTLN(F("[SIM7070G] 10s wait after set APN done, checking IP..."));
    updateState(MODEM_CHECK_IP_AFTER_APN);
    return DURATION_IMMEDIATELY;
  }

  case MODEM_CHECK_IP_AFTER_APN:
  {
    // Process async operations
    if (_modem) {
      _modem->loop();
    }
    char ip[16];
    if (_modem->getIPAddress(ip, sizeof(ip))) {
      DEBUG_PRINT(F("[SIM7070G] IP check after set APN success: "));
      DEBUG_PRINTLN(ip);
      // Success, proceed to MQTT
      updateState(MODEM_MQTT_CONNECTING_HANDSHAKE);
      _retryCount = 0;
      return DURATION_IMMEDIATELY;
    } else {
      DEBUG_PRINTLN(F("[SIM7070G] IP check after set APN failed, retrying..."));
      _retryCount++;
      if (_retryCount >= 5) {
        updateState(MODEM_ERROR);
        return 10000;
      }
      return 10000; // Retry after 10 seconds
    }
  }
  case MODEM_SYNC_NTP_TIME:
  {
    // Process async operations
    if (_modem) {
      _modem->loop();
    }
    
    DEBUG_PRINTLN(F("[SIM7070G] Synchronizing NTP time..."));
    
    // Try to sync NTP time with multiple servers
    bool ntpSuccess = _modem->syncTimeUTC_any(28, 12000);
    
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
      updateState(MODEM_MQTT_CONNECTED_FIRST);
      _retryCount = 0;
      return 5000;
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
        updateState(MODEM_MQTT_CONNECTED_FIRST);
        _retryCount = 0;
        return 5000;
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
    if(_modem->isMQTTConnected())
    {
      DEBUG_PRINTLN(F("[SIM7070G] MQTT already connected"));
      _retryCount = 0; // Reset retry count when entering MODEM_MQTT_CONNECTED_FIRST
      updateState(MODEM_SYNC_NTP_TIME);
      return 10000;
    }
    
    if (_modem->checkSendCommandSync("AT+SMCONN", "OK", 30000))
    {
      _retryCount = 0; // MODEM_SYNC_NTP_TIME
      updateState(MODEM_SYNC_NTP_TIME);
      return 5000;
    }
    else
    {
      _modem->checkSendCommandSync("AT+SMDISC", "OK", 60000);      
      DEBUG_PRINTLN(F("[SIM7070G] MQTT connect failed, disconnecting and retrying..."));
      return 30000;
    }
  }
  case MODEM_MQTT_CONNECTED_FIRST:
  {
    // Process async operations
    if (_modem) {
      _modem->loop();
      
      // Subscribe to topic
      if(_modem->mqttSubscribe(_subTopic, 1))
      {
        DEBUG_PRINTLN(F("[SIM7070G] Subscribed to MQTT topic successfully"));
        // Success, reset retry count
        _retryCount = 0;
        
        // Publish online status
        char msg[160];
        snprintf(msg, sizeof(msg), "{\"id\":\"%s\",\"status\":\"online\"}", _nodeId);
        _modem->mqttPublish(_pubTopic, msg, 0, false);
        
        _lastHeartbeatMs = millis();
        updateState(MODEM_MQTT_CONNECTED);
        return 200;
      }
      else
      {
        _retryCount++;
        DEBUG_PRINT(F("[SIM7070G] Failed to subscribe to MQTT topic (attempt "));
        DEBUG_PRINT(_retryCount);
        DEBUG_PRINTLN(F(")"));
        
        if (_retryCount >= 5) {
          DEBUG_PRINTLN(F("[SIM7070G] Subscribe failed after 5 attempts, closing and reinitializing MQTT..."));
          // Close MQTT connection
          if (_modem->isMQTTConnected()) {
            _modem->mqttDisconnect();
            DEBUG_PRINTLN(F("[SIM7070G] MQTT connection closed"));
          }
          // Reset MQTT to reinitialize
          _mqttBegun = false;
          _retryCount = 0;
          // Reinitialize MQTT
          updateState(MODEM_MQTT_CONNECTING_HANDSHAKE);
          return DURATION_IMMEDIATELY;
        } else {
          // Retry subscribe after 10 seconds
          return 10000;
        }
      }
    }
    
    // If _modem null, switch to error
    updateState(MODEM_ERROR);
    return 10000;
  }

  case MODEM_WAITING_SUCCESS:
  {
    if (_modem) {
      _modem->loop();
    }
    if (_waitingDone) {
      _waitingDone = false;
      ModemState next = _waitingSuccessNextState;
      _waitingSuccessNextState = MODEM_OFF;
      DEBUG_PRINTLN(F("[SIM7070G] Waiting success done, transitioning to next state"));
      updateState(next);
      return DURATION_IMMEDIATELY;
    }
    return 200;  // Poll lại sau 200 ms
  }

  case MODEM_MQTT_CONNECTED:
    // CRITICAL: Must call loop() to process async operations
    handleIncomingData();

    // if (!_modem || !_modem->isMQTTConnected())
    // {
    //   _stats.mqttDisconnects++;
    //   // Recheck IP and reconnect
    //   updateState(MODEM_CHECK_IP_FIRST);
    //   return 2000;
    // }

    // if (!_modem->isNetworkConnected())
    // {
    //   // Network lost, retry from start
    //   updateState(MODEM_ACTIVATE_CNACT_FIRST);
    //   return 5000;
    // }

    // if (_lastHeartbeatMs == 0 || (millis() - _lastHeartbeatMs) >= 30000UL)
    // {
    //   char hb[200];
    //   int8_t rssi;
    //   if (_modem && _modem->getSignalQuality(&rssi, nullptr)) {
    //     snprintf(hb, sizeof(hb), "{\"id\":\"%s\",\"csq\":%d,\"uptime\":%lu}",
    //              _nodeId, (int)rssi, (unsigned long)(millis() / 1000UL));
    //     _modem->mqttPublish(_pubTopic, hb, 0, false);
    //     _lastHeartbeatMs = millis();
    //   }
    // }

    // Auto-flush buffered MQTT messages (e.g. telemetry)
    if (_bufferCount > 0) {
      flushMessageBuffer();
    }

    return 20000;

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
    case MODEM_POWER_ON: return F("MODEM_POWER_ON");
    case MODEM_ACTIVATE_CNACT_FIRST: return F("MODEM_ACTIVATE_CNACT_FIRST");
    case MODEM_WAIT_FOR_IP_FIRST: return F("MODEM_WAIT_FOR_IP_FIRST");
    case MODEM_CHECK_IP_FIRST: return F("MODEM_CHECK_IP_FIRST");
    case MODEM_APPLY_BOOT_CONFIG: return F("MODEM_APPLY_BOOT_CONFIG");
    case MODEM_CHECK_SIM_AFTER_CONFIG: return F("MODEM_CHECK_SIM_AFTER_CONFIG");
    case MODEM_CHECK_SIGNAL_AFTER_CONFIG: return F("MODEM_CHECK_SIGNAL_AFTER_CONFIG");
    case MODEM_SET_APN: return F("MODEM_SET_APN");
    case MODEM_WAIT_FOR_IP_AFTER_APN: return F("MODEM_WAIT_FOR_IP_AFTER_APN");
    case MODEM_CHECK_IP_AFTER_APN: return F("MODEM_CHECK_IP_AFTER_APN");
    case MODEM_SYNC_NTP_TIME: return F("MODEM_SYNC_NTP_TIME");
    case MODEM_MQTT_CONNECTING_HANDSHAKE: return F("MODEM_MQTT_CONNECTING_HANDSHAKE");
    case MODEM_MQTT_CONNECTING: return F("MODEM_MQTT_CONNECTING");
    case MODEM_MQTT_CONNECTED_FIRST: return F("MODEM_MQTT_CONNECTED_FIRST");
    case MODEM_MQTT_CONNECTED: return F("MODEM_MQTT_CONNECTED");
    case MODEM_WAITING_SUCCESS: return F("MODEM_WAITING_SUCCESS");
    case MODEM_ERROR: return F("MODEM_ERROR");
    default: return F("MODEM_UNKNOWN");
  }
}

void Sim7070GDevice::gotoWaitingSuccess(ModemState nextState)
{
  _waitingDone = false;
  _waitingSuccessNextState = nextState;
  updateState(MODEM_WAITING_SUCCESS);
}

void Sim7070GDevice::setWaitingDone()
{
  _waitingDone = true;
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

bool Sim7070GDevice::isMqttConnected() const {
  return _state == MODEM_MQTT_CONNECTED && _modem && _modem->isMQTTConnected();
}

bool Sim7070GDevice::publishTelemetryNow(const char* payload) {
  if (!isMqttConnected() || !_modem || !payload) return false;
  return _modem->mqttPublish(_pubTopic, payload, 1, false);
}

bool Sim7070GDevice::enqueueTelemetry(const char* payload) {
  if (!payload) return false;
  return enqueueMessage(_pubTopic, payload, 1);
}

int Sim7070GDevice::getCSQ() {
  int8_t rssi = -1;
  if (!_modem || !_modem->getSignalQuality(&rssi, nullptr)) return -1;
  if (rssi < 0 || rssi > 31) return 0;
  return (int)rssi;
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
