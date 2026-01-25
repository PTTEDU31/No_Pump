#include "Sim7070G.h"
#include "DEBUG.h"

Sim7070G *Sim7070G::_instance = nullptr;

Sim7070G::Sim7070G(HardwareSerial *serial, uint8_t powerPin, uint32_t baudRate)
    : _serial(serial),
      _powerPin(powerPin),
      _baudRate(baudRate),
      _at(serial, 512),
      _state(Sim7070GState::IDLE),
      _pdpContextId(0),
      _mqttPort(1883),
      _mqttKeepalive(60),
      _mqttCleanSession(true),
      _mqttSSLIndex(0),
      _httpState(0),
      _httpSSLIndex(0),
      _networkStateCallback(nullptr),
      _mqttMessageCallback(nullptr),
      _mqttStateCallback(nullptr),
      _atCommandCount(0),
      _atErrorCount(0)
{
  _instance = this;

  memset(&_networkInfo, 0, sizeof(_networkInfo));
  _networkInfo.rssi = 99;
  _networkInfo.ber = 99;
  _networkInfo.regStatus = NetworkRegStatus::UNKNOWN;

  _apn[0] = '\0';
  _apnUsername[0] = '\0';
  _apnPassword[0] = '\0';

  _mqttClientId[0] = '\0';
  _mqttServer[0] = '\0';
  _mqttUsername[0] = '\0';
  _mqttPassword[0] = '\0';

  _httpURL[0] = '\0';

  _mqttState = MQTTState::DISCONNECTED;
}

Sim7070G::~Sim7070G()
{
  if (_instance == this)
  {
    _instance = nullptr;
  }
}

bool Sim7070G::begin()
{
  pinMode(_powerPin, OUTPUT);
  powerOn();

  if (!_at.begin(_baudRate))
  {
    return false;
  }

  // Register URC handlers
  _at.registerURCHandler("+SMSTATE", onURC_SMSTATE);
  _at.registerURCHandler("+SMPUB", onURC_SMPUB);
  _at.registerURCHandler("+SMSUB", onURC_SMSUB);
  _at.registerURCHandler("+CGREG", onURC_CGREG);
  _at.registerURCHandler("+CGNAPN", onURC_CGNAPN);
  _at.registerURCHandler("+CNACT", onURC_CNACT);
  _at.registerURCHandler("+APP PDP", onURC_APP_PDP);

  updateState(Sim7070GState::INITIALIZING);

  return true;
}

void Sim7070G::loop()
{
  _at.loop();
  processURCs();
}

void Sim7070G::powerOn()
{
  digitalWrite(_powerPin, LOW);
  delay(50);
  digitalWrite(_powerPin, HIGH);
}

void Sim7070G::powerOff()
{
  digitalWrite(_powerPin, LOW);
}

bool Sim7070G::isPoweredOn()
{
  return _at.isAlive();
}

bool Sim7070G::isAlive(unsigned long timeout)
{
  char response[64];
  return _at.sendCommandSync("AT", timeout, response, sizeof(response));
}

bool Sim7070G::checkSIM()
{
  // Check if SIM is ready - response should contain "READY"
  return checkSendCommandSync("AT+CPIN?", "READY", 1000);
}

bool Sim7070G::setPreferredRAT(uint8_t rat)
{
  // AT+CMNB: 1=CAT-M, 2=NB-IoT, 3=CAT-M and NB-IoT
  char cmd[32];
  snprintf(cmd, sizeof(cmd), "AT+CMNB=%d", rat);
  return checkSendCommandSync(cmd, "OK", 5000);
}

bool Sim7070G::setNetworkMode(uint8_t mode)
{
  // AT+CNMP: 2=Automatic, 13=GSM only, 38=LTE only, 51=GSM and LTE only
  char cmd[32];
  snprintf(cmd, sizeof(cmd), "AT+CNMP=%d", mode);
  return checkSendCommandSync(cmd, "OK", 5000);
}

bool Sim7070G::getSignalQuality(int8_t *rssi, uint8_t *ber)
{
  char response[64];
  if (!_at.sendCommandSync("AT+CSQ", 5000, response, sizeof(response)))
  {
    return false;
  }

  // Parse +CSQ: <rssi>,<ber>
  int r, b;
  if (sscanf(response, "+CSQ: %d,%d", &r, &b) == 2)
  {
    if (rssi)
      *rssi = (int8_t)r;
    if (ber)
      *ber = (uint8_t)b;
    _networkInfo.rssi = (int8_t)r;
    _networkInfo.ber = (uint8_t)b;
    return true;
  }

  return false;
}

bool Sim7070G::getNetworkRegistration(NetworkRegStatus *status)
{
  char response[64];
  if (!_at.sendCommandSync("AT+CGREG?", 5000, response, sizeof(response)))
  {
    return false;
  }

  // Parse +CGREG: <n>,<stat>
  int n, stat;
  if (sscanf(response, "+CGREG: %d,%d", &n, &stat) == 2)
  {
    NetworkRegStatus regStatus;
    switch (stat)
    {
    case 0:
      regStatus = NetworkRegStatus::NOT_REGISTERED;
      break;
    case 1:
      regStatus = NetworkRegStatus::REGISTERED_HOME;
      break;
    case 2:
      regStatus = NetworkRegStatus::SEARCHING;
      break;
    case 3:
      regStatus = NetworkRegStatus::REGISTRATION_DENIED;
      break;
    case 5:
      regStatus = NetworkRegStatus::REGISTERED_ROAMING;
      break;
    default:
      regStatus = NetworkRegStatus::UNKNOWN;
      break;
    }

    if (status)
      *status = regStatus;
    _networkInfo.regStatus = regStatus;
    return true;
  }

  return false;
}

bool Sim7070G::getAPN(char *apn, size_t len)
{
  char response[128];
  if (!_at.sendCommandSync("AT+CGNAPN", 5000, response, sizeof(response)))
  {
    return false;
  }

  // Parse +CGNAPN: <index>,<APN>
  int index;
  char apnName[64];
  if (sscanf(response, "+CGNAPN: %d,\"%63[^\"]\"", &index, apnName) == 2)
  {
    if (apn && len > 0)
    {
      strncpy(apn, apnName, len - 1);
      apn[len - 1] = '\0';
    }
    strncpy(_networkInfo.apn, apnName, sizeof(_networkInfo.apn) - 1);
    return true;
  }

  return false;
}

bool Sim7070G::setAPN(uint8_t cid, const char *apn, const char *username, const char *password)
{
  if (!apn || strlen(apn) == 0)
  {
    DEBUG_PRINTLN(F("[APN] No APN provided"));
    return false;
  }

  char cmd[256];
  char response[256];

  // Step 1: Disable RF - AT+CFUN=0
  DEBUG_PRINTLN(F("[APN] Step 1: Disabling RF..."));
  if (!_at.sendCommandSync("AT+CFUN=0", 5000))
  {
    DEBUG_PRINTLN(F("[APN] Failed to disable RF"));
    return false;
  }
  DEBUG_PRINTLN(F("[APN] RF disabled"));
  delay(500);

  // Step 2: Set APN manually - AT+CGDCONT=<cid>,"IP","<apn>"
  // Some operators need to set APN first when registering the network
  DEBUG_PRINT(F("[APN] Step 2: Setting APN manually (CID="));
  DEBUG_PRINT(cid);
  DEBUG_PRINT(F(", APN="));
  DEBUG_PRINT(apn);
  DEBUG_PRINTLN(F(")..."));
  snprintf(cmd, sizeof(cmd), "AT+CGDCONT=%d,\"IP\",\"%s\"", cid, apn);
  if (!_at.sendCommandSync(cmd, 5000))
  {
    DEBUG_PRINTLN(F("[APN] Failed to set APN manually"));
    // Try to enable RF anyway
    _at.sendCommandSync("AT+CFUN=1", 5000);
    return false;
  }
  DEBUG_PRINTLN(F("[APN] APN set manually"));
  delay(500);

  // Step 3: Enable RF - AT+CFUN=1
  DEBUG_PRINTLN(F("[APN] Step 3: Enabling RF..."));
  if (!_at.sendCommandSync("AT+CFUN=1", 10000))
  {
    DEBUG_PRINTLN(F("[APN] Failed to enable RF"));
    return false;
  }
  DEBUG_PRINTLN(F("[APN] RF enabled"));
  delay(2000); // Wait for SIM to be ready after RF enable

  // Step 4: Check PS service - AT+CGATT?
  DEBUG_PRINTLN(F("[APN] Step 4: Checking PS service attachment..."));
  unsigned long startTime = millis();
  bool psAttached = false;
  while ((millis() - startTime) < 30000)
  { // Wait up to 30 seconds
    if (!_at.sendCommandSync("AT+CGATT?", 5000, response, sizeof(response)))
    {
      delay(1000);
      continue;
    }
    // Parse +CGATT: <state> (1 indicates PS has attached)
    int cgattState = 0;
    if (sscanf(response, "+CGATT: %d", &cgattState) == 1)
    {
      if (cgattState == 1)
      {
        psAttached = true;
        DEBUG_PRINTLN(F("[APN] PS service is attached"));
        break;
      }
    }
    delay(2000);
  }

  if (!psAttached)
  {
    DEBUG_PRINTLN(F("[APN] PS service not attached after timeout"));
    return false;
  }

  // Step 5: Query APN from network - AT+CGNAPN
  DEBUG_PRINTLN(F("[APN] Step 5: Querying APN from network..."));
  char networkAPN[64] = {0};
  if (getAPN(networkAPN, sizeof(networkAPN)))
  {
    DEBUG_PRINT(F("[APN] Network APN: "));
    DEBUG_PRINTLN(networkAPN);
  }

  // Step 6: Set APN configuration - AT+CNCFG=<cid>,<type>,<apn>
  // Use provided APN (network APN may be empty under GSM network)
  DEBUG_PRINT(F("[APN] Step 6: Setting APN configuration (CID="));
  DEBUG_PRINT(cid);
  DEBUG_PRINT(F(", APN="));
  DEBUG_PRINT(apn);
  DEBUG_PRINTLN(F(")..."));

  // AT+CNCFG=<cid>,<type>,<apn>[,<username>,<password>]
  // type: 1 = IPv4, 2 = IPv6, 3 = IPv4v6
  if (username && password && strlen(username) > 0 && strlen(password) > 0)
  {
    snprintf(cmd, sizeof(cmd), "AT+CNCFG=%d,%d,\"%s\",\"%s\",\"%s\"", cid, 1, apn, username, password);
  }
  else
  {
    snprintf(cmd, sizeof(cmd), "AT+CNCFG=%d,%d,\"%s\"", cid, 1, apn);
  }

  if (!_at.sendCommandSync(cmd, 5000))
  {
    DEBUG_PRINTLN(F("[APN] Failed to set APN configuration"));
    return false;
  }
  DEBUG_PRINTLN(F("[APN] APN configuration set successfully"));

  // Save APN information
  strncpy(_apn, apn, sizeof(_apn) - 1);
  _apn[sizeof(_apn) - 1] = '\0';
  if (username)
  {
    strncpy(_apnUsername, username, sizeof(_apnUsername) - 1);
    _apnUsername[sizeof(_apnUsername) - 1] = '\0';
  }
  else
  {
    _apnUsername[0] = '\0';
  }
  if (password)
  {
    strncpy(_apnPassword, password, sizeof(_apnPassword) - 1);
    _apnPassword[sizeof(_apnPassword) - 1] = '\0';
  }
  else
  {
    _apnPassword[0] = '\0';
  }

  DEBUG_PRINTLN(F("[APN] APN set successfully"));
  return true;
}

bool Sim7070G::getSystemInfo(char *info, size_t len)
{
  char response[256];
  if (!_at.sendCommandSync("AT+CPSI?", 5000, response, sizeof(response)))
  {
    return false;
  }

  // Parse +CPSI: <system>,<operator>,<long_eons>,<short_eons>,<rat>,<mcc>,<mnc>,<lac>,<cell_id>,<ch>,<rssi>,<sinr>
  if (info && len > 0)
  {
    strncpy(info, response, len - 1);
    info[len - 1] = '\0';
  }

  return true;
}

Sim7070GState Sim7070G::getState()
{
  return _state;
}
// Part 1: Steps 1-3 - Initial preparation (SIM, RF signal, PS service)
bool Sim7070G::attacthService()
{
  char response[256];

  // Step 1: Check SIM card status - AT+CPIN?
  DEBUG_PRINTLN(F("[PDP] Step 1: Checking SIM card status..."));
  if (!checkSIM())
  {
    DEBUG_PRINTLN(F("[PDP] SIM card not ready"));
    return false;
  }
  DEBUG_PRINTLN(F("[PDP] SIM card is READY"));

  // Step 2: Check RF signal - AT+CSQ
  DEBUG_PRINTLN(F("[PDP] Step 2: Checking RF signal..."));
  int8_t rssi;
  if (!getSignalQuality(&rssi, nullptr))
  {
    DEBUG_PRINTLN(F("[PDP] Failed to get signal quality"));
    return false;
  }
  DEBUG_PRINT(F("[PDP] Signal quality: "));
  DEBUG_PRINTLN(rssi);

  // Step 3: Check PS service - AT+CGATT?
  // DEBUG_PRINTLN(F("[PDP] Step 3: Checking PS service attachment..."));
  // if (!_at.sendCommandSync("AT+CGATT?", 5000, response, sizeof(response))) {
  //   DEBUG_PRINTLN(F("[PDP] Failed to check PS service"));
  //   return false;
  // }
  // // Parse +CGATT: <state> (1 indicates PS has attached)
  // int cgattState = 0;
  // if (sscanf(response, "+CGATT: %d", &cgattState) == 1) {
  //   if (cgattState != 1) {
  //     DEBUG_PRINT(F("[PDP] PS service not attached (state="));
  //     DEBUG_PRINT(cgattState);
  //     DEBUG_PRINTLN(F(")"));
  //     return false;
  //   }
  //   DEBUG_PRINTLN(F("[PDP] PS service is attached"));
  // } else {
  //   DEBUG_PRINTLN(F("[PDP] Failed to parse CGATT response"));
  //   return false;
  // }
  // step3: activate application network
  sendCommand("AT+CGDCONT?", 1000);
  if (!checkSendCommandSync("AT+CNACT=1,1", "OK", 5000))
  {
    DEBUG_PRINTLN(F("[PDP] Failed to attach service"));
    return false;
  }
  DEBUG_PRINTLN(F("[PDP] Service attached successfully"));
  return true;
}

// Part 2: Step 4 - Query network information
bool Sim7070G::queryNetworkInfo()
{
  char response[256];

  // Step 4: Query Network information - AT+COPS?
  // This command can take longer when network is not fully ready
  DEBUG_PRINTLN(F("[PDP] Step 4: Querying network information..."));
  if (!_at.sendCommandSync("AT+COPS?", 30000, response, sizeof(response)))
  {
    DEBUG_PRINTLN(F("[PDP] Failed to query network"));
    return false;
  }
  // Parse +COPS: <mode>,<format>,<oper>,<AcT>
  // Mode 9 means NB-IoT network
  int mode, format, act;
  char oper[64] = {0};
  if (sscanf(response, "+COPS: %d,%d,\"%63[^\"]\",%d", &mode, &format, oper, &act) >= 3)
  {
    DEBUG_PRINT(F("[PDP] Network: "));
    DEBUG_PRINT(oper);
    DEBUG_PRINT(F(", Mode: "));
    DEBUG_PRINT(mode);
    DEBUG_PRINT(F(", Act: "));
    DEBUG_PRINTLN(act);
  }

  return true;
}

// Part 3: Steps 5-8 - Complete PDP context activation (APN, activate, get IP)
bool Sim7070G::PDPContextActivation(uint8_t cid, const char *apn,
                                    const char *username, const char *password)
{
  char cmd[256];

  // Step 5: Query APN - AT+CGNAPN
  DEBUG_PRINTLN(F("[PDP] Step 5: Querying APN from network..."));
  char networkAPN[64] = {0};
  if (getAPN(networkAPN, sizeof(networkAPN)))
  {
    DEBUG_PRINT(F("[PDP] Network APN: "));
    DEBUG_PRINTLN(networkAPN);
  }

  // Step 6: Set APN configuration - AT+CNCFG=<cid>,<type>,<apn>
  // Use provided APN or network APN
  const char *apnToUse = apn;
  if (!apnToUse || strlen(apnToUse) == 0)
  {
    if (strlen(networkAPN) > 0)
    {
      apnToUse = networkAPN;
    }
    else
    {
      DEBUG_PRINTLN(F("[PDP] No APN provided and network APN is empty"));
      return false;
    }
  }

  DEBUG_PRINT(F("[PDP] Step 6: Setting APN configuration: "));
  DEBUG_PRINTLN(apnToUse);

  // AT+CNCFG=<cid>,<type>,<apn>
  // type: 1 = IPv4, 2 = IPv6, 3 = IPv4v6
  snprintf(cmd, sizeof(cmd), "AT+CNCFG=%d,%d,\"%s\"", cid, 1, apnToUse);
  if (!_at.sendCommandSync(cmd, 5000))
  {
    DEBUG_PRINTLN(F("[PDP] Failed to set APN configuration"));
    return false;
  }
  DEBUG_PRINTLN(F("[PDP] APN configuration set successfully"));

  // Step 7: Activate PDP context - AT+CNACT=<cid>,1
  DEBUG_PRINT(F("[PDP] Step 7: Activating PDP context (CID="));
  DEBUG_PRINT(cid);
  DEBUG_PRINTLN(F(")..."));
  snprintf(cmd, sizeof(cmd), "AT+CNACT=%d,%d", cid, 1);
  if (!_at.sendCommandSync(cmd, 5000))
  {
    DEBUG_PRINTLN(F("[PDP] Failed to activate PDP context"));
    return false;
  }
  DEBUG_PRINTLN(F("[PDP] PDP context activation command sent"));

  // Wait a bit for +APP PDP: <cid>,ACTIVE URC
  delay(10000);

  // Step 8: Get local IP - AT+CNACT?
  DEBUG_PRINTLN(F("[PDP] Step 8: Getting local IP address..."));
  char ip[16];
  if (!getIPAddress(ip, sizeof(ip)))
  {
    DEBUG_PRINTLN(F("[PDP] Failed to get IP address"));
    return false;
  }
  DEBUG_PRINT(F("[PDP] Local IP: "));
  DEBUG_PRINTLN(ip);

  // // Save context information
  // _pdpContextId = cid;
  // if (apnToUse) {
  //   strncpy(_apn, apnToUse, sizeof(_apn) - 1);
  //   _apn[sizeof(_apn) - 1] = '\0';
  // }
  if (username)
  {
    strncpy(_apnUsername, username, sizeof(_apnUsername) - 1);
    _apnUsername[sizeof(_apnUsername) - 1] = '\0';
  }
  if (password)
  {
    strncpy(_apnPassword, password, sizeof(_apnPassword) - 1);
    _apnPassword[sizeof(_apnPassword) - 1] = '\0';
  }

  DEBUG_PRINTLN(F("[PDP] PDP context activated successfully"));
  return true;
}

bool Sim7070G::activatePDPContext(uint8_t cid, const char *apn,
                                  const char *username, const char *password)
{
  // Part 1: Steps 1-3 - Initial preparation
  if (!attacthService())
  {
    return false;
  }

  // Part 2: Step 4 - Query network information
  if (!queryNetworkInfo())
  {
    return false;
  }

  // Part 3: Steps 5-8 - Complete activation
  if (!PDPContextActivation(cid, apn, username, password))
  {
    return false;
  }

  return true;
}

bool Sim7070G::deactivatePDPContext(uint8_t cid)
{
  char cmd[32];
  snprintf(cmd, sizeof(cmd), "AT+CNACT=%d,%d", cid, 0);
  return _at.sendCommandSync(cmd, 10000);
}

bool Sim7070G::getIPAddress(char *ip, size_t len)
{
  char response[128];
  if (!_at.sendCommandSync("AT+CNACT?", 15000, response, sizeof(response)))
  {
    return false;
  }

  // Parse +CNACT: <cid>,<state>,<ip_address>
  // Response may contain multiple lines, need to find the one with state=1 (ACTIVE)
  // const char *line = response;
  // while (line && *line)
  // {
  //   // Find next +CNACT: line
  //   const char *cnactLine = strstr(line, "+CNACT:");
  //   if (!cnactLine)
  //   {
  //     break; // No more +CNACT: lines
  //   }

  //   // Parse this line: +CNACT: <cid>,<state>,"<ip_address>"
  //   int cid, state;
  //   char ipAddr[16];
  //   if (sscanf(cnactLine, "+CNACT: %d,%d,\"%15[^\"]\"", &cid, &state, ipAddr) == 3)
  //   {
  //     if (state == 1)
  //     { // Found ACTIVE context
  //       if (ip && len > 0)
  //       {
  //         strncpy(ip, ipAddr, len - 1);
  //         ip[len - 1] = '\0';
  //         strncpy(_networkInfo.ipAddress, ipAddr, sizeof(_networkInfo.ipAddress) - 1);
  //         _networkInfo.ipAddress[sizeof(_networkInfo.ipAddress) - 1] = '\0';
  //         return true;
  //       }
  //     }
  //   }

  //   // Move to next line (after newline or end of string)
  //   line = strchr(cnactLine, '\n');
  //   if (line)
  //   {
  //     line++; // Skip the newline
  //   }
  //   else
  //   {
  //     break;
  //   }
  // }

  return true;
}

NetworkInfo Sim7070G::getNetworkInfo()
{
  return _networkInfo;
}

bool Sim7070G::isNetworkConnected()
{
  char ip[16];
  return getIPAddress(ip, sizeof(ip));
}

// MQTT Client Implementation
bool Sim7070G::mqttBegin()
{
  // MQTT initialization is done via mqttSetConfig
  return true;
}

bool Sim7070G::mqttSetConfig(const char *clientId, const char *server, uint16_t port,
                             const char *username, const char *password,
                             uint16_t keepalive, bool cleanSession)
{
  if (!clientId || !server)
  {
    return false;
  }

  strncpy(_mqttClientId, clientId, sizeof(_mqttClientId) - 1);
  strncpy(_mqttServer, server, sizeof(_mqttServer) - 1);
  _mqttPort = port;
  _mqttKeepalive = keepalive;
  _mqttCleanSession = cleanSession;

  if (username)
  {
    strncpy(_mqttUsername, username, sizeof(_mqttUsername) - 1);
  }
  else
  {
    _mqttUsername[0] = '\0';
  }

  if (password)
  {
    strncpy(_mqttPassword, password, sizeof(_mqttPassword) - 1);
  }
  else
  {
    _mqttPassword[0] = '\0';
  }

  // Set MQTT configuration: AT+SMCONF=<t>,<v>
  // t: type (CLIENTID, URL, KEEPTIME, CLEANSS, QOS, RETAIN, etc.)
  char cmd[256];

  // Set server URL
  snprintf(cmd, sizeof(cmd), "AT+SMCONF=\"URL\",%s,%d", _mqttServer, _mqttPort);
  if (!checkSendCommandSync(cmd, "OK", 20000))
    return false;
  delay(100);

  // Set keepalive
  snprintf(cmd, sizeof(cmd), "AT+SMCONF=\"KEEPTIME\",%d", _mqttKeepalive);
  if (!checkSendCommandSync(cmd, "OK", 5000))
    return false;
  // Set client ID
  snprintf(cmd, sizeof(cmd), "AT+SMCONF=\"CLIENTID\",%s", _mqttClientId);
  if (!checkSendCommandSync(cmd, "OK", 5000))
    return false;
  // Set clean session
  snprintf(cmd, sizeof(cmd), "AT+SMCONF=\"CLEANSS\",%d", _mqttCleanSession ? 1 : 0);
  if (!checkSendCommandSync(cmd, "OK", 5000))
    return false;
  // Set username if provided
  if(_mqttUsername[0] != '\0')
  {
    snprintf(cmd, sizeof(cmd), "AT+SMCONF=\"USERNAME\",%s", _mqttUsername);
    if (!checkSendCommandSync(cmd, "OK", 5000))
      return false;
  }
  // Set password if provided
  if(_mqttPassword[0] != '\0')
  {
  snprintf(cmd, sizeof(cmd), "AT+SMCONF=\"PASSWORD\",%s", _mqttPassword);
    if (!checkSendCommandSync(cmd, "OK", 5000))
      return false;
  }
  return true;
}

bool Sim7070G::mqttSetSSL(uint8_t sslIndex)
{
  _mqttSSLIndex = sslIndex;
  char cmd[32];
  snprintf(cmd, sizeof(cmd), "AT+SMSSL=%d", sslIndex);
  return _at.sendCommandSync(cmd, 5000);
}

bool Sim7070G::mqttConnect()
{
  updateMQTTState(MQTTState::CONNECTING);

  if (!_at.sendCommandSync("AT+SMCONN", 30000))
  {
    updateMQTTState(MQTTState::ERROR);
    return false;
  }

  // Check connection state - state 1 means connected
  if (checkSendCommandSync("AT+SMSTATE?", "+SMSTATE: 1", 5000))
  {
    updateMQTTState(MQTTState::CONNECTED);
    return true;
  }

  updateMQTTState(MQTTState::DISCONNECTED);
  return false;
}

bool Sim7070G::mqttDisconnect()
{
  updateMQTTState(MQTTState::DISCONNECTING);

  if (!_at.sendCommandSync("AT+SMDISC", 10000))
  {
    return false;
  }

  updateMQTTState(MQTTState::DISCONNECTED);
  return true;
}

bool Sim7070G::mqttPublish(const char *topic, const char *payload, uint8_t qos, bool retain)
{
  if (!topic || !payload)
  {
    return false;
  }

  // AT+SMPUB=<topic>,<qos>,<retain>,<length>
  // Then send payload
  char cmd[256];
  size_t payloadLen = strlen(payload);
  snprintf(cmd, sizeof(cmd), "AT+SMPUB=\"%s\",%d,%d,%d", topic, qos, retain ? 1 : 0, payloadLen);

  if (!_at.sendCommandSync(cmd, 5000))
  {
    return false;
  }

  // Send payload
  _serial->print(payload);
  _serial->print("\r\n");
  _serial->flush();

  // Wait for OK
  delay(100);
  return _at.sendCommandSync("AT", 5000);
}

bool Sim7070G::mqttPublishHex(const char *topic, const uint8_t *payload, size_t len, uint8_t qos, bool retain)
{
  if (!topic || !payload || len == 0)
  {
    return false;
  }

  // Set hex mode
  if (!_at.sendCommandSync("AT+SMPUBHEX=1", 5000))
  {
    return false;
  }

  // Publish
  char cmd[256];
  snprintf(cmd, sizeof(cmd), "AT+SMPUB=\"%s\",%d,%d,%d", topic, qos, retain ? 1 : 0, len);

  if (!_at.sendCommandSync(cmd, 5000))
  {
    return false;
  }

  // Send hex payload
  for (size_t i = 0; i < len; i++)
  {
    if (i > 0 && (i % 32 == 0))
    {
      _serial->print("\r\n");
    }
    char hex[3];
    snprintf(hex, sizeof(hex), "%02X", payload[i]);
    _serial->print(hex);
  }
  _serial->print("\r\n");
  _serial->flush();

  delay(100);
  bool result = _at.sendCommandSync("AT", 5000);

  // Reset to text mode
  _at.sendCommandSync("AT+SMPUBHEX=0", 5000);

  return result;
}

bool Sim7070G::mqttSubscribe(const char *topic, uint8_t qos)
{
  if (!topic)
  {
    return false;
  }

  char cmd[256];
  snprintf(cmd, sizeof(cmd), "AT+SMSUB=\"%s\",%d", topic, qos);
  return _at.sendCommandSync(cmd, 10000);
}

bool Sim7070G::mqttUnsubscribe(const char *topic)
{
  if (!topic)
  {
    return false;
  }

  char cmd[256];
  snprintf(cmd, sizeof(cmd), "AT+SMUNSUB=\"%s\"", topic);
  return _at.sendCommandSync(cmd, 10000);
}

bool Sim7070G::mqttGetState(MQTTState *state)
{
  if (state)
  {
    *state = _mqttState;
  }
  return true;
}

bool Sim7070G::isMQTTConnected()
{
  char response[64];
  if (!_at.sendCommandSync("AT+SMSTATE?", 5000, response, sizeof(response)))
  {
    return false;
  }

  int state;
  if (sscanf(response, "+SMSTATE: %d", &state) == 1)
  {
    return (state == 1);
  }

  return false;
}

// HTTP Client Implementation
bool Sim7070G::httpBegin()
{
  return true;
}

bool Sim7070G::httpSetSSL(uint8_t sslIndex)
{
  _httpSSLIndex = sslIndex;
  char cmd[32];
  snprintf(cmd, sizeof(cmd), "AT+SHSSL=%d", sslIndex);
  return _at.sendCommandSync(cmd, 5000);
}

bool Sim7070G::httpSetURL(const char *url)
{
  if (!url)
  {
    return false;
  }

  strncpy(_httpURL, url, sizeof(_httpURL) - 1);
  char cmd[512];
  snprintf(cmd, sizeof(cmd), "AT+SHPARA=\"URL\",\"%s\"", url);
  return _at.sendCommandSync(cmd, 5000);
}

bool Sim7070G::httpAddHeader(const char *header)
{
  if (!header)
  {
    return false;
  }

  char cmd[256];
  snprintf(cmd, sizeof(cmd), "AT+SHAHEAD=\"%s\"", header);
  return _at.sendCommandSync(cmd, 5000);
}

bool Sim7070G::httpClearHeaders()
{
  return _at.sendCommandSync("AT+SHCHEAD", 5000);
}

bool Sim7070G::httpSetBody(const char *body)
{
  if (!body)
  {
    return false;
  }

  char cmd[512];
  size_t len = strlen(body);
  snprintf(cmd, sizeof(cmd), "AT+SHBOD=%d,\"%s\"", len, body);
  return _at.sendCommandSync(cmd, 5000);
}

bool Sim7070G::httpSetBody(const uint8_t *body, size_t len)
{
  if (!body || len == 0)
  {
    return false;
  }

  // Convert to hex string
  char hexBody[1024];
  size_t hexLen = 0;
  for (size_t i = 0; i < len && hexLen < (sizeof(hexBody) - 1); i++)
  {
    hexLen += snprintf(hexBody + hexLen, sizeof(hexBody) - hexLen, "%02X", body[i]);
  }

  char cmd[512];
  snprintf(cmd, sizeof(cmd), "AT+SHBOD=%d,\"%s\"", len, hexBody);
  return _at.sendCommandSync(cmd, 5000);
}

bool Sim7070G::httpGET(HTTPResponseCallback callback, void *userData)
{
  char response[128];
  if (!_at.sendCommandSync("AT+SHREQ=\"GET\"", 30000, response, sizeof(response)))
  {
    return false;
  }

  // Parse response: +SHREQ: <status_code>
  int statusCode;
  if (sscanf(response, "+SHREQ: %d", &statusCode) == 1)
  {
    if (callback)
    {
      // Read response data
      char data[1024];
      size_t bytesRead = 0;
      if (httpReadResponse(data, sizeof(data), &bytesRead))
      {
        callback(statusCode, data, bytesRead, userData);
      }
      else
      {
        callback(statusCode, nullptr, 0, userData);
      }
    }
    return (statusCode >= 200 && statusCode < 300);
  }

  return false;
}

bool Sim7070G::httpPOST(HTTPResponseCallback callback, void *userData)
{
  char response[128];
  if (!_at.sendCommandSync("AT+SHREQ=\"POST\"", 30000, response, sizeof(response)))
  {
    return false;
  }

  int statusCode;
  if (sscanf(response, "+SHREQ: %d", &statusCode) == 1)
  {
    if (callback)
    {
      char data[1024];
      size_t bytesRead = 0;
      if (httpReadResponse(data, sizeof(data), &bytesRead))
      {
        callback(statusCode, data, bytesRead, userData);
      }
      else
      {
        callback(statusCode, nullptr, 0, userData);
      }
    }
    return (statusCode >= 200 && statusCode < 300);
  }

  return false;
}

bool Sim7070G::httpPUT(HTTPResponseCallback callback, void *userData)
{
  char response[128];
  if (!_at.sendCommandSync("AT+SHREQ=\"PUT\"", 30000, response, sizeof(response)))
  {
    return false;
  }

  int statusCode;
  if (sscanf(response, "+SHREQ: %d", &statusCode) == 1)
  {
    if (callback)
    {
      char data[1024];
      size_t bytesRead = 0;
      if (httpReadResponse(data, sizeof(data), &bytesRead))
      {
        callback(statusCode, data, bytesRead, userData);
      }
      else
      {
        callback(statusCode, nullptr, 0, userData);
      }
    }
    return (statusCode >= 200 && statusCode < 300);
  }

  return false;
}

bool Sim7070G::httpDELETE(HTTPResponseCallback callback, void *userData)
{
  char response[128];
  if (!_at.sendCommandSync("AT+SHREQ=\"DELETE\"", 30000, response, sizeof(response)))
  {
    return false;
  }

  int statusCode;
  if (sscanf(response, "+SHREQ: %d", &statusCode) == 1)
  {
    if (callback)
    {
      char data[1024];
      size_t bytesRead = 0;
      if (httpReadResponse(data, sizeof(data), &bytesRead))
      {
        callback(statusCode, data, bytesRead, userData);
      }
      else
      {
        callback(statusCode, nullptr, 0, userData);
      }
    }
    return (statusCode >= 200 && statusCode < 300);
  }

  return false;
}

bool Sim7070G::httpReadResponse(char *buffer, size_t bufferSize, size_t *bytesRead)
{
  if (!buffer || bufferSize == 0)
  {
    return false;
  }

  char response[128];
  if (!_at.sendCommandSync("AT+SHREAD", 10000, response, sizeof(response)))
  {
    return false;
  }

  // Parse +SHREAD: <length>,<data>
  int length;
  if (sscanf(response, "+SHREAD: %d,", &length) == 1)
  {
    // Extract data part (after comma)
    const char *dataStart = strchr(response, ',');
    if (dataStart)
    {
      dataStart++; // Skip comma
      size_t copyLen = (length < (bufferSize - 1)) ? length : (bufferSize - 1);
      strncpy(buffer, dataStart, copyLen);
      buffer[copyLen] = '\0';
      if (bytesRead)
      {
        *bytesRead = copyLen;
      }
      return true;
    }
  }

  return false;
}

bool Sim7070G::httpGetState(int *state)
{
  char response[64];
  if (!_at.sendCommandSync("AT+SHSTATE?", 5000, response, sizeof(response)))
  {
    return false;
  }

  int s;
  if (sscanf(response, "+SHSTATE: %d", &s) == 1)
  {
    if (state)
      *state = s;
    _httpState = s;
    return true;
  }

  return false;
}

bool Sim7070G::httpDisconnect()
{
  return _at.sendCommandSync("AT+SHDISC", 5000);
}

// TCP/UDP Socket Implementation
bool Sim7070G::socketSetSSL(int socketId, uint8_t sslIndex)
{
  char cmd[64];
  snprintf(cmd, sizeof(cmd), "AT+CASSLCFG=%d,%d", socketId, sslIndex);
  return _at.sendCommandSync(cmd, 5000);
}

bool Sim7070G::socketOpen(int socketId, const char *host, uint16_t port, bool tcp)
{
  if (!host)
  {
    return false;
  }

  char cmd[256];
  snprintf(cmd, sizeof(cmd), "AT+CAOPEN=%d,\"%s\",%d,\"%s\"",
           socketId, tcp ? "TCP" : "UDP", port, host);
  return _at.sendCommandSync(cmd, 30000);
}

bool Sim7070G::socketSend(int socketId, const uint8_t *data, size_t len)
{
  if (!data || len == 0)
  {
    return false;
  }

  char cmd[64];
  snprintf(cmd, sizeof(cmd), "AT+CASEND=%d,%d", socketId, len);

  if (!_at.sendCommandSync(cmd, 5000))
  {
    return false;
  }

  // Send data
  _serial->write(data, len);
  _serial->flush();

  delay(100);
  return _at.sendCommandSync("AT", 5000);
}

bool Sim7070G::socketReceive(int socketId, uint8_t *buffer, size_t bufferSize, size_t *bytesRead)
{
  if (!buffer || bufferSize == 0)
  {
    return false;
  }

  char cmd[32];
  snprintf(cmd, sizeof(cmd), "AT+CARECV=%d,%d", socketId, bufferSize);

  char response[512];
  if (!_at.sendCommandSync(cmd, 10000, response, sizeof(response)))
  {
    return false;
  }

  // Parse +CARECV: <socket_id>,<length>,<data>
  int sid, length;
  if (sscanf(response, "+CARECV: %d,%d,", &sid, &length) == 2)
  {
    const char *dataStart = strchr(response, ',');
    if (dataStart)
    {
      dataStart = strchr(dataStart + 1, ',');
      if (dataStart)
      {
        dataStart++; // Skip comma
        size_t copyLen = (length < bufferSize) ? length : bufferSize;
        memcpy(buffer, dataStart, copyLen);
        if (bytesRead)
        {
          *bytesRead = copyLen;
        }
        return true;
      }
    }
  }

  return false;
}

bool Sim7070G::socketClose(int socketId)
{
  char cmd[32];
  snprintf(cmd, sizeof(cmd), "AT+CACLOSE=%d", socketId);
  return _at.sendCommandSync(cmd, 5000);
}

bool Sim7070G::socketSetTransparentMode(int socketId, bool enable)
{
  char cmd[64];
  snprintf(cmd, sizeof(cmd), "AT+CACFG=%d,%d", socketId, enable ? 1 : 0);
  return _at.sendCommandSync(cmd, 5000);
}

bool Sim7070G::socketSwitchTransparent(int socketId, bool enable)
{
  char cmd[64];
  snprintf(cmd, sizeof(cmd), "AT+CASWITCH=%d,%d", socketId, enable ? 1 : 0);
  return _at.sendCommandSync(cmd, 5000);
}

// Internal methods
void Sim7070G::updateState(Sim7070GState newState)
{
  if (_state != newState)
  {
    _state = newState;
    if (_networkStateCallback)
    {
      _networkStateCallback(newState);
    }
  }
}

void Sim7070G::updateMQTTState(MQTTState newState)
{
  if (_mqttState != newState)
  {
    _mqttState = newState;
    if (_mqttStateCallback)
    {
      _mqttStateCallback(newState);
    }
  }
}

void Sim7070G::processURCs()
{
  // URCs are handled by registered handlers in Sim7070G_AT
  // This method can be used for additional processing if needed
}

// URC Handlers
void Sim7070G::onURC_SMSTATE(const char *urc, const char *data)
{
  if (!_instance)
    return;

  // Parse MQTT state change
  int state;
  if (sscanf(data, "%d", &state) == 1)
  {
    MQTTState mqttState;
    switch (state)
    {
    case 0:
      mqttState = MQTTState::DISCONNECTED;
      break;
    case 1:
      mqttState = MQTTState::CONNECTED;
      break;
    default:
      mqttState = MQTTState::ERROR;
      break;
    }
    _instance->updateMQTTState(mqttState);
  }
}

void Sim7070G::onURC_SMPUB(const char *urc, const char *data)
{
  // Handle published message acknowledgment
  // Format: +SMPUB: <topic>,<result>
}

void Sim7070G::onURC_SMSUB(const char *urc, const char *data)
{
  if (!_instance || !_instance->_mqttMessageCallback)
    return;

  // Parse incoming MQTT message
  // Format: +SMSUB: <topic>,<qos>,<payload_len>,<payload>
  char topic[128];
  int qos, payloadLen;

  if (sscanf(data, "\"%127[^\"]\",%d,%d,", topic, &qos, &payloadLen) >= 3)
  {
    // Extract payload (after last comma)
    const char *payloadStart = strrchr(data, ',');
    if (payloadStart)
    {
      payloadStart++; // Skip comma
      uint8_t payload[512];
      size_t len = (payloadLen < sizeof(payload)) ? payloadLen : sizeof(payload);

      // Convert hex string to bytes if needed, or use as-is
      memcpy(payload, payloadStart, len);

      _instance->_mqttMessageCallback(topic, payload, len);
    }
  }
}

void Sim7070G::onURC_CGREG(const char *urc, const char *data)
{
  if (!_instance)
    return;

  // Parse network registration status
  int n, stat;
  if (sscanf(data, "%d,%d", &n, &stat) == 2)
  {
    NetworkRegStatus regStatus;
    switch (stat)
    {
    case 0:
      regStatus = NetworkRegStatus::NOT_REGISTERED;
      break;
    case 1:
      regStatus = NetworkRegStatus::REGISTERED_HOME;
      break;
    case 2:
      regStatus = NetworkRegStatus::SEARCHING;
      break;
    case 3:
      regStatus = NetworkRegStatus::REGISTRATION_DENIED;
      break;
    case 5:
      regStatus = NetworkRegStatus::REGISTERED_ROAMING;
      break;
    default:
      regStatus = NetworkRegStatus::UNKNOWN;
      break;
    }
    _instance->_networkInfo.regStatus = regStatus;

    if (regStatus == NetworkRegStatus::REGISTERED_HOME ||
        regStatus == NetworkRegStatus::REGISTERED_ROAMING)
    {
      _instance->updateState(Sim7070GState::NETWORK_CONNECTED);
    }
    else
    {
      _instance->updateState(Sim7070GState::NETWORK_DISCONNECTED);
    }
  }
}

void Sim7070G::onURC_CGNAPN(const char *urc, const char *data)
{
  if (!_instance)
    return;

  // Parse APN info
  int index;
  char apn[64];
  if (sscanf(data, "%d,\"%63[^\"]\"", &index, apn) == 2)
  {
    strncpy(_instance->_networkInfo.apn, apn, sizeof(_instance->_networkInfo.apn) - 1);
  }
}

void Sim7070G::onURC_CNACT(const char *urc, const char *data)
{
  if (!_instance)
    return;

  // Parse PDP context activation status
  // Format: +CNACT: <cid>,<state>,<ip_address>
  int cid, state;
  char ip[16];
  if (sscanf(data, "%d,%d,\"%15[^\"]\"", &cid, &state, ip) == 3)
  {
    if (state == 1)
    {
      strncpy(_instance->_networkInfo.ipAddress, ip, sizeof(_instance->_networkInfo.ipAddress) - 1);
      _instance->updateState(Sim7070GState::NETWORK_CONNECTED);
    }
    else
    {
      _instance->updateState(Sim7070GState::NETWORK_DISCONNECTED);
    }
  }
}
void Sim7070G::onURC_APP_PDP(const char *urc, const char *data)
{
  if (!_instance)
    return;

  // Parse PDP context activation status
  // Format: +APP PDP: <cid>,ACTIVE or +APP PDP: <cid>,DEACTIVE
  int cid;
  char state[16];
  DEBUG_PRINTLN(F("[URC] APP PDP: "));
  DEBUG_PRINTLN(data);
  if (sscanf(data, "%d,%15s", &cid, state) == 2)
  {
    if (strcmp(state, "ACTIVE") == 0)
    {
      DEBUG_PRINTLN(F("[URC] PDP context activated"));
      _instance->updateState(Sim7070GState::NETWORK_CONNECTED);
    }
    else if (strcmp(state, "DEACTIVE") == 0)
    {
      DEBUG_PRINTLN(F("[URC] PDP context deactivated"));
      _instance->updateState(Sim7070GState::NETWORK_DISCONNECTED);
    }
  }
}

// Command wrappers - expose Sim7070G_AT methods
bool Sim7070G::sendCommandSync(const char *command, unsigned long timeout,
                               char *response, size_t responseSize)
{
  return _at.sendCommandSync(command, timeout, response, responseSize);
}

int Sim7070G::sendCommand(const char *command, unsigned long timeout,
                          ATCallback callback, void *userData)
{
  return _at.sendCommand(command, timeout, callback, userData);
}

bool Sim7070G::checkSendCommandSync(const char *command, const char *expectedResult,
                                    unsigned long timeout)
{
  if (!command || !expectedResult)
  {
    return false;
  }

  char response[256];
  if (!_at.sendCommandSync(command, timeout, response, sizeof(response)))
  {
    DEBUG_PRINT(F("[CMD] Command failed: "));
    DEBUG_PRINTLN(command);
    return false;
  }

  // Remove command echo from response if present
  // Echo is typically the first line, so skip it
  char *responsePtr = response;
  DEBUG_PRINTLN(F("[CMD] Response: "));
  DEBUG_PRINTLN(response);
  if (strncmp(responsePtr, command, strlen(command)) == 0)
  {
    // Skip echo line
    responsePtr = strchr(responsePtr, '\n');
    if (responsePtr)
    {
      responsePtr++; // Skip newline
      // Skip any additional whitespace
      while (*responsePtr == '\r' || *responsePtr == '\n' || *responsePtr == ' ')
      {
        responsePtr++;
      }
    }
  }

  // Check if response contains expected result
  bool match = (strstr(responsePtr, expectedResult) != nullptr);

  if (match)
  {
    DEBUG_PRINT(F("[CMD] Command OK, expected found: "));
    DEBUG_PRINTLN(expectedResult);
  }
  else
  {
    DEBUG_PRINT(F("[CMD] Command OK but expected not found: "));
    DEBUG_PRINT(expectedResult);
    DEBUG_PRINT(F(", got: "));
    DEBUG_PRINTLN(responsePtr);
  }

  return match;
}
