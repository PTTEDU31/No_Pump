#include "sim7070g.h"
#include <Adafruit_SleepyDog.h>

// Macro for watchdog reset
#define WDT_RESET() Watchdog.reset()

// Debug macros (will use the ones from config.h)
#if DEBUG
#define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
#define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#define DEBUG_PRINTLN(...)
#endif

#if DIAG
#define DIAG_PRINT(...) Serial.print(__VA_ARGS__)
#define DIAG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
#define DIAG_PRINT(...)
#define DIAG_PRINTLN(...)
#endif

// Constructor
SIM7070G::SIM7070G(HardwareSerial& serial, uint8_t pwrPin)
  : _serial(serial), _pwrPin(pwrPin), mqttConnected(false),
    cntModemRestarts(0), cntGprsConnects(0), cntMqttConnects(0),
    atFailStreak(0), gprsFailStreak(0), mqttFailStreak(0),
    lastATokMs(0), lastGPRSokMs(0), lastMQTTokMs(0), lastRestartMs(0),
    _lastUartActivity(0) {
}

// Initialization
void SIM7070G::begin(unsigned long baudRate) {
  pinMode(_pwrPin, OUTPUT);
  digitalWrite(_pwrPin, LOW);
  _serial.begin(baudRate);
}

void SIM7070G::applyBootConfig() {
  sendAT("AT+CMEE=2", 120);
  sendAT("AT+CSOCKSETPN=1", 200);
  sendAT("AT+SMCONF=\"CONTEXTID\",1", 200);
  sendAT("AT+CSCLK=0", 150);
}

// UART activity tracking
void SIM7070G::noteUartActivity() {
  _lastUartActivity = millis();
}

bool SIM7070G::canRunHealthCheck() {
  return (millis() - _lastUartActivity) > _healthQuietMs;
}

// AT command helpers
void SIM7070G::sendAT(const char* command, unsigned long wait) {
  unsigned long t0 = millis();
  while (millis() - t0 < 20) {
    while (_serial.available()) {
      (void)_serial.read();
      noteUartActivity();
    }
    delay(1);
  }

  _serial.println(command);
  noteUartActivity();
  if (DIAG) {
    DIAG_PRINT(F("[DIAG] sendAT \""));
    DIAG_PRINT(command);
    DIAG_PRINT(F("\" wait="));
    DIAG_PRINTLN(wait);
  }
  delay(wait);
  readResponse();
}

bool SIM7070G::sendATwaitOK(const char* cmd, char* out, size_t outCap, unsigned long overallMs) {
  if (outCap == 0) return false;
  out[0] = '\0';

  unsigned long t0 = millis();
  while (millis() - t0 < 20) {
    while (_serial.available()) {
      (void)_serial.read();
      noteUartActivity();
    }
    delay(1);
  }

  if (DIAG) {
    DIAG_PRINT(F("[DIAG] sendATwaitOK avail-before="));
    DIAG_PRINTLN(_serial.available());
  }

  _serial.println(cmd);
  noteUartActivity();

  unsigned long tstart = millis();
  size_t w = 0;
  bool sawOK = false, sawERROR = false;

  while (millis() - tstart < overallMs) {
    while (_serial.available()) {
      char c = _serial.read();
      noteUartActivity();
      if (w < outCap - 1) out[w++] = c;

      if (c == '\n') {
        out[w] = '\0';
        if (strstr(out, "\nOK") || strstr(out, "OK\r") || strstr(out, "OK\n")) {
          sawOK = true;
          goto done;
        }
        if (strstr(out, "ERROR") || strstr(out, "+CME ERROR")) {
          sawERROR = true;
          goto done;
        }
      }
    }
    WDT_RESET();
  }

  out[w] = '\0';
  if (strstr(out, "OK")) sawOK = true;
  if (strstr(out, "ERROR") || strstr(out, "+CME ERROR")) sawERROR = true;

done:
  out[w] = '\0';
  if (sawOK && !sawERROR) lastATokMs = millis();
  if (DIAG) {
    DIAG_PRINT(F("[DIAG] sendATwaitOK result="));
    DIAG_PRINT(sawOK && !sawERROR ? "OK" : "FAIL");
    DIAG_PRINT(F(" bytes="));
    DIAG_PRINTLN(w);
  }
  return sawOK && !sawERROR;
}

void SIM7070G::sendCommandGetResponse(const char* command, char* buffer, size_t bufferSize, unsigned long timeout) {
  if (bufferSize == 0) return;
  buffer[0] = '\0';

  if (DIAG) {
    DIAG_PRINT(F("[DIAG] sendCmd avail-before="));
    DIAG_PRINT(_serial.available());
    DIAG_PRINT(F(" cmd="));
    DIAG_PRINTLN(command);
  }

  _serial.println(command);
  noteUartActivity();

  unsigned long start = millis();
  size_t idx = 0;
  bool sawTerminator = false;

  while (millis() - start < timeout && idx < bufferSize - 1) {
    while (_serial.available() && idx < bufferSize - 1) {
      char c = _serial.read();
      noteUartActivity();
      buffer[idx++] = c;

      if (c == '\n') {
        buffer[idx] = '\0';
        if (strstr(buffer, "\nOK") || strstr(buffer, "OK\r") || strstr(buffer, "OK\n") || 
            strstr(buffer, "ERROR") || strstr(buffer, "+CME ERROR")) {
          sawTerminator = true;
          goto endread;
        }
      }
    }
    WDT_RESET();
  }

endread:
  buffer[idx] = '\0';
  if (strstr(buffer, "OK")) lastATokMs = millis();

  if (DIAG) {
    DIAG_PRINT(F("[DIAG] sendCmd done bytes="));
    DIAG_PRINT(idx);
    DIAG_PRINT(F(" sawTerm="));
    DIAG_PRINTLN(sawTerminator ? "Y" : "N");
  }
}

void SIM7070G::readResponse() {
  String receivedMessage = "";
  unsigned long timeout = millis() + READ_RESPONSE_WAIT_MS;
  while (millis() < timeout) {
    while (_serial.available()) {
      char c = _serial.read();
      noteUartActivity();
      receivedMessage += c;
      if (receivedMessage.length() > 550) break;
    }
    WDT_RESET();
  }
  if (receivedMessage.length() > 0) {
    DEBUG_PRINTLN(receivedMessage);
  }
}

void SIM7070G::flushBuffer() {
  while (_serial.available()) (void)_serial.read();
}

void SIM7070G::safeFlushAfterReset() {
  unsigned long t0 = millis();
  while (millis() - t0 < 200) {
    while (_serial.available()) {
      (void)_serial.read();
      noteUartActivity();
    }
    delay(5);
  }
}

// Power management
bool SIM7070G::isAlive() {
  unsigned long t0 = millis();
  int drained = 0;
  while (millis() - t0 < 30) {
    while (_serial.available()) {
      (void)_serial.read();
      DEBUG_PRINTLN("The SIM7070G was emptied SIM7070G Modem Alive Ping");
      noteUartActivity();
      drained++;
    }
    delay(2);
  }

  _serial.write('\r');
  noteUartActivity();
  delay(40);

  auto tryAT = [&](uint32_t waitMs) -> bool {
    char buf[192] = { 0 };
    size_t idx = 0;
    _serial.println("AT");
    noteUartActivity();
    unsigned long start = millis();
    while (millis() - start < waitMs) {
      while (_serial.available()) {
        char c = _serial.read();
        noteUartActivity();
        if (idx < sizeof(buf) - 1) buf[idx++] = c;
        if (c == '\n') {
          buf[idx] = '\0';
          if (strstr(buf, "\nOK") || strstr(buf, "OK\r") || strstr(buf, "OK\n") || strstr(buf, "OK")) {
            lastATokMs = millis();
            return true;
          }
        }
      }
      WDT_RESET();
      delay(1);
    }
    buf[idx] = '\0';
    if (strstr(buf, "OK")) {
      lastATokMs = millis();
      return true;
    }
    return false;
  };

  if (tryAT(400)) return true;
  if (tryAT(1200)) return true;
  return false;
}

void SIM7070G::verifySIMConnected() {
  if (isAlive()) {
    DEBUG_PRINTLN(F("SIM7070G responds."));
    return;
  }
  DEBUG_PRINTLN(F("There is no 'OK'. Restarting modem..."));
  restart();
}

void SIM7070G::restart() {
  DEBUG_PRINTLN(F("Elegant restart of modem (PWRKEY and verification)…"));

  digitalWrite(_pwrPin, LOW);

  auto pwrkeyPulse = [&]() {
    digitalWrite(_pwrPin, HIGH);
    delay(PWRKEY_MS);
    digitalWrite(_pwrPin, LOW);
  };

  auto softDrain = [&]() {
    unsigned long t0 = millis();
    while (millis() - t0 < PRE_DRAIN_MS) {
      while (_serial.available()) (void)_serial.read();
      delay(1);
      WDT_RESET();
    }
  };

  softDrain();
  if (isAlive()) {
    DEBUG_PRINTLN(F("Modem already responds. No power cycle required."));
    return;
  }

  DEBUG_PRINTLN(F("Without response. Sending the first PWRKEY pulse (toggle)…"));
  pwrkeyPulse();

  {
    unsigned long t0 = millis();
    while (millis() - t0 < OFF_WAIT_MS) {
      WDT_RESET();
      delay(10);
    }
  }
  softDrain();

  if (isAlive()) {
    DEBUG_PRINTLN(F("Modem responds after first toggle."));
    {
      unsigned long t0 = millis();
      while (millis() - t0 < BOOT_WAIT_MS) {
        WDT_RESET();
        delay(10);
      }
    }
    safeFlushAfterReset();
    lastRestartMs = millis();
    applyBootConfig();
    cntModemRestarts++;
    return;
  }

  DEBUG_PRINTLN(F("Still without response. Sending second PWRKEY pulse. (forcing ON)…"));
  pwrkeyPulse();

  {
    unsigned long t0 = millis();
    while (millis() - t0 < BOOT_WAIT_MS) {
      WDT_RESET();
      delay(10);
    }
  }
  softDrain();
  safeFlushAfterReset();

  lastRestartMs = millis();
  DEBUG_PRINTLN(F("Restart sequence completed. (doble toggle PWRKEY)."));
  applyBootConfig();
  cntModemRestarts++;
}

bool SIM7070G::ensureATorPowerCycle(uint8_t atAttempts) {
  for (uint8_t i = 0; i < atAttempts; i++) {
    if (isAlive()) {
      DEBUG_PRINTLN(F("AT OK (without power cycle)."));
      return true;
    }
    DEBUG_PRINT(F("AT does not respond (try number: "));
    DEBUG_PRINT(i + 1);
    DEBUG_PRINTLN(F(")."));
    delay(250);
    WDT_RESET();
  }

  DEBUG_PRINTLN(F("Without AT after 3 retries. Cycling with PWRKEY (modemRestart)..."));
  restart();

  unsigned long t0 = millis();
  while (millis() - t0 < 20000UL) {
    if (isAlive()) {
      DEBUG_PRINTLN(F("AT OK after power cycle."));
      return true;
    }
    delay(200);
    WDT_RESET();
  }

  DEBUG_PRINTLN(F("AT not available after power cycle."));
  return false;
}

// GPRS functions
bool SIM7070G::gprsConnect() {
  DEBUG_PRINTLN(F("Verifying GPRS state..."));

  char response[160] = { 0 };

  // Get CCLK before anything else
  {
    char cclkResp[128] = { 0 };
    char iso[32] = { 0 };

    DEBUG_PRINTLN(F("[GPRS] Getting CCLK before PDP..."));
    sendCommandGetResponse("AT+CCLK?", cclkResp, sizeof(cclkResp), 2000);

    DEBUG_PRINT(F("[GPRS] Raw CCLK response: "));
    DEBUG_PRINTLN(cclkResp);

    if (parseCCLKToISO(cclkResp, iso, sizeof(iso))) {
      DEBUG_PRINT(F("[GPRS] Parsed UTC time: "));
      DEBUG_PRINTLN(iso);
    } else {
      DEBUG_PRINTLN(F("[GPRS] Could not parse CCLK into ISO-8601."));
    }
  }

  sendCommandGetResponse("AT+CNACT?", response, sizeof(response), 2000);

  if (response[0] == '\0') {
    DEBUG_PRINTLN(F("CNACT? returned 0 bytes. Ensuring AT (modem on) before touching CFUN/CNACT..."));
    if (!ensureATorPowerCycle(3)) {
      DEBUG_PRINTLN(F("Aborting GPRS: there is no AT."));
      return false;
    }
    memset(response, 0, sizeof(response));
    sendCommandGetResponse("AT+CNACT?", response, sizeof(response), 2000);
  }

  if (strstr(response, "+CNACT: 1,1")) {
    DEBUG_PRINTLN(F("Connecting to data context (CID=1)."));
    lastGPRSokMs = millis();
    cntGprsConnects++;

    // Synchronize UTC time via multiple NTP servers
    syncTimeUTC_any(PDP_CID, 12000UL);

    return true;
  }

  {
    char fun[64] = { 0 };
    sendCommandGetResponse("AT+CFUN?", fun, sizeof(fun), 1500);

    if (fun[0] == '\0') {
      DEBUG_PRINTLN(F("CFUN? returned 0 bytes. Ensuring AT before CFUN..."));
      if (!ensureATorPowerCycle(3)) {
        DEBUG_PRINTLN(F("Aborting GPRS: there is no AT."));
        return false;
      }
      memset(fun, 0, sizeof(fun));
      sendCommandGetResponse("AT+CFUN?", fun, sizeof(fun), 1500);
    }

    if (!strstr(fun, "+CFUN: 1")) {
      DEBUG_PRINT(F("Current CFUN: "));
      DEBUG_PRINTLN(fun);
      sendAT("AT+CFUN=1", 2500);
      delay(1500);
      WDT_RESET();
    }
  }

  sendAT("AT+CPSMS=0", 200);
  sendAT("AT+CGDCONT=1,\"IP\",\"" PDP_APN "\"", SIM_WAIT_MS);
  sendAT("AT+CNMP=2", SIM_WAIT_MS);
  sendAT("AT+CSOCKSETPN=1", 200);

  for (uint8_t attempt = 0; attempt < 2; attempt++) {
    DEBUG_PRINT(F("Activating data context (try number: "));
    DEBUG_PRINT(attempt + 1);
    DEBUG_PRINTLN(F(")..."));

    if (!isAlive()) {
      DEBUG_PRINTLN(F("AT failed before CNACT. Ensuring AT..."));
      if (!ensureATorPowerCycle(3)) {
        DEBUG_PRINTLN(F("Aborting GPRS: there is no AT."));
        return false;
      }
    }

    sendAT("AT+CNACT=1,0", 800);
    sendAT("AT+CNACT=1,1", 2000);

    memset(response, 0, sizeof(response));
    sendCommandGetResponse("AT+CNACT?", response, sizeof(response), 2500);

    if (response[0] == '\0') {
      DEBUG_PRINTLN(F("CNACT? returned 0 bytes after activation. Ensuring AT and rereading state..."));
      if (!ensureATorPowerCycle(3)) {
        DEBUG_PRINTLN(F("Aborting GPRS: there is no AT."));
        return false;
      }
      memset(response, 0, sizeof(response));
      sendCommandGetResponse("AT+CNACT?", response, sizeof(response), 2500);
    }

    DEBUG_PRINT(F("Response CNACT (post intent): "));
    DEBUG_PRINTLN(response);
    if (strstr(response, "+CNACT: 1,1")) {
      DEBUG_PRINTLN(F("Connected to data context (CID=1)."));
      lastGPRSokMs = millis();
      cntGprsConnects++;

      // Synchronize UTC time via multiple NTP servers
      syncTimeUTC_any(PDP_CID, 12000UL);

      return true;
    }

    WDT_RESET();
    delay(1000);
  }

  DEBUG_PRINTLN(F("Error: Unable to connect to data context (wihtout recursion)."));
  return false;
}

bool SIM7070G::gprsIsConnected() {
  char response[120];
  sendCommandGetResponse("AT+CNACT?", response, sizeof(response), 1500);
  bool ok = strstr(response, "+CNACT: 1,1") != NULL;
  if (ok) lastGPRSokMs = millis();
  return ok;
}

// MQTT functions
void SIM7070G::mqttConnect() {
  if (!gprsIsConnected()) {
    DEBUG_PRINTLN(F("mqttConnect: GPRS not connected. Aborting MQTT connection attempt."));
    mqttConnected = false;
    return;
  }
  if (checkMQTTConnection()) {
    return;
  }

  if (!ensureATorPowerCycle(3)) {
    DEBUG_PRINTLN(F("mqttConnect: there are no AT after retry. Aborting."));
    mqttConnected = false;
    return;
  }
  if (checkMQTTConnection()) {
    DEBUG_PRINTLN(F("MQTT already connected after ensuring AT. Ommitting reconnection."));
    return;
  }

  DEBUG_PRINTLN(F("Connecting to MQTT"));

  sendAT("AT+SMDISC", 200);

  char cmd[96];
  sendAT("AT+SMCONF=\"URL\",\"\"", 100);
  snprintf(cmd, sizeof(cmd), "AT+SMCONF=\"URL\",\"%s\"", mqtt_server);
  sendAT(cmd, 120);
  snprintf(cmd, sizeof(cmd), "AT+SMCONF=\"PORT\",\"%d\"", mqtt_port);
  sendAT(cmd, 120);
  snprintf(cmd, sizeof(cmd), "AT+SMCONF=\"CLIENTID\",\"%s\"", clientID);
  sendAT(cmd, 120);

  sendAT("AT+SMCONF=\"CLEANSS\",1", 100);
  sendAT("AT+SMCONF=\"QOS\",\"0\"", 100);
  sendAT("AT+SMCONF=\"RETAIN\",\"0\"", 100);
  sendAT("AT+SMCONF=\"KEEPTIME\",60", 100);

  snprintf(cmd, sizeof(cmd), "AT+SMCONF=\"USERNAME\",\"%s\"", mqtt_username);
  sendAT(cmd, 120);
  snprintf(cmd, sizeof(cmd), "AT+SMCONF=\"PASSWORD\",\"%s\"", mqtt_password);
  sendAT(cmd, 120);

  DEBUG_PRINTLN(F("Connecting to broker..."));
  sendAT("AT+SMCONN", 2500);
  delay(800);

  if (checkMQTTConnection()) {
    cntMqttConnects++;
    snprintf(cmd, sizeof(cmd), "AT+SMSUB=\"%s\",1", topic_sub);
    sendAT(cmd, 500);
    DEBUG_PRINT(F("Subscribed to: "));
    DEBUG_PRINTLN(topic_sub);
    readResponse();
    lastMQTTokMs = millis();
  } else {
    DEBUG_PRINTLN(F("MQTT connection failed"));
  }
}

bool SIM7070G::checkMQTTConnection() {
  char response[96] = { 0 };
  sendCommandGetResponse("AT+SMSTATE?", response, sizeof(response), 1500);
  mqttConnected = strstr(response, "+SMSTATE: 1") != NULL;

  if (mqttConnected) {
    DEBUG_PRINTLN(F("Connected to MQTT"));
    lastMQTTokMs = millis();
  } else {
    DEBUG_PRINT(F("MQTT NOT connected. State: "));
    DEBUG_PRINTLN(response);
  }
  return mqttConnected;
}

bool SIM7070G::publishMessage(const char* topic, const char* payload) {
  if (!mqttConnected) return false;

  int len = strlen(payload);
  _serial.print(F("AT+SMPUB=\""));
  _serial.print(topic);
  _serial.print(F("\","));
  _serial.print(len);
  _serial.println(F(",1,0"));
  noteUartActivity();
  delay(120);
  _serial.print(payload);
  noteUartActivity();
  delay(350);
  noteUartActivity();

  return true;
}

// Time synchronization
bool SIM7070G::syncTimeUTC_viaCNTP(uint8_t cid, const char* server, uint32_t waitTotalMs) {
  char buf[128] = { 0 };
  char cmd[96];
  char timeBefore[160] = { 0 };
  char timeAfter[160] = { 0 };

  DEBUG_PRINT(F("[NTP] Synchronizing with server: "));
  DEBUG_PRINTLN(server);

  // Capture time BEFORE sync attempt
  sendCommandGetResponse("AT+CCLK?", timeBefore, sizeof(timeBefore), 1200);

  // Configure NTP server
  snprintf(cmd, sizeof(cmd), "AT+CNTPCID=%u", cid);
  sendCommandGetResponse(cmd, buf, sizeof(buf), 1200);

  snprintf(cmd, sizeof(cmd), "AT+CNTP=\"%s\",0", server);
  sendCommandGetResponse(cmd, buf, sizeof(buf), 1500);

  // Trigger NTP sync
  sendCommandGetResponse("AT+CNTP", buf, sizeof(buf), 1500);

  // Poll for result
  bool ok = false;
  bool explicitFailure = false;
  unsigned long t0 = millis();

  while (millis() - t0 < waitTotalMs) {
    memset(buf, 0, sizeof(buf));
    sendCommandGetResponse("AT+CNTP?", buf, sizeof(buf), 900);

    // Check for success
    if (strstr(buf, "+CNTP: 1")) {
      ok = true;
      DEBUG_PRINTLN(F("[NTP] Sync successful!"));
      break;
    }

    // Check for explicit failure
    if (strstr(buf, "+CNTP: 0")) {
      explicitFailure = true;
      DEBUG_PRINTLN(F("[NTP] Explicit failure (+CNTP: 0)"));
      break;
    }

    if (strstr(buf, "ERROR") || strstr(buf, "+CME ERROR")) {
      explicitFailure = true;
      DEBUG_PRINTLN(F("[NTP] Error response"));
      break;
    }

    delay(600);
    WDT_RESET();
  }

  if (!ok && !explicitFailure) {
    DEBUG_PRINTLN(F("[NTP] Timeout waiting for response"));
  }

  // Capture time AFTER sync attempt
  sendCommandGetResponse("AT+CCLK?", timeAfter, sizeof(timeAfter), 1200);

  // Verify time actually changed
  if (ok && strcmp(timeBefore, timeAfter) == 0) {
    DEBUG_PRINTLN(F("[NTP] Warning: Time didn't change after NTP sync"));
  }

  DEBUG_PRINT(F("[NTP] Time before: "));
  DEBUG_PRINTLN(timeBefore);
  DEBUG_PRINT(F("[NTP] Time after:  "));
  DEBUG_PRINTLN(timeAfter);

  if (ok) {
    DEBUG_PRINTLN(F("[NTP] Time synchronized successfully"));
  } else {
    DEBUG_PRINT(F("[NTP] Failed to synchronize with "));
    DEBUG_PRINTLN(server);
  }

  return ok;
}

bool SIM7070G::syncTimeUTC_any(uint8_t cid, uint32_t waitTotalMs) {
  const char* servers[] = {
    "time.google.com",
    "time.cloudflare.com",
    "pool.ntp.org",
    "time.windows.com",
    "time.nist.gov"
  };
  const size_t nServers = sizeof(servers) / sizeof(servers[0]);

  for (size_t i = 0; i < nServers; i++) {
    DEBUG_PRINT(F("[NTP] Trying server: "));
    DEBUG_PRINTLN(servers[i]);

    if (syncTimeUTC_viaCNTP(cid, servers[i], waitTotalMs)) {
      DEBUG_PRINT(F("[NTP] Time synchronized using: "));
      DEBUG_PRINTLN(servers[i]);
      return true;
    }
  }

  DEBUG_PRINTLN(F("[NTP] All NTP servers failed. Keeping previous clock / network time."));
  return false;
}

bool SIM7070G::getNetworkTimeISO8601(char* isoOut, size_t outCap) {
  char resp[96] = { 0 };
  sendCommandGetResponse("AT+CCLK?", resp, sizeof(resp), 1500);
  if (strstr(resp, "+CCLK:") == NULL) return false;
  return parseCCLKToISO(resp, isoOut, outCap);
}

bool SIM7070G::parseCCLKToISO(const char* cclkResp, char* isoOut, size_t outCap) {
  if (!cclkResp || !isoOut || outCap < 21) return false;

  const char* p = strchr(cclkResp, '\"');
  if (!p) return false;
  const char* q = strchr(p + 1, '\"');
  if (!q) return false;

  char core[32];
  size_t n = (size_t)(q - (p + 1));
  if (n >= sizeof(core)) n = sizeof(core) - 1;
  memcpy(core, p + 1, n);
  core[n] = '\0';

  int yy = 0, MM = 0, dd = 0, hh = 0, mi = 0, ss = 0;
  char tzStr[6] = { 0 };
  int scanned = sscanf(core, "%2d/%2d/%2d,%2d:%2d:%2d%5s", &yy, &MM, &dd, &hh, &mi, &ss, tzStr);
  if (scanned < 7) return false;

  int y = (yy <= 69) ? (2000 + yy) : (1900 + yy);

  int tzQuarter = atoi(tzStr);
  long totalMin = (long)hh * 60L + (long)mi - (long)tzQuarter * 15L;

  int dayShift = 0;
  if (totalMin < 0) {
    totalMin += 1440;
    dayShift = -1;
  } else if (totalMin >= 1440) {
    totalMin -= 1440;
    dayShift = 1;
  }

  int uh = (int)(totalMin / 60L);
  int um = (int)(totalMin % 60L);
  int us = ss;

  auto isLeap = [](int Y) -> bool {
    return ((Y % 4 == 0) && (Y % 100 != 0)) || (Y % 400 == 0);
  };
  auto dim = [&](int Y, int M) -> int {
    switch (M) {
      case 1: return 31;
      case 2: return isLeap(Y) ? 29 : 28;
      case 3: return 31;
      case 4: return 30;
      case 5: return 31;
      case 6: return 30;
      case 7: return 31;
      case 8: return 31;
      case 9: return 30;
      case 10: return 31;
      case 11: return 30;
      case 12: return 31;
      default: return 30;
    }
  };

  if (dayShift == -1) {
    dd -= 1;
    if (dd <= 0) {
      MM -= 1;
      if (MM <= 0) {
        MM = 12;
        y -= 1;
      }
      dd = dim(y, MM);
    }
  } else if (dayShift == 1) {
    dd += 1;
    if (dd > dim(y, MM)) {
      dd = 1;
      MM += 1;
      if (MM > 12) {
        MM = 1;
        y += 1;
      }
    }
  }

  snprintf(isoOut, outCap, "%04d-%02d-%02dT%02d:%02d:%02dZ", y, MM, dd, uh, um, us);
  return true;
}

bool SIM7070G::getNetworkHHMMSS(char* hms, size_t cap) {
  char resp[96] = { 0 };
  sendCommandGetResponse("AT+CCLK?", resp, sizeof(resp), 2000);

  const char* p = strchr(resp, '\"');
  if (!p) return false;
  const char* q = strchr(p + 1, '\"');
  if (!q) return false;

  char core[32];
  size_t n = (size_t)(q - (p + 1));
  if (n >= sizeof(core)) n = sizeof(core) - 1;
  memcpy(core, p + 1, n);
  core[n] = '\0';

  int H = 0, M = 0, S = 0;
  if (sscanf(core, "%*2d/%*2d/%*2d,%2d:%2d:%2d", &H, &M, &S) != 3) return false;

  if (cap < 9) return false;
  snprintf(hms, cap, "%02d:%02d:%02d", H, M, S);
  return true;
}

// Signal quality
int SIM7070G::getCSQ_RSSI() {
  char buf[128] = { 0 };

  if (!sendATwaitOK("AT+CSQ", buf, sizeof(buf), 1500)) {
    return -1;
  }

  const char* p = strstr(buf, "+CSQ:");
  if (!p) return -1;

  int rssi = 99, ber = 0;
  if (sscanf(p, "+CSQ: %d,%d", &rssi, &ber) != 2) return -1;

  if (rssi == 99) return -1;
  if (rssi < 0) rssi = 0;
  if (rssi > 31) rssi = 31;
  return rssi;
}

int SIM7070G::csqToDbm(int rssi) {
  if (rssi < 0 || rssi > 31) return -113;
  return -113 + (rssi * 2);
}
