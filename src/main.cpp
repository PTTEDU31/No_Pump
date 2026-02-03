#include <Arduino.h>
#include <Arduino_LSM6DS3.h>
#include <TemperatureZero.h>
#include <Adafruit_SleepyDog.h>
#include <Crypto.h>
#include <ChaChaPoly.h>
#include <math.h>
#include "wiring_private.h"
#include "config.h"
#include "crypto_json_utils.h"

// Node credentials from config.h
const char* NODE_ID = CONFIG_NODE_ID;
const char* USERNAME = CONFIG_USERNAME;
const char* PASSWORD = CONFIG_PASSWORD;
const char* CRYPTOKEY = CONFIG_CRYPTOKEY;

TemperatureZero tempSAMD;

// ----------------- CONFIG DEBUG/DIAGNOSTIC -----------------
#define DEBUG true  // <--- Put false to silence the serial prints in the monitor
#define DIAG false  // <--- Put false to silence the detailed diagnostics

#if DEBUG
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#define DEBUG_PRINT_F(x, p) Serial.print((x), (p))
#define DEBUG_PRINTLN_F(x, p) Serial.println((x), (p))
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINT_F(x, p)
#define DEBUG_PRINTLN_F(x, p)
#endif

#if DIAG
#define DIAG_PRINT(x) Serial.print(x)
#define DIAG_PRINTLN(x) Serial.println(x)
#else
#define DIAG_PRINT(x)
#define DIAG_PRINTLN(x)
#endif

// Pins, PDP, WDT from config.h (sim7070, MODEM_PWR_PIN, CONTACT_PIN, PWR_ON_PIN, PWR_OFF_PIN, ADC_*, BATT_VOLTS, Input_Supply_V, RS485_*, PDP_CID, PDP_APN, HW_WDT_TIMEOUT_SEC)

static inline void WDT_INIT() {
  Watchdog.enable(HW_WDT_TIMEOUT_SEC * 1000);
}
static inline void WDT_RESET() {
  Watchdog.reset();
}

// MQTT server/port from config.h (defines extern mqtt_server[], mqtt_port)
const char mqtt_server[] = CONFIG_MQTT_SERVER;
const int mqtt_port = CONFIG_MQTT_PORT;
char topic_sub[96];
char topic_pub[96];
char clientID[64];

char mqtt_username[64];
char mqtt_password[64];


// ----------------- MQTT STATE -----------------
bool mqttConnected = false;
unsigned long lastPublish = 0;

// PUBLISH / BATTERY from config.h (BAT_THRESH_PCT, BAT_HYST_PCT, PUB_FAST_MS, PUB_SLOW_MS)

// PREVIOUS STATE OF THE MAIN POWER LINE ----------
bool prevPwrOn = false;  // it is initialized in setup()

unsigned long publishIntervalMs = PUB_FAST_MS;

// CHECK CONNECTIONS INTERVAL from config.h (CHECK_INTERVAL_MS)
unsigned long lastCheck = 0;

byte pump = 0;
int onPulse = ON_PULSE_MS;
bool serverOnCommand = false;
String receivedMessage = "";

// TIMINGS from config.h (READ_RESPONSE_WAIT_MS, SIM_WAIT_MS)
int readResponseWait = READ_RESPONSE_WAIT_MS;
int simWait = SIM_WAIT_MS;

// UART Quiet Period from config.h (HEALTH_QUIET_MS)
static unsigned long lastUartActivity = 0;
inline void noteUartActivity() {
  lastUartActivity = millis();
}
inline bool canRunHealthCheck() {
  return (millis() - lastUartActivity) > HEALTH_QUIET_MS;
}
// Modem restart timing from config.h (PWRKEY_MS, PRE_DRAIN_MS, OFF_WAIT_MS, BOOT_WAIT_MS)

// ---- COUNTERS FOR RESTARTS/RECONNECTIONS ----
volatile uint32_t cntModemRestarts = 0;          // restarts via PWRKEY
volatile uint32_t cntGprsConnects = 0;           // successful PDP activations
volatile uint32_t cntMqttConnects = 0;           // successful MQTT connections
// RECONN_INTERVAL_MS from config.h

// RESTART DEBOUNCE from config.h (MIN_RESTART_GAP_MS)
unsigned long lastRestartMs = 0;

// CPU TEMPERATURE CHECK from config.h (TEMP_CHECK_INTERVAL_MS)
unsigned long lastTempCheck = 0;

// -------- VIBRATION ---------
int vibrationCount = 0;
int EventCountTotal = 0;

// ---- MODEM HEALTH (anti-false negatives) ----
static uint8_t atFailStreak = 0;
static uint8_t gprsFailStreak = 0;
static uint8_t mqttFailStreak = 0;

static unsigned long lastATokMs = 0;
static unsigned long lastGPRSokMs = 0;
static unsigned long lastMQTTokMs = 0;

// MODEM HEALTH THRESHOLDS from config.h (AT_FAILS_BEFORE_RESTART, GPRS_FAILS_BEFORE_RESTART, MQTT_FAILS_BEFORE_RESTART, RECENT_OK_GRACE_MS)

// PUMP ON-PROTECTION from config.h (PROTECT_ON_MS, PROTECT_TICK_MS)

volatile bool protectOnStart = false;
unsigned long protectStartMs = 0;
unsigned long protectLastAnnounce = 0;

// To distinguish if the pump was turned off via HEX (X=0)
volatile bool lastOffByHexFlag = false;

// To detect "turn off without HEX" by flank
bool prevPumpOn = false;  // It is initialized in setup() after reading the pin

// ADC & BATTERY VOLTAGE from config.h (ADC_VREF, ADC_MAX, R_TOP, R_BOTTOM)

// ------------- RS 485 ------------------

// We instantiate a UART in SERCOM0
Uart RS485(&sercom0, RS485_RX_PIN, RS485_TX_PIN, SERCOM_RX_PAD_1, UART_TX_PAD_0);

byte messege[256];
uint8_t index_request = 2;
float parameters[64];

// MODBUS / METER addresses from config.h (METER_FLOW_RATE_ADDRESS, METER_CUMULATIVE_ADDRESS, METER_ADDRESS, METER_FUNCTION_CODE, METER_NUM_REGISTERS)

struct modbus_transmit {
  uint8_t address = METER_ADDRESS;
  uint8_t function = METER_FUNCTION_CODE;

  uint32_t startByte_H = 0xFF000000;
  uint32_t startByte_L = 0x00FF0000;

  uint16_t endByte_H = 0x0000FF00;
  uint16_t endByte_L = 0x000000FF;

} MODBUS_REQ;



// ----------------- PROTOTYPES -----------------
void sendAT(const char* command, unsigned long wait = 100);  // Sends command AT to the modem
bool sendATwaitOK(const char* cmd, char* out, size_t outCap, unsigned long overallMs = 2500);
void sendCommandGetResponse(const char* command, char* response, size_t maxLen, unsigned long timeout = 1200);
static bool mainGetCCLKResponse(char* buf, size_t cap);
void readResponse();
void modemRestart();
void mqttConnect();
bool checkMQTTConnection();
bool publishMessage();
bool gprsConnect();
bool gprsIsConnected();
void handleMQTTMessages();
void flushSIMBuffer();
void checkConnections();
void turnOnPump();
void turnOffPump();
void verifySIMConnected();
void latchRelaySafetySetup();
int readAverage(uint8_t pin);
void safeFlushAfterReset();
void latchRelaySafetyLoop();
void printCPUTemperature();
void processVibrations(int& vibrationCount, int& EventCountTotal);
bool getNetworkHHMMSS(char* hms, size_t cap);
bool modemAlivePing();
static inline void applyBootConfig();
void diagSnapshot(const char* label);
static inline void servicePumpEdgeAndStatus();

// NEW: ENSURE AT (MODEM ON) BEFORE CFUN/CNACT WHEN CNACT? RETURNS 0 bytes
static bool ensureATorPowerCycle(uint8_t atAttempts = 3);

// PROTECTION HELPERS
static inline bool isPumpOn();
static inline void armStartProtection(const char* motivo);
static inline void serviceStartProtection();

bool syncTimeUTC_viaCNTP(uint8_t timeZone, const char* server, uint32_t waitTotalMs);
bool syncTimeUTC_any(uint8_t timeZone, uint32_t waitTotalMs);


int getCSQ_RSSI();       // returns 0..31; -1 if unknown
int csqToDbm(int rssi);  // useful optional: converts CSQ to dBm

// FUNCTION TO READ VOLTAGES
float readBatteryVolts();

// FUNCTION TO SEE IF THERE IS MAIN POWER PRESENT
static inline bool isPwrPresent();

// Mandatory ISR to manage interruptions of SERCOM0

void SERCOM0_Handler() {
  RS485.IrqHandler();
}

// Parsing of RS485 data

void READING_PARAM(uint32_t PARAM);
uint16_t RTU_Vcrc(uint8_t RTU_Buff[], uint16_t RTU_Leng);

bool PARSING_PARAM(uint32_t data[1]);
float get_param(uint32_t regAddr);

void RS485_loop();


// ----------------- SETUP -----------------
void setup() {
  WDT_INIT();
  delay(200);
  pinMode(PWR_ON_PIN, OUTPUT);
  pinMode(PWR_OFF_PIN, OUTPUT);
  pinMode(MODEM_PWR_PIN, OUTPUT);
  pinMode(CONTACT_PIN, INPUT);

  pinMode(BATT_VOLTS, INPUT);

  pinMode(Input_Supply_V, INPUT);

  // Initializing RS485
  Serial.begin(115200);
  DEBUG_PRINTLN(F("Initializing RS485..."));
  pinPeripheral(RS485_RX_PIN, PIO_SERCOM_ALT);
  pinPeripheral(RS485_TX_PIN, PIO_SERCOM_ALT);
  RS485.begin(9600);
  delay(100);


  sim7070.begin(115200);

  tempSAMD.init();
  analogReadResolution(12);

  // PUMP INITIAL STATE
  prevPumpOn = isPumpOn();

  // POWER INITIAL STATE (MAIN LINE)
  prevPwrOn = isPwrPresent();

  DEBUG_PRINTLN(F("Production_Node_Nano33IoT (115200 bps, CID=1)"));
  delay(500);
  WDT_RESET();

  DEBUG_PRINTLN(F("Verifying communication with the SIM7070G"));
  verifySIMConnected();
  setGetCCLKResponse(mainGetCCLKResponse);

  applyBootConfig();

  DEBUG_PRINTLN(F("Initializing SIM7070G..."));

  gprsConnect();

  delay(1000);

  WDT_RESET();

  snprintf(topic_sub, sizeof(topic_sub), "xtr/server/%s", NODE_ID);
  snprintf(topic_pub, sizeof(topic_pub), "xtr/nodes/%s", NODE_ID);
  snprintf(clientID, sizeof(clientID), "%s", NODE_ID);

  snprintf(mqtt_username, sizeof(mqtt_username), "%s", USERNAME);
  snprintf(mqtt_password, sizeof(mqtt_password), "%s", PASSWORD);

  mqttConnect();

  WDT_RESET();
  DEBUG_PRINTLN("Leaving Setup");
}

// ----------------- LOOP -----------------
void loop() {

  WDT_RESET();
  serviceStartProtection();

  handleMQTTMessages();

  checkConnections();

  servicePumpEdgeAndStatus();

  //DEBUG_PRINT("Pump On?: ");
  //DEBUG_PRINTLN(isPumpOn());

  {
    unsigned long now = millis();
    if ((long)(now - lastPublish) >= (long)publishIntervalMs) {
      if (mqttConnected) {
        RS485_loop();
        if (publishMessage()) {
          lastPublish = now;  // We advance to timer only if we have published
        }
      } else {
        lastPublish = now;
        DEBUG_PRINTLN(F("Skipping publication: MQTT not connected."));
      }
    }
  }
}


// ----------------- HANDLERS -----------------
void handleMQTTMessages() {
  static char incoming[600];
  static byte idx = 0;

  auto secsFromHMS = [](int H, int M, int S) -> int {
    if (H < 0) H = 0;
    if (H > 23) H = 23;
    if (M < 0) M = 0;
    if (M > 59) M = 59;
    if (S < 0) S = 0;
    if (S > 59) S = 59;
    return H * 3600 + M * 60 + S;
  };

  auto circDiffSecs = [&](int a, int b) -> int {
    const int DAY = 86400;
    int d = abs(a - b);
    if (d > DAY) d %= DAY;
    return min(d, DAY - d);
  };

  while (sim7070.available()) {
    char c = sim7070.read();
    noteUartActivity();
    if (c == '\n' || c == '\r') {
      incoming[idx] = '\0';
      if (idx > 0) {
        if (strncmp(incoming, "+SMSUB", 6) == 0) {
          char* payload = strchr(incoming, ',');
          if (payload) payload = strchr(payload + 1, '"');
          if (payload) {
            payload++;
            char* end = strchr(payload, '"');
            if (end) *end = '\0';

            DEBUG_PRINT(F("[RX] HEX payload (raw), len="));
            DEBUG_PRINTLN(strlen(payload));
            DEBUG_PRINTLN(payload);

            const char* hexStr = payload;
            size_t hexLen = strlen(hexStr);
            if ((hexLen % 2) == 0 && hexLen >= (12 + 16) * 2) {
              uint8_t buf[480];
              size_t bytes = 0;
              if (hexDecode(hexStr, buf, sizeof(buf), bytes) && bytes >= (12 + 16)) {
                uint8_t* nonce = buf;
                uint8_t* tag = buf + (bytes - 16);
                uint8_t* ct = buf + 12;
                size_t ctLen = bytes - 12 - 16;

                int nH = nonce[4];
                int nM = nonce[5];
                int nS = nonce[6];
                int secMsg = secsFromHMS(nH, nM, nS);

                // === Multiple tries to obtain ISO time ===
                char isoNow[21] = { 0 };
                int rH = 0, rM = 0, rS = 0;
                bool timeOk = false;

                for (uint8_t attempt = 1; attempt <= 5; attempt++) {
                  DEBUG_PRINT(F("[TIME] Try number "));
                  DEBUG_PRINT(attempt);
                  DEBUG_PRINTLN(F(" from 3 to obtain ISO time..."));

                  memset(isoNow, 0, sizeof(isoNow));
                  if (getNetworkTimeISO8601(isoNow, sizeof(isoNow))) {
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
                  WDT_RESET();
                }

                if (!timeOk) {
                  DEBUG_PRINTLN(F("[TIME] Unable to obtain hour from network after 3 tries. Command discarded."));
                  idx = 0;
                  continue;
                }

                // === TIME OBTAINED WITH SUCCESS ===
                const int secNow = secsFromHMS(rH, rM, rS);
                const int dsec = circDiffSecs(secMsg, secNow);

                DIAG_PRINT(F(" | NET HMS="));
                DIAG_PRINT(isoNow + 11);
                DIAG_PRINT(F(" | Δs="));
                DIAG_PRINTLN(dsec);

                // === DECIPHER → clear JSON ===
                char plain[420];
                bool ok = decryptPayload(ct, ctLen, tag, nonce, plain, sizeof(plain));
                DEBUG_PRINT(F("[DECRYPT] Result: "));
                DEBUG_PRINTLN(ok ? F("OK") : F("FAIL"));
                if (!ok) {
                  idx = 0;
                  continue;
                }

                DEBUG_PRINT(F("[RX] clear JSON: "));
                DEBUG_PRINTLN(plain);

                // Window of 30 s
                const int WINDOW_SEC = 30;
                if (dsec > WINDOW_SEC) {
                  DEBUG_PRINTLN(F("[TIME] Command outside of time window (>30 s). Ignored."));
                  idx = 0;
                  if (publishMessage()) lastPublish = millis();
                  continue;
                }

                // === PROCESS X FIELD ===
                long xVal = -1;
                if (jsonGetInt(plain, "X", xVal)) {
                  if (xVal == 1) {
                    if (protectOnStart) {
                      unsigned long elapsed = millis() - protectStartMs;
                      DEBUG_PRINT(F("[ACT] ON command arrived, ignored for protectio ("));
                      DEBUG_PRINT(elapsed / 1000UL);
                      DEBUG_PRINTLN(F(" s)."));
                      if (publishMessage()) lastPublish = millis();
                    } else {
                      serverOnCommand = true;
                      turnOnPump();
                    }
                  } else if (xVal == 0) {
                    serverOnCommand = false;
                    lastOffByHexFlag = true;
                    turnOffPump();
                    armStartProtection("Turned off by HEX command");
                  } else {
                    DEBUG_PRINT(F("[ACT] Invalid X Value: "));
                    DEBUG_PRINTLN(xVal);
                  }
                } else {
                  DEBUG_PRINTLN(F("[ACT] JSON without X field."));
                }

                long newPulse;
                if (jsonGetInt(plain, "onPulse", newPulse)) {
                  onPulse = constrain((int)newPulse, 500, 10000);
                  DEBUG_PRINT(F("[ACT] onPulse updated to "));
                  DEBUG_PRINTLN(onPulse);
                }

              } else {
                DEBUG_PRINTLN(F("[RX] Invalid HEX HEX (deciphering o length)."));
              }
            } else {
              DEBUG_PRINTLN(F("[RX] Payload does not appear to be a valid HEX value or is too short."));
            }
          }
        } else if (DIAG) {
          DIAG_PRINT("[URC] ");
          DIAG_PRINTLN(incoming);
        }
      }
      idx = 0;
    } else if (idx < sizeof(incoming) - 1) {
      incoming[idx++] = c;
    }
  }
}



// ----------------- ON PROTECTION: HELPERS -----------------
// CONTACT_PIN HIGH = ON, LOW = OFF
// 4 READINGS WITH 20 ms; 4/4 HIGH => ON, 4/4 LOW => OFF, MIXTURES => KEEPS PREVIOUS STATE
static inline bool isPumpOn() {
  byte highs = 0;
  highs += (digitalRead(CONTACT_PIN) == HIGH);
  delay(20);
  WDT_RESET();
  highs += (digitalRead(CONTACT_PIN) == HIGH);
  delay(20);
  WDT_RESET();
  highs += (digitalRead(CONTACT_PIN) == HIGH);
  delay(20);
  WDT_RESET();
  highs += (digitalRead(CONTACT_PIN) == HIGH);

  if (highs == 4) return true;   // on
  if (highs == 0) return false;  // off
  return prevPumpOn;             // ambiguous zone: keep state
}

static inline void armStartProtection(const char* reason) {
  protectOnStart = true;
  protectStartMs = millis();
  protectLastAnnounce = 0;
  DEBUG_PRINTLN(F(">>> Time of pump on-protection ACTIVATED (120 s)"));
  if (reason) {
    DEBUG_PRINT(F("Reason: "));
    DEBUG_PRINTLN(reason);
  }
}

static inline void serviceStartProtection() {
  if (!protectOnStart) return;

  unsigned long elapsed = millis() - protectStartMs;
  if (elapsed >= PROTECT_ON_MS) {
    protectOnStart = false;
    DEBUG_PRINTLN(F(">>> Pump on-protection finalized (120 s fulfilled)."));
    return;
  }

  if (millis() - protectLastAnnounce >= PROTECT_TICK_MS) {
    protectLastAnnounce = millis();
    DEBUG_PRINT(F("Time of on-protection of pump: "));
    DEBUG_PRINT(elapsed / 1000UL);
    DEBUG_PRINTLN(F(" s"));
  }
}

// MODEM CONFIGURATION
static inline void applyBootConfig() {
  //sendAT("ATE0", 150);
  // sendAT("AT+CMEE=2", 120);
  sendAT("AT+CSCLK=0", 150);
}

// ----------------- DIAGNOSTIC -----------------
void diagSnapshot(const char* label) {
  if (!DIAG) return;
  DIAG_PRINT(F("[DIAG] "));
  DIAG_PRINT(label);
  DIAG_PRINT(F(" | t="));
  DIAG_PRINT(millis());
  DIAG_PRINT(F("ms | avail="));
  DIAG_PRINT(sim7070.available());
  DIAG_PRINT(F(" | ATstreak="));
  DIAG_PRINT(atFailStreak);
  DIAG_PRINT(F(" | GPRSstreak="));
  DIAG_PRINT(gprsFailStreak);
  DIAG_PRINT(F(" | MQTTstreak="));
  DIAG_PRINT(mqttFailStreak);
  DIAG_PRINT(F(" | since ATok="));
  DIAG_PRINT(millis() - lastATokMs);
  DIAG_PRINT(F(" | since GPRSk="));
  DIAG_PRINT(millis() - lastGPRSokMs);
  DIAG_PRINT(F(" | since MQTTk="));
  DIAG_PRINT(millis() - lastMQTTokMs);
  DIAG_PRINTLN("");
}

// ----------------- HEALTH/RECONNECTIONS -----------------
void checkConnections() {
  if ((long)(millis() - lastCheck) < (long)CHECK_INTERVAL_MS) return;

  if (!canRunHealthCheck()) {
    if (DIAG) DIAG_PRINTLN(F("[DIAG] checkConnections posponed: recent UART activity"));
    return;
  }
  /*
  unsigned long t0 = millis();
  while (millis() - t0 < 40) {
    while (sim7070.available()) { (void)sim7070.read(); }
    DEBUG_PRINTLN("The SIM7070G has been emptied");
    delay(1);
  }
*/
  if (!canRunHealthCheck()) {
    if (DIAG) DIAG_PRINTLN(F("[DIAG] posponed after soft-drain"));
    return;
  }

  lastCheck = millis();

  bool atOk = false;
  for (byte attempt = 0; attempt < 3; attempt++) {
    if (modemAlivePing()) {
      atOk = true;
      break;
    }
    DEBUG_PRINT(F("Without 'AT' response. Short retry... (try number: "));
    DEBUG_PRINT(attempt + 1);
    DEBUG_PRINTLN(F(")..."));
    delay(180);
    WDT_RESET();
  }

  if (!atOk) {
    atFailStreak++;
    DEBUG_PRINT(F("AT fail streak = "));
    DEBUG_PRINTLN(atFailStreak);

    if ((millis() - lastATokMs) < RECENT_OK_GRACE_MS && atFailStreak < AT_FAILS_BEFORE_RESTART) {
      DEBUG_PRINTLN(F("Recent Ok and few failures: DO NOT restart (grace period)."));
      return;
    }
    if (atFailStreak < AT_FAILS_BEFORE_RESTART) {
      DEBUG_PRINTLN(F("AT failed under threshold. Will try later."));
      return;
    }
    if (millis() - lastRestartMs < MIN_RESTART_GAP_MS) {
      DEBUG_PRINTLN(F("Restart blocked by debounce. Waiting for next cycle."));
      return;
    }

    DEBUG_PRINTLN(F("Modem not responding consistently. Restarting..."));
    modemRestart();
    atFailStreak = 0;
    if (!gprsConnect()) return;
    mqttConnect();
    return;
  } else {
    atFailStreak = 0;
    lastATokMs = millis();
  }
  if (!gprsIsConnected()) {
    DEBUG_PRINTLN(F("GPRS not connected. Skipping MQTT check and attempting GPRS reconnection..."));
    if (gprsConnect()) {
      mqttConnect();
    }
    return;
  }
  {
    bool mqttOk = false;

    if (checkMQTTConnection()) {
      mqttOk = true;
    } else {
      DEBUG_PRINTLN(F("MQTT NOT connected. Trying to reconnect up to 3 times..."));
      for (byte attempt = 0; attempt < 3; attempt++) {
        mqttConnect();
        if (mqttConnected) {
          mqttOk = true;
          break;
        }
        delay(900);
        WDT_RESET();
      }
    }

    if (mqttOk) {
      mqttFailStreak = 0;
      lastMQTTokMs = millis();
      if (DIAG) diagSnapshot("MQTT OK (we ommit GPRS)");
      return;
    }

    DEBUG_PRINTLN(F("MQTT still disconnected after tries. Verifying GPRS..."));
  }

  {
    bool gprsOk = false;
    for (byte attempt = 0; attempt < 3; attempt++) {
      DEBUG_PRINT(F("Verifying GPRS (try number: "));
      DEBUG_PRINT(attempt + 1);
      DEBUG_PRINTLN(F(")..."));

      if (gprsIsConnected()) {
        gprsOk = true;
        break;
      }
      DEBUG_PRINTLN(F("GPRS disconnected. Trying to reconnect..."));
      if (gprsConnect()) {
        gprsOk = true;
        break;
      }

      delay(700);
      WDT_RESET();
    }

    if (!gprsOk) {
      gprsFailStreak++;
      DEBUG_PRINT(F("GPRS fail streak = "));
      DEBUG_PRINTLN(gprsFailStreak);

      if ((millis() - lastGPRSokMs) < RECENT_OK_GRACE_MS && gprsFailStreak < GPRS_FAILS_BEFORE_RESTART) {
        DEBUG_PRINTLN(F("GPRS failed but it was OK recently. Do not restart."));
        return;
      }
      if (gprsFailStreak < GPRS_FAILS_BEFORE_RESTART) {
        DEBUG_PRINTLN(F("GPRS failed under threshold. Retry later."));
        return;
      }
      if (millis() - lastRestartMs < MIN_RESTART_GAP_MS) {
        DEBUG_PRINTLN(F("GPRS failed; restart blocked by debounce."));
        return;
      }

      DEBUG_PRINTLN(F("GPRS failed consistently. Restarting modem..."));
      modemRestart();
      gprsFailStreak = 0;
      if (!gprsConnect()) return;

      DEBUG_PRINTLN(F("GPRS OK after restart. Retrying MQTT..."));
      mqttConnect();
      if (mqttConnected) {
        mqttFailStreak = 0;
        lastMQTTokMs = millis();
      } else {
        mqttFailStreak++;
      }
      return;
    } else {
      gprsFailStreak = 0;
      lastGPRSokMs = millis();
    }
  }

  {
    if (!mqttConnected) {
      DEBUG_PRINTLN(F("GPRS OK. Retrying MQTT after GPRS..."));
      for (byte attempt = 0; attempt < 3; attempt++) {
        mqttConnect();
        if (mqttConnected) {
          mqttFailStreak = 0;
          lastMQTTokMs = millis();
          return;
        }
        delay(700);
        WDT_RESET();
      }

      mqttFailStreak++;
      DEBUG_PRINT(F("MQTT fail streak = "));
      DEBUG_PRINTLN(mqttFailStreak);

      if ((millis() - lastMQTTokMs) < RECENT_OK_GRACE_MS && mqttFailStreak < MQTT_FAILS_BEFORE_RESTART) {
        DEBUG_PRINTLN(F("MQTT failed but it was OK recently. Do not restart."));
        return;
      }
      if (mqttFailStreak < MQTT_FAILS_BEFORE_RESTART) {
        DEBUG_PRINTLN(F("MQTT failed under threshold. Retry later."));
        return;
      }
      if (millis() - lastRestartMs < MIN_RESTART_GAP_MS) {
        DEBUG_PRINTLN(F("MQTT failed; restart blocked by debounce."));
        return;
      }

      DEBUG_PRINTLN(F("MQTT consistently. Restarting modem..."));
      modemRestart();
      mqttFailStreak = 0;
      if (!gprsConnect()) return;
      mqttConnect();
      return;
    } else {
      mqttFailStreak = 0;
      lastMQTTokMs = millis();
      return;
    }
  }
}


// ----------------- MQTT/GPRS -----------------
bool publishMessage() {
  if (!mqttConnected) return false;

  int V1 = readAverage(ADC_PIN_1);
  int V2 = readAverage(ADC_PIN_2);
  int V3 = readAverage(ADC_PIN_3);

  // CSQ (0..31; if fails, 0)
  int csq = getCSQ_RSSI();
  int CSQ = (csq < 0) ? 0 : csq;

  // Battery (V -> %)
  float Bv_raw = readBatteryVolts();
  float Bv = roundf(Bv_raw * 100.0f) / 100.0f;
  const float V_MIN = 3.00f, V_MAX = 4.05f;
  int B = (int)lroundf(constrain((Bv - V_MIN) * (100.0f / (V_MAX - V_MIN)), 0.0f, 100.0f));

  float cpuC = tempSAMD.readInternalTemperature();
  float cpuC_final = isnan(cpuC) ? -127.0f : cpuC;

  // States
  pump = isPumpOn() ? 1 : 0;         // X
  int P = protectOnStart ? 1 : 0;    // Protection of pump restart
  int PWR = isPwrPresent() ? 1 : 0;  // Main pwer line present with debounce
  DEBUG_PRINT("PIN Voltage: ");
  DEBUG_PRINTLN((digitalRead(Input_Supply_V)));

  // === Publish rate with PWR conditional ===
  if (PWR == 1) {
    // With main power line, always publish fast
    if (publishIntervalMs != PUB_FAST_MS) {
      publishIntervalMs = PUB_FAST_MS;
      DEBUG_PRINT(F("[RATE] Forced to fast publication because PWR=1: "));
      DEBUG_PRINTLN(publishIntervalMs);
    }
  } else {
    // With battery power (PWR=0): apply hysteresis by lever battery
    if (publishIntervalMs == PUB_FAST_MS && B < BAT_THRESH_PCT) {
      publishIntervalMs = PUB_SLOW_MS;
      DEBUG_PRINT(F("[RATE] Slow publish because of low battery and PWR=0: "));
      DEBUG_PRINTLN(publishIntervalMs);
    } else if (publishIntervalMs == PUB_SLOW_MS && B >= BAT_THRESH_PCT + BAT_HYST_PCT) {
      publishIntervalMs = PUB_FAST_MS;
      DEBUG_PRINT(F("[RATE] Returns to fast publish because battery level is restored (PWR=0): "));
      DEBUG_PRINTLN(publishIntervalMs);
    }
  }
  // RS485 values already updated by RS485_loop() right before publishMessage()
  float FLOW = parameters[0];  // current flow
  float TOT = parameters[1];   // cumulative

  // AT_UTC
  char tsZ[21] = { 0 };  // "YYYY-MM-DDTHH:MM:SSZ"
  bool haveTs = getNetworkTimeISO8601(tsZ, sizeof(tsZ));

  // JSON (without HMS)
  char plain[420];
  if (haveTs) {
    snprintf(plain, sizeof(plain),
             "{\"V1\":%d,\"V2\":%d,\"V3\":%d,"
             "\"X\":%d,\"P\":%d,\"B\":%d,\"CSQ\":%d,"
             "\"TCPU\":%.1f,"
             "\"FLOW\":%.3f,\"TOT\":%.1f,"
             "\"RSIM\":%lu,\"RGPRS\":%lu,\"RMQTT\":%lu,"
             "\"PWR\":%d,\"AT_UTC\":\"%s\"}",
             V1, V2, V3,
             pump, P, B, CSQ,
             cpuC_final,
             FLOW, TOT,
             (unsigned long)cntModemRestarts,
             (unsigned long)cntGprsConnects,
             (unsigned long)cntMqttConnects,
             PWR, tsZ);
  } else {
    snprintf(plain, sizeof(plain),
             "{\"V1\":%d,\"V2\":%d,\"V3\":%d,"
             "\"X\":%d,\"P\":%d,\"B\":%d,\"CSQ\":%d,"
             "\"TCPU\":%.1f,"
             "\"FLOW\":%.3f,\"TOT\":%.1f,"
             "\"RSIM\":%lu,\"RGPRS\":%lu,\"RMQTT\":%lu,"
             "\"PWR\":%d,\"AT_UTC\":null}",
             V1, V2, V3,
             pump, P, B, CSQ,
             cpuC_final,
             FLOW, TOT,
             (unsigned long)cntModemRestarts,
             (unsigned long)cntGprsConnects,
             (unsigned long)cntMqttConnects,
             PWR);
  }

  DEBUG_PRINT(F("[TX] Clear JSON to be published: "));
  DEBUG_PRINTLN(plain);

  // CIPHER AND PUBLISH
  uint8_t ct[420], tag[16], nonce[12];
  makeNonce12(nonce);
  size_t ctLen = encryptPayload(plain, ct, tag, nonce);

  char hexPayload[(12 + 420 + 16) * 2 + 1];
  size_t o = 0;
  hexEncode(nonce, 12, hexPayload + o, sizeof(hexPayload) - o, true);
  o += 12 * 2;
  hexEncode(ct, ctLen, hexPayload + o, sizeof(hexPayload) - o, true);
  o += ctLen * 2;
  hexEncode(tag, 16, hexPayload + o, sizeof(hexPayload) - o, true);
  o += 16 * 2;
  hexPayload[o] = '\0';

  DEBUG_PRINTLN("Published HEX: ");
  DEBUG_PRINTLN(hexPayload);

  int len = strlen(hexPayload);
  sim7070.print(F("AT+SMPUB=\""));
  sim7070.print(topic_pub);
  sim7070.print(F("\","));
  sim7070.print(len);
  sim7070.println(F(",1,0"));
  noteUartActivity();
  delay(120);
  sim7070.print(hexPayload);
  noteUartActivity();
  delay(350);
  noteUartActivity();

  EventCountTotal = 0;
  return true;
}


bool checkMQTTConnection() {
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

void mqttConnect() {

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


// ====== NEW: Ensure AT before CFUN/CNACT when CNACT? returns 0 bytes ======
static bool ensureATorPowerCycle(uint8_t atAttempts) {
  for (uint8_t i = 0; i < atAttempts; i++) {
    if (modemAlivePing()) {
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
  modemRestart();

  unsigned long t0 = millis();
  while (millis() - t0 < 20000UL) {
    if (modemAlivePing()) {
      DEBUG_PRINTLN(F("AT OK after power cycle."));
      return true;
    }
    delay(200);
    WDT_RESET();
  }

  DEBUG_PRINTLN(F("AT not available after power cycle."));
  return false;
}

bool gprsConnect() {
  DEBUG_PRINTLN(F("Verifying GPRS state..."));


  char response[160] = { 0 };

  // ============================================
  // 1) READ + PRINT TIME BEFORE ANYTHING ELSE
  // ============================================
  {
    char cclkResp[128] = { 0 };
    char iso[32] = { 0 };

    DEBUG_PRINTLN(F("[GPRS] Getting CCLK before PDP..."));
    sendCommandGetResponse("AT+CCLK?", cclkResp, sizeof(cclkResp), 2000);

    DEBUG_PRINT(F("[GPRS] Raw CCLK response: "));
    DEBUG_PRINTLN(cclkResp);

    if (parseCCLKToISO(cclkResp, iso, sizeof(iso))) {
      DEBUG_PRINT(F("[GPRS] Parsed UTC time: "));
      DEBUG_PRINTLN(iso);  // Example: 2025-12-16T22:15:30Z
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


    syncTimeUTC_any(0, 12000UL);

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
  sendAT("AT+CGDCONT=1,\"IP\",\"" PDP_APN "\"", simWait);
  sendAT("AT+CNMP=2", simWait);    // auto
  sendAT("AT+CSOCKSETPN=1", 200);  // reinforcement

  for (uint8_t attempt = 0; attempt < 2; attempt++) {
    DEBUG_PRINT(F("Activating data context (try number: "));
    DEBUG_PRINT(attempt + 1);
    DEBUG_PRINTLN(F(")..."));

    if (!modemAlivePing()) {
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

      syncTimeUTC_any(0, 12000UL);

      return true;
    }

    WDT_RESET();
    delay(1000);
  }

  DEBUG_PRINTLN(F("Error: Unable to connect to data context (wihtout recursion)."));
  return false;
}

bool gprsIsConnected() {
  char response[120];
  sendCommandGetResponse("AT+CNACT?", response, sizeof(response), 1500);
  bool ok = strstr(response, "+CNACT: 1,1") != NULL;
  if (ok) lastGPRSokMs = millis();
  return ok;
}

// ----------------- AT HELPERS -----------------
void sendAT(const char* command, unsigned long wait) {
  unsigned long t0 = millis();
  while (millis() - t0 < 20) {
    while (sim7070.available()) {
      (void)sim7070.read();
      noteUartActivity();
    }
    delay(1);
  }

  sim7070.println(command);
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

bool syncTimeUTC_viaCNTP(uint8_t timeZone, const char* server, uint32_t waitTotalMs) {
  char buf[128] = { 0 };
  char cmd[96];
  char timeBefore[160] = { 0 };
  char timeAfter[160] = { 0 };

  DEBUG_PRINT(F("[NTP] Synchronizing with server: "));
  DEBUG_PRINTLN(server);

  sendCommandGetResponse("AT+CCLK?", timeBefore, sizeof(timeBefore), 5000);

  snprintf(cmd, sizeof(cmd), "AT+CNTP=\"%s\",%d", server, (int)timeZone);
  sendCommandGetResponse(cmd, buf, sizeof(buf), 5000);
  if (strstr(buf, "OK") == nullptr) {
    DEBUG_PRINTLN(F("[NTP] Failed to configure NTP server"));
    return false;
  }

  sendCommandGetResponse("AT+CNTP", buf, sizeof(buf), 5000);
  if (strstr(buf, "OK") == nullptr) {
    DEBUG_PRINTLN(F("[NTP] Failed to trigger NTP sync"));
    return false;
  }

  bool ok = false;
  bool explicitFailure = false;
  unsigned long t0 = millis();

  while (millis() - t0 < waitTotalMs) {
    memset(buf, 0, sizeof(buf));
    sendCommandGetResponse("AT+CNTP?", buf, sizeof(buf), 900);
    if (strstr(buf, "OK") == nullptr) {
      delay(600);
      WDT_RESET();
      continue;
    }

    const char* cntpPtr = strstr(buf, "+CNTP:");
    if (cntpPtr) {
      cntpPtr += 6;
      while (*cntpPtr == ' ' || *cntpPtr == '\t') cntpPtr++;

      int simpleStatus = -1;
      if (sscanf(cntpPtr, "%d", &simpleStatus) == 1) {
        if (simpleStatus == 1) {
          ok = true;
          DEBUG_PRINTLN(F("[NTP] Sync successful! (simple format)"));
          break;
        }
        if (simpleStatus == 0) {
          explicitFailure = true;
          DEBUG_PRINTLN(F("[NTP] Explicit failure (+CNTP: 0)"));
          break;
        }
      } else {
        char serverName[64];
        int tz = -1, offset = 0, mode = -1;
        if (sscanf(cntpPtr, "\"%63[^\"]\",%d,%d,%d", serverName, &tz, &offset, &mode) == 4 ||
            sscanf(cntpPtr, "%63[^,],%d,%d,%d", serverName, &tz, &offset, &mode) == 4) {
          if (mode == 2) {
            ok = true;
            DEBUG_PRINTLN(F("[NTP] Sync successful! (mode=2)"));
            break;
          }
          if (mode != 0) {
            explicitFailure = true;
            DEBUG_PRINT(F("[NTP] Error mode: "));
            DEBUG_PRINTLN(mode);
            break;
          }
        }
      }
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

  sendCommandGetResponse("AT+CCLK?", timeAfter, sizeof(timeAfter), 1200);
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

bool syncTimeUTC_any(uint8_t timeZone, uint32_t waitTotalMs) {
  const char* servers[] = {
    "time.google.com",
    "time.cloudflare.com",
    "pool.ntp.org",
  };
  const size_t nServers = sizeof(servers) / sizeof(servers[0]);

  for (size_t i = 0; i < nServers; i++) {
    DEBUG_PRINT(F("[NTP] Trying server: "));
    DEBUG_PRINTLN(servers[i]);

    if (syncTimeUTC_viaCNTP(timeZone, servers[i], waitTotalMs)) {
      DEBUG_PRINT(F("[NTP] Time synchronized using: "));
      DEBUG_PRINTLN(servers[i]);
      return true;
    }
  }

  DEBUG_PRINTLN(F("[NTP] All NTP servers failed. Keeping previous clock / network time."));
  return false;
}


bool sendATwaitOK(const char* cmd, char* out, size_t outCap, unsigned long overallMs) {
  if (outCap == 0) return false;
  out[0] = '\0';

  unsigned long t0 = millis();
  while (millis() - t0 < 20) {
    while (sim7070.available()) {
      (void)sim7070.read();
      noteUartActivity();
    }
    delay(1);
  }

  if (DIAG) {
    DIAG_PRINT(F("[DIAG] sendATwaitOK avail-before="));
    DIAG_PRINTLN(sim7070.available());
  }

  sim7070.println(cmd);
  noteUartActivity();

  unsigned long tstart = millis();
  size_t w = 0;
  bool sawOK = false, sawERROR = false;

  while (millis() - tstart < overallMs) {
    while (sim7070.available()) {
      char c = sim7070.read();
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

void flushSIMBuffer() {
  while (sim7070.available()) (void)sim7070.read();
}

void sendCommandGetResponse(const char* command, char* buffer, size_t bufferSize, unsigned long timeout) {
  if (bufferSize == 0) return;
  buffer[0] = '\0';
  /*
  unsigned long t0 = millis();
  while (millis() - t0 < 20) {
    while (sim7070.available()) {
      (void)sim7070.read();
      noteUartActivity();
    }
    delay(1);
  }
*/
  if (DIAG) {
    DIAG_PRINT(F("[DIAG] sendCmd avail-before="));
    DIAG_PRINT(sim7070.available());
    DIAG_PRINT(F(" cmd="));
    DIAG_PRINTLN(command);
  }

  sim7070.println(command);
  noteUartActivity();

  unsigned long start = millis();
  size_t idx = 0;
  bool sawTerminator = false;

  while (millis() - start < timeout && idx < bufferSize - 1) {
    while (sim7070.available() && idx < bufferSize - 1) {
      char c = sim7070.read();
      noteUartActivity();
      buffer[idx++] = c;

      if (c == '\n') {
        buffer[idx] = '\0';
        if (strstr(buffer, "\nOK") || strstr(buffer, "OK\r") || strstr(buffer, "OK\n") || strstr(buffer, "ERROR") || strstr(buffer, "+CME ERROR")) {
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

static bool mainGetCCLKResponse(char* buf, size_t cap) {
  sendCommandGetResponse("AT+CCLK?", buf, cap, 1500);
  return strstr(buf, "+CCLK:") != nullptr;
}

void safeFlushAfterReset() {
  unsigned long t0 = millis();
  while (millis() - t0 < 200) {
    while (sim7070.available()) {
      (void)sim7070.read();
      noteUartActivity();
    }
    delay(5);
  }
}

void readResponse() {
  receivedMessage = "";
  unsigned long timeout = millis() + readResponseWait;
  while (millis() < timeout) {
    while (sim7070.available()) {
      char c = sim7070.read();
      noteUartActivity();
      receivedMessage += c;
      if (receivedMessage.length() > 550) break;
    }
    WDT_RESET();
  }
  if (receivedMessage.length() > 0) {
    DEBUG_PRINTLN(receivedMessage);
    receivedMessage = "";
  }
}


int getCSQ_RSSI() {
  char buf[128] = { 0 };

  // Use the helper that reads the reponse and detects "OK"
  if (!sendATwaitOK("AT+CSQ", buf, sizeof(buf), 1500)) {
    return -1;  // there was no OK
  }

  // Looks for the line +CSQ: <rssi>,<ber>
  const char* p = strstr(buf, "+CSQ:");
  if (!p) return -1;

  int rssi = 99, ber = 0;
  if (sscanf(p, "+CSQ: %d,%d", &rssi, &ber) != 2) return -1;

  if (rssi == 99) return -1;  // 99 = unknown
  if (rssi < 0) rssi = 0;
  if (rssi > 31) rssi = 31;
  return rssi;  // 0..31
}



// ----------------- EQUIPMENT CONTROL -----------------
void turnOffPump() {
  if (!isPumpOn()) {
    DEBUG_PRINTLN(F("Pump is already off."));
    if (publishMessage()) lastPublish = millis();
    return;
  }
  DEBUG_PRINTLN(F("Turn off equipment (pump)"));
  digitalWrite(PWR_OFF_PIN, HIGH);
  delay(1000);
  digitalWrite(PWR_OFF_PIN, LOW);
  delay(10);
  if (publishMessage()) lastPublish = millis();
}

void turnOnPump() {
  if (protectOnStart) {
    unsigned long elapsed = millis() - protectStartMs;
    DEBUG_PRINT(F("Turn ON request ignored: currently in protection. ("));
    DEBUG_PRINT(elapsed / 1000UL);
    DEBUG_PRINTLN(F(" s)."));
    return;
  }

  WDT_RESET();
  if (isPumpOn()) {
    DEBUG_PRINTLN(F("Pump is already off."));
    return;
  }
  DEBUG_PRINTLN(F("Turn on equipment (pump)"));
  digitalWrite(PWR_ON_PIN, HIGH);
  delay(onPulse);
  WDT_RESET();
  delay(onPulse);
  digitalWrite(PWR_ON_PIN, LOW);
  delay(10);
  if (publishMessage()) lastPublish = millis();
}

// ----------------- MODEM POWER -----------------
void verifySIMConnected() {
  if (modemAlivePing()) {
    DEBUG_PRINTLN(F("SIM7070G responds."));
    return;
  }
  DEBUG_PRINTLN(F("There is no 'OK'. Restarting modem..."));
  modemRestart();
}

void modemRestart() {

  DEBUG_PRINTLN(F("Elegant restart of modem (PWRKEY and verification)…"));
  diagSnapshot("pre-restart");

  pinMode(MODEM_PWR_PIN, OUTPUT);
  digitalWrite(MODEM_PWR_PIN, LOW);

  auto pwrkeyPulse = [&]() {
    digitalWrite(MODEM_PWR_PIN, HIGH);
    delay(PWRKEY_MS);
    digitalWrite(MODEM_PWR_PIN, LOW);
  };

  auto softDrain = [&]() {
    unsigned long t0 = millis();
    while (millis() - t0 < PRE_DRAIN_MS) {
      while (sim7070.available()) (void)sim7070.read();
      delay(1);
      WDT_RESET();
    }
  };

  softDrain();
  if (modemAlivePing()) {
    DEBUG_PRINTLN(F("Modem already responds. No power cycle required."));
    diagSnapshot("no-restart-needed");
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

  if (modemAlivePing()) {
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
    diagSnapshot("post-restart");
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
  diagSnapshot("post-restart");
  applyBootConfig();
  cntModemRestarts++;
}



// === Ping AT suave con drenaje y doble ventana ===
bool modemAlivePing() {
  unsigned long t0 = millis();
  int drained = 0;
  while (millis() - t0 < 30) {
    while (sim7070.available()) {
      (void)sim7070.read();
      DEBUG_PRINTLN("The SIM7070G was emptied SIM7070G Modem Alive Ping");
      noteUartActivity();
      drained++;
    }
    delay(2);
  }

  sim7070.write('\r');
  noteUartActivity();
  delay(40);

  auto tryAT = [&](uint32_t waitMs) -> bool {
    char buf[192] = { 0 };
    size_t idx = 0;
    sim7070.println("AT");
    noteUartActivity();
    unsigned long start = millis();
    while (millis() - start < waitMs) {
      while (sim7070.available()) {
        char c = sim7070.read();
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


// ----------------- USEFUL -----------------
int readAverage(uint8_t pin) {
  long sum = 0;
  for (byte i = 0; i < 3; i++) {
    sum += analogRead(pin);
    delay(10);
  }
  return (sum / 3) * 0.1299;  // 0.08059 para Arduino Uno, 0.1299 para Nano33IoT
}

void latchRelaySafetySetup() {
  if (!isPumpOn()) {
    DEBUG_PRINTLN(F("Initial state: pump OFF. Reset Latch."));
    digitalWrite(PWR_OFF_PIN, HIGH);
    delay(1000);
    digitalWrite(PWR_OFF_PIN, LOW);
    delay(20);
  } else {
    DEBUG_PRINTLN(F("Initial state: pump OFF."));
  }
}

void latchRelaySafetyLoop() {
  if (!isPumpOn() && serverOnCommand == true) {
    DEBUG_PRINTLN("Turning pump OFF for safety (inside loop).");
    digitalWrite(PWR_OFF_PIN, HIGH);
    delay(1000);
    digitalWrite(PWR_OFF_PIN, LOW);
    delay(20);
    serverOnCommand = false;
    if (publishMessage()) lastPublish = millis();
  }
}

void printCPUTemperature() {
  if ((long)millis() - (long)lastTempCheck <= (long)TEMP_CHECK_INTERVAL_MS) return;
  lastTempCheck = millis();

  float cpuC = tempSAMD.readInternalTemperature();
  DEBUG_PRINT("CPU Temp: ");
  if (isnan(cpuC)) {
    DEBUG_PRINTLN("N/A");
  } else {
    DEBUG_PRINT(cpuC);
    DEBUG_PRINTLN(" °C");
  }
}

// ============== "HH:MM:SS" ==============
bool getNetworkHHMMSS(char* hms, size_t cap) {
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

static inline void servicePumpEdgeAndStatus() {
  bool nowOn = isPumpOn();

  // Detection of flank: ON -> OFF (turned off by contact, without HEX)
  if (prevPumpOn && !nowOn) {
    if (!lastOffByHexFlag) {
      armStartProtection("Turned OFF detected by contact (without HEX)");
    } else {
      lastOffByHexFlag = false;
    }
  }

  prevPumpOn = nowOn;

  // X (pump): 1 on, 0 off
  pump = nowOn ? 1 : 0;
}

float readBatteryVolts() {
  long sum = 0;
  for (int i = 0; i < 8; i++) {
    sum += analogRead(BATT_VOLTS);
    delay(2);
  }
  float raw = sum / 8.0f;
  float v_adc = raw * (ADC_VREF / ADC_MAX);               // volts on the pin
  float v_bat = v_adc * ((R_TOP + R_BOTTOM) / R_BOTTOM);  // voltage divider scale
  return v_bat;
}

// ----------------- Presence of main power line (PWR): helpers -----------------
// Input_Supply_V HIGH = there is main power line, LOW = there is no main power line
// 4 readings con 20 ms; 4/4 HIGH => present, 4/4 LOW => missing, mixtures => keeps previous state
static inline bool isPwrPresent() {
  byte highs = 0;
  highs += (digitalRead(Input_Supply_V) == HIGH);
  delay(20);
  WDT_RESET();
  highs += (digitalRead(Input_Supply_V) == HIGH);
  delay(20);
  WDT_RESET();
  highs += (digitalRead(Input_Supply_V) == HIGH);
  delay(20);
  WDT_RESET();
  highs += (digitalRead(Input_Supply_V) == HIGH);

  if (highs == 4) {
    prevPwrOn = true;
    return true;
  }
  if (highs == 0) {
    prevPwrOn = false;
    return false;
  }
  return prevPwrOn;  // ambiguous zone: keep state
}
// -------------------------------------------------------
void RS485_loop() {
  uint32_t param[index_request] = {
    METER_FLOW_RATE_ADDRESS,
    METER_CUMULATIVE_ADDRESS,
    //TOTAL_ACTIVE_POWER_ADDR
  };

  for (int i = 0; i < index_request; i++) {
    READING_PARAM(param[i]);
    delay(50);

    parameters[i] = get_param(param[i]);
    delay(250);
  }

  Serial.println(F("========== READINGS ZM194-D9Y =========="));

  DEBUG_PRINT(F("CURRENT WATER FLOW: "));
  DEBUG_PRINT_F(parameters[0], 3);
  DEBUG_PRINTLN();
  DEBUG_PRINT(F("CUMULATIVE WATER FLOW: "));
  DEBUG_PRINT_F(parameters[1], 1);


  Serial.println(F("=========================================\n"));
}

//---------------------------------------------------------------------
// Send petition to Modbus
//---------------------------------------------------------------------
void READING_PARAM(uint32_t PARAM) {
  messege[0] = MODBUS_REQ.address;
  messege[1] = MODBUS_REQ.function;
  messege[2] = (MODBUS_REQ.startByte_H & PARAM) >> 24;
  messege[3] = (MODBUS_REQ.startByte_L & PARAM) >> 16;
  messege[4] = (MODBUS_REQ.endByte_H & PARAM) >> 8;
  messege[5] = (MODBUS_REQ.endByte_L & PARAM);

  uint16_t crc = RTU_Vcrc(messege, 6);
  messege[6] = crc >> 8;
  messege[7] = crc;

  RS485.write(messege, 8);
  RS485.flush();
  delayMicroseconds(800);

  DEBUG_PRINT(F("TX: "));
  for (int i = 0; i < 8; i++) {
    Serial.print(messege[i], HEX);
    Serial.print(' ');
  }
  Serial.println();
}


//---------------------------------------------------------------------
// CRC MODBUS
//---------------------------------------------------------------------
uint16_t RTU_Vcrc(uint8_t RTU_Buff[], uint16_t RTU_Leng) {
  uint16_t temp = 0xFFFF, temp2, flag;
  for (uint16_t i = 0; i < RTU_Leng; i++) {
    temp ^= RTU_Buff[i];
    for (uint8_t e = 1; e <= 8; e++) {
      flag = temp & 0x0001;
      temp >>= 1;
      if (flag) temp ^= 0xA001;
    }
  }

  temp2 = temp >> 8;
  temp = (temp << 8) | temp2;
  return temp;
}

//---------------------------------------------------------------------
// Parse Modbus response
//---------------------------------------------------------------------
bool PARSING_PARAM(uint32_t data[1]) {
  const uint8_t EXPECT = 9;            // fixed for 0x03, 2 regs
  const uint16_t FIRST_BYTE_TO = 200;  // ms
  const uint16_t INTER_BYTE_TO = 30;   // ms (tune 10..50)
  uint8_t recv[EXPECT];
  uint8_t u = 0;

  // Wait for first byte
  unsigned long t0 = millis();
  while (!RS485.available() && (millis() - t0) < FIRST_BYTE_TO) { /* wait */
  }
  if (!RS485.available()) {
    Serial.println(F("Modbus: no data (first byte timeout)."));
    data[0] = 0;
    return false;
  }

  // Read until EXPECT bytes, using inter-byte timeout
  unsigned long lastByte = millis();
  while (u < EXPECT) {
    while (RS485.available() && u < EXPECT) {
      recv[u++] = RS485.read();
      lastByte = millis();
    }
    if ((millis() - lastByte) > INTER_BYTE_TO) break;  // frame stalled
  }

  Serial.print(F("RX("));
  Serial.print(u);
  Serial.print(F("): "));
  for (int i = 0; i < u; i++) {
    Serial.print(recv[i], HEX);
    Serial.print(' ');
  }
  Serial.println();

  if (u != EXPECT) {
    Serial.println(F("Modbus: incomplete frame."));
    data[0] = 0;
    return false;
  }

  // CRC check (note: your RTU_Vcrc swaps bytes at end, so compare same way)
  uint16_t crcCalc = RTU_Vcrc(recv, EXPECT - 2);
  uint16_t crcResp = ((uint16_t)recv[EXPECT - 2] << 8) | recv[EXPECT - 1];
  if (crcCalc != crcResp) {
    Serial.println(F("Modbus: CRC error."));
    data[0] = 0;
    return false;
  }

  // Basic sanity
  if (recv[0] != MODBUS_REQ.address || recv[1] != MODBUS_REQ.function || recv[2] != (METER_NUM_REGISTERS * 2)) {
    Serial.println(F("Modbus: unexpected header."));
    data[0] = 0;
    return false;
  }

  uint32_t raw =
    ((uint32_t)recv[3] << 24) | ((uint32_t)recv[4] << 16) | ((uint32_t)recv[5] << 8) | ((uint32_t)recv[6]);

  data[0] = raw;
  return true;
}


//---------------------------------------------------------------------
// Escalate value
//---------------------------------------------------------------------
float get_param(uint32_t regAddr) {
  uint32_t raw[1];

  if (!PARSING_PARAM(raw)) {
    return 0.0f;
  }

  float value = 0.0f;

  switch (regAddr) {

    case METER_FLOW_RATE_ADDRESS:
      value = raw[0] / 1000.0f;  // Flow rate scaled
      DEBUG_PRINT(F("[CURRENT] RAW="));
      DEBUG_PRINT(raw[0]);
      DEBUG_PRINT(F("  SCALED="));
      DEBUG_PRINT_F(value, 3);
      DEBUG_PRINTLN("");
      break;

    case METER_CUMULATIVE_ADDRESS:
      value = (float)raw[0];  // Cumulative stays raw
      DEBUG_PRINT(F("[CUMULATIVE] RAW="));
      DEBUG_PRINTLN(raw[0]);
      break;

    default:
      value = (float)raw[0];
      DEBUG_PRINT(F("[UNKNOWN REG] RAW="));
      DEBUG_PRINTLN(raw[0]);
      break;
  }

  return value;
}
