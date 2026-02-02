#include <Arduino.h>
#include <Arduino_LSM6DS3.h>
#include <TemperatureZero.h>
#include <Adafruit_SleepyDog.h>
#include <Crypto.h>
#include <ChaChaPoly.h>
#include <math.h>
#include "wiring_private.h"
#include "config.h"
#include "sim7070g.h"
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

TemperatureZero tempSAMD;

// SIM7070G Modem instance
SIM7070G modem(sim7070, MODEM_PWR_PIN);

// ----------------- PINS -----------------
#define sim7070 Serial1
#define MODEM_PWR_PIN 12  // Pin to turn on/off the modem
#define CONTACT_PIN 20    // Pin to detect whether the pump is on or off
#define PWR_ON_PIN 4      // Pin to turn un pump (on relay)
#define PWR_OFF_PIN 3     // Pin to turn pump off (off relay)

#define ADC_PIN_1 A0       // AC Transducer input (line 1)
#define ADC_PIN_2 A1       // AC Transducer input (line 2)
#define ADC_PIN_3 A2       // AC Transducer input (line 3)
#define BATT_VOLTS A3      // Battery voltage input pin
#define Input_Supply_V 21  // Main power detection pin (detects whether there is power from the main line)

#define RS485_RX_PIN 5  // PIN RX for RS485 module
#define RS485_TX_PIN 6  // PIN TX for RS485 module

// ----------------- PDP CONTEXT -----------------
#define PDP_CID 1
#define PDP_APN "iot.1nce.net"

// ----------------- WATCHDOG -----------------
#define HW_WDT_TIMEOUT_SEC 16
static inline void WDT_INIT() {
  Watchdog.enable(HW_WDT_TIMEOUT_SEC * 1000);
}
static inline void WDT_RESET() {
  Watchdog.reset();
}



// ====== CRYPTO =======
static const char* KEY_HEX = CRYPTOKEY;
static uint8_t CRYPTO_KEY[32];

// ----------------- MQTT STATE -----------------
unsigned long lastPublish = 0;

// Battery/publish constants moved to include/config.h

// PREVIOUS STATE OF THE MAIN POWER LINE ----------
bool prevPwrOn = false;  // it is initialized in setup()

unsigned long publishIntervalMs = PUB_FAST_MS;

// -------------CHECK CONNECTIONS INTERVAL----
unsigned long lastCheck = 0;

byte pump = 0;
int onPulse = ON_PULSE_MS;
bool serverOnCommand = false;

// TIMINGS
// read/sim wait times moved to include/config.h

// -------- MONITOR CPU TEMPERATURE -----------
unsigned long lastTempCheck = 0;

// -------- VIBRATION ---------
int vibrationCount = 0;
int EventCountTotal = 0;

// Health thresholds moved to include/config.h

// PUMP protection timings moved to include/config.h

volatile bool protectOnStart = false;
unsigned long protectStartMs = 0;
unsigned long protectLastAnnounce = 0;

// To distinguish if the pump was turned off via HEX (X=0)
volatile bool lastOffByHexFlag = false;

// To detect "turn off without HEX" by flank
bool prevPumpOn = false;  // It is initialized in setup() after reading the pin

// ADC / battery calibration moved to include/config.h

// ------------- RS 485 ------------------

// We instantiate a UART in SERCOM0
Uart RS485(&sercom0, RS485_RX_PIN, RS485_TX_PIN, SERCOM_RX_PAD_1, UART_TX_PAD_0);

byte messege[256];
uint8_t index_request = 2;
float parameters[64];

// Constant water flow address
#define METER_CURRENT_ADDRESS 0x00000002

// Cumulative water flow address
#define METER_CUMULATIVE_ADDRESS 0x00380002


struct modbus_transmit {
  uint8_t address = 0x01;
  uint8_t function = 0x03;

  uint32_t startByte_H = 0xFF000000;
  uint32_t startByte_L = 0x00FF0000;

  uint16_t endByte_H = 0x0000FF00;
  uint16_t endByte_L = 0x000000FF;

} MODBUS_REQ;



// ----------------- PROTOTYPES -----------------
void handleMQTTMessages();
void checkConnections();
void turnOnPump();
void turnOffPump();
void latchRelaySafetySetup();
int readAverage(uint8_t pin);
void latchRelaySafetyLoop();
void printCPUTemperature();
static inline void makeNonce12(uint8_t nonce[12]);
size_t encryptPayload(const char* plaintext, uint8_t* ciphertext, uint8_t tag[16], const uint8_t nonce12[12]);
bool decryptPayload(const uint8_t* ciphertext, size_t ctLen, const uint8_t tag[16], const uint8_t nonce12[12], char* outPlain, size_t outPlainCap);
void processVibrations(int& vibrationCount, int& EventCountTotal);
void diagSnapshot(const char* label);
static inline void servicePumpEdgeAndStatus();
bool publishMessage();

// PROTECTION HELPERS
static inline bool isPumpOn();
static inline void armStartProtection(const char* motivo);
static inline void serviceStartProtection();

// HEX HELPERS
void hexEncode(const uint8_t* in, size_t inLen, char* out, size_t outCap, bool uppercase = true);
bool hexDecode(const char* hex, uint8_t* out, size_t outCap, size_t& outLen);

// MINIMAL JSON HELPERS
bool jsonGetString(const char* json, const char* key, char* out, size_t cap);
bool jsonGetBool(const char* json, const char* key, bool& out);
bool jsonGetInt(const char* json, const char* key, long& out);

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


  // Initialize SIM7070G modem
  modem.begin(115200);

  tempSAMD.init();
  analogReadResolution(12);

  // --- DECODE HEX KEY TO BYTES (CRYPTO_KEY[32]) ---
  {
    size_t outLen = 0;
    if (!hexDecode(KEY_HEX, CRYPTO_KEY, sizeof(CRYPTO_KEY), outLen) || outLen != 32) {
      DEBUG_PRINTLN(F("ERROR: INVALID CRYPTO KEY (must be 64 hex = 32 bytes)."));
    } else {
      DEBUG_PRINTLN(F("CRYPTO KEY loaded from HEX."));
    }
  }

  // PUMP INITIAL STATE
  prevPumpOn = isPumpOn();

  // POWER INITIAL STATE (MAIN LINE)
  prevPwrOn = isPwrPresent();

  DEBUG_PRINTLN(F("Production_Node_Nano33IoT (115200 bps, CID=1)"));
  delay(500);
  WDT_RESET();

  DEBUG_PRINTLN(F("Verifying communication with the SIM7070G"));
  modem.verifySIMConnected();

  modem.applyBootConfig();

  DEBUG_PRINTLN(F("Initializing SIM7070G..."));

  modem.gprsConnect();

  delay(1000);

  WDT_RESET();

  snprintf(topic_sub, sizeof(topic_sub), "xtr/server/%s", NODE_ID);
  snprintf(topic_pub, sizeof(topic_pub), "xtr/nodes/%s", NODE_ID);
  snprintf(clientID, sizeof(clientID), "%s", NODE_ID);

  snprintf(mqtt_username, sizeof(mqtt_username), "%s", USERNAME);
  snprintf(mqtt_password, sizeof(mqtt_password), "%s", PASSWORD);

  modem.mqttConnect();

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
      if (modem.mqttConnected) {
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
    modem.noteUartActivity();
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
                    if (modem.getNetworkTimeISO8601(isoNow, sizeof(isoNow))) {
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
  DIAG_PRINT(modem.atFailStreak);
  DIAG_PRINT(F(" | GPRSstreak="));
  DIAG_PRINT(modem.gprsFailStreak);
  DIAG_PRINT(F(" | MQTTstreak="));
  DIAG_PRINT(modem.mqttFailStreak);
  DIAG_PRINT(F(" | since ATok="));
  DIAG_PRINT(millis() - modem.lastATokMs);
  DIAG_PRINT(F(" | since GPRSk="));
  DIAG_PRINT(millis() - modem.lastGPRSokMs);
  DIAG_PRINT(F(" | since MQTTk="));
  DIAG_PRINT(millis() - modem.lastMQTTokMs);
  DIAG_PRINTLN("");
}

// ----------------- HEALTH/RECONNECTIONS -----------------
void checkConnections() {
  if ((long)(millis() - lastCheck) < (long)CHECK_INTERVAL_MS) return;

  if (!modem.canRunHealthCheck()) {
    if (DIAG) DIAG_PRINTLN(F("[DIAG] checkConnections posponed: recent UART activity"));
    return;
  }

  if (!modem.canRunHealthCheck()) {
    if (DIAG) DIAG_PRINTLN(F("[DIAG] posponed after soft-drain"));
    return;
  }

  lastCheck = millis();

  bool atOk = false;
  for (byte attempt = 0; attempt < 3; attempt++) {
    if (modem.isAlive()) {
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
    modem.atFailStreak++;
    DEBUG_PRINT(F("AT fail streak = "));
    DEBUG_PRINTLN(modem.atFailStreak);

    if ((millis() - modem.lastATokMs) < RECENT_OK_GRACE_MS && modem.atFailStreak < AT_FAILS_BEFORE_RESTART) {
      DEBUG_PRINTLN(F("Recent Ok and few failures: DO NOT restart (grace period)."));
      return;
    }
    if (modem.atFailStreak < AT_FAILS_BEFORE_RESTART) {
      DEBUG_PRINTLN(F("AT failed under threshold. Will try later."));
      return;
    }
    if (millis() - modem.lastRestartMs < MIN_RESTART_GAP_MS) {
      DEBUG_PRINTLN(F("Restart blocked by debounce. Waiting for next cycle."));
      return;
    }

    DEBUG_PRINTLN(F("Modem not responding consistently. Restarting..."));
    modem.restart();
    modem.atFailStreak = 0;
    if (!modem.gprsConnect()) return;
    modem.mqttConnect();
    return;
  } else {
    modem.atFailStreak = 0;
    modem.lastATokMs = millis();
  }
  if (!modem.gprsIsConnected()) {
    DEBUG_PRINTLN(F("GPRS not connected. Skipping MQTT check and attempting GPRS reconnection..."));
    if (modem.gprsConnect()) {
      modem.mqttConnect();
    }
    return;
  }
  {
    bool mqttOk = false;

    if (modem.checkMQTTConnection()) {
      mqttOk = true;
    } else {
      DEBUG_PRINTLN(F("MQTT NOT connected. Trying to reconnect up to 3 times..."));
      for (byte attempt = 0; attempt < 3; attempt++) {
        modem.mqttConnect();
        if (modem.mqttConnected) {
          mqttOk = true;
          break;
        }
        delay(900);
        WDT_RESET();
      }
    }

    if (mqttOk) {
      modem.mqttFailStreak = 0;
      modem.lastMQTTokMs = millis();
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

      if (modem.gprsIsConnected()) {
        gprsOk = true;
        break;
      }
      DEBUG_PRINTLN(F("GPRS disconnected. Trying to reconnect..."));
      if (modem.gprsConnect()) {
        gprsOk = true;
        break;
      }

      delay(700);
      WDT_RESET();
    }

    if (!gprsOk) {
      modem.gprsFailStreak++;
      DEBUG_PRINT(F("GPRS fail streak = "));
      DEBUG_PRINTLN(modem.gprsFailStreak);

      if ((millis() - modem.lastGPRSokMs) < RECENT_OK_GRACE_MS && modem.gprsFailStreak < GPRS_FAILS_BEFORE_RESTART) {
        DEBUG_PRINTLN(F("GPRS failed but it was OK recently. Do not restart."));
        return;
      }
      if (modem.gprsFailStreak < GPRS_FAILS_BEFORE_RESTART) {
        DEBUG_PRINTLN(F("GPRS failed under threshold. Retry later."));
        return;
      }
      if (millis() - modem.lastRestartMs < MIN_RESTART_GAP_MS) {
        DEBUG_PRINTLN(F("GPRS failed; restart blocked by debounce."));
        return;
      }

      DEBUG_PRINTLN(F("GPRS failed consistently. Restarting modem..."));
      modem.restart();
      modem.gprsFailStreak = 0;
      if (!modem.gprsConnect()) return;

      DEBUG_PRINTLN(F("GPRS OK after restart. Retrying MQTT..."));
      modem.mqttConnect();
      if (modem.mqttConnected) {
        modem.mqttFailStreak = 0;
        modem.lastMQTTokMs = millis();
      } else {
        modem.mqttFailStreak++;
      }
      return;
    } else {
      modem.gprsFailStreak = 0;
      modem.lastGPRSokMs = millis();
    }
  }

  {
    if (!modem.mqttConnected) {
      DEBUG_PRINTLN(F("GPRS OK. Retrying MQTT after GPRS..."));
      for (byte attempt = 0; attempt < 3; attempt++) {
        modem.mqttConnect();
        if (modem.mqttConnected) {
          modem.mqttFailStreak = 0;
          modem.lastMQTTokMs = millis();
          return;
        }
        delay(700);
        WDT_RESET();
      }

      modem.mqttFailStreak++;
      DEBUG_PRINT(F("MQTT fail streak = "));
      DEBUG_PRINTLN(modem.mqttFailStreak);

      if ((millis() - modem.lastMQTTokMs) < RECENT_OK_GRACE_MS && modem.mqttFailStreak < MQTT_FAILS_BEFORE_RESTART) {
        DEBUG_PRINTLN(F("MQTT failed but it was OK recently. Do not restart."));
        return;
      }
      if (modem.mqttFailStreak < MQTT_FAILS_BEFORE_RESTART) {
        DEBUG_PRINTLN(F("MQTT failed under threshold. Retry later."));
        return;
      }
      if (millis() - modem.lastRestartMs < MIN_RESTART_GAP_MS) {
        DEBUG_PRINTLN(F("MQTT failed; restart blocked by debounce."));
        return;
      }

      DEBUG_PRINTLN(F("MQTT consistently. Restarting modem..."));
      modem.restart();
      modem.mqttFailStreak = 0;
      if (!modem.gprsConnect()) return;
      modem.mqttConnect();
      return;
    } else {
      modem.mqttFailStreak = 0;
      modem.lastMQTTokMs = millis();
      return;
    }
  }
}


// ----------------- MQTT/GPRS -----------------
bool publishMessage() {
  if (!modem.mqttConnected) return false;

  int V1 = readAverage(ADC_PIN_1);
  int V2 = readAverage(ADC_PIN_2);
  int V3 = readAverage(ADC_PIN_3);

  // CSQ (0..31; if fails, 0)
  int csq = modem.getCSQ_RSSI();
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
  bool haveTs = modem.getNetworkTimeISO8601(tsZ, sizeof(tsZ));

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
             (unsigned long)modem.cntModemRestarts,
             (unsigned long)modem.cntGprsConnects,
             (unsigned long)modem.cntMqttConnects,
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
             (unsigned long)modem.cntModemRestarts,
             (unsigned long)modem.cntGprsConnects,
             (unsigned long)modem.cntMqttConnects,
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

  bool result = modem.publishMessage(topic_pub, hexPayload);

  EventCountTotal = 0;
  return result;
}


// ----------------- MQTT/GPRS (moved to SIM7070G library) -----------------

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

// ----------------- NEW makeNonce12 -----------------
static inline void makeNonce12(uint8_t nonce[12]) {
  static uint64_t perSecondCtr = 0;  // 40-bit used
  static uint16_t lastY = 0;
  static uint8_t lastM = 0;
  static uint8_t lastD = 0;
  static uint8_t lastH = 0;
  static uint8_t lastMin = 0;
  static uint8_t lastS = 0;

  int Y = 0, Mo = 0, D = 0, H = 0, Mi = 0, S = 0;
  char iso[32] = { 0 };

  bool haveNetwork = false;
  if (modem.getNetworkTimeISO8601(iso, sizeof(iso))) {
    if (sscanf(iso, "%4d-%2d-%2dT%2d:%2d:%2d", &Y, &Mo, &D, &H, &Mi, &S) == 6) {
      haveNetwork = true;
    }
  }

  if (!haveNetwork) {
    uint32_t t = millis() / 1000UL;
    H = (t / 3600UL) % 24;
    Mi = (t / 60UL) % 60;
    S = (t) % 60;
    Y = 0xFFFF;
    Mo = 0;
    D = 0;
  }

  if ((uint16_t)Y != lastY || (uint8_t)Mo != lastM || (uint8_t)D != lastD || (uint8_t)H != lastH || (uint8_t)Mi != lastMin || (uint8_t)S != lastS) {
    perSecondCtr = 0;
    lastY = (uint16_t)Y;
    lastM = (uint8_t)Mo;
    lastD = (uint8_t)D;
    lastH = (uint8_t)H;
    lastMin = (uint8_t)Mi;
    lastS = (uint8_t)S;
  }

  nonce[0] = (uint8_t)((((uint16_t)Y) >> 8) & 0xFF);
  nonce[1] = (uint8_t)(((uint16_t)Y) & 0xFF);
  nonce[2] = (uint8_t)Mo;
  nonce[3] = (uint8_t)D;
  nonce[4] = (uint8_t)H;
  nonce[5] = (uint8_t)Mi;
  nonce[6] = (uint8_t)S;

  uint64_t c = perSecondCtr++;
  c &= 0x000000FFFFFFFFFFULL;
  nonce[7] = (uint8_t)((c >> 32) & 0xFF);
  nonce[8] = (uint8_t)((c >> 24) & 0xFF);
  nonce[9] = (uint8_t)((c >> 16) & 0xFF);
  nonce[10] = (uint8_t)((c >> 8) & 0xFF);
  nonce[11] = (uint8_t)(c & 0xFF);

  if (DIAG) {
    DIAG_PRINT(F("[DIAG] nonce(Y-M-D H:M:S ctr): "));
    DIAG_PRINT(lastY);
    DIAG_PRINT('-');
    DIAG_PRINT(lastM);
    DIAG_PRINT('-');
    DIAG_PRINT(lastD);
    DIAG_PRINT(' ');
    DIAG_PRINT(lastH);
    DIAG_PRINT(':');
    DIAG_PRINT(lastMin);
    DIAG_PRINT(':');
    DIAG_PRINT(lastS);
    DIAG_PRINT(F(" ctr="));
    DIAG_PRINT(perSecondCtr - 1);
    DIAG_PRINTLN("");
  }
}

size_t encryptPayload(const char* plaintext, uint8_t* ciphertext, uint8_t tag[16], const uint8_t nonce12[12]) {
  size_t len = strlen(plaintext);
  ChaChaPoly aead;
  aead.setKey(CRYPTO_KEY, sizeof(CRYPTO_KEY));
  aead.setIV(nonce12, 12);
  aead.encrypt(ciphertext, (const uint8_t*)plaintext, len);
  aead.computeTag(tag, 16);
  return len;
}

bool decryptPayload(const uint8_t* ciphertext, size_t ctLen, const uint8_t tag[16],
                    const uint8_t nonce12[12], char* outPlain, size_t outPlainCap) {
  if (outPlainCap < ctLen + 1) return false;
  ChaChaPoly aead;
  aead.setKey(CRYPTO_KEY, sizeof(CRYPTO_KEY));
  aead.setIV(nonce12, 12);
  aead.decrypt((uint8_t*)outPlain, ciphertext, ctLen);
  bool ok = aead.checkTag(tag, 16);
  if (ok) outPlain[ctLen] = '\0';
  return ok;
}

// ----------------- HEX helpers -----------------
void hexEncode(const uint8_t* in, size_t inLen, char* out, size_t outCap, bool uppercase) {
  static const char* TAB_U = "0123456789ABCDEF";
  static const char* TAB_L = "0123456789abcdef";
  const char* TAB = uppercase ? TAB_U : TAB_L;

  size_t need = inLen * 2 + 1;
  if (outCap < need) {
    if (outCap) out[0] = '\0';
    return;
  }

  size_t o = 0;
  for (size_t i = 0; i < inLen; i++) {
    out[o++] = TAB[(in[i] >> 4) & 0x0F];
    out[o++] = TAB[in[i] & 0x0F];
  }
  out[o] = '\0';
}

bool hexDecode(const char* hex, uint8_t* out, size_t outCap, size_t& outLen) {
  auto nib = [](char c) -> int {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
  };
  size_t n = strlen(hex);
  if (n % 2) return false;
  size_t bytes = n / 2;
  if (bytes > outCap) return false;
  for (size_t i = 0; i < bytes; i++) {
    int hi = nib(hex[2 * i]);
    int lo = nib(hex[2 * i + 1]);
    if (hi < 0 || lo < 0) return false;
    out[i] = (uint8_t)((hi << 4) | lo);
  }
  outLen = bytes;
  return true;
}

// ----------------- JSON "mini" getters -----------------
bool jsonGetString(const char* json, const char* key, char* out, size_t cap) {
  const char* p = strstr(json, key);
  if (!p) return false;
  p = strchr(p, ':');
  if (!p) return false;
  p = strchr(p, '"');
  if (!p) return false;
  p++;
  const char* e = strchr(p, '"');
  if (!e) return false;
  size_t n = (size_t)(e - p);
  if (n >= cap) n = cap - 1;
  memcpy(out, p, n);
  out[n] = '\0';
  return true;
}
bool jsonGetBool(const char* json, const char* key, bool& out) {
  const char* p = strstr(json, key);
  if (!p) return false;
  p = strchr(p, ':');
  if (!p) return false;
  p++;
  while (*p == ' ' || *p == '\t') p++;
  if (!strncmp(p, "true", 4)) {
    out = true;
    return true;
  }
  if (!strncmp(p, "false", 5)) {
    out = false;
    return true;
  }
  return false;
}
bool jsonGetInt(const char* json, const char* key, long& out) {
  const char* p = strstr(json, key);
  if (!p) return false;
  p = strchr(p, ':');
  if (!p) return false;
  p++;
  out = strtol(p, NULL, 10);
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
    METER_CURRENT_ADDRESS,
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
  if (recv[0] != MODBUS_REQ.address || recv[1] != MODBUS_REQ.function || recv[2] != 4) {
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

    case METER_CURRENT_ADDRESS:
      value = raw[0] / 1000.0f;  // Current scaled
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
