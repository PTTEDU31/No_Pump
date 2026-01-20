# AI Reference Guide - IoT Pump Control System

## Quick Overview

**Project**: Remote water pump monitoring and control via LTE-M IoT  
**MCU**: Arduino Nano 33 IoT (SAMD21)  
**Modem**: SIM7070G (LTE-M/NB-IoT)  
**Protocol**: MQTT over TCP with ChaCha20-Poly1305 encryption  
**Purpose**: Monitor pump status, water flow, battery, and control pump remotely with security

---

## System Architecture (High-Level)

```
┌─────────────────┐     ┌──────────────┐     ┌─────────────────┐
│  MQTT Server    │────▶│  SIM7070G    │────▶│  Arduino Nano   │
│  (Commands)     │◀────│  LTE-M Modem │◀────│  33 IoT (MCU)   │
└─────────────────┘     └──────────────┘     └─────────────────┘
                                                      │
                        ┌─────────────────────────────┼─────────────────────────────┐
                        │                             │                             │
                   ┌────▼────┐               ┌────────▼────────┐          ┌────────▼────────┐
                   │ Sensors │               │  Pump Control   │          │  RS485 Modbus   │
                   │ • AC I  │               │  • Relay ON/OFF │          │  • Water Meter  │
                   │ • Batt  │               │  • Contact Sens │          │  • Flow: m³/h   │
                   │ • Power │               │  • Protection   │          │  • Total: m³    │
                   └─────────┘               └─────────────────┘          └─────────────────┘
```

---

## Core Modules & Responsibilities

### 1. **Setup (One-time Initialization)**
- Initialize watchdog timer (16s)
- Configure GPIO pins (relays, sensors, ADC)
- Setup UART interfaces (SIM7070G: 115200, RS485: 9600)
- Decode crypto key (64 hex → 32 bytes)
- Detect initial pump & power state
- Verify modem connection (restart if needed)
- Connect GPRS (activate PDP context)
- Connect MQTT & subscribe to command topic

### 2. **Main Loop (Continuous Operation)**
```cpp
loop() {
  WDT_RESET();                    // Pet watchdog every cycle
  serviceStartProtection();        // Manage 120s protection timer
  handleMQTTMessages();           // Process incoming commands (real-time)
  checkConnections();             // Health check (every 15s)
  servicePumpEdgeAndStatus();     // Detect pump state changes
  publishMessage();               // Send telemetry (every 15s/30s)
}
```

### 3. **Key Functions**

#### **handleMQTTMessages()**
```
Receive URC "+SMSUB" → Extract HEX payload → Decode HEX to bytes
→ Extract [nonce(12)|ciphertext|tag(16)]
→ Get timestamp from nonce[4:6] (H:M:S)
→ Get network time (retry 5x) → Verify time window (±30s)
→ Decrypt ChaCha20-Poly1305 → Parse JSON {"X":0/1, "onPulse":ms}
→ Execute command (turnOnPump/turnOffPump)
```

**Time Window Security**:
- Message timestamp from nonce: `secMsg = H*3600 + M*60 + S`
- Network time from AT+CCLK: `secNow`
- Circular difference: `Δt = circular_diff(secMsg, secNow)`
- **Reject if Δt > 30s** (replay attack protection)

#### **publishMessage()**
```
Collect sensor data → Adjust publish rate (15s/30s based on PWR & battery)
→ Build JSON plain → Encrypt ChaCha20-Poly1305 → HEX encode
→ AT+SMPUB to xtr/nodes/{NODE_ID}
```

**Sensors Published**:
| Field | Source | Description |
|-------|--------|-------------|
| V1, V2, V3 | ADC (A0/A1/A2) | AC current transducers |
| X | CONTACT_PIN | Pump state (0=OFF, 1=ON) |
| P | protectOnStart | Protection active (0/1) |
| B | Battery ADC | Battery % (3.0V-4.05V = 0-100%) |
| CSQ | AT+CSQ | Signal strength (0-31) |
| TCPU | SAMD21 sensor | CPU temperature (°C) |
| FLOW | RS485 Modbus | Current water flow (m³/h) |
| TOT | RS485 Modbus | Cumulative flow (m³) |
| RSIM/RGPRS/RMQTT | Counters | Restart/reconnect stats |
| PWR | Input_Supply_V | Main power present (0/1) |
| AT_UTC | AT+CCLK | ISO-8601 timestamp |

**Adaptive Publish Rate**:
```python
if PWR == 1:  # Main power present
    interval = 15s  # Always fast
else:  # Battery power
    if B < 90%:
        interval = 30s  # Slow (save power)
    elif B >= 93%:
        interval = 15s  # Fast (hysteresis)
```

#### **checkConnections()**
Health check every 15s with fail streak tracking:

```
Check AT (modemAlivePing) → 3 retries
  ├─ Fail streak >= 2 → modemRestart()
  └─ Grace: if OK within 16s → tolerate failures

Check GPRS (AT+CNACT?) → 3 retries
  ├─ Fail streak >= 3 → modemRestart()
  └─ Grace: if OK within 16s → tolerate failures

Check MQTT (AT+SMSTATE?) → 3 retries
  ├─ Fail streak >= 3 → modemRestart()
  └─ Grace: if OK within 16s → tolerate failures
```

**Debounce**: Minimum 15s gap between modem restarts

#### **Pump Control & Protection**

**turnOnPump()**:
```
if protectOnStart:
    reject("Protection active, Xs elapsed")
if isPumpOn():
    skip("Already ON")
digitalWrite(PWR_ON_PIN, HIGH)
delay(onPulse)  # Default: 5000ms, configurable 500-10000ms
digitalWrite(PWR_ON_PIN, LOW)
publishMessage()
```

**turnOffPump()**:
```
if !isPumpOn():
    skip("Already OFF")
digitalWrite(PWR_OFF_PIN, HIGH)
delay(1000)
digitalWrite(PWR_OFF_PIN, LOW)
publishMessage()
```

**Protection Mechanism (120s after OFF)**:
- **Trigger**: When X=0 command OR edge detection (ON→OFF without command)
- **Duration**: 120 seconds
- **Effect**: Block all ON commands (X=1)
- **Announce**: Every 10s log "Protection: Xs elapsed"
- **Purpose**: Prevent rapid cycling (damage to motor)

**State Detection (Debounce)**:
```cpp
isPumpOn() {
    highs = 0;
    for (i=0; i<4; i++) {
        highs += (digitalRead(CONTACT_PIN) == HIGH);
        delay(20);
    }
    if (highs == 4) return true;   // Definitely ON
    if (highs == 0) return false;  // Definitely OFF
    return prevPumpOn;             // Ambiguous → keep previous
}
```

#### **Modem Management**

**modemRestart()** - Hardware reset via PWRKEY:
```
Drain UART buffer (40ms)
if modemAlivePing(): return  # Already on

PWRKEY pulse #1 (1600ms HIGH) → toggle ON/OFF
Wait 9000ms (shutdown time)
if modemAlivePing(): done

PWRKEY pulse #2 (1600ms HIGH) → force ON
Wait 6000ms (boot time)
applyBootConfig() → AT+CMEE=2, AT+CSOCKSETPN=1, etc.
cntModemRestarts++
```

**gprsConnect()** - Activate PDP context:
```
AT+CNACT? → Check if "+CNACT: 1,1" (already connected)
AT+CFUN? → Ensure radio ON (must be 1)
AT+CGDCONT=1,"IP","iot.1nce.net"
AT+CNACT=1,1 → Activate (retry 2x)
syncTimeUTC_any() → NTP sync (5 servers fallback)
cntGprsConnects++
```

**mqttConnect()** - Connect to broker:
```
Verify GPRS connected
AT+SMDISC → Disconnect old session
Configure: URL, PORT, CLIENTID, USERNAME, PASSWORD, KEEPTIME=60
AT+SMCONN → Connect
AT+SMSUB="xtr/server/{NODE_ID}",1 → Subscribe QoS 1
cntMqttConnects++
```

**syncTimeUTC_any()** - NTP sync with fallback:
```
Try servers in order:
  1. time.google.com
  2. time.cloudflare.com
  3. pool.ntp.org
  4. time.windows.com
  5. time.nist.gov

For each: AT+CNTPCID=1, AT+CNTP="{server}",0, AT+CNTP
Poll AT+CNTP? until "+CNTP: 1" (success) or timeout
```

---

## Cryptography (ChaCha20-Poly1305 AEAD)

### Key Management
- **CRYPTOKEY**: 64 hex characters (constant in code)
- **CRYPTO_KEY**: 32 bytes binary (decoded once in setup)
- **Shared secret** between node and server

### Nonce Structure (12 bytes)
```
[Year(2) | Month | Day | Hour | Minute | Second | Counter(5)]
 uint16     u8     u8    u8      u8       u8       uint40

Year: Big-endian, e.g., 0x07EA = 2026
Counter: Increments per nonce, resets when second changes
```

### Encryption (TX: Node → Server)
```cpp
makeNonce12(nonce);  // Get time + counter
encryptPayload(plain, ct, tag, nonce);
  ├─ ChaChaPoly.setKey(CRYPTO_KEY, 32)
  ├─ ChaChaPoly.setIV(nonce, 12)
  ├─ ChaChaPoly.encrypt(ct, plain, len)
  └─ ChaChaPoly.computeTag(tag, 16)
hexEncode([nonce(12) | ct | tag(16)])
```

### Decryption (RX: Server → Node)
```cpp
hexDecode(payload) → [nonce(12) | ct | tag(16)]
Extract H:M:S from nonce[4:6] → secMsg
Get network time → secNow
if circular_diff(secMsg, secNow) > 30s: REJECT
decryptPayload(ct, ctLen, tag, nonce, plain, cap)
  ├─ ChaChaPoly.setKey(CRYPTO_KEY, 32)
  ├─ ChaChaPoly.setIV(nonce, 12)
  ├─ ChaChaPoly.decrypt(plain, ct, ctLen)
  └─ bool ok = ChaChaPoly.checkTag(tag, 16)
```

---

## RS485 Modbus RTU (Water Meter)

### Registers
- **METER_CURRENT_ADDRESS** = 0x00000002 (Current flow, scaled ÷1000 → m³/h)
- **METER_CUMULATIVE_ADDRESS** = 0x00380002 (Total flow, raw → m³)

### Request Frame (8 bytes)
```
[Address | Function | RegH | RegL | LenH | LenL | CRCH | CRCL]
   0x01      0x03     4-byte register      0x0002    CRC-16
```

### Response Frame (9 bytes)
```
[Address | Function | ByteCount | Data(4 bytes) | CRCH | CRCL]
   0x01      0x03        0x04       uint32 BE      CRC-16
```

### Read Sequence
```cpp
RS485_loop() {
    READING_PARAM(METER_CURRENT_ADDRESS);
    delay(50);
    parameters[0] = get_param(METER_CURRENT_ADDRESS);  // Flow (m³/h)
    delay(250);
    
    READING_PARAM(METER_CUMULATIVE_ADDRESS);
    delay(50);
    parameters[1] = get_param(METER_CUMULATIVE_ADDRESS);  // Total (m³)
    delay(250);
}
```

**Timeouts**:
- First byte: 200ms
- Inter-byte: 30ms
- Total read: 250ms

---

## Configuration Constants

### Timing
| Constant | Value | Purpose |
|----------|-------|---------|
| HW_WDT_TIMEOUT_SEC | 16 | Watchdog timeout |
| checkInterval | 15000 | Health check interval (ms) |
| PUB_FAST_MS | 15000 | Fast publish rate |
| PUB_SLOW_MS | 30000 | Slow publish rate |
| PROTECT_ON_MS | 120000 | Protection duration (2 min) |
| minRestartGapMs | 15000 | Debounce between restarts |
| recentOkGraceMs | 16000 | Grace period for "recent OK" |

### Thresholds
| Constant | Value | Purpose |
|----------|-------|---------|
| AT_FAILS_BEFORE_RESTART | 2 | Modem restart after 2 AT fails |
| GPRS_FAILS_BEFORE_RESTART | 3 | Modem restart after 3 GPRS fails |
| MQTT_FAILS_BEFORE_RESTART | 3 | Modem restart after 3 MQTT fails |
| BAT_THRESH_PCT | 90 | Battery threshold for rate change |
| BAT_HYST_PCT | 3 | Hysteresis to avoid jitter |

### Battery
| Constant | Value | Purpose |
|----------|-------|---------|
| V_MIN | 3.00 | 0% battery voltage |
| V_MAX | 4.05 | 100% battery voltage |
| R_TOP | 100kΩ | Voltage divider resistor |
| R_BOTTOM | 220kΩ | Voltage divider resistor |

---

## Pin Mapping

| Pin | Function | Description |
|-----|----------|-------------|
| 12 | MODEM_PWR_PIN | SIM7070G PWRKEY |
| 4 | PWR_ON_PIN | Relay ON coil |
| 3 | PWR_OFF_PIN | Relay OFF coil |
| 20 | CONTACT_PIN | Pump state (HIGH=ON) |
| A0 | ADC_PIN_1 | AC current line 1 |
| A1 | ADC_PIN_2 | AC current line 2 |
| A2 | ADC_PIN_3 | AC current line 3 |
| A3 | BATT_VOLTS | Battery voltage |
| 21 | Input_Supply_V | Main power detection |
| 5 | RS485_RX_PIN | SERCOM0 RX |
| 6 | RS485_TX_PIN | SERCOM0 TX |

---

## JSON Protocol

### Subscribe (Server → Node): `xtr/server/{NODE_ID}`
```json
{
  "X": 0,          // 0=OFF, 1=ON
  "onPulse": 5000  // Optional: ON pulse duration (500-10000 ms)
}
```

### Publish (Node → Server): `xtr/nodes/{NODE_ID}`
```json
{
  "V1": 123,       // AC current line 1
  "V2": 456,       // AC current line 2
  "V3": 789,       // AC current line 3
  "X": 1,          // Pump state (0=OFF, 1=ON)
  "P": 0,          // Protection active (0=NO, 1=YES)
  "B": 85,         // Battery percentage
  "CSQ": 18,       // Signal strength (0-31)
  "TCPU": 32.5,    // CPU temperature (°C)
  "FLOW": 1.234,   // Current flow (m³/h)
  "TOT": 1234.5,   // Cumulative flow (m³)
  "RSIM": 5,       // Modem restart counter
  "RGPRS": 12,     // GPRS connect counter
  "RMQTT": 12,     // MQTT connect counter
  "PWR": 1,        // Main power (0=NO, 1=YES)
  "AT_UTC": "2026-01-20T15:30:45Z"  // ISO-8601 timestamp
}
```

---

## State Machine: Connection Management

```
                    ┌──────────────┐
                    │  POWER ON    │
                    └──────┬───────┘
                           │
                    ┌──────▼───────┐
                    │  AT Command  │◀─┐
                    │  Ping (3x)   │  │
                    └──────┬───────┘  │
                           │          │
                    ┌──────▼───────┐  │
                    │  GPRS Check  │  │ Fail Streak
                    │  (AT+CNACT?) │  │ >= Threshold
                    └──────┬───────┘  │
                           │          │
                    ┌──────▼───────┐  │
                    │  MQTT Check  │  │
                    │  (AT+SMSTATE)│  │
                    └──────┬───────┘  │
                           │          │
              ┌────────────┴────────┐ │
              │                     │ │
         ┌────▼────┐         ┌─────▼──▼──┐
         │  OK     │         │  FAIL      │
         │  (Pub)  │         │  (Restart) │
         └────┬────┘         └─────┬──────┘
              │                    │
              │   15s              │
              └────────────────────┘
```

---

## Troubleshooting Quick Reference

| Symptom | Check | Fix |
|---------|-------|-----|
| Modem restart loop | CSQ, AT+CNACT?, cntModemRestarts | Verify antenna, APN settings |
| MQTT disconnect | mqttFailStreak, lastMQTTokMs | Increase KEEPTIME, check broker |
| Command not working | Time window log, decrypt result | Sync NTP, verify CRYPTOKEY match |
| Pump won't turn ON | P field, protection timer | Wait 120s, check CONTACT_PIN wiring |
| Battery drain | publishIntervalMs, PWR field | Fix PWR detection, increase PUB_SLOW_MS |
| RS485 no data | RX buffer, CRC errors | Check A/B wiring, verify 9600 baud |

---

## Important Design Rationale

1. **ChaCha20-Poly1305**: AEAD (auth + encrypt in one), lightweight for Cortex-M0+
2. **30s time window**: Balance between security and network latency tolerance
3. **120s protection**: Pump industry standard to prevent cycling damage
4. **Adaptive publish**: Save battery when on backup power
5. **4×20ms debounce**: Filter relay/motor EMI noise
6. **16s grace period**: Sync with watchdog, tolerate network hiccups
7. **Fail streak thresholds**: Layered recovery (AT→GPRS→MQTT)
8. **5 NTP servers**: Reliability through geographic diversity

---

## Code Entry Points (For AI Code Analysis)

| Function | Purpose | Called From |
|----------|---------|-------------|
| `setup()` | One-time init | Arduino framework |
| `loop()` | Main event loop | Arduino framework |
| `handleMQTTMessages()` | Parse URC, decrypt, execute | `loop()` |
| `publishMessage()` | Collect sensors, encrypt, publish | `loop()` |
| `checkConnections()` | Health check, auto-reconnect | `loop()` |
| `modemRestart()` | Hardware reset via PWRKEY | `checkConnections()` |
| `gprsConnect()` | Activate PDP context | `setup()`, `checkConnections()` |
| `mqttConnect()` | Connect & subscribe MQTT | `setup()`, `checkConnections()` |
| `turnOnPump()` | Actuate relay ON | `handleMQTTMessages()` |
| `turnOffPump()` | Actuate relay OFF | `handleMQTTMessages()` |
| `RS485_loop()` | Read Modbus water meter | `loop()` (before publish) |

---

## Dependencies (PlatformIO lib_deps)

```ini
lib_deps = 
    arduino-libraries/Arduino_LSM6DS3@^1.0.3
    adafruit/Adafruit SleepyDog Library
    arduino-libraries/Arduino_TemperatureZero
    rweather/Crypto@^0.4.0
```

---

## Credentials & Config (Change before deployment)

```cpp
const char* NODE_ID = "5a06bafb-e479-4dc3-87d9-d79734d71f13";
const char* USERNAME = "node_d935168a3a77";
const char* PASSWORD = "G9XqOmsUYuSgq3mmWd0ecTmP";
const char* CRYPTOKEY = "B262DF3DCFCAEB149785BFDB3E84CF1535EF0F849FCB702449CD9A5DC037545F";

#define PDP_APN "iot.1nce.net"
const char mqtt_server[] = "broker.remotextr.com";
const int mqtt_port = 1883;
```

---

## Debug Flags

```cpp
#define DEBUG true   // Enable Serial.print debug messages
#define DIAG false   // Enable detailed diagnostics
```

When `DEBUG=true`: All `DEBUG_PRINT()` macros output to Serial (115200 bps)  
When `DIAG=true`: All `DIAG_PRINT()` macros output (low-level UART/timing info)

---

**Last Updated**: 2026-01-20  
**Firmware Version**: Production  
**Hardware**: Arduino Nano 33 IoT + SIM7070G + RS485 Modbus
