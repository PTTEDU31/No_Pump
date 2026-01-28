# SIM7070G Modem Device (dev_sim7070g)

## 1. Overview

**Sim7070GDevice** is a non-blocking driver that:

- Manages the SIM7070G cellular modem (power, AT commands, GPRS/NB-IoT).
- Brings up the PDP context (CNACT, APN) and waits for an assigned IP.
- Configures and connects to MQTT; subscribes to a command topic and publishes telemetry.
- Buffers outbound MQTT messages when disconnected and flushes them when connected.
- Handles incoming MQTT commands: supports encrypted HEX payloads (decode + time-window check) and plain JSON; drives the pump via `PumpDevice` (e.g. `"X":1` = ON, `"X":0` = OFF with protection).

It implements the **Device** interface and runs on **core 0**. It does not subscribe to framework events (`EVENT_NONE`); scheduling is entirely timeout-driven.

---

## 2. Hardware and Dependencies

| Item | Description |
|------|-------------|
| **Modem** | SIM7070G over `HardwareSerial` (baud from `config.h`: `MODEM_BAUD_RATE`). |
| **Power** | `MODEM_POWER_PIN` (e.g. 12) for PWRKEY pulse. |
| **Library** | `Sim7070G` (`lib/sim7070g`). |
| **Config** | MQTT broker/port, credentials, APN, node ID from `config.h` and globals (`mqtt_server`, `mqtt_port`, `USERNAME`, `PASSWORD`, `PDP_APN`). |

Topics are derived from `nodeId`: publish topic = `nodeId`, subscribe topic = `nodeId/sub`.

---

## 3. Execution Flow Summary

```
setup()
   │
   ├── initialize()
   │      • Create Sim7070G, call begin(), set MQTT message callback
   │      • updateState(MODEM_OFF)
   │
   └── start()
          • Build _pubTopic, _subTopic, MQTT/GPRS credentials from config
          • updateState(MODEM_POWER_ON)
          • return DURATION_IMMEDIATELY

loop (scheduler calls timeout() when due)
   │
   └── timeout() — switch (_state)
          │
          ├── MODEM_POWER_ON
          │      • powerPulse(); updateState(MODEM_ACTIVATE_CNACT_FIRST)
          │      • return 10000 (boot wait)
          │
          ├── MODEM_ACTIVATE_CNACT_FIRST
          │      • If IP already assigned → MODEM_MQTT_CONNECTING_HANDSHAKE
          │      • Else AT+CNACT=0,1 → success: MODEM_WAIT_FOR_IP_FIRST (7s); fail 3×: MODEM_APPLY_BOOT_CONFIG
          │
          ├── MODEM_WAIT_FOR_IP_FIRST → MODEM_CHECK_IP_FIRST (immediately)
          ├── MODEM_CHECK_IP_FIRST
          │      • getIPAddress() → OK: MODEM_MQTT_CONNECTING_HANDSHAKE; fail: MODEM_APPLY_BOOT_CONFIG
          │
          ├── MODEM_APPLY_BOOT_CONFIG
          │      • applyBootConfig() (RAT, network mode); fail → MODEM_ERROR
          │      • → MODEM_CHECK_SIM_AFTER_CONFIG
          ├── MODEM_CHECK_SIM_AFTER_CONFIG
          │      • checkSIM() → OK: MODEM_CHECK_SIGNAL_AFTER_CONFIG; fail 6×: MODEM_ERROR
          ├── MODEM_CHECK_SIGNAL_AFTER_CONFIG
          │      • getSignalQuality() → OK: MODEM_SET_APN; fail 5×: MODEM_ERROR
          ├── MODEM_SET_APN
          │      • setAPN(), then AT+CNACT=1,1 → MODEM_WAIT_FOR_IP_AFTER_APN (10s) or retry/ERROR
          ├── MODEM_WAIT_FOR_IP_AFTER_APN → MODEM_CHECK_IP_AFTER_APN
          ├── MODEM_CHECK_IP_AFTER_APN
          │      • getIPAddress() → OK: MODEM_MQTT_CONNECTING_HANDSHAKE; fail 5×: MODEM_ERROR
          │
          ├── MODEM_MQTT_CONNECTING_HANDSHAKE
          │      • mqttSetConfig(); updateState(MODEM_MQTT_CONNECTING)
          │      • return 2000
          ├── MODEM_MQTT_CONNECTING
          │      • AT+SMCONN → OK: MODEM_SYNC_NTP_TIME; fail: disconnect, return 30000
          ├── MODEM_SYNC_NTP_TIME
          │      • syncTimeUTC_any(); → MODEM_MQTT_CONNECTED_FIRST (5s); retry up to 2× then proceed
          ├── MODEM_MQTT_CONNECTED_FIRST
          │      • mqttSubscribe(_subTopic) → OK: enqueue "online", updateState(MODEM_MQTT_CONNECTED); fail 5×: close MQTT, MODEM_MQTT_CONNECTING_HANDSHAKE
          │
          ├── MODEM_MQTT_CONNECTED
          │      • handleIncomingData() (_modem->loop())
          │      • if _bufferCount > 0: flushMessageBuffer() (sends buffered telemetry; may enter MODEM_WAITING_SUCCESS)
          │      • return 20000
          │
          ├── MODEM_WAITING_SUCCESS
          │      • When _waitingDone: transition to _waitingSuccessNextState
          │      • Timeout 5 min → MODEM_MQTT_CONNECTED
          │
          ├── MODEM_ERROR
          │      • _stats.modemRestarts++; _mqttBegun = false; updateState(MODEM_INITIALIZING)
          │      • return 15000
          ├── MODEM_INITIALIZING
          │      • AT, AT+CPOWD=1; updateState(MODEM_RESET); return DURATION_IMMEDIATELY
          ├── MODEM_RESET
          │      • digitalWrite(POWER, LOW); updateState(MODEM_POWER_ON)
          │      • return 1700
          │
          └── default → return 10000
```

Recovery path: **MODEM_ERROR** → **MODEM_INITIALIZING** → **MODEM_RESET** → **MODEM_POWER_ON** → then full sequence again.

---

## 4. State Machine (ModemState)

| State | Purpose |
|-------|--------|
| `MODEM_OFF` | Initial state after `initialize()`. |
| `MODEM_POWER_ON` | Apply PWRKEY pulse, then wait for boot. |
| `MODEM_RESET` | Drive power pin LOW before power-on (used after INITIALIZING). |
| `MODEM_ACTIVATE_CNACT_FIRST` | First PDP activation (AT+CNACT); skip if IP already present. |
| `MODEM_WAIT_FOR_IP_FIRST` | Wait 7 s after first CNACT. |
| `MODEM_CHECK_IP_FIRST` | Verify IP; on failure go to boot config path. |
| `MODEM_APPLY_BOOT_CONFIG` | Set RAT (CAT-M/NB-IoT) and network mode. |
| `MODEM_CHECK_SIM_AFTER_CONFIG` | Verify SIM. |
| `MODEM_CHECK_SIGNAL_AFTER_CONFIG` | Get signal quality. |
| `MODEM_SET_APN` | Set APN and activate CNACT=1,1. |
| `MODEM_WAIT_FOR_IP_AFTER_APN` | Wait 10 s after APN/CNACT. |
| `MODEM_CHECK_IP_AFTER_APN` | Verify IP again. |
| `MODEM_SYNC_NTP_TIME` | Sync NTP (best-effort); then move to MQTT. |
| `MODEM_MQTT_CONNECTING_HANDSHAKE` | MQTT config (client id, broker, port, credentials). |
| `MODEM_MQTT_CONNECTING` | AT+SMCONN. |
| `MODEM_MQTT_CONNECTED_FIRST` | Subscribe to `nodeId/sub`, enqueue online status. |
| `MODEM_MQTT_CONNECTED` | Steady state: `loop()`, flush buffer. |
| `MODEM_WAITING_SUCCESS` | Temporary state until `setWaitingDone()` or 5 min timeout. |
| `MODEM_ERROR` | Triggers full reinit (INITIALIZING → RESET → POWER_ON). |
| `MODEM_INITIALIZING` | AT + CPOWD=1 before RESET. |

---

## 5. MQTT Message Handling

- **Incoming** (subscribe topic): Delivered via library callback → `onMqttMessage()`.
  - **Encrypted path:** Payload is HEX (nonce + ciphertext + tag). Decode, check time window (30 s vs network time), decrypt; on success call `handleMqttCommandJson(plain)`.
  - **Plain path:** Treat as UTF-8 text and call `handleMqttCommandJson(plain)`.

- **handleMqttCommandJson(plain):**
  - `"X": 1` → pump ON (if not in protection); `"X": 0` → pump OFF, set last-off-by-HEX flag and arm protection.
  - `"onPulse"` → optional config (e.g. 500–10000 ms).

- **Outgoing:** Publish topic = `nodeId`. `enqueueTelemetry(payload)` buffers; when connected, `flushMessageBuffer()` sends buffered messages (and may use `MODEM_WAITING_SUCCESS` around publish). Buffer size from `config.h`: `MQTT_BUFFER_SIZE` (e.g. 10); oldest message dropped when full.

---

## 6. Public API

| Method | Description |
|--------|-------------|
| `initialize()` | Create modem, `begin()`, set MQTT callback, state OFF. |
| `start()` | Fill topics/credentials, state POWER_ON, return DURATION_IMMEDIATELY. |
| `timeout()` | Run one step of state machine; return next timeout (ms). |
| `event()` | No-op; returns DURATION_IGNORE. |
| `getModem()` | Raw `Sim7070G*` for advanced use. |
| `isMqttConnected()` | True if state is MQTT_CONNECTED and modem reports connected. |
| `publishTelemetryNow(payload)` | Publish once; fails if not connected. |
| `enqueueTelemetry(payload)` | Add to buffer; sent when connected. |
| `getCSQ()` | Signal quality 0–31 or -1. |
| `getModemRestarts()`, `getGprsConnects()`, `getMqttConnects()` | Statistics. |
| `gotoWaitingSuccess(nextState)` | Switch to WAITING_SUCCESS; on `setWaitingDone()` go to `nextState`. |
| `setWaitingDone()` | Set flag so next timeout() leaves WAITING_SUCCESS. |

---

## 7. Design Notes

- **Single instance:** Static `s_instance` used for MQTT message callback.
- **Core 0:** Device runs on core 0 to keep AT/UART work off the main loop core.
- **Blocking in callbacks:** Decryption and time fetches use `delay()`/blocking; keep payloads and retries small.
- **Retries:** Each state has its own retry count and limit; repeated failure leads to MODEM_ERROR and full restart.
- **Buffer:** Circular MQTT buffer; flush on MODEM_MQTT_CONNECTED; on publish failure the message is re-enqueued and flush stops.
