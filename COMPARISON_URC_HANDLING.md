# 📊 So sánh xử lý URC và nhận MQTT message

## 🔍 Tổng quan

| Aspect | **main_old.cpp** | **Sim7070GDevice.cpp** |
|--------|------------------|------------------------|
| **Architecture** | Monolithic (tất cả trong 1 file) | Modular (class-based) |
| **URC Handling** | Manual parsing trong `loop()` | Library callback từ `Sim7070G_AT.cpp` |
| **MQTT Config** | Unknown (không thấy SUBHEX config) | **SUBHEX=1** (explicit) |
| **Payload Format** | HEX string từ `+SMSUB:` URC | HEX string từ callback |

---

## 📥 Flow nhận MQTT Message

### **main_old.cpp** (Manual URC Parsing)

```
┌─────────────────────────────────────────────────────────────┐
│ 1. loop() calls handleMQTTMessages()                        │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│ 2. while (sim7070.available())                              │
│    - Read char by char from Serial                          │
│    - Build line in static buffer incoming[600]              │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│ 3. if (strncmp(incoming, "+SMSUB", 6) == 0)                 │
│    - Manual string parsing to extract payload               │
│    - Find first ',' then first '"'                          │
│    - Extract HEX string between quotes                      │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│ 4. Process HEX payload                                      │
│    - hexDecode() → binary                                   │
│    - Extract nonce, ct, tag                                 │
│    - Time validation                                        │
│    - Decrypt                                                │
└─────────────────────────────────────────────────────────────┘
```

**Code:**

```cpp
void handleMQTTMessages() {
  static char incoming[600];
  static byte idx = 0;

  while (sim7070.available()) {
    char c = sim7070.read();
    noteUartActivity();
    if (c == '\n' || c == '\r') {
      incoming[idx] = '\0';
      if (idx > 0) {
        // Check for +SMSUB: URC
        if (strncmp(incoming, "+SMSUB", 6) == 0) {
          // Manual parsing: find payload between quotes
          char* payload = strchr(incoming, ',');
          if (payload) payload = strchr(payload + 1, '"');
          if (payload) {
            payload++;
            char* end = strchr(payload, '"');
            if (end) *end = '\0';

            DEBUG_PRINT(F("[RX] HEX payload (raw), len="));
            DEBUG_PRINTLN(strlen(payload));
            DEBUG_PRINTLN(payload);

            // Process HEX payload
            const char* hexStr = payload;
            size_t hexLen = strlen(hexStr);
            if ((hexLen % 2) == 0 && hexLen >= (12 + 16) * 2) {
              uint8_t buf[480];
              size_t bytes = 0;
              if (hexDecode(hexStr, buf, sizeof(buf), bytes) && bytes >= (12 + 16)) {
                // Extract nonce, tag, ciphertext
                uint8_t* nonce = buf;
                uint8_t* tag = buf + (bytes - 16);
                uint8_t* ct = buf + 12;
                size_t ctLen = bytes - 12 - 16;

                // Time validation
                int nH = nonce[4];
                int nM = nonce[5];
                int nS = nonce[6];
                int secMsg = secsFromHMS(nH, nM, nS);

                // Get network time (5 retries)
                char isoNow[21] = { 0 };
                int rH = 0, rM = 0, rS = 0;
                bool timeOk = false;

                for (uint8_t attempt = 1; attempt <= 5; attempt++) {
                  if (getNetworkTimeISO8601(isoNow, sizeof(isoNow))) {
                    if (sscanf(isoNow + 11, "%2d:%2d:%2d", &rH, &rM, &rS) == 3) {
                      timeOk = true;
                      break;
                    }
                  }
                  delay(200 * attempt);
                  WDT_RESET();
                }

                if (!timeOk) {
                  DEBUG_PRINTLN(F("[TIME] Unable to obtain hour from network after 3 tries. Command discarded."));
                  idx = 0;
                  continue;
                }

                // Calculate time difference
                const int secNow = secsFromHMS(rH, rM, rS);
                const int dsec = circDiffSecs(secMsg, secNow);

                DIAG_PRINT(F(" | NET HMS="));
                DIAG_PRINT(isoNow + 11);
                DIAG_PRINT(F(" | Δs="));
                DIAG_PRINTLN(dsec);

                // Decrypt
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

                // Time window check
                const int WINDOW_SEC = 30;
                if (dsec > WINDOW_SEC) {
                  DEBUG_PRINTLN(F("[TIME] Command outside of time window (>30 s). Ignored."));
                  idx = 0;
                  if (publishMessage()) lastPublish = millis();
                  continue;
                }

                // Process JSON (X field, onPulse, etc.)
                // ...
              }
            }
          }
        }
      }
      idx = 0;
    } else if (idx < sizeof(incoming) - 1) {
      incoming[idx++] = c;
    }
  }
}
```

---

### **Sim7070GDevice.cpp** (Library Callback)

```
┌─────────────────────────────────────────────────────────────┐
│ 1. Sim7070G_AT library receives URC                         │
│    - Parses +SMSUB: automatically                           │
│    - Extracts topic and payload                             │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│ 2. Library calls registered callback:                       │
│    onMqttMessage(topic, payload, len)                       │
│    - payload is already extracted (uint8_t*)                │
│    - len is payload length                                  │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│ 3. Sim7070GDevice::onMqttMessage()                          │
│    - Convert payload to null-terminated string              │
│    - Process as HEX string                                  │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│ 4. Same processing as main_old.cpp                          │
│    - hexDecode() → binary                                   │
│    - Extract nonce, ct, tag                                 │
│    - Time validation                                        │
│    - Decrypt                                                │
└─────────────────────────────────────────────────────────────┘
```

**Code:**

```cpp
void Sim7070GDevice::onMqttMessage(const char *topic, const uint8_t *payload, uint32_t len)
{
  DEBUG_PRINT(F("[SIM7070G] MQTT RX topic="));
  DEBUG_PRINT(topic ? topic : "(null)");
  DEBUG_PRINT(F(" len="));
  DEBUG_PRINTLN(len);

  if (!payload || len == 0) {
    return;
  }

  // Convert payload to null-terminated string
  char hexPayload[600];
  size_t hexLen = (len > sizeof(hexPayload) - 1) ? sizeof(hexPayload) - 1 : len;
  memcpy(hexPayload, payload, hexLen);
  hexPayload[hexLen] = '\0';

  DEBUG_PRINT(F("[RX] HEX payload len="));
  DEBUG_PRINTLN(strlen(hexPayload));

  const char *hexStr = hexPayload;
  size_t hexStrLen = strlen(hexStr);

  // Encrypted path: valid HEX length and successful decode
  if ((hexStrLen % 2) == 0 && hexStrLen >= (12 + 16) * 2)
  {
    DEBUG_PRINTLN(F("[DEBUG] HEX length check passed"));
    uint8_t buf[480];
    size_t bytes = 0;
    bool decodeOk = hexDecode(hexStr, buf, sizeof(buf), bytes);
    DEBUG_PRINT(F("[DEBUG] hexDecode result: "));
    DEBUG_PRINT(decodeOk ? F("OK") : F("FAIL"));
    DEBUG_PRINT(F(", bytes="));
    DEBUG_PRINTLN(bytes);
    
    if (decodeOk && bytes >= (12 + 16))
    {
      DEBUG_PRINT(F("[RX] Decoded binary, bytes="));
      DEBUG_PRINTLN(bytes);

      // Extract nonce, ciphertext, and tag
      uint8_t *nonce = buf;
      uint8_t *tag = buf + (bytes - 16);
      uint8_t *ct = buf + 12;
      size_t ctLen = bytes - 12 - 16;

      // Debug: print nonce in HEX
      DEBUG_PRINT(F("[DEBUG] Nonce hex: "));
      for (int i = 0; i < 12; i++) {
        if (nonce[i] < 0x10) DEBUG_PRINT(F("0"));
        DEBUG_PRINT(nonce[i], HEX);
      }
      DEBUG_PRINTLN();

      // Extract time from nonce
      int nH = nonce[4];
      int nM = nonce[5];
      int nS = nonce[6];
      int secMsg = nH * 3600 + nM * 60 + nS;
      
      DEBUG_PRINT(F("[TIME] Nonce time: "));
      DEBUG_PRINT(nH);
      DEBUG_PRINT(F(":"));
      DEBUG_PRINT(nM);
      DEBUG_PRINT(F(":"));
      DEBUG_PRINT(nS);
      DEBUG_PRINT(F(" (secMsg="));
      DEBUG_PRINT(secMsg);
      DEBUG_PRINTLN(F(")"));

      // Get network time UTC (from CCLK converted to ISO) for comparison
      char isoNow[21] = {0};
      int rH = 0, rM = 0, rS = 0;
      bool timeOk = false;

      for (uint8_t attempt = 1; attempt <= 5; attempt++)
      {
        DEBUG_PRINT(F("[TIME] Try number "));
        DEBUG_PRINT(attempt);
        DEBUG_PRINTLN(F(" to obtain ISO time (UTC)..."));

        memset(isoNow, 0, sizeof(isoNow));
        if (_modem && _modem->getNetworkTimeISO8601(isoNow, sizeof(isoNow)))
        {
          DEBUG_PRINT(F("[TIME] Response CCLK ISO: "));
          DEBUG_PRINTLN(isoNow);

          if (sscanf(isoNow + 11, "%2d:%2d:%2d", &rH, &rM, &rS) == 3)
          {
            DEBUG_PRINT(F("[TIME] Successful parsing of time on try number: "));
            DEBUG_PRINTLN(attempt);
            timeOk = true;
            break;
          }
          else
          {
            DEBUG_PRINTLN(F("[TIME] Failure to parse format HH:MM:SS."));
          }
        }
        else
        {
          DEBUG_PRINTLN(F("[TIME] getNetworkTimeISO8601() failed (without response)."));
        }

        delay(200 * attempt);
      }

      if (!timeOk)
      {
        DEBUG_PRINTLN(F("[TIME] Unable to obtain UTC from network after 5 tries. Command discarded."));
        return;
      }

      // Calculate time difference (circular difference for 24-hour clock) - both sides use UTC
      const int secNow = rH * 3600 + rM * 60 + rS;
      const int DAY = 86400;
      int d = abs(secMsg - secNow);
      if (d > DAY)
        d %= DAY;
      int dsec = min(d, DAY - d);

      DEBUG_PRINT(F(" | NET UTC HMS="));
      DEBUG_PRINT(isoNow + 11);
      DEBUG_PRINT(F(" | Δs="));
      DEBUG_PRINTLN(dsec);

      // Check time window (30 seconds)
      const int WINDOW_SEC = 30;
      if (dsec > WINDOW_SEC)
      {
        DEBUG_PRINTLN(F("[TIME] Command outside of time window (>30 s). Ignored."));
        return;
      }

      // Decrypt payload
      char plain[420];
      bool ok = decryptPayload(ct, ctLen, tag, nonce, plain, sizeof(plain));
      DEBUG_PRINT(F("[DECRYPT] Result: "));
      DEBUG_PRINTLN(ok ? F("OK") : F("FAIL"));
      if (!ok)
      {
        return;
      }

      DEBUG_PRINT(F("[RX] clear JSON: "));
      DEBUG_PRINTLN(plain);

      // Process JSON...
    }
  }
}
```

---

## 🔑 Key Differences

### 1. **URC Parsing**

| **main_old.cpp** | **Sim7070GDevice.cpp** |
|------------------|------------------------|
| ❌ Manual parsing char-by-char | ✅ Library handles parsing |
| ❌ String manipulation (`strchr`, `strncmp`) | ✅ Clean callback interface |
| ❌ Static buffer management | ✅ Library manages buffers |
| ❌ Error-prone (quote matching, etc.) | ✅ Robust parsing in library |

**Example URC format:**
```
+SMSUB: "xtr/server/5a06bafb-e479-4dc3-87d9-d79734d71f13","07EA011F0E2E09..."
```

**main_old.cpp** phải:
1. Tìm dấu `,` đầu tiên
2. Tìm dấu `"` sau đó
3. Tìm dấu `"` kết thúc
4. Extract string giữa 2 dấu `"`

**Sim7070GDevice.cpp**:
- Library đã làm tất cả, chỉ nhận `payload` pointer!

---

### 2. **Payload Reception**

| **main_old.cpp** | **Sim7070GDevice.cpp** |
|------------------|------------------------|
| Receives: `char*` string | Receives: `uint8_t*` + `len` |
| Uses `strlen()` to get length | Length provided by library |
| Assumes null-terminated | Safer with explicit length |

**Problem với main_old.cpp:**
- Nếu payload có null byte (0x00), `strlen()` sẽ trả về length sai!
- Nhưng với HEX encoding, không có null byte trong string

---

### 3. **MQTT Configuration**

**main_old.cpp:**
- Không thấy explicit SUBHEX config
- Có thể default là SUBHEX=1 (HEX mode)

**Sim7070GDevice.cpp:**
```cpp
// Line 313-314
_modem->mqttSetConfig("SUBHEX", "1");  // Explicit HEX mode

// Line 916-917
_modem->mqttSetConfig("SUBHEX", "1");  // Set again before connect
```

✅ **Explicit configuration** = better reliability!

---

### 4. **Time Validation Logic**

| Feature | **main_old.cpp** | **Sim7070GDevice.cpp** |
|---------|------------------|------------------------|
| Retry attempts | 5 | 5 |
| Retry delay | `200 * attempt` ms | `200 * attempt` ms |
| Time window | 30 seconds | 30 seconds |
| Circular diff | ✅ Same algorithm | ✅ Same algorithm |
| WDT reset | ✅ In retry loop | ❌ Not in retry loop |

**Identical logic!** ✅

---

### 5. **Decryption**

| Feature | **main_old.cpp** | **Sim7070GDevice.cpp** |
|---------|------------------|------------------------|
| Algorithm | ChaCha20-Poly1305 | ChaCha20-Poly1305 |
| Function | `decryptPayload()` | `decryptPayload()` |
| Nonce extraction | `nonce[4,5,6]` for H:M:S | `nonce[4,5,6]` for H:M:S |
| Buffer size | `char plain[420]` | `char plain[420]` |

**Identical!** ✅

---

## 🐛 Current Problem Analysis

### **Vấn đề hiện tại:**

```
ESP32 log:
[SIM7070G] MQTT RX len=144 ❌ (Should be 72!)
[DEBUG] Nonce hex: 303745413031314630453245 ❌ (ASCII "07EA011F0E2E")
```

### **Root Cause:**

Python đang gửi **HEX string** `"07EA011F0E2E..."` (72 chars)
↓
paho-mqtt converts string → bytes: `b'07EA011F0E2E...'` (72 bytes of ASCII)
↓
MQTT Broker forwards: 72 bytes of ASCII
↓
SIM7070G (SUBHEX=1) encodes ASCII bytes → HEX: `"303745413031..."` (144 chars)
↓
ESP32 receives: 144 chars (HEX of HEX) ❌

### **Solution Applied:**

```python
# OLD (wrong):
payload = encode_mqtt_message(plaintext)  # Returns HEX string
client.publish(topic, payload)  # paho converts to ASCII bytes

# NEW (correct):
hex_payload = encode_mqtt_message(plaintext)  # HEX string
payload = bytes.fromhex(hex_payload)  # Convert to binary bytes
client.publish(topic, payload)  # paho sends binary as-is
```

**Expected result:**

Python sends: 36 binary bytes
↓
SIM7070G (SUBHEX=1) encodes → 72 chars HEX
↓
ESP32 receives: 72 chars HEX ✅
↓
hexDecode → 36 binary bytes ✅
↓
Nonce: `07EA011F0E2E...` ✅

---

## 📝 Recommendations

### **For main_old.cpp:**

1. ✅ **Keep manual parsing** if you don't want library dependency
2. ⚠️ **Add WDT_RESET()** in time retry loop (already has it)
3. ⚠️ **Add explicit SUBHEX=1** config for clarity

### **For Sim7070GDevice.cpp:**

1. ✅ **Library-based approach** is cleaner and more robust
2. ✅ **Explicit SUBHEX=1** configuration
3. ⚠️ **Add WDT_RESET()** in time retry loop (missing)
4. ✅ **Better debug logging** (nonce hex, decode result)

### **For Python client:**

1. ✅ **Send binary bytes** (not HEX string) to prevent double encoding
2. ✅ **Add debug logging** for payload type and length
3. ✅ **Restart client** after code changes!

---

## 🎯 Next Steps

1. **Restart Python client** with updated code
2. **Test MQTT message** with `{"X": 1}`
3. **Verify ESP32 log:**
   - `len=72` ✅
   - `Nonce hex: 07EA011F...` ✅
   - `Δs < 30` ✅
   - `DECRYPT: OK` ✅

**Hãy restart Python và test!** 🚀
