# ⚠️ PYTHON CLIENT CHƯA ĐƯỢC RESTART!

## Evidence

ESP32 nhận được **HEX của HEX**:

```
[DEBUG] Nonce hex: 303745413031314630453242
```

Decode thành ASCII:
```
30 37 = '0' '7'
45 41 = 'E' 'A'
30 31 = '0' '1'
31 46 = '1' 'F'
30 45 = '0' 'E'
32 42 = '2' 'B'
```

String: `"07EA011F0E2B"`

## What This Means

Python is sending **HEX string**, but it's being **encoded to HEX again**!

Flow:
```
Python: {"X":1} → encrypt → HEX string "07EA011F..."
MQTT: Receives HEX string
SIM7070G (SUBHEX=1): Encodes to HEX again → "303745413031..."
ESP32: Decodes → Gets ASCII "07EA..." instead of binary
```

## Root Cause

**Python client is still running OLD code!**

Old code was using `encode_mqtt_message_binary()` which returns `bytes`.

New code should use `encode_mqtt_message()` which returns HEX `str`.

But if Python wasn't restarted, it's still using old code!

## Verification

Check Python output when sending command:

**Expected (NEW code):**
```
[DEBUG] Encrypted payload: 72 chars (HEX), type=<class 'str'>
[DEBUG] Publishing string: len=72, content=07EA011F0E2B25...
```

**Actual (OLD code still running):**
```
[DEBUG] Encrypted payload: 36 bytes, type=<class 'bytes'>
[DEBUG] Publishing bytes: len=36, hex=07ea011f0e2b25...
```

## Solution

### 1. CLOSE Python Client Completely

```bash
# In Python terminal/GUI:
# Press Ctrl+C or close window
# Make SURE it's completely closed!
```

### 2. Verify Code is Updated

```bash
cd src/dev_sim7070g
python -c "from mqtt_client import encode_mqtt_message; print('HEX mode ready!')"
```

Should output: `HEX mode ready!`

### 3. Restart Python Client

```bash
python mqtt_client_gui.py
```

### 4. Send Command and Check Output

**Must see:**
```
[DEBUG] Encrypted payload: 72 chars (HEX), type=<class 'str'>
```

**NOT:**
```
[DEBUG] Encrypted payload: 36 bytes, type=<class 'bytes'>
```

## Expected ESP32 Output (After Python Restart)

```
[SIM7070G] MQTT RX len=72 ← NOT 144!
[RX] HEX payload len=72
[RX] Decoded binary, bytes=36
[DEBUG] Nonce hex: 07EA011F0E2B25000000 ← Correct!
[TIME] Nonce time: 14:43:37 (secMsg=52817) ← Correct!
| NET UTC HMS=14:43:44Z | Δs=7 ← < 30!
[DECRYPT] Result: OK
[RX] clear JSON: {"X": 1}
```

## Why This Happened

1. ✅ C++ code updated to use SUBHEX=1
2. ✅ Python code updated to send HEX string
3. ❌ Python client NOT restarted
4. ❌ Still running old code (binary mode)
5. ❌ MQTT broker receives bytes
6. ❌ SIM7070G (SUBHEX=1) encodes bytes to HEX
7. ❌ ESP32 receives HEX of bytes (double encoded)

## Checklist

- [ ] Python client completely closed
- [ ] Verified code is updated
- [ ] Python client restarted
- [ ] Sent test command
- [ ] Checked Python output shows "72 chars (HEX)"
- [ ] Checked ESP32 output shows correct nonce hex
- [ ] Δs < 30 seconds
- [ ] DECRYPT: OK

## Status

⏳ **Waiting for Python client restart...**
