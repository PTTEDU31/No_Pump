# ✅ Ready to Test: HEX Mode (SUBHEX=1)

## Changes Summary

### Problem Found:
- Binary payload contains **null bytes** (0x00) in nonce counter
- SUBHEX=0 mode treats payload as C string → truncates at null byte
- ESP32 only received 7 bytes instead of 36 bytes

### Solution:
- Use **SUBHEX=1** (HEX mode) instead of SUBHEX=0 (binary mode)
- HEX string has no null bytes → full payload received
- Decode HEX to binary on ESP32 side

## Files Modified

### ✅ C++ (Firmware)
- `src/dev_sim7070g/Sim7070GDevice.cpp`
  - Set `subhex=true` in `mqttSetConfig()`
  - Restore HEX decoding logic
  - Decode HEX string → binary → decrypt

### ✅ Python (Client)
- `src/dev_sim7070g/mqtt_client.py`
  - Use `encode_mqtt_message()` (HEX) instead of `encode_mqtt_message_binary()`
  - Both `publish_command()` and `publish_telemetry()` updated

## Testing Steps

### 1. Close Serial Monitor
```bash
# Press Ctrl+C in serial monitor terminal
```

### 2. Upload Firmware
```bash
cd d:\git\No_Pump_
pio run --target upload
```

### 3. Open Serial Monitor
```bash
pio device monitor
```

### 4. Restart Python Client
```bash
cd src\dev_sim7070g
python mqtt_client_gui.py
```

### 5. Send Command
- Click **"Bat (X=1)"** button in GUI

## Expected Results

### Python Output:
```
[DEBUG] Encrypted payload: 72 chars (HEX), type=<class 'str'>
[DEBUG] Publishing string: len=72, content=07EA011F0E2415...
[2026-01-31 XX:XX:XX] [TX] xtr/server/5a06bafb-e479-4dc3-87d9-d79734d71f13 (encrypt=True) -> {"X": 1}
```

### ESP32 Output:
```
[SIM7070G] MQTT RX topic=xtr/server/5a06bafb-e479-4dc3-87d9-d79734d71f13 len=72
[RX] HEX payload len=72
[RX] Decoded binary, bytes=36
[TIME] Nonce time: 14:XX:XX (secMsg=XXXXX)
[TIME] Try number 1 to obtain ISO time (UTC)...
[TIME] Response CCLK ISO: 2026-01-31T14:XX:XXZ
[TIME] Successful parsing of time on try number: 1
 | NET UTC HMS=14:XX:XXZ | Δs=X
[DECRYPT] Result: OK
[RX] clear JSON: {"X": 1}
[ACT] Pump ON (X=1)
```

## Key Indicators of Success

✅ Python: `72 chars (HEX)` not `36 bytes`
✅ ESP32: `len=72` not `len=7`
✅ ESP32: `Decoded binary, bytes=36`
✅ ESP32: `Δs < 30` (time window check passes)
✅ ESP32: `DECRYPT: OK`
✅ ESP32: `{"X": 1}` decoded correctly

## Performance Comparison

| Metric | Binary (SUBHEX=0) | HEX (SUBHEX=1) |
|--------|-------------------|----------------|
| **Works?** | ❌ No (null bytes) | ✅ Yes |
| **Payload size** | 36 bytes | 72 chars |
| **Over-the-air** | 36 bytes | 72 bytes |
| **CPU encode** | None | Minimal |
| **CPU decode** | None | Minimal |
| **Reliability** | ❌ Fails | ✅ Works |

**Verdict:** HEX mode is the ONLY reliable option!

## Troubleshooting

### If Python still shows "36 bytes":
- Python client not restarted
- Old code still in memory
- **Solution:** Close Python completely and restart

### If ESP32 still shows "len=7":
- Firmware not uploaded
- Still using old code
- **Solution:** Upload firmware again

### If "DECRYPT: FAIL":
- Key mismatch
- Payload corrupted
- **Solution:** Check `CONFIG_CRYPTOKEY` matches `DEFAULT_CRYPTOKEY_HEX`

### If "Time window fail (Δs > 30)":
- ESP32 time not synced
- Python time incorrect
- **Solution:** Wait for ESP32 to sync NTP time

## Next Steps

1. ✅ Code updated
2. ⏳ Upload firmware
3. ⏳ Restart Python client
4. ⏳ Test command
5. ⏳ Verify logs match expected output

Once all steps pass → **System is working!** 🎉
