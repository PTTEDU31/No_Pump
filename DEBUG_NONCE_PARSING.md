# 🐛 Debug: Nonce Parsing Issue

## Current Problem

ESP32 is reading **wrong nonce values**!

### Expected vs Actual:

**HEX Payload:**
```
07 EA 01 1F 0E 29 05 00 00 00 00 00 FDE7A858...
```

**Expected Nonce:**
```
Byte 0-1: 07 EA → Year 2026
Byte 2:   01    → Month 1
Byte 3:   1F    → Day 31
Byte 4:   0E    → Hour 14 ✅
Byte 5:   29    → Minute 41 ✅
Byte 6:   05    → Second 5 ✅

Time: 14:41:05 UTC
```

**Actual Reading (from ESP32 log):**
```
[TIME] Nonce time: 48:49:49 (secMsg=175789) ❌

48 = 0x30 (ASCII '0')
49 = 0x31 (ASCII '1')
49 = 0x31 (ASCII '1')
```

## Root Cause

ESP32 is reading **ASCII characters** instead of **binary bytes**!

This means:
- `hexDecode()` is **NOT working**
- Or buffer is **not being used correctly**
- Or pointer is **pointing to wrong location**

## Possible Causes

### 1. hexDecode() Failed Silently

```cpp
if (hexDecode(hexStr, buf, sizeof(buf), bytes) && bytes >= (12 + 16))
{
  // This block should execute
  uint8_t *nonce = buf; // Should point to decoded binary
  // ...
}
```

If `hexDecode()` returns `false`, this block is skipped!

### 2. Buffer Corruption

```cpp
uint8_t buf[480];
// After hexDecode(), buf should contain binary data
// But maybe it's still pointing to HEX string?
```

### 3. Wrong Pointer

```cpp
uint8_t *nonce = buf;
// Maybe buf is not at the right position?
```

## Debug Strategy

Added debug log to print nonce in HEX:

```cpp
DEBUG_PRINT(F("[DEBUG] Nonce hex: "));
for (int i = 0; i < 12; i++) {
  if (nonce[i] < 0x10) DEBUG_PRINT(F("0"));
  DEBUG_PRINT(nonce[i], HEX);
}
DEBUG_PRINTLN();
```

This will show us what ESP32 is actually reading!

## Expected Output (After Upload)

```
[RX] HEX payload len=144
[RX] Decoded binary, bytes=72
[DEBUG] Nonce hex: 07EA011F0E290500000000 ← Should match!
[TIME] Nonce time: 14:41:5 (secMsg=52865) ← Should be correct!
| NET UTC HMS=14:41:11Z | Δs=6 ← Should be < 30!
[DECRYPT] Result: OK
```

## If Still Wrong

### Check hexDecode() Function

```cpp
// In crypto_json_utils.cpp
bool hexDecode(const char* hex, uint8_t* out, size_t outCap, size_t& outLen) {
  // Verify this function is working correctly
  // Maybe add debug logs here
}
```

### Verify Buffer

```cpp
// After hexDecode()
DEBUG_PRINT(F("[DEBUG] First 12 bytes: "));
for (int i = 0; i < 12; i++) {
  DEBUG_PRINT(buf[i], HEX);
  DEBUG_PRINT(F(" "));
}
DEBUG_PRINTLN();
```

## Next Steps

1. ✅ Added debug log for nonce hex
2. ⏳ Upload firmware
3. ⏳ Test and check new log
4. ⏳ If nonce hex is correct → time calculation is wrong
5. ⏳ If nonce hex is wrong → hexDecode() has bug

## Manual Verification

From HEX string `07EA011F0E290500...`:

```python
hex_str = "07EA011F0E290500000000"
binary = bytes.fromhex(hex_str)
print(f"Hour: {binary[4]} (0x{binary[4]:02X})")
print(f"Minute: {binary[5]} (0x{binary[5]:02X})")
print(f"Second: {binary[6]} (0x{binary[6]:02X})")

# Output:
# Hour: 14 (0x0E)
# Minute: 41 (0x29)
# Second: 5 (0x05)
```

Time: **14:41:05 UTC** ✅

Network time: **14:41:11 UTC**

Expected Δs: **6 seconds** ✅

## Status

⏳ Waiting for firmware upload with debug logs...
