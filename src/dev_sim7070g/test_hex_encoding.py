#!/usr/bin/env python3
"""
Test HEX encoding for MQTT (SUBHEX=1 mode)
"""

import sys
import os

_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _dir)

from mqtt_crypto import encode_mqtt_message
import json

print("=" * 60)
print("Testing HEX Encoding (SUBHEX=1)")
print("=" * 60)

# Test payload
plaintext = json.dumps({"X": 1})
print(f"\nPlaintext: {plaintext} ({len(plaintext)} bytes)")

# Encode to HEX
hex_payload = encode_mqtt_message(plaintext)

print(f"\nEncoded payload:")
print(f"  Type: {type(hex_payload)}")
print(f"  Length: {len(hex_payload)} characters")
print(f"  First 40 chars: {hex_payload[:40]}...")
print(f"  Full payload: {hex_payload}")

# Check for null bytes in original binary
from mqtt_crypto import encode_mqtt_message_binary
binary = encode_mqtt_message_binary(plaintext)
null_positions = [i for i, b in enumerate(binary) if b == 0]

print(f"\n" + "=" * 60)
print("Binary Analysis (why we need HEX):")
print("=" * 60)
print(f"Binary length: {len(binary)} bytes")
print(f"Binary hex: {binary.hex()}")
print(f"Null byte positions: {null_positions}")
print(f"Number of null bytes: {len(null_positions)}")

if null_positions:
    print(f"\n⚠️  WARNING: Binary contains {len(null_positions)} null bytes!")
    print(f"   First null at position {null_positions[0]}")
    print(f"   This would truncate payload in SUBHEX=0 mode!")
    print(f"\n✅ SOLUTION: Use HEX encoding (SUBHEX=1)")
    print(f"   HEX string has no null bytes: {hex_payload}")
else:
    print(f"\n✅ No null bytes in binary (rare!)")

print("\n" + "=" * 60)
print("Expected ESP32 behavior:")
print("=" * 60)
print(f"1. Receive HEX string: len={len(hex_payload)}")
print(f"2. Decode HEX → binary: {len(binary)} bytes")
print(f"3. Decrypt binary → JSON: {plaintext}")
print("\n✅ Ready to test with ESP32!")
