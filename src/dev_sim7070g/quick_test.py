#!/usr/bin/env python3
"""Quick test to verify encode_mqtt_message_binary works"""

import sys
import os
_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _dir)

from mqtt_crypto import encode_mqtt_message_binary

plaintext = '{"X":1}'
print(f"Input: {plaintext} ({len(plaintext)} chars)")

result = encode_mqtt_message_binary(plaintext)
print(f"Output type: {type(result)}")
print(f"Output length: {len(result)} bytes")
print(f"Output (hex): {result.hex()}")
print(f"First 12 bytes (nonce): {result[:12].hex()}")

if len(result) >= 28:  # 12 (nonce) + 16 (tag) minimum
    print("✅ PASS: Binary encoding works!")
else:
    print("❌ FAIL: Output too short!")
