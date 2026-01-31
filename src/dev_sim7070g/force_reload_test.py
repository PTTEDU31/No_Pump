#!/usr/bin/env python3
"""
Force reload modules and test binary encoding.
Useful for development when you don't want to restart the entire client.
"""

import sys
import os
import importlib

_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _dir)

print("=" * 60)
print("Force reloading mqtt_crypto module...")
print("=" * 60)

# Remove from cache if exists
if 'mqtt_crypto' in sys.modules:
    print("✓ Found mqtt_crypto in cache, removing...")
    del sys.modules['mqtt_crypto']

# Import fresh
import mqtt_crypto
print("✓ Imported mqtt_crypto")

# Check if new function exists
if hasattr(mqtt_crypto, 'encode_mqtt_message_binary'):
    print("✓ _binary exists!")
else:
    print("✗ encode_mqtt_message_binary NOT FOUND!")
    print("  Available functions:", dir(mqtt_crypto))
    sys.exit(1)

# Test it
plaintext = '{"X":1}'
print(f"\nTesting with: {plaintext}")

result = mqtt_crypto.encode_mqtt_message_binary(plaintext)
print(f"✓ Result type: {type(result)}")
print(f"✓ Result length: {len(result)} bytes")
print(f"✓ First 12 bytes (nonce): {result[:12].hex()}")

if len(result) >= 28:
    print("\n" + "=" * 60)
    print("✅ SUCCESS: Binary encoding works correctly!")
    print("=" * 60)
else:
    print("\n" + "=" * 60)
    print("❌ FAIL: Output too short!")
    print("=" * 60)
    sys.exit(1)
