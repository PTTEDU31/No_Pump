#!/usr/bin/env python3
"""
Test script to verify what Python is actually publishing to MQTT broker.
"""

import sys
import os
import time

_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _dir)

from mqtt_client import MqttClient, CONFIG

print("=" * 60)
print("Testing MQTT Publish")
print("=" * 60)

# Create client
client = MqttClient(
    node_id=CONFIG["NODE_ID"],
    username=CONFIG["USERNAME"],
    password=CONFIG["PASSWORD"],
    key_hex=CONFIG["CRYPTOKEY"],
)

# Connect
print("\nConnecting to MQTT broker...")
client.connect()
time.sleep(2)

if not client._connected:
    print("❌ Failed to connect!")
    sys.exit(1)

print("✅ Connected!")

# Test publish
print("\n" + "=" * 60)
print("Publishing command: {'X': 1}")
print("=" * 60)

result = client.publish_command({"X": 1}, encrypt=True)

if result:
    print("✅ Publish successful!")
else:
    print("❌ Publish failed!")

# Wait a bit
time.sleep(2)

# Disconnect
client.disconnect()

print("\n" + "=" * 60)
print("Test complete!")
print("=" * 60)
print("\nCheck the output above for:")
print("  [DEBUG] Encrypted payload: XX bytes")
print("\nExpected: 36 bytes for {'X': 1}")
print("If you see 7 bytes, Python is NOT encrypting!")
