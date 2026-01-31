# -*- coding: utf-8 -*-
"""
MQTT client: tu dong sub + giai ma tin nhan theo config.h (Sim7070GDevice).

- Khi chay: tu dong subscribe xtr/nodes/<NODE_ID> (telemetry) va xtr/server/<NODE_ID> (lenh).
- Tin nhan nhan duoc: tu dong giai ma (payload HEX -> JSON) va in ra [decrypted] hoac [plain].

Topic: xtr/nodes/<id> (nhan telemetry), xtr/server/<id> (nhan lenh).
Config: config.h (override bang env MQTT_SERVER, NODE_ID, ...).
Cai dat: pip install paho-mqtt pycryptodome
"""

from __future__ import annotations

import json
import os
import sys
import time
from typing import Any, Callable, Optional

import paho.mqtt.client as mqtt

# Import crypto từ cùng thư mục
from datetime import datetime, timezone

from mqtt_crypto import (
    DEFAULT_CRYPTOKEY_HEX,
    decode_mqtt_message_or_plain,
    encode_mqtt_message,
    encode_mqtt_message_binary,
)

# -----------------------------------------------------------------------------
# Config (giống include/config.h – override bằng env: MQTT_SERVER, MQTT_PORT, NODE_ID, ...)
# Client ID: mặc định = 5 ký tự đầu của NODE_ID (khi không điền --client-id).
# -----------------------------------------------------------------------------
CONFIG = {
    "NODE_ID": os.environ.get("NODE_ID", "5a06bafb-e479-4dc3-87d9-d79734d71f13"),
    "USERNAME": os.environ.get("MQTT_USERNAME", "node_d935168a3a77"),
    "PASSWORD": os.environ.get("MQTT_PASSWORD", "G9XqOmsUYuSgq3mmWd0ecTmP"),
    "CRYPTOKEY": os.environ.get("CRYPTOKEY", DEFAULT_CRYPTOKEY_HEX),
    "MQTT_SERVER": os.environ.get("MQTT_SERVER", "mqtt.myvpn.id.vn"),
    "MQTT_PORT": int(os.environ.get("MQTT_PORT", "1883")),
}
MQTT_KEEPALIVE_SEC = 60


def topic_pub(node_id: Optional[str] = None) -> str:
    """Topic thiết bị publish telemetry: xtr/nodes/<NODE_ID>."""
    return f"xtr/nodes/{node_id or CONFIG['NODE_ID']}"


def topic_sub(node_id: Optional[str] = None) -> str:
    """Topic thiết bị subscribe nhận lệnh: xtr/server/<NODE_ID>."""
    return f"xtr/server/{node_id or CONFIG['NODE_ID']}"


# -----------------------------------------------------------------------------
# MQTT client với encode/decode
# -----------------------------------------------------------------------------


class MqttClient:
    """
    Client MQTT: kết nối broker theo config.h, sub vào topic nhận lệnh và topic nhận telemetry,
    gửi/nhận payload được mã hóa hoặc plain (giống Sim7070GDevice).
    """

    def __init__(
        self,
        node_id: Optional[str] = None,
        username: Optional[str] = None,
        password: Optional[str] = None,
        broker: Optional[str] = None,
        port: Optional[int] = None,
        key_hex: Optional[str] = None,
        client_id: Optional[str] = None,
        verify_time_window: bool = False,
        window_sec: int = 30,
    ):
        self.node_id = node_id or CONFIG["NODE_ID"]
        self.username = username or CONFIG["USERNAME"]
        self.password = password or CONFIG["PASSWORD"]
        self.broker = broker or CONFIG["MQTT_SERVER"]
        self.port = port or CONFIG["MQTT_PORT"]
        self.key_hex = key_hex or CONFIG["CRYPTOKEY"]
        self.verify_time_window = verify_time_window
        self.window_sec = window_sec
        # Khi khong dien: client_id = 5 ky tu dau cua node_id (giong firmware shortId)
        self.client_id = client_id if client_id is not None else (self.node_id[:5] if len(self.node_id) >= 5 else self.node_id)

        self._client: Optional[mqtt.Client] = None
        self._connected = False
        self._on_message_callback: Optional[Callable[[str, str, Optional[str]], None]] = None
        self._sub_telemetry = topic_pub(self.node_id)  # nhận telemetry từ thiết bị
        self._sub_commands = topic_sub(self.node_id)   # nhận lệnh (khi chạy ở phía server)

    def _on_connect(self, client: mqtt.Client, userdata: Any, flags: dict, rc: int) -> None:
        self._connected = rc == 0
        if rc == 0:
            print(f"[MQTT] Connected to {self.broker}:{self.port}")
            # Subscribe topic nhận telemetry từ thiết bị (xtr/nodes/<id>)
            client.subscribe(self._sub_telemetry, qos=0)
            # Subscribe topic nhận lệnh gửi tới thiết bị (xtr/server/<id>) để debug/echo
            client.subscribe(self._sub_commands, qos=0)
            print(f"[MQTT] Subscribed: {self._sub_telemetry}, {self._sub_commands}")
        else:
            print(f"[MQTT] Connect failed, rc={rc}")

    def _on_disconnect(self, client: mqtt.Client, userdata: Any, *args: Any) -> None:
        self._connected = False
        # paho 1.x: (rc,); paho 2.x: (flags, reason_code, properties)
        rc = args[1] if len(args) >= 2 else (args[0] if args else 0)
        rc = getattr(rc, "value", rc)
        if rc != 0:
            print(f"[MQTT] Disconnected, rc={rc}")

    def _on_message(self, client: mqtt.Client, userdata: Any, msg: mqtt.MQTTMessage) -> None:
        topic = msg.topic
        try:
            payload_str = msg.payload.decode("utf-8", errors="replace")
        except Exception:
            payload_str = str(msg.payload)
        ref_sec = None
        if self.verify_time_window:
            now = datetime.now(timezone.utc)
            ref_sec = now.hour * 3600 + now.minute * 60 + now.second
        plain = decode_mqtt_message_or_plain(
            payload_str,
            key_hex=self.key_hex,
            verify_time_window=self.verify_time_window,
            ref_time_sec=ref_sec,
            window_sec=self.window_sec,
        )
        was_encrypted = (len(payload_str.strip()) % 2 == 0 and len(payload_str.strip()) >= (12 + 16) * 2)
        ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        if plain is None and was_encrypted:
            if self._on_message_callback:
                self._on_message_callback(topic, payload_str, None)
            else:
                print(f"[{ts}] [RX] {topic} -> [rejected] decrypt failed or outside time window (>={self.window_sec}s)")
            return
        if plain is None:
            plain = payload_str
        was_decrypted = plain != payload_str
        if self._on_message_callback:
            self._on_message_callback(topic, payload_str, plain)
        else:
            label = "decrypted" if was_decrypted else "plain"
            print(f"[{ts}] [RX] {topic} -> [{label}] {plain}")

    def set_on_message(self, callback: Callable[[str, str, Optional[str]], None]) -> None:
        """Set callback(topic, raw_payload, decoded_plain). decoded_plain=None nếu rejected (time window/tag)."""
        self._on_message_callback = callback

    def connect(self) -> bool:
        """Kết nối broker (username/password nếu có)."""
        self._client = mqtt.Client(
            client_id=self.client_id,
            protocol=mqtt.MQTTv311,
            clean_session=True,
        )
        if self.username or self.password:
            self._client.username_pw_set(self.username, self.password or None)
        self._client.on_connect = self._on_connect
        self._client.on_disconnect = self._on_disconnect
        self._client.on_message = self._on_message
        try:
            self._client.connect(self.broker, self.port, MQTT_KEEPALIVE_SEC)
            return True
        except Exception as e:
            print(f"[MQTT] Connect error: {e}")
            return False

    def loop_start(self) -> None:
        """Chạy loop trong thread nền."""
        if self._client:
            self._client.loop_start()

    def loop_stop(self) -> None:
        if self._client:
            self._client.loop_stop()

    def loop_forever(self) -> None:
        """Chạy loop chặn (blocking)."""
        if self._client:
            self._client.loop_forever()

    def disconnect(self) -> None:
        if self._client:
            self._client.disconnect()
            self._client.loop_stop()

    def publish_telemetry(self, plaintext: str | dict, encrypt: bool = True) -> bool:
        """
        Gửi telemetry lên topic xtr/nodes/<NODE_ID> (giống thiết bị).
        :param plaintext: Chuỗi JSON hoặc dict (sẽ chuyển thành JSON).
        :param encrypt: True = mã hóa BINARY (tiết kiệm CPU), False = gửi plain.
        """
        if isinstance(plaintext, dict):
            plaintext = json.dumps(plaintext, ensure_ascii=False)
        topic = topic_pub(self.node_id)
        if encrypt:
            # Use HEX encoding (SUBHEX=1) - but send as bytes to prevent double encoding
            hex_payload = encode_mqtt_message(plaintext, key_hex=self.key_hex)
            if hex_payload is None:
                print("[MQTT] Encode failed")
                return False
            # Convert HEX string to bytes for MQTT transmission
            payload = bytes.fromhex(hex_payload)
        else:
            payload = plaintext
        if not self._client or not self._connected:
            print("[MQTT] Not connected")
            return False
        self._client.publish(topic, payload, qos=0, retain=False)
        # Preview: show hex for binary payload
        if isinstance(payload, bytes):
            preview = payload.hex()[:80] + "..." if len(payload.hex()) > 80 else payload.hex()
        else:
            preview = (payload[:80] + "...") if len(payload) > 80 else payload
        ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        print(f"[{ts}] [TX] {topic} (encrypt={encrypt}) -> {preview}")
        return True

    def publish_command(self, plaintext: str | dict, encrypt: bool = True) -> bool:
        """
        Gửi lệnh tới thiết bị trên topic xtr/server/<NODE_ID>.
        :param plaintext: JSON string hoặc dict, ví dụ {"X": 1} bật bơm, {"X": 0} tắt.
        :param encrypt: True = mã hóa BINARY (thiết bị chấp nhận, tiết kiệm CPU), False = plain.
        """
        if isinstance(plaintext, dict):
            plaintext = json.dumps(plaintext, ensure_ascii=False)
        topic = topic_sub(self.node_id)
        if encrypt:
            # Use HEX encoding (SUBHEX=1) - but send as bytes to prevent double encoding
            try:
                hex_payload = encode_mqtt_message(plaintext, key_hex=self.key_hex)
                if hex_payload is None:
                    print("[MQTT] Encode failed (returned None)")
                    return False
                # Convert HEX string to bytes for MQTT transmission
                # This prevents paho-mqtt from encoding the string again
                payload = bytes.fromhex(hex_payload)
                print(f"[DEBUG] Encrypted payload: {len(hex_payload)} chars HEX → {len(payload)} bytes")
            except Exception as e:
                print(f"[MQTT] Encode exception: {e}")
                import traceback
                traceback.print_exc()
                return False
        else:
            payload = plaintext
        if not self._client or not self._connected:
            print("[MQTT] Not connected")
            return False
        # Debug: print payload info before publishing
        if isinstance(payload, bytes):
            print(f"[DEBUG] Publishing bytes: len={len(payload)}, hex={payload[:20].hex()}...")
        else:
            print(f"[DEBUG] Publishing string: len={len(payload)}, content={payload[:50]}")
        
        result = self._client.publish(topic, payload, qos=0, retain=False)
        print(f"[DEBUG] Publish result: rc={result.rc}, mid={result.mid}")
        
        ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        print(f"[{ts}] [TX] {topic} (encrypt={encrypt}) -> {plaintext}")
        return True


# -----------------------------------------------------------------------------
# CLI: chạy client, sub + gửi lệnh / telemetry
# -----------------------------------------------------------------------------

def main() -> None:
    import argparse
    parser = argparse.ArgumentParser(description="MQTT client (config from config.h)")
    parser.add_argument("--broker", default=CONFIG["MQTT_SERVER"], help="MQTT broker host")
    parser.add_argument("--port", type=int, default=CONFIG["MQTT_PORT"], help="MQTT port")
    parser.add_argument("--node", default=CONFIG["NODE_ID"], help="NODE_ID for topics (xtr/nodes/<id>, xtr/server/<id>)")
    parser.add_argument("--client-id", dest="client_id", default=None, help="MQTT client ID (default: 5 ky tu dau cua --node)")
    parser.add_argument("--user", default=CONFIG["USERNAME"], help="MQTT username")
    parser.add_argument("--pass", dest="password", default=CONFIG["PASSWORD"], help="MQTT password")
    parser.add_argument("--cmd", type=str, help="Send command JSON, e.g. '{\"X\":1}' or {\"X\":0}")
    parser.add_argument("--telem", type=str, help="Send telemetry JSON (encrypted to xtr/nodes/<id>)")
    parser.add_argument("--no-encrypt", action="store_true", help="Send --cmd/--telem as plain text")
    parser.add_argument("--once", action="store_true", help="Send then exit (no subscribe loop)")
    parser.add_argument("--verify-time-window", action="store_true", help="Khi nhan tin: kiem tra time window 30s (chong replay, giong firmware)")
    args = parser.parse_args()

    client = MqttClient(
        node_id=args.node,
        client_id=args.client_id,
        username=args.user,
        password=args.password,
        broker=args.broker,
        port=args.port,
        verify_time_window=args.verify_time_window,
    )
    if not args.client_id:
        print(f"[MQTT] Client ID: {client.client_id} (5 ky tu dau cua NODE_ID)")
    if not client.connect():
        sys.exit(1)

    # Tu dong sub: xtr/nodes/<id> (telemetry), xtr/server/<id> (lenh) -> giai ma va in ra
    sub_telem = topic_pub(args.node)
    sub_cmd = topic_sub(args.node)
    print(f"[MQTT] Sub: {sub_telem}")
    print(f"[MQTT] Sub: {sub_cmd}")
    print("[MQTT] Tin nhan nhan duoc se tu dong giai ma (hex -> JSON) va in ra.")

    client.loop_start()
    for _ in range(25):
        time.sleep(0.2)
        if client._connected:
            break
    if not client._client or not client._connected:
        print("[MQTT] Not connected after 5s, exit.")
        sys.exit(1)

    if args.cmd:
        try:
            cmd = json.loads(args.cmd) if args.cmd.strip().startswith("{") else {"X": int(args.cmd)}
        except (json.JSONDecodeError, ValueError):
            cmd = {"raw": args.cmd}
        client.publish_command(cmd, encrypt=not args.no_encrypt)
    if args.telem:
        try:
            telem = json.loads(args.telem)
        except json.JSONDecodeError:
            telem = {"body": args.telem}
        client.publish_telemetry(telem, encrypt=not args.no_encrypt)

    if args.once and (args.cmd or args.telem):
        time.sleep(0.5)
        client.disconnect()
        return

    print("[MQTT] Dang nhan tin (Ctrl+C thoat)...")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    client.disconnect()


if __name__ == "__main__":
    main()
