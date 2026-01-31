# -*- coding: utf-8 -*-
"""
Mã hóa / giải mã tin nhắn MQTT tương thích với Sim7070GDevice.cpp và crypto_json_utils.

Định dạng payload HEX: nonce(12 bytes) + ciphertext + tag(16 bytes) → tất cả encode hex nối liền.
Thuật toán: ChaCha20-Poly1305 (AEAD), key 32 bytes (64 ký tự hex).

Cài đặt: pip install pycryptodome
"""

from __future__ import annotations

import struct
import sys
from datetime import datetime, timezone
from typing import Optional, Tuple

# ChaCha20-Poly1305: pycryptodome (pip install pycryptodome)
try:
    from Cryptodome.Cipher import ChaCha20_Poly1305
except ImportError:
    from Crypto.Cipher import ChaCha20_Poly1305


# -----------------------------------------------------------------------------
# Mặc định key (giống config.h CONFIG_CRYPTOKEY) – nên override bằng biến môi trường
# -----------------------------------------------------------------------------
DEFAULT_CRYPTOKEY_HEX = (
    "B262DF3DCFCAEB149785BFDB3E84CF1535EF0F849FCB702449CD9A5DC037545F"
)


def hex_encode(data: bytes, uppercase: bool = True) -> str:
    """Encode binary sang chuỗi hex (giống hexEncode trong C++)."""
    s = data.hex()
    return s.upper() if uppercase else s.lower()


def hex_decode(hex_str: str) -> Optional[bytes]:
    """Decode chuỗi hex sang bytes. Trả về None nếu không hợp lệ (giống hexDecode C++)."""
    hex_str = hex_str.strip()
    if len(hex_str) % 2 != 0:
        return None
    try:
        return bytes.fromhex(hex_str)
    except ValueError:
        return None


def _key_from_hex(key_hex: str) -> Optional[bytes]:
    """Chuyển key hex (64 ký tự) thành 32 bytes. Trả về None nếu sai format."""
    raw = hex_decode(key_hex)
    if raw is None or len(raw) != 32:
        return None
    return raw


# Counter per second để tạo nonce unique trong cùng giây (giống makeNonce12 C++)
_per_second_counter = 0
_last_nonce_second: Optional[Tuple[int, int, int]] = None


def make_nonce_12(
    dt: Optional[datetime] = None,
    counter_override: Optional[int] = None,
) -> bytes:
    """
    Tạo nonce 12 bytes giống crypto_json_utils (makeNonce12).
    Layout: Y(2) | Mo | D | H | Mi | S | counter(5 bytes, 40-bit big-endian).

    :param dt: Thời điểm dùng cho nonce (mặc định UTC now).
    :param counter_override: Nếu set, dùng giá trị này thay vì counter tự tăng.
    :return: 12 bytes nonce.
    """
    global _per_second_counter, _last_nonce_second

    if dt is None:
        dt = datetime.now(timezone.utc)
    if dt.tzinfo is None:
        dt = dt.replace(tzinfo=timezone.utc)

    y = dt.year
    mo, d = dt.month, dt.day
    h, mi, s = dt.hour, dt.minute, dt.second

    key_sec = (y, mo, d, h, mi, s)
    if _last_nonce_second != key_sec:
        _per_second_counter = 0
        _last_nonce_second = key_sec

    if counter_override is not None:
        c = counter_override & 0xFFFFFFFFFF
    else:
        c = _per_second_counter & 0xFFFFFFFFFF
        _per_second_counter += 1

    # Y 2 bytes big-endian, Mo, D, H, Mi, S, counter 5 bytes big-endian
    nonce = bytearray(12)
    nonce[0] = (y >> 8) & 0xFF
    nonce[1] = y & 0xFF
    nonce[2] = mo
    nonce[3] = d
    nonce[4] = h
    nonce[5] = mi
    nonce[6] = s
    nonce[7] = (c >> 32) & 0xFF
    nonce[8] = (c >> 24) & 0xFF
    nonce[9] = (c >> 16) & 0xFF
    nonce[10] = (c >> 8) & 0xFF
    nonce[11] = c & 0xFF
    return bytes(nonce)


def parse_nonce_time(nonce: bytes) -> Tuple[int, int, int]:
    """Lấy (hour, minute, second) từ nonce 12 bytes (index 4,5,6 như trong Sim7070GDevice)."""
    if len(nonce) < 7:
        return 0, 0, 0
    return int(nonce[4]), int(nonce[5]), int(nonce[6])


def time_window_seconds(
    msg_sec: int,
    ref_sec: int,
    window_sec: int = 30,
    day_sec: int = 86400,
) -> bool:
    """
    Kiểm tra khoảng thời gian (giống Sim7070GDevice: 30 giây, đồng hồ 24h).
    Trả về True nếu |msg_sec - ref_sec| (theo vòng 24h) <= window_sec.
    """
    d = abs(msg_sec - ref_sec)
    if d > day_sec:
        d %= day_sec
    dsec = min(d, day_sec - d)
    return dsec <= window_sec


# -----------------------------------------------------------------------------
# Mã hóa / giải mã payload (ChaCha20-Poly1305)
# -----------------------------------------------------------------------------


def encrypt_payload(
    plaintext: str | bytes,
    nonce: bytes,
    key: bytes,
) -> Tuple[bytes, bytes]:
    """
    Mã hóa plaintext với ChaCha20-Poly1305 (giống encryptPayload C++).
    :param plaintext: Chuỗi hoặc bytes cần mã hóa.
    :param nonce: 12 bytes nonce.
    :param key: 32 bytes key.
    :return: (ciphertext, tag) với tag 16 bytes.
    """
    if isinstance(plaintext, str):
        plaintext = plaintext.encode("utf-8")
    if len(nonce) != 12 or len(key) != 32:
        raise ValueError("nonce phải 12 bytes, key phải 32 bytes")
    cipher = ChaCha20_Poly1305.new(key=key, nonce=nonce)
    ct = cipher.encrypt(plaintext)
    tag = cipher.digest()
    return ct, tag


def decrypt_payload(
    ciphertext: bytes,
    tag: bytes,
    nonce: bytes,
    key: bytes,
) -> Optional[bytes]:
    """
    Giải mã và xác thực tag (giống decryptPayload C++).
    :return: Plaintext bytes hoặc None nếu thất bại (tag sai).
    """
    if len(nonce) != 12 or len(key) != 32 or len(tag) != 16:
        return None
    try:
        cipher = ChaCha20_Poly1305.new(key=key, nonce=nonce)
        plain = cipher.decrypt_and_verify(ciphertext, tag)
        return plain
    except (ValueError, KeyError):
        return None


# -----------------------------------------------------------------------------
# Encode / decode tin nhắn MQTT (payload HEX)
# -----------------------------------------------------------------------------


def encode_mqtt_message_binary(
    plaintext: str | bytes,
    key_hex: Optional[str] = None,
    nonce: Optional[bytes] = None,
    dt: Optional[datetime] = None,
) -> Optional[bytes]:
    """
    Mã hóa tin nhắn thành payload BINARY để gửi MQTT (nonce + ciphertext + tag).
    Dùng khi SIM7070G config SUBHEX=0 (tiết kiệm CPU, không cần encode/decode HEX).
    :param plaintext: Nội dung cần mã hóa (string hoặc bytes).
    :param key_hex: Key 64 ký tự hex (mặc định DEFAULT_CRYPTOKEY_HEX).
    :param nonce: Nonce 12 bytes (nếu None sẽ tạo từ dt hoặc thời gian hiện tại).
    :param dt: Dùng để tạo nonce nếu nonce=None.
    :return: Binary bytes (nonce + ciphertext + tag) hoặc None nếu lỗi (key sai).
    """
    key_hex = key_hex or DEFAULT_CRYPTOKEY_HEX
    key = _key_from_hex(key_hex)
    if key is None:
        return None
    if nonce is None:
        nonce = make_nonce_12(dt=dt)
    ct, tag = encrypt_payload(plaintext, nonce, key)
    return nonce + ct + tag


def encode_mqtt_message(
    plaintext: str | bytes,
    key_hex: Optional[str] = None,
    nonce: Optional[bytes] = None,
    dt: Optional[datetime] = None,
    hex_uppercase: bool = True,
) -> Optional[str]:
    """
    Mã hóa tin nhắn thành payload HEX để gửi MQTT (giống main.cpp: nonce_hex + ct_hex + tag_hex).
    Dùng khi SIM7070G config SUBHEX=1 (modem trả về HEX string).
    :param plaintext: Nội dung cần mã hóa (string hoặc bytes).
    :param key_hex: Key 64 ký tự hex (mặc định DEFAULT_CRYPTOKEY_HEX).
    :param nonce: Nonce 12 bytes (nếu None sẽ tạo từ dt hoặc thời gian hiện tại).
    :param dt: Dùng để tạo nonce nếu nonce=None.
    :param hex_uppercase: True = hex in hoa (giống C++).
    :return: Chuỗi hex hoặc None nếu lỗi (key sai).
    """
    key_hex = key_hex or DEFAULT_CRYPTOKEY_HEX
    key = _key_from_hex(key_hex)
    if key is None:
        return None
    if nonce is None:
        nonce = make_nonce_12(dt=dt)
    ct, tag = encrypt_payload(plaintext, nonce, key)
    parts = [
        hex_encode(nonce, hex_uppercase),
        hex_encode(ct, hex_uppercase),
        hex_encode(tag, hex_uppercase),
    ]
    return "".join(parts)


def decode_mqtt_message(
    hex_payload: str,
    key_hex: Optional[str] = None,
    verify_time_window: bool = False,
    ref_time_sec: Optional[int] = None,
    window_sec: int = 30,
) -> Optional[str]:
    """
    Giải mã payload MQTT HEX nhận từ thiết bị (giống onMqttMessage trong Sim7070GDevice.cpp).
    Payload hợp lệ: chuỗi hex có độ dài chẵn và >= (12+16)*2, decode ra nonce(12) + ciphertext + tag(16).
    :param hex_payload: Chuỗi hex (nonce + ciphertext + tag).
    :param key_hex: Key 64 ký tự hex (mặc định DEFAULT_CRYPTOKEY_HEX).
    :param verify_time_window: Nếu True, kiểm tra thời gian trong nonce nằm trong window_sec so với ref_time_sec.
    :param ref_time_sec: Giây trong ngày (H*3600 + M*60 + S) của thời gian tham chiếu (ví dụ từ NTP).
    :param window_sec: Cửa sổ cho phép (mặc định 30 giây như firmware).
    :return: Chuỗi plaintext (UTF-8) hoặc None nếu không phải payload mã hóa hợp lệ / tag sai / fail time window.
    """
    key_hex = key_hex or DEFAULT_CRYPTOKEY_HEX
    key = _key_from_hex(key_hex)
    if key is None:
        return None

    raw = hex_decode(hex_payload)
    if raw is None or len(raw) < 12 + 16:
        return None

    nonce = raw[:12]
    tag = raw[-16:]
    ct = raw[12:-16]

    if verify_time_window and ref_time_sec is not None:
        h, m, s = parse_nonce_time(nonce)
        msg_sec = h * 3600 + m * 60 + s
        if not time_window_seconds(msg_sec, ref_time_sec, window_sec):
            return None

    plain = decrypt_payload(ct, tag, nonce, key)
    if plain is None:
        return None
    return plain.decode("utf-8", errors="replace")


def decode_mqtt_message_or_plain(
    payload: str | bytes,
    key_hex: Optional[str] = None,
    verify_time_window: bool = False,
    ref_time_sec: Optional[int] = None,
    window_sec: int = 30,
) -> Optional[str]:
    """
    Giải mã nếu payload là HEX mã hóa (giống onMqttMessage firmware), ngược lại trả về payload plain.
    Có thể bật verify_time_window + ref_time_sec để chống replay (cửa sổ window_sec giây, mặc định 30).
    :param payload: Chuỗi hex hoặc plain text (có thể bytes → decode utf-8).
    :param verify_time_window: Nếu True, kiểm tra thời gian trong nonce nằm trong window_sec so với ref_time_sec.
    :param ref_time_sec: Giây trong ngày UTC (H*3600+M*60+S). Cần set khi verify_time_window=True.
    :param window_sec: Cửa sổ cho phép (mặc định 30, giống firmware).
    :return: Chuỗi plain text, hoặc None nếu payload là HEX mã hóa nhưng giải mã/time-window thất bại.
    """
    if isinstance(payload, bytes):
        payload = payload.decode("utf-8", errors="replace")
    hex_len = len(payload.strip())
    # Encrypted path: hex chẵn và đủ dài (12+16)*2
    if (hex_len % 2 == 0) and hex_len >= (12 + 16) * 2:
        decoded = decode_mqtt_message(
            payload,
            key_hex=key_hex,
            verify_time_window=verify_time_window,
            ref_time_sec=ref_time_sec,
            window_sec=window_sec,
        )
        if decoded is not None:
            return decoded
        # Payload là encrypted nhưng giải mã hoặc time window thất bại
        return None
    # Plain text path
    return payload


# -----------------------------------------------------------------------------
# CLI: giải mã / mã hóa từ dòng lệnh
# -----------------------------------------------------------------------------

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="MQTT crypto: encode/decode (Sim7070GDevice compatible)")
    parser.add_argument("--encrypt", "-e", metavar="TEXT", help="Ma hoa plaintext (JSON) thanh hex")
    parser.add_argument("--decrypt", "-d", metavar="HEX", help="Giai ma chuoi hex thanh plaintext")
    parser.add_argument("--key", default=DEFAULT_CRYPTOKEY_HEX, help="Key 64 hex chars (default: config)")
    parser.add_argument("--no-time-window", action="store_true", help="Khong kiem tra time window khi giai ma")
    args = parser.parse_args()

    key = args.key

    if args.decrypt is not None:
        # Giai ma: paste hex -> in ra plaintext
        hex_str = args.decrypt.strip()
        if not hex_str and not sys.stdin.isatty():
            hex_str = sys.stdin.read().strip()
        if not hex_str:
            print("Error: --decrypt can chuoi hex hoac pipe/redirect", file=sys.stderr)
            sys.exit(1)
        ref_sec = None
        if not args.no_time_window:
            now = datetime.now(timezone.utc)
            ref_sec = now.hour * 3600 + now.minute * 60 + now.second
        plain = decode_mqtt_message(
            hex_str,
            key_hex=key,
            verify_time_window=not args.no_time_window,
            ref_time_sec=ref_sec,
        )
        if plain is None:
            print("Decrypt failed (tag sai hoac ngoai time window). Thu --no-time-window.", file=sys.stderr)
            sys.exit(1)
        print(plain)

    elif args.encrypt is not None:
        # Ma hoa: plaintext -> hex
        text = args.encrypt.strip()
        if not text and not sys.stdin.isatty():
            text = sys.stdin.read().strip()
        if not text:
            print("Error: --encrypt can plaintext hoac pipe/redirect", file=sys.stderr)
            sys.exit(1)
        hex_out = encode_mqtt_message(text, key_hex=key)
        if hex_out is None:
            print("Encrypt failed (key sai?).", file=sys.stderr)
            sys.exit(1)
        print(hex_out)

    else:
        # Khong tham so: chay test nhanh
        msg = '{"X":1}'
        hex_msg = encode_mqtt_message(msg, key_hex=key)
        print("Encoded (hex):", hex_msg[:80] + "..." if hex_msg and len(hex_msg) > 80 else hex_msg)
        if hex_msg:
            dec = decode_mqtt_message(hex_msg, key_hex=key)
            print("Decoded:", dec)
            assert dec == msg, "Roundtrip failed"
        print("OK: encode/decode compatible with Sim7070GDevice.")
        print("  Giai ma: python mqtt_crypto.py --decrypt \"HEXSTRING\"")
        print("  Ma hoa:  python mqtt_crypto.py --encrypt '{\"X\":1}'")
