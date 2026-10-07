# -*- coding: utf-8 -*-
"""
STM32 G474 측 코드에 맞춘 프로토콜 구현:
- Write: Extended ID = (mode_id << 8) | driver_id
  * mode 1(Current): payload = s32 big-endian (단위 mA)
  * mode 3(Velocity): payload = s32 big-endian (단위 eRPM)
  * DLC=8 (앞 4바이트 payload + 0 패딩)
- Broadcast(8B):
  [0..1] int16 BE => pos_deg     = /10
  [2..3] int16 BE => speed_erpm  = *10
  [4..5] int16 BE => current_A   = /1000
  [6]    int8      => temp_C
  [7]    uint8     => error
"""
import struct
from typing import Dict, Tuple
from config import (
    MODE_CURRENT,
    MODE_VELOCITY,
    MODE_POSITION,
    ERROR_FLAG_CAN,
    ERROR_FLAG_SPI,
    ERROR_FLAG_DRIVER,
)

def build_ext_id(mode_id: int, driver_id: int) -> int:
    return ((mode_id & 0xFF) << 8) | (driver_id & 0xFF)

def pack_s32_be(v: int) -> bytes:
    return struct.pack(">i", int(v))

def unpack_s16_be(b0: int, b1: int) -> int:
    return struct.unpack(">h", bytes([b0, b1]))[0]

def unpack_s8(b: int) -> int:
    return struct.unpack("b", bytes([b & 0xFF]))[0]

def pack_command_payload(mode_id: int, value_float: float) -> bytes:
    """
    mode에 맞춰 s32(big-endian) 4바이트를 만든 뒤 0x00 * 4 패딩해 총 8바이트 반환.
    - MODE_CURRENT: 입력 단위 A → mA로 ×1000 후 s32
    - MODE_VELOCITY: 입력 단위 eRPM → 그대로 s32
    """
    if mode_id == MODE_CURRENT:
        raw = int(round(value_float * 1000.0))  # A -> mA
    elif mode_id == MODE_VELOCITY:
        raw = int(round(value_float))           # eRPM 원값
    elif mode_id == MODE_POSITION:
        raw = int(round(value_float * 100))           # 0.01 deg 단위로 환산
    else:
        raise ValueError(f"Unknown mode_id: {mode_id}")

    return pack_s32_be(raw) + b"\x00\x00\x00\x00"

def parse_broadcast_frame(data: bytes) -> Dict[str, float]:
    """
    8바이트 브로드캐스트 프레임을 해석해 dict로 반환.
    """
    if len(data) != 8:
        raise ValueError("Broadcast frame must be 8 bytes.")

    pos_scaled = unpack_s16_be(data[0], data[1])      # deg * 10
    spd_scaled = unpack_s16_be(data[2], data[3])      # erpm / 10 저장
    cur_scaled = unpack_s16_be(data[4], data[5])      # A * 1000
    tmp_i8 = unpack_s8(data[6])
    err_u8 = data[7]

    return {
        "pos_deg": pos_scaled / 10.0,
        "spd_erpm": spd_scaled * 10.0,
        "cur_A": cur_scaled / 1000.0,
        "temp_C": int(tmp_i8),
        "error": int(err_u8),
    }

def format_error_code(error_code: int) -> str:
    """
    에러 코드 -> 사람이 읽기 쉬운 문자열.
    예: 0x03 (CAN|SPI)
    """
    code = int(error_code) & 0xFF
    if code == 0:
        return "0x00 (OK)"

    parts = []
    if code & ERROR_FLAG_CAN:
        parts.append("CAN")
    if code & ERROR_FLAG_SPI:
        parts.append("SPI")
    if code & ERROR_FLAG_DRIVER:
        parts.append("DRIVER_FAULT")

    known_mask = ERROR_FLAG_CAN | ERROR_FLAG_SPI | ERROR_FLAG_DRIVER
    unknown = code & (~known_mask & 0xFF)
    if unknown:
        parts.append(f"UNKNOWN(0x{unknown:02X})")

    return f"0x{code:02X} (" + "|".join(parts) + ")"

def id_fields_from_eid(eid: int) -> Tuple[int, int]:
    """eid → (mode_id, driver_id)"""
    return ((eid >> 8) & 0xFF), (eid & 0xFF)

def pack_calibration_payload() -> bytes:
    """
    캘리브레이션 트리거용 페이로드.
    기본: 8바이트 0x00 패딩 (값 미사용 가정).
    펌웨어가 특정 키값을 요구한다면 여기서 4바이트 s32(BE)로 채워 넣어라.
      예) struct.pack(">i", 1) + b"\x00\x00\x00\x00"
    """
    return b"\x00\x00\x00\x00\x00\x00\x00\x00"
