#!/usr/bin/env python3
"""
Jetson Orin Nano + DYNAMIXEL Shield MKR 모터 제어 스크립트
 - UART(Pin 8/10) + GPIO 방향제어(Pin 7) 반이중 통신
 - DYNAMIXEL X 시리즈, Protocol 2.0 기준 (XL430, XM430, XC430, XL330 등)

설치:
    pip3 install dynamixel-sdk Jetson.GPIO pyserial

사용 예:
    python3 dxl_jetson.py scan                 # 연결된 모터 ID 찾기
    python3 dxl_jetson.py info 1               # 상태 확인
    python3 dxl_jetson.py pos 1 90             # ID 1을 90도로 이동
    python3 dxl_jetson.py pos 1 180 --speed 10 # 10rpm으로 천천히 이동
    python3 dxl_jetson.py vel 1 30 --duration 3  # 30rpm으로 3초 회전
    python3 dxl_jetson.py off 1                # 토크 해제
    python3 dxl_jetson.py repl 1               # 대화형 제어
"""
import argparse
import sys
import time

import Jetson.GPIO as GPIO
from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler

# ---------------- X 시리즈 컨트롤 테이블 ----------------
ADDR_RETURN_DELAY = 9          # 1B
ADDR_OPERATING_MODE = 11       # 1B
ADDR_VELOCITY_LIMIT = 44       # 4B
ADDR_TORQUE_ENABLE = 64        # 1B
ADDR_LED = 65                  # 1B
ADDR_HW_ERROR = 70             # 1B
ADDR_GOAL_VELOCITY = 104       # 4B
ADDR_PROFILE_ACCEL = 108       # 4B
ADDR_PROFILE_VELOCITY = 112    # 4B
ADDR_GOAL_POSITION = 116       # 4B
ADDR_MOVING = 122              # 1B
ADDR_PRESENT_VELOCITY = 128    # 4B
ADDR_PRESENT_POSITION = 132    # 4B
ADDR_PRESENT_VOLTAGE = 144     # 2B
ADDR_PRESENT_TEMP = 146        # 1B

MODE_VELOCITY = 1
MODE_POSITION = 3
MODE_EXT_POSITION = 4

POS_PER_DEG = 4096 / 360.0     # 0.088 deg/unit
RPM_PER_UNIT = 0.229           # 속도 단위


def to_signed32(v):
    return v - (1 << 32) if v & 0x80000000 else v


# ---------------- GPIO 방향제어 포트 ----------------
class GpioDirPortHandler(PortHandler):
    """송신 시에만 TX_EN을 켜고, 전송 완료 즉시 수신 모드로 전환하는 PortHandler."""

    def __init__(self, port_name, dir_pin, invert=False):
        super().__init__(port_name)
        self.dir_pin = dir_pin
        self.tx_level = GPIO.LOW if invert else GPIO.HIGH
        self.rx_level = GPIO.HIGH if invert else GPIO.LOW
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(dir_pin, GPIO.OUT, initial=self.rx_level)

    def writePort(self, packet):
        GPIO.output(self.dir_pin, self.tx_level)
        n = self.ser.write(bytes(packet))
        self.ser.flush()                       # 실제 송신 완료까지 대기 (tcdrain)
        GPIO.output(self.dir_pin, self.rx_level)
        return n



# ---------------- 모터 제어 클래스 ----------------
class DxlError(RuntimeError):
    pass


class Dynamixel:
    def __init__(self, port="/dev/ttyTHS1", baud=57600, dir_pin=7, invert=False, retries=3):
        self.port = GpioDirPortHandler(port, dir_pin, invert)
        self.pkt = PacketHandler(2.0)
        self.retries = retries
        try:
            ok = self.port.openPort()
        except Exception as e:   # 권한 없음, 장치 없음 등
            GPIO.cleanup(dir_pin)
            raise DxlError(f"포트 열기 실패: {port} ({e}) - dialout 그룹/포트 이름 확인")
        if not ok:
            raise DxlError(f"포트 열기 실패: {port}")
        if not self.port.setBaudRate(baud):
            self.port.closePort()
            raise DxlError(f"보드레이트 설정 실패: {baud}")

    def close(self):
        try:
            self.port.closePort()
        finally:
            GPIO.cleanup(self.port.dir_pin)

    def __enter__(self):
        return self

    def __exit__(self, *a):
        self.close()

    # --- 저수준 읽기/쓰기 (실패 시 재시도) ---
    def _call(self, fn, what, *args):
        last = ""
        for _ in range(self.retries):
            res = fn(self.port, *args)
            value, comm, err = (res if len(res) == 3 else (None, *res))
            if comm == COMM_SUCCESS:
                if err:
                    raise DxlError(f"{what}: {self.pkt.getRxPacketError(err)}")
                return value
            last = self.pkt.getTxRxResult(comm)
            time.sleep(0.005)
        raise DxlError(f"{what}: {last}")

    def w1(self, i, a, v): self._call(self.pkt.write1ByteTxRx, f"write1 id{i}@{a}", i, a, v)
    def w4(self, i, a, v): self._call(self.pkt.write4ByteTxRx, f"write4 id{i}@{a}", i, a, v & 0xFFFFFFFF)
    def r1(self, i, a): return self._call(self.pkt.read1ByteTxRx, f"read1 id{i}@{a}", i, a)
    def r2(self, i, a): return self._call(self.pkt.read2ByteTxRx, f"read2 id{i}@{a}", i, a)
    def r4(self, i, a): return self._call(self.pkt.read4ByteTxRx, f"read4 id{i}@{a}", i, a)

    # --- 기본 기능 ---
    def ping(self, dxl_id):
        model, comm, err = self.pkt.ping(self.port, dxl_id)
        return model if comm == COMM_SUCCESS else None

    def scan(self, max_id=20):
        found = []
        for i in range(max_id + 1):
            m = self.ping(i)
            if m is not None:
                found.append((i, m))
        return found

    def torque(self, dxl_id, on):
        self.w1(dxl_id, ADDR_TORQUE_ENABLE, 1 if on else 0)

    def led(self, dxl_id, on):
        self.w1(dxl_id, ADDR_LED, 1 if on else 0)

    def set_mode(self, dxl_id, mode):
        if self.r1(dxl_id, ADDR_OPERATING_MODE) != mode:
            self.torque(dxl_id, False)          # 모드 변경은 토크 OFF 상태에서만 가능
            self.w1(dxl_id, ADDR_OPERATING_MODE, mode)
        self.torque(dxl_id, True)

    # --- 위치 제어 ---
    def get_deg(self, dxl_id):
        return to_signed32(self.r4(dxl_id, ADDR_PRESENT_POSITION)) / POS_PER_DEG

    def move_deg(self, dxl_id, deg, speed_rpm=30.0, wait=True, timeout=10.0, tol_deg=1.0):
        if not 0 <= deg <= 360:
            raise ValueError("위치 모드에서는 0~360도 범위만 가능합니다.")
        self.set_mode(dxl_id, MODE_POSITION)
        self.w4(dxl_id, ADDR_PROFILE_VELOCITY, int(abs(speed_rpm) / RPM_PER_UNIT))
        self.w4(dxl_id, ADDR_GOAL_POSITION, min(4095, int(round(deg * POS_PER_DEG))))
        if not wait:
            return
        t0 = time.time()
        while time.time() - t0 < timeout:
            cur = self.get_deg(dxl_id)
            if abs(cur - deg) <= tol_deg and not self.r1(dxl_id, ADDR_MOVING):
                return cur
            time.sleep(0.02)
        raise DxlError(f"id{dxl_id}: {timeout}s 안에 목표 위치 도달 실패 (현재 {self.get_deg(dxl_id):.1f}도)")

    # --- 속도 제어 ---
    def set_rpm(self, dxl_id, rpm):
        self.set_mode(dxl_id, MODE_VELOCITY)
        self.w4(dxl_id, ADDR_GOAL_VELOCITY, int(rpm / RPM_PER_UNIT))

    def stop(self, dxl_id):
        mode = self.r1(dxl_id, ADDR_OPERATING_MODE)
        if mode == MODE_VELOCITY:
            self.w4(dxl_id, ADDR_GOAL_VELOCITY, 0)
        else:
            self.w4(dxl_id, ADDR_GOAL_POSITION, self.r4(dxl_id, ADDR_PRESENT_POSITION))

    # --- 상태 ---
    def status(self, dxl_id):
        return {
            "model": self.ping(dxl_id),
            "mode": self.r1(dxl_id, ADDR_OPERATING_MODE),
            "torque": self.r1(dxl_id, ADDR_TORQUE_ENABLE),
            "position_deg": round(self.get_deg(dxl_id), 2),
            "velocity_rpm": round(to_signed32(self.r4(dxl_id, ADDR_PRESENT_VELOCITY)) * RPM_PER_UNIT, 2),
            "velocity_limit_rpm": round(self.r4(dxl_id, ADDR_VELOCITY_LIMIT) * RPM_PER_UNIT, 1),
            "voltage_V": self.r2(dxl_id, ADDR_PRESENT_VOLTAGE) / 10.0,
            "temp_C": self.r1(dxl_id, ADDR_PRESENT_TEMP),
            "hw_error": self.r1(dxl_id, ADDR_HW_ERROR),
        }


# ---------------- 대화형 모드 ----------------
REPL_HELP = """명령어:
  p <deg>     위치 이동 (0~360)       예) p 90
  sp <rpm>    위치 이동 속도 설정       예) sp 10
  v <rpm>     속도 회전 (음수=역방향)   예) v -20
  s           정지
  i           상태 출력
  on / off    토크 ON / OFF
  led 1|0     LED
  q           종료 (토크 OFF)"""


def repl(dxl, dxl_id):
    speed = 30.0
    print(REPL_HELP)
    while True:
        try:
            line = input(f"dxl[{dxl_id}]> ").strip().split()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not line:
            continue
        cmd, args = line[0].lower(), line[1:]
        try:
            if cmd == "q":
                break
            elif cmd == "p":
                print(f"도착: {dxl.move_deg(dxl_id, float(args[0]), speed):.1f}도")
            elif cmd == "sp":
                speed = float(args[0]); print(f"위치 이동 속도 = {speed} rpm")
            elif cmd == "v":
                dxl.set_rpm(dxl_id, float(args[0]))
            elif cmd == "s":
                dxl.stop(dxl_id)
            elif cmd == "i":
                for k, v in dxl.status(dxl_id).items():
                    print(f"  {k:20s} {v}")
            elif cmd == "on":
                dxl.torque(dxl_id, True)
            elif cmd == "off":
                dxl.torque(dxl_id, False)
            elif cmd == "led":
                dxl.led(dxl_id, args and args[0] == "1")
            else:
                print(REPL_HELP)
        except (DxlError, ValueError, IndexError) as e:
            print(f"오류: {e}")
    try:
        dxl.stop(dxl_id)
        dxl.torque(dxl_id, False)
    except DxlError:
        pass


# ---------------- CLI ----------------
def main():
    ap = argparse.ArgumentParser(description="Jetson + DYNAMIXEL Shield MKR 제어")
    ap.add_argument("--port", default="/dev/ttyTHS1")
    ap.add_argument("--baud", type=int, default=57600)
    ap.add_argument("--dir-pin", type=int, default=7, help="TX_EN에 연결한 40핀 번호 (BOARD)")
    ap.add_argument("--dir-invert", action="store_true", help="송신=LOW 인 경우")
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("scan"); p.add_argument("--max-id", type=int, default=20)
    p = sub.add_parser("info"); p.add_argument("id", type=int)
    p = sub.add_parser("pos"); p.add_argument("id", type=int); p.add_argument("deg", type=float)
    p.add_argument("--speed", type=float, default=30.0, help="이동 속도 rpm")
    p = sub.add_parser("vel"); p.add_argument("id", type=int); p.add_argument("rpm", type=float)
    p.add_argument("--duration", type=float, default=3.0, help="회전 시간(초), 이후 정지")
    p = sub.add_parser("off"); p.add_argument("id", type=int)
    p = sub.add_parser("repl"); p.add_argument("id", type=int)
    a = ap.parse_args()

    try:
        with Dynamixel(a.port, a.baud, a.dir_pin, a.dir_invert) as dxl:
            if a.cmd == "scan":
                found = dxl.scan(a.max_id)
                if not found:
                    print("모터를 찾지 못했습니다. 전원/배선/보드레이트/--dir-invert를 확인하세요.")
                for i, m in found:
                    print(f"ID {i}: model {m}")
            elif a.cmd == "info":
                for k, v in dxl.status(a.id).items():
                    print(f"{k:20s} {v}")
            elif a.cmd == "pos":
                print(f"도착: {dxl.move_deg(a.id, a.deg, a.speed):.1f}도")
            elif a.cmd == "vel":
                dxl.set_rpm(a.id, a.rpm)
                try:
                    time.sleep(a.duration)
                finally:
                    dxl.stop(a.id)
            elif a.cmd == "off":
                dxl.torque(a.id, False)
            elif a.cmd == "repl":
                repl(dxl, a.id)
    except DxlError as e:
        print(f"오류: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()