# -*- coding: utf-8 -*-
"""
손가락 한 개 위치 제어 GUI - 관절 4개 (position_control_app.py 확장, 3D_app.py 기반).

- 관절: MCP A/A = Channel 0 (CAN), MCP F/E = 다이나믹셀 (U2D2 USB, DYNAMIXEL SDK),
        PIP F/E = Channel 1 + 2 (CAN), DIP F/E = Channel 3 (CAN).
- CAN 쪽(연결, 위치 제어, 게인, Smoothing, Soft start, Initialize)은 position_control_app.py와 같다.
- ROM 보호: CAN 관절의 현재/명령 위치가 ROM 끝 근처(ROM_GUARD_DEG)면 Kd를 슬라이더 최대로 올리고,
  벗어나면 원래 Kd로 되돌린다 (구역 안에서 Kd 슬라이더를 직접 바꾸면 복귀하지 않음).
  ROM Guard 버튼으로 ON/OFF하며 기본값은 ON. OFF 시 자동으로 올린 Kd를 복원한다.
- MCP F/E 섹션: [DXL 연결] = 포트 open + 상태 수신 스레드 시작, [위치 제어] = Extended Position 모드 토크 ON.
  내부 속도/가속도 프로파일로 가감속하며, Smoothing과 Initialize(영점)도 같이 적용된다.
- 굽힘 평면 뷰: 주황 손끝을 드래그하면 최소 구동 IK로 MCP/PIP/DIP 목표를 정한다.
  같은 손끝 위치를 내는 해 중 현재 목표 관절각에서 가장 적게 움직이는 해를 고른다.
  MCP F/E 위치 제어가 OFF면 MCP는 현재 각(수신 없으면 0)으로 고정하고 PIP/DIP만 푼다.
- 3D 뷰 (보기 전용): 현재 자세(초록), 목표 자세(주황), 손끝 도달 가능 영역(파란 선, MCP F/E 포함).

실행: python3 one_finger.py
"""
import fcntl
import math
import sys
import termios
import threading
import time
from functools import lru_cache

import numpy as np
import pyqtgraph.opengl as gl
from PyQt5 import QtCore, QtGui, QtWidgets

from position_control_app import (AA_AXIS_Y, FINGER_LINKS, FINGER_SECTIONS, FINGER_TIP, GAIN_STEPS, RAMP_MS,
                                  REF_MAX_DEG, REF_MIN_DEG, REF_SCALE, SECTIONS, STALE_S, TIP_LENGTH, TIP_RADIUS,
                                  FingerView, PositionControlApp, clamp, finger_points, finger_workspace,
                                  heading, make_indicator, make_slider, nearest_in_range, rotate)

# =====================================================================
# 1. 설정값
# =====================================================================
AA_SECTION = 0                 # MCP A/A를 구동하는 SECTIONS 인덱스 (Channel 0)
AA_LIMITS = (-17.0, 17.0)      # MCP A/A 출력축 위치 제약 [deg]
AA_SIGN = 1.0                  # A/A + 방향 = 손가락이 +Z(화면 밖)로 기운다. 반대면 -1.0
JOINT_NAMES = {AA_SECTION: "MCP A/A", FINGER_SECTIONS[0]: "PIP F/E", FINGER_SECTIONS[1]: "DIP F/E"}

# 섹션별 게인 슬라이더 최대값 (Kp [A/deg], Kd [A·s/deg]). 시작 위치 = 최대값 * GAIN_INIT_RATIO
GAIN_MAX = {AA_SECTION: (0.005, 0.0005), FINGER_SECTIONS[0]: (0.01, 0.0005), FINGER_SECTIONS[1]: (0.005, 0.0005)}
GAIN_INIT_RATIO = 0.2
# ROM 보호: 현재/명령 위치가 위치 제약 끝에서 이 범위 [deg] 안이면 Kd를 슬라이더 최대값으로 올린다.
# (A/A: -17~-13, 13~17 deg / PIP, DIP: 0~10, 80~90 deg). 나갈 때는 ROM_GUARD_HYST_DEG만큼 더 벗어나야 복귀.
ROM_GUARD_DEG = {AA_SECTION: 4.0, FINGER_SECTIONS[0]: 10.0, FINGER_SECTIONS[1]: 10.0}
ROM_GUARD_HYST_DEG = 0.5
MCP_FE_LIMITS = (0.0, 90.0)    # MCP F/E 위치 제약 [deg] (임시값, 하드웨어 확인 필요)

# 다이나믹셀 (MCP F/E): U2D2가 송수신 방향을 자동 제어하므로 Jetson GPIO는 사용하지 않는다.
DXL_PORT = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTBIN7BJ-if00-port0"
DXL_BAUD = 1_000_000
DXL_ID = 1                     # 연결된 XM335-T323-T에서 확인한 ID
DXL_GEAR = 1.0                 # 다이나믹셀 축 deg / MCP F/E deg (임시값)
DXL_SIGN = 1.0                 # 다이나믹셀 + 방향 = MCP 굽힘(+X)이면 1.0, 반대면 -1.0
DXL_ZERO_DEG = 0.0             # MCP F/E = 0일 때 다이나믹셀 위치 [deg]. Initialize가 실행 중 다시 잡는다.
DXL_PROFILE_VELOCITY = 45      # 단위 0.229 rpm, 45 ≈ 61.8 deg/s (다이나믹셀 축)
DXL_PROFILE_ACCELERATION = 15  # 단위 214.577 rev/min², 15 ≈ 321.9 deg/s² (다이나믹셀 축)
DXL_PERIOD_S = 0.02            # 다이나믹셀 읽기/쓰기 주기 (50 Hz)
DXL_LOG_S = 1.0                # 통신 오류 로그 최소 간격 [s]

IK_STEP_DEG = 0.1              # 최소 구동 IK의 MCP F/E 탐색 간격 (슬라이더 해상도)
IK_TOL_MM = 0.05               # 손끝 오차가 최소값 + 이 값 이내인 해를 "같은 위치"로 본다

MCP_FE_STEP = 1.0              # 도달 영역 합집합을 만들 MCP F/E 간격 [deg]
WORKSPACE_AA_STEPS = 5         # 3D 도달 영역 경계를 그릴 A/A 각도 개수

# 카메라 프리셋: (elevation, azimuth) [deg]. Side = 굽힘 평면 정면 (X 오른쪽, Y 위, Z 화면 밖)
VIEWS = {"Side": (0, -90), "Front": (0, 0), "Top": (90, -90), "Iso": (25, -60)}

GREEN = (0.18, 0.63, 0.26, 1.0)
GRAY = (0.53, 0.53, 0.53, 1.0)
DARK = (0.33, 0.33, 0.33, 1.0)
ORANGE = (0.88, 0.63, 0.19, 1.0)
JOINT = (0.83, 0.83, 0.83, 1.0)
WORKSPACE = (0.34, 0.61, 0.84, 0.35)


# =====================================================================
# 2. 기구학 (굽힘 평면: X = 굽힘 방향, Y = 손가락 방향, 원점 = MCP F/E 축, 단위 mm)
# =====================================================================
def plane_points(angles):
    """순기구학: (MCP, PIP, DIP) [deg, 앞 링크 기준] → MCP, PIP, DIP, 반구 중심 좌표."""
    return [rotate(p, angles[0]) for p in finger_points(angles[1:])]


def solve_pip_dip(target, q0):
    """MCP F/E = q0일 때 손끝 = target인 (PIP, DIP, 손끝 오차 mm). 닿지 않으면 가까운 각도.

    MCP를 되돌린 좌표에서 두 링크(PIP→DIP, DIP→반구 중심)를 코사인 법칙으로 푼다.
    DIP가 0~90 deg로 한쪽으로만 굽으므로 해는 하나다.
    """
    tx, ty = rotate(target, -q0)
    (px, py), middle, distal = FINGER_LINKS
    rx, ry = tx - px, ty - py
    a, b = math.hypot(*middle), math.hypot(*distal)
    cos_rel = clamp((rx * rx + ry * ry - a * a - b * b) / (2 * a * b), -1.0, 1.0)
    q2 = clamp(math.degrees(math.acos(cos_rel)) - (heading(distal) - heading(middle)),
               REF_MIN_DEG, REF_MAX_DEG)
    dx, dy = rotate(distal, q2)
    q1 = nearest_in_range(heading((rx, ry)) - heading((middle[0] + dx, middle[1] + dy)))
    # q1이 범위에 잘렸을 수 있으므로 q2를 다시 손끝 쪽으로 맞춘다 (finger_ik와 같은 방식).
    mx, my = rotate(middle, q1)
    jx, jy = px + mx, py + my
    q2 = nearest_in_range(heading((tx - jx, ty - jy)) - heading(distal) - q1)
    dx, dy = rotate(distal, q1 + q2)
    return q1, q2, math.hypot(jx + dx - tx, jy + dy - ty)


def finger_ik_min_motion(target, reference, q0_range):
    """최소 구동 IK: 손끝 target (x, y) → (MCP, PIP, DIP) [deg].

    q0_range 안의 MCP 각마다 PIP/DIP를 풀고, 손끝 오차가 가장 작은 해들(IK_TOL_MM 이내) 중
    reference(현재 목표 관절각)와의 거리 제곱합이 가장 작은 해를 고른다.
    닿을 수 없는 목표는 가장 가까운 도달 위치를 내는 해 중에서 고른다.
    """
    low, high = q0_range
    steps = max(1, round((high - low) / IK_STEP_DEG))
    candidates = []
    for k in range(steps + 1):
        q0 = low + (high - low) * k / steps
        q1, q2, error = solve_pip_dip(target, q0)
        cost = sum((q - r) ** 2 for q, r in zip((q0, q1, q2), reference))
        candidates.append((error, cost, (q0, q1, q2)))
    best = min(c[0] for c in candidates)
    return min((c for c in candidates if c[0] <= best + IK_TOL_MM), key=lambda c: c[1])[2]


@lru_cache(maxsize=None)
def plane_workspace():
    """MCP F/E까지 고려한 굽힘 평면 손끝 도달 영역 경계 [mm].

    PIP/DIP 영역(MCP F/E = 0)을 MCP F/E 범위에 걸쳐 MCP 축(원점) 둘레로 돌린 다각형들의 합집합.
    """
    boundary = finger_workspace()
    low, high = MCP_FE_LIMITS
    region = QtGui.QPainterPath()
    for k in range(round((high - low) / MCP_FE_STEP) + 1):
        path = QtGui.QPainterPath()
        path.addPolygon(QtGui.QPolygonF([QtCore.QPointF(*rotate(p, low + k * MCP_FE_STEP))
                                         for p in boundary]))
        path.closeSubpath()
        region = region.united(path)
    outline = max(region.simplified().toFillPolygons(), key=len)  # 범위 안에서는 한 덩어리
    return tuple((p.x(), p.y()) for p in outline)


# ------------- 3D (GL은 z가 위쪽이라 (X, -Z, Y)로 그린다) -------------
def to_gl(points, aa):
    """굽힘 평면 점 (X, Y) [mm]를 A/A 축(X 방향, Y = AA_AXIS_Y) 둘레로 aa [deg] 돌려 GL 좌표로."""
    a = math.radians(AA_SIGN * aa)
    return np.array([(x, -(y - AA_AXIS_Y) * math.sin(a), AA_AXIS_Y + (y - AA_AXIS_Y) * math.cos(a))
                     for x, y in points])


def finger_points_3d(aa, angles):
    """순기구학: A/A, (MCP, PIP, DIP) [deg] → A/A, MCP, PIP, DIP, 반구 중심 GL 좌표 (5 x 3) [mm]."""
    return to_gl([(0.0, AA_AXIS_Y), *plane_points(angles)], aa)


def tip_frame(points, aa):
    """손끝 원기둥 좌표계: 원점 = DIP 축, z = 원기둥 축(DIP → 반구 중심), x = F/E 축(Z를 A/A만큼 돌린 방향)."""
    z = points[4] - points[3]
    z /= np.linalg.norm(z)
    a = math.radians(AA_SIGN * aa)
    x = np.array((0.0, -math.cos(a), -math.sin(a)))  # 사용자 Z축 → GL (0, -1, 0), A/A로 회전
    y = np.cross(z, x)
    (ox, oy, oz), rows = points[3], np.column_stack((x, y, z))
    return QtGui.QMatrix4x4(*rows[0], ox, *rows[1], oy, *rows[2], oz, 0, 0, 0, 1)


def tip_outline(n=32, meridians=8):
    """손끝 형상 외곽선 (원기둥 + 반구), 손끝 좌표계 기준 선분 점 쌍. z = 원기둥 축."""
    def pairs(points):
        return [p for start, end in zip(points, points[1:]) for p in (start, end)]

    def ring(radius, z):
        return pairs([(radius * math.cos(t), radius * math.sin(t), z) for t in np.linspace(0, 2 * math.pi, n + 1)])

    segments = ring(TIP_RADIUS, 0.0) + ring(TIP_RADIUS, TIP_LENGTH)
    for lat in (math.pi / 6, math.pi / 3):  # 반구 위도선
        segments += ring(TIP_RADIUS * math.cos(lat), TIP_LENGTH + TIP_RADIUS * math.sin(lat))
    for phi in np.linspace(0, 2 * math.pi, meridians, endpoint=False):
        c, s = math.cos(phi), math.sin(phi)
        segments += [(TIP_RADIUS * c, TIP_RADIUS * s, 0.0), (TIP_RADIUS * c, TIP_RADIUS * s, TIP_LENGTH)]
        segments += pairs([(TIP_RADIUS * math.cos(t) * c, TIP_RADIUS * math.cos(t) * s,
                            TIP_LENGTH + TIP_RADIUS * math.sin(t)) for t in np.linspace(0, math.pi / 2, 9)])
    return np.array(segments)


FLOOR = min(AA_AXIS_Y, *(y for _, y in plane_workspace())) - 10.0  # 3D 바닥 격자 높이 [mm]


def workspace_lines():
    """A/A 각도별 굽힘 평면 도달 영역 경계를 3D로 돌린 선분들 (GL 'lines' 모드용 점 쌍)."""
    boundary = plane_workspace()
    segments = []
    for k in range(WORKSPACE_AA_STEPS):
        loop = to_gl(boundary, AA_LIMITS[0] + (AA_LIMITS[1] - AA_LIMITS[0]) * k / (WORKSPACE_AA_STEPS - 1))
        segments += [p for start, end in zip(loop, np.roll(loop, -1, axis=0)) for p in (start, end)]
    return np.array(segments)


# =====================================================================
# 3. 다이나믹셀 (MCP F/E): 통신 스레드 + 섹션 위젯
# =====================================================================
DXL_POS_PER_DEG = 4096 / 360.0
DXL_ADDR_DRIVE_MODE = 10
DXL_ADDR_OPERATING_MODE = 11
DXL_ADDR_TORQUE_ENABLE = 64
DXL_ADDR_PROFILE_ACCELERATION = 108
DXL_ADDR_PROFILE_VELOCITY = 112
DXL_ADDR_GOAL_POSITION = 116
DXL_ADDR_PRESENT_POSITION = 132
DXL_MODE_EXT_POSITION = 4
DXL_DRIVE_TIME_BASED_PROFILE = 0x04


class DxlError(RuntimeError):
    pass


def dxl_signed32(value):
    return value - (1 << 32) if value & 0x80000000 else value


class U2D2Dynamixel:
    """Protocol 2.0 USB 통신. GPIO를 사용하는 dxl_jetson.py와 독립적으로 동작한다."""

    def __init__(self, port, baud, retries=3):
        # GUI 자체는 SDK가 없어도 열 수 있게 연결할 때만 import한다.
        from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler

        self.port_name, self.baud = port, baud
        self.retries, self.comm_success = retries, COMM_SUCCESS
        self.port, self.packet = PortHandler(port), PacketHandler(2.0)
        self._exclusive = False
        try:
            if not self.port.openPort():
                raise DxlError("포트 열기 실패")
            if not self.port.setBaudRate(baud):
                raise DxlError(f"보드레이트 설정 실패: {baud}")
            # SDK의 setBaudRate가 포트를 다시 열므로 최종 핸들에 독점 접근을 설정한다.
            fcntl.ioctl(self.port.ser.fileno(), termios.TIOCEXCL)
            self._exclusive = True
        except Exception as exc:
            self.close()
            raise DxlError(f"U2D2 포트 열기 실패: {port} ({exc})") from exc

    def close(self):
        if self.port.is_open:
            if self._exclusive:
                try:
                    fcntl.ioctl(self.port.ser.fileno(), termios.TIOCNXCL)
                except OSError:
                    pass  # USB가 분리됐어도 핸들은 닫는다.
                self._exclusive = False
            self.port.closePort()

    def _call(self, fn, what, *args):
        last = ""
        for attempt in range(self.retries):
            try:
                result = fn(self.port, *args)
            except OSError as exc:
                self.port.is_using = False  # SDK는 시리얼 예외 시 통신 중 플래그를 해제하지 않는다.
                raise DxlError(f"{what}: {exc} ({self.port_name})") from exc
            value, comm, error = result if len(result) == 3 else (None, *result)
            if comm == self.comm_success:
                if error:
                    raise DxlError(f"{what}: {self.packet.getRxPacketError(error)}")
                return value
            last = self.packet.getTxRxResult(comm)
            if attempt + 1 < self.retries:
                time.sleep(0.005)
        raise DxlError(f"{what}: {last} ({self.port_name}, {self.baud} bps)")

    def ping(self, dxl_id):
        return self._call(self.packet.ping, f"ping ID {dxl_id}", dxl_id)

    def r1(self, i, a):
        return self._call(self.packet.read1ByteTxRx, f"read1 ID {i}@{a}", i, a)

    def r2(self, i, a):
        return self._call(self.packet.read2ByteTxRx, f"read2 ID {i}@{a}", i, a)

    def r4(self, i, a):
        return self._call(self.packet.read4ByteTxRx, f"read4 ID {i}@{a}", i, a)

    def w1(self, i, a, v):
        self._call(self.packet.write1ByteTxRx, f"write1 ID {i}@{a}", i, a, v)

    def w4(self, i, a, v):
        self._call(self.packet.write4ByteTxRx, f"write4 ID {i}@{a}", i, a, v & 0xFFFFFFFF)

    def torque(self, dxl_id, on):
        self.w1(dxl_id, DXL_ADDR_TORQUE_ENABLE, int(on))


def open_dynamixel():
    """현재 연결된 U2D2를 연다. 모드/위치/토크는 변경하지 않는다."""
    return U2D2Dynamixel(DXL_PORT, DXL_BAUD)


class DxlWorker(threading.Thread):
    """DXL_PERIOD_S마다 요청된 토크/목표를 반영하고 현재 위치를 읽는다. 포트는 이 스레드만 쓴다."""

    def __init__(self, dxl, dxl_id):
        super().__init__(daemon=True)
        self.dxl, self.id = dxl, dxl_id
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.torque = False   # 요청: 토크 ON/OFF
        self.goal = None      # 요청: 목표 위치 [다이나믹셀 raw]
        self.sample = None    # (수신 시각, 현재 위치 raw)
        self.errors = []
        self._torque_on, self._goal_sent = None, None  # 실제로 반영한 값

    def request(self, torque, goal=None):
        with self.lock:
            self.torque = torque
            if goal is not None:
                self.goal = goal

    def take_errors(self):
        with self.lock:
            errors, self.errors = self.errors, []
        return errors

    def _apply(self, torque, goal):
        dxl, i = self.dxl, self.id
        if torque != self._torque_on:
            if torque:
                dxl.torque(i, False)  # 모드는 토크 OFF에서만 바꿀 수 있다 (EEPROM이라 다를 때만 쓴다)
                if dxl.r1(i, DXL_ADDR_OPERATING_MODE) != DXL_MODE_EXT_POSITION:
                    dxl.w1(i, DXL_ADDR_OPERATING_MODE, DXL_MODE_EXT_POSITION)
                # 시간 기반 비트만 해제해 속도 기반 프로파일을 선택하고 회전 방향 등은 보존한다.
                drive_mode = dxl.r1(i, DXL_ADDR_DRIVE_MODE)
                if drive_mode & DXL_DRIVE_TIME_BASED_PROFILE:
                    dxl.w1(i, DXL_ADDR_DRIVE_MODE, drive_mode & ~DXL_DRIVE_TIME_BASED_PROFILE)
                # Operating Mode 변경 시 프로파일 값이 초기화되므로 모드 설정 뒤에 적용한다.
                dxl.w4(i, DXL_ADDR_PROFILE_ACCELERATION, DXL_PROFILE_ACCELERATION)
                dxl.w4(i, DXL_ADDR_PROFILE_VELOCITY, DXL_PROFILE_VELOCITY)
                if goal is None:
                    goal = dxl_signed32(dxl.r4(i, DXL_ADDR_PRESENT_POSITION))
                dxl.w4(i, DXL_ADDR_GOAL_POSITION, goal)
                dxl.torque(i, True)
                self._goal_sent = goal
            else:
                dxl.torque(i, False)
            self._torque_on = torque
        elif torque and goal is not None and goal != self._goal_sent:
            dxl.w4(i, DXL_ADDR_GOAL_POSITION, goal)
            self._goal_sent = goal

    def run(self):
        while not self.stop_event.is_set():
            start = time.monotonic()
            with self.lock:
                torque, goal = self.torque, self.goal
            try:
                self._apply(torque, goal)
                raw = dxl_signed32(self.dxl.r4(self.id, DXL_ADDR_PRESENT_POSITION))
                with self.lock:
                    self.sample = (time.monotonic(), raw)
            except DxlError as exc:
                with self.lock:
                    self.errors.append(str(exc))
            self.stop_event.wait(max(0.0, DXL_PERIOD_S - (time.monotonic() - start)))
        try:
            self.dxl.torque(self.id, False)
        except DxlError:
            pass


class DxlSection(QtWidgets.QGroupBox):
    """MCP F/E 섹션. MotorSection과 같은 모양이지만 게인/코깅/오프셋 없이 위치만 다룬다."""
    connect_toggled = QtCore.pyqtSignal(bool)
    control_toggled = QtCore.pyqtSignal(bool)
    ref_changed = QtCore.pyqtSignal()

    def __init__(self):
        super().__init__(f"MCP F/E  -  Dynamixel ID {DXL_ID}   (gear {DXL_GEAR:.2f})")
        self.name = "MCP F/E"
        self.low, self.high = MCP_FE_LIMITS
        self.cb_connect = QtWidgets.QCheckBox("DXL 연결")
        self.cb_control = QtWidgets.QCheckBox("위치 제어")
        self.lbl_status = QtWidgets.QLabel()
        self.sld_ref = make_slider(round(self.low * REF_SCALE), round(self.high * REF_SCALE),
                                   round(clamp(0.0, self.low, self.high) * REF_SCALE))
        self.lbl_ref, self.lbl_actual = QtWidgets.QLabel(), QtWidgets.QLabel()
        self.bar = make_indicator(self.low, self.high)

        top = QtWidgets.QHBoxLayout()
        top.addWidget(self.cb_connect)
        top.addWidget(self.cb_control)
        top.addStretch()
        top.addWidget(self.lbl_status)
        grid = QtWidgets.QGridLayout(self)
        grid.addLayout(top, 0, 0, 1, 3)
        for row, (text, widget, label) in enumerate((("Position", self.sld_ref, self.lbl_ref),
                                                     ("DXL actual", self.bar, self.lbl_actual)), start=1):
            label.setMinimumWidth(190)
            grid.addWidget(QtWidgets.QLabel(text), row, 0)
            grid.addWidget(widget, row, 1)
            grid.addWidget(label, row, 2)
        grid.setColumnStretch(1, 1)

        self.cb_connect.toggled.connect(self.connect_toggled)
        self.cb_control.toggled.connect(self.sld_ref.setEnabled)
        self.cb_control.toggled.connect(self.control_toggled)
        self.sld_ref.valueChanged.connect(self._on_ref_slider)
        self._update_label()
        self.set_connected(False)

    @property
    def ref_deg(self):
        return self.sld_ref.value() / REF_SCALE

    def set_ref(self, deg):
        """프로그램이 목표를 옮길 때 사용 (ref_changed 미발생)."""
        self.sld_ref.blockSignals(True)
        self.sld_ref.setValue(round(clamp(deg, self.low, self.high) * REF_SCALE))
        self.sld_ref.blockSignals(False)
        self._update_label()

    def force_off(self, checkbox):
        """시그널 없이 체크 해제 (실패/오류 처리용)."""
        checkbox.blockSignals(True)
        checkbox.setChecked(False)
        checkbox.blockSignals(False)
        self.sld_ref.setEnabled(self.cb_control.isChecked())

    def set_actual(self, deg, raw=None):
        """현재 MCP 각도 표시. None = 수신 없음."""
        if deg is None:
            self.bar.setValue(self.bar.minimum())
            self.lbl_actual.setText("  --  deg")
            self.lbl_status.setText("no data" if self.cb_connect.isChecked() else "disconnected")
        else:
            self.bar.setValue(round(clamp(deg, self.low, self.high) * REF_SCALE))
            self.lbl_actual.setText(f"{deg:5.1f} deg  (raw {raw})")
            self.lbl_status.setText(f"ID {DXL_ID}: {deg:6.1f} deg")

    def set_connected(self, connected):
        self.cb_control.setEnabled(connected)
        self.sld_ref.setEnabled(connected and self.cb_control.isChecked())
        if not connected:
            self.set_actual(None)

    def _update_label(self):
        self.lbl_ref.setText(f"{self.ref_deg:5.1f} deg")

    def _on_ref_slider(self):
        self._update_label()
        self.ref_changed.emit()


# =====================================================================
# 4. 그림: 굽힘 평면 뷰 (드래그) + 3D 뷰
# =====================================================================
class PlaneView(FingerView):
    """굽힘 평면 뷰. angles / targets = [MCP, PIP, DIP] [deg], None = 수신 없음 / 위치 제어 OFF.

    PIP/DIP가 위치 제어 중일 때 주황 손끝을 드래그하면 tip_dragged(x, y) [mm]를 보낸다.
    """
    tip_dragged = QtCore.pyqtSignal(float, float)

    def __init__(self):
        super().__init__()
        self.angles = [None] * 3
        self.targets = [None] * 3
        self.workspace = plane_workspace()
        xs, ys = zip((0.0, AA_AXIS_Y), *finger_points((0.0, 0.0)), *self.workspace)
        self.bounds = (min(xs) - TIP_RADIUS, max(xs) + TIP_RADIUS, min(ys), max(ys) + TIP_RADIUS)

    def _can_drag(self):
        return None not in self.targets[1:]

    def _target_points(self):
        # 위치 제어 OFF인 관절은 현재 각(없으면 0)으로 채워 목표 자세를 잇는다.
        return plane_points([a if t is None else t
                             for t, a in zip(self.targets, (a or 0.0 for a in self.angles))])

    def mouseMoveEvent(self, event):
        if not (self.dragging and self._can_drag()):
            return
        ox, oy, scale = self._transform()
        pos = event.localPos()
        self.tip_dragged.emit((pos.x() - ox) / scale, (oy - pos.y()) / scale)

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.fillRect(self.rect(), QtGui.QColor("#252526"))
        scale = self._transform()[2]
        width = max(8.0, TIP_RADIUS * scale)  # 링크 굵기 = 손끝 직경의 절반

        painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(QtGui.QColor(86, 156, 214, 45))
        painter.drawPolygon(QtGui.QPolygonF([self._to_screen(p) for p in self.workspace]))

        base, mcp = self._to_screen((0.0, AA_AXIS_Y)), self._to_screen((0.0, 0.0))
        painter.setPen(QtGui.QPen(QtGui.QColor("#666666"), 4))
        painter.drawLine(base - QtCore.QPointF(50, 0), base + QtCore.QPointF(50, 0))  # A/A 축 (X 방향)
        pen = QtGui.QPen(QtGui.QColor("#888888"), width)
        pen.setCapStyle(QtCore.Qt.RoundCap)
        painter.setPen(pen)
        painter.drawLine(base, mcp)  # A/A → MCP F/E

        current = [a or 0.0 for a in self.angles]
        points = [self._to_screen(p) for p in plane_points(current)]  # MCP, PIP, DIP, 반구 중심
        color = QtGui.QColor("#555555")
        for i, angle in enumerate(self.angles):  # 링크 i는 관절 i가 움직인다
            if angle is None:
                pen = QtGui.QPen(QtGui.QColor("#555555"), width, QtCore.Qt.DashLine)
            else:
                pen = QtGui.QPen(QtGui.QColor("#2ea043"), width)
            pen.setCapStyle(QtCore.Qt.RoundCap)
            painter.setPen(pen)
            painter.drawLine(points[i], points[i + 1])
            color = pen.color()

        # 손끝 형상: DIP 축에서 시작하는 원기둥 + 반구. 원기둥 축은 기준 자세에서 +Y이므로
        # 관절 각의 합만큼 돌린다.
        radius, length = TIP_RADIUS * scale, TIP_LENGTH * scale
        outline = QtGui.QPainterPath(QtCore.QPointF(-radius, 0))
        outline.lineTo(-radius, -length)
        outline.arcTo(QtCore.QRectF(-radius, -length - radius, 2 * radius, 2 * radius), 180, -180)
        outline.lineTo(radius, 0)
        outline.closeSubpath()
        painter.save()
        painter.translate(points[-2])
        painter.rotate(sum(current))
        painter.setPen(QtGui.QPen(color, 2))
        painter.setBrush(QtCore.Qt.NoBrush)
        painter.drawPath(outline)
        painter.restore()

        painter.setPen(QtGui.QPen(QtGui.QColor("#1e1e1e"), 2))
        painter.setBrush(QtGui.QColor("#d4d4d4"))
        for point in (base, *points[:-1]):  # A/A, MCP, PIP, DIP
            painter.drawEllipse(point, width * 0.45, width * 0.45)

        if any(t is not None for t in self.targets):
            target = [self._to_screen(p) for p in self._target_points()]
            orange = QtGui.QColor("#e0a030")
            painter.setPen(QtGui.QPen(orange, 3, QtCore.Qt.DashLine))
            painter.setBrush(QtCore.Qt.NoBrush)
            painter.drawPolyline(QtGui.QPolygonF(target))
            painter.setPen(QtGui.QPen(orange, 2))
            painter.setBrush(orange if self._can_drag() else QtCore.Qt.NoBrush)
            painter.drawEllipse(target[-1], self.HANDLE_PX, self.HANDLE_PX)

        def fmt(angle):
            return "  --  " if angle is None else f"{angle:5.1f}°"

        painter.setPen(QtGui.QColor("#d4d4d4"))
        sources = ("Dynamixel", *(SECTIONS[i][0] for i in FINGER_SECTIONS))
        for row, (name, source, angle, target) in enumerate(
                zip(("MCP", "PIP", "DIP"), sources, self.angles, self.targets)):
            painter.drawText(points[row] + QtCore.QPointF(width, -width), name)
            painter.drawText(12, 22 * (row + 1), f"{name} ({source}): now {fmt(angle)}   target {fmt(target)}")
        if not self._can_drag():
            hint = "드래그: PIP/DIP 위치 제어 ON 필요"
        elif self.targets[0] is None:
            hint = "주황 원 드래그 (MCP 위치 제어 OFF → MCP 고정)"
        else:
            hint = "주황 원 드래그 (최소 구동 IK)"
        painter.drawText(12, self.height() - 12, hint)


class FingerView3D(QtWidgets.QWidget):
    """angles / targets = [A/A, MCP, PIP, DIP] [deg]. None = 수신 없음 / 위치 제어 OFF."""

    def __init__(self):
        super().__init__()
        self.setMinimumSize(460, 460)
        self.angles = [None] * 4
        self.targets = [None] * 4

        self.lbl_state = QtWidgets.QLabel()
        buttons = QtWidgets.QHBoxLayout()
        buttons.addStretch()
        for name, (elevation, azimuth) in VIEWS.items():
            button = QtWidgets.QPushButton(name)
            button.setStyleSheet("padding: 4px 10px;")
            button.clicked.connect(lambda _, e=elevation, a=azimuth: self.set_view(e, a))
            buttons.addWidget(button)

        self.view = gl.GLViewWidget()
        self.view.setBackgroundColor("#252526")
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addLayout(buttons)
        layout.addWidget(self.lbl_state)
        layout.addWidget(self.view, 1)

        grid = gl.GLGridItem()  # 바닥 = 도달 영역과 A/A 축보다 아래
        grid.setSize(260, 260)
        grid.setSpacing(10, 10)
        grid.setColor((80, 80, 80, 120))
        grid.translate(0, 0, FLOOR)
        self.view.addItem(grid)
        self.view.addItem(gl.GLLinePlotItem(pos=np.array([(-20, 0, AA_AXIS_Y), (30, 0, AA_AXIS_Y)]),
                                            color=GRAY, width=2, antialias=True))  # A/A 축 (X 방향)
        self.view.addItem(gl.GLLinePlotItem(pos=workspace_lines(), mode="lines", color=WORKSPACE,
                                            width=1, antialias=True, glOptions="translucent"))

        self.links = gl.GLLinePlotItem(mode="lines", width=8, antialias=True)
        self.joints = gl.GLScatterPlotItem(size=10, color=JOINT, pxMode=False)
        # 손끝: DIP 축에서 시작하는 원기둥 + 반구 외곽선
        self.tip = gl.GLLinePlotItem(pos=tip_outline(), mode="lines", width=1.5, antialias=True)
        self.target = gl.GLLinePlotItem(color=ORANGE, width=3, antialias=True)
        self.target_tip = gl.GLScatterPlotItem(size=8, color=ORANGE, pxMode=False)
        self.labels = [gl.GLTextItem(text=name, color=QtGui.QColor("#d4d4d4"))
                       for name in ("A/A", "MCP", "PIP", "DIP")]
        for item in (self.links, self.joints, self.tip, self.target, self.target_tip, *self.labels):
            self.view.addItem(item)

        self.set_view(*VIEWS["Iso"])
        self._redraw()

    def set_view(self, elevation, azimuth):
        xs = [x for x, _ in plane_workspace()]
        height = FINGER_TIP[1] - FLOOR
        self.view.setCameraPosition(pos=QtGui.QVector3D((min(xs + [0.0]) + max(xs)) / 2, 0,
                                                        (FINGER_TIP[1] + FLOOR) / 2),
                                    distance=1.5 * height, elevation=elevation, azimuth=azimuth)

    def set_state(self, angles, targets):
        if (angles, targets) != (self.angles, self.targets):
            self.angles, self.targets = angles, targets
            self._redraw()

    def _redraw(self):
        current = [a or 0.0 for a in self.angles]
        points = finger_points_3d(current[0], current[1:])

        # 링크 i (A/A→MCP, MCP→PIP, PIP→DIP, DIP→반구 중심)는 관절 i의 수신 여부로 색칠
        colors = [GREEN if a is not None else DARK for a in self.angles]
        self.links.setData(pos=np.repeat(points, 2, axis=0)[1:-1],
                           color=np.repeat(np.array(colors), 2, axis=0))
        self.joints.setData(pos=points[:-1])
        for label, point in zip(self.labels, points[:-1]):
            label.setData(pos=point + (-28.0, 0.0, 0.0))
        self.tip.setTransform(tip_frame(points, current[0]))
        self.tip.setData(color=colors[-1])

        # 위치 제어 OFF인 관절은 현재 각(없으면 0)으로 채워 목표 자세를 잇는다.
        visible = any(t is not None for t in self.targets)
        if visible:
            target = [a if t is None else t for t, a in zip(self.targets, current)]
            target_points = finger_points_3d(target[0], target[1:])
            self.target.setData(pos=target_points)
            self.target_tip.setData(pos=target_points[-1:])
        self.target.setVisible(visible)
        self.target_tip.setVisible(visible)

        def fmt(angle):
            return "--" if angle is None else f"{angle:.1f}°"

        self.lbl_state.setText("    ".join(f"{name} now {fmt(a)}  target {fmt(t)}" for name, a, t
                                           in zip(("A/A", "MCP"), self.angles, self.targets)))


# =====================================================================
# 5. 메인 창: position_control_app의 창에 MCP F/E(다이나믹셀), 최소 구동 IK, 3D 뷰를 더한다.
# =====================================================================
def find_layout(layout, widget):
    """widget을 직접 담고 있는 (하위) 레이아웃."""
    for k in range(layout.count()):
        item = layout.itemAt(k)
        if item.widget() is widget:
            return layout
        if item.layout() is not None:
            found = find_layout(item.layout(), widget)
            if found is not None:
                return found
    return None


class OneFingerApp(PositionControlApp):
    SECTION_LIMITS = {AA_SECTION: AA_LIMITS}

    def __init__(self):
        self.dxl, self.dxl_worker = None, None
        self.dxl_zero = DXL_ZERO_DEG  # MCP F/E = 0인 다이나믹셀 위치 [deg]
        self.dxl_errors = 0           # 연결 후 누적 통신 오류 수
        self.dxl_pending = []         # 아직 로그에 안 남긴 통신 오류
        self.dxl_log_time = 0.0
        self.kd_guard = {}            # ROM 보호 중인 섹션 -> 복귀할 Kd 슬라이더 값 (None = 사용자 변경, 복귀 안 함)
        super().__init__()
        self.setWindowTitle("One Finger Control")
        self.resize(1750, 940)
        for index, joint in JOINT_NAMES.items():
            sec = self.sections[index]
            sec.setTitle(f"{joint}  -  {sec.title()}")
        for index, (kp_max, kd_max) in GAIN_MAX.items():
            sec = self.sections[index]
            sec.kp_max, sec.kd_max = kp_max, kd_max
            for slider in (sec.sld_kp, sec.sld_kd):
                slider.blockSignals(True)  # 시작 시에는 연결 전이므로 전송하지 않는다
                slider.setValue(round(GAIN_STEPS * GAIN_INIT_RATIO))
                slider.blockSignals(False)
            sec._update_labels()

        # MCP F/E 섹션을 A/A 섹션 바로 아래에 넣는다 (관절 순서: A/A, MCP, PIP, DIP).
        self.mcp = DxlSection()
        self.mcp.connect_toggled.connect(self._on_mcp_connect)
        self.mcp.control_toggled.connect(self._on_mcp_control)
        self.mcp.ref_changed.connect(lambda: self._send_ref(self.mcp))
        left = find_layout(self.centralWidget().layout(), self.sections[AA_SECTION])
        left.insertWidget(left.indexOf(self.sections[AA_SECTION]) + 1, self.mcp)

        self.btn_kd_guard = QtWidgets.QPushButton("ROM Guard ON")
        self.btn_kd_guard.setCheckable(True)
        self.btn_kd_guard.setChecked(True)
        self.btn_kd_guard.setToolTip("CAN 관절(MCP A/A, PIP, DIP)의 상·하한 근처에서 Kd 자동 증가. "
                                     "OFF 시 자동으로 올린 Kd를 복원하며 수동 변경값은 유지합니다.")
        self.btn_kd_guard.toggled.connect(self._on_kd_guard)
        header = find_layout(self.centralWidget().layout(), self.btn_cogging)
        header.insertWidget(header.indexOf(self.btn_cogging) + 1, self.btn_kd_guard)

    def _build_finger(self):
        self.finger = PlaneView()
        self.finger.tip_dragged.connect(self._on_tip_drag)
        plane_box = QtWidgets.QGroupBox("Finger (bending plane)")
        QtWidgets.QVBoxLayout(plane_box).addWidget(self.finger)
        self.finger3d = FingerView3D()
        box3d = QtWidgets.QGroupBox("Finger (3D)")
        QtWidgets.QVBoxLayout(box3d).addWidget(self.finger3d)
        splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        splitter.addWidget(plane_box)
        splitter.addWidget(box3d)
        splitter.setStretchFactor(1, 1)
        return splitter

    # ------------- MCP F/E 위치 변환 -------------
    def _mcp_deg(self):
        """현재 MCP F/E [deg]. 연결/수신이 없으면 None."""
        sample = self._mcp_sample()
        return None if sample is None else self._raw_to_deg(sample[1])

    def _mcp_sample(self):
        worker = self.dxl_worker
        if worker is None:
            return None
        with worker.lock:
            sample = worker.sample
        if sample is None or time.monotonic() - sample[0] > STALE_S:
            return None
        return sample

    def _raw_to_deg(self, raw):
        return DXL_SIGN * (raw / DXL_POS_PER_DEG - self.dxl_zero) / DXL_GEAR

    def _deg_to_raw(self, deg):
        return round((self.dxl_zero + DXL_SIGN * deg * DXL_GEAR) * DXL_POS_PER_DEG)

    # ------------- 명령: MCP F/E는 다이나믹셀로 보낸다 -------------
    def _command(self, sec, deg):
        if sec is not self.mcp:
            return super()._command(sec, deg)
        target = clamp(deg, sec.low, sec.high)  # 위치 제약 최종 확인
        self.cmd[sec] = target
        self.dxl_worker.request(True, self._deg_to_raw(target))
        return True

    def _ramp_command(self):
        """Smoothing ON: CAN 섹션과 같은 1차 선형 보간을 MCP F/E에도 적용한다."""
        super()._ramp_command()
        cmd = self.cmd.get(self.mcp)
        if not (self.btn_smooth.isChecked() and self.mcp.cb_control.isChecked()) or cmd is None:
            return
        max_step = self.spin_slope.value() * RAMP_MS / 1000.0
        error = clamp(self.mcp.ref_deg, self.mcp.low, self.mcp.high) - cmd
        if error != 0.0:
            self._command(self.mcp, cmd + clamp(error, -max_step, max_step))

    # ------------- ROM 보호: 위치 제약 끝 근처에서 Kd 최대 -------------
    def _ramp(self):
        super()._ramp()
        self._update_kd_guard()

    def _on_kd_guard(self, checked):
        self.btn_kd_guard.setText("ROM Guard ON" if checked else "ROM Guard OFF")
        if checked:
            self._update_kd_guard()
        else:
            for sec, saved in self.kd_guard.items():
                # 마지막 주기 이후 사용자가 바꾼 값이나 조작 중인 슬라이더도 덮어쓰지 않는다.
                if saved is not None and not sec.sld_kd.isSliderDown() and sec.sld_kd.value() == GAIN_STEPS:
                    self._set_kd_slider(sec, saved)
            self.kd_guard.clear()
        self._log(f"ROM Guard {'ON' if checked else 'OFF'}")

    def _update_kd_guard(self):
        """현재 위치나 명령 위치가 ROM 끝 ROM_GUARD_DEG 안이면 Kd 슬라이더를 최대로 올리고,
        구역을 벗어나면 원래 값으로 되돌린다. 구역 안에서 사용자가 Kd 슬라이더를 만지면
        그 값을 존중하고 복귀하지 않는다. 경계 떨림을 막기 위해 나갈 때는 ROM_GUARD_HYST_DEG만큼 더 벗어나야 한다.
        """
        if not self.btn_kd_guard.isChecked():
            return
        for index, band in ROM_GUARD_DEG.items():
            sec = self.sections[index]
            guarded = sec in self.kd_guard
            positions = []
            if sec.cb_control.isChecked():
                positions = [p for p in (self._section_deg(sec, sec.active_ids), self.cmd.get(sec))
                             if p is not None]
            margin = band + ROM_GUARD_HYST_DEG if guarded else band
            near = any(p <= sec.low + margin or p >= sec.high - margin for p in positions)

            if near and not guarded:
                if sec.sld_kd.isSliderDown():  # 사용자가 조작 중이면 건드리지 않는다
                    self.kd_guard[sec] = None
                    continue
                self.kd_guard[sec] = sec.sld_kd.value()
                self._set_kd_slider(sec, GAIN_STEPS)
                self._log(f"{sec.name}: ROM 끝 {band:g} deg 이내 → Kd 최대 {sec.kd:.6f}")
            elif near and self.kd_guard[sec] is not None:
                if sec.sld_kd.isSliderDown() or sec.sld_kd.value() != GAIN_STEPS:
                    self.kd_guard[sec] = None  # 사용자가 바꿨으므로 구역을 나가도 복귀하지 않는다
                    self._log(f"{sec.name}: Kd 수동 변경, ROM 구역을 벗어나도 자동 복귀 안 함")
            elif not near and guarded:
                saved = self.kd_guard.pop(sec)
                if saved is not None and not sec.sld_kd.isSliderDown():
                    self._set_kd_slider(sec, saved)
                    self._log(f"{sec.name}: ROM 구역 벗어남 → Kd 복귀 {sec.kd:.6f}")

    def _set_kd_slider(self, sec, value):
        """Kd 슬라이더를 프로그램이 옮긴다. 위치 제어 중일 때만 게인을 전송한다."""
        sec.sld_kd.blockSignals(True)
        sec.sld_kd.setValue(value)
        sec.sld_kd.blockSignals(False)
        sec._update_labels()
        if sec.cb_control.isChecked():
            self._on_gains_changed(sec)  # 소프트 스타트 중이면 램프가 슬라이더 값을 읽어 반영한다

    # ------------- MCP F/E 입력 처리 -------------
    def _on_mcp_connect(self, checked):
        if checked:
            try:
                self.dxl = open_dynamixel()
                model = self.dxl.ping(DXL_ID)
            except Exception as exc:
                self._log(f"MCP F/E: 다이나믹셀 연결 실패: {exc}")
                self._close_dxl()
                self.mcp.force_off(self.mcp.cb_connect)
                return
            self.dxl_errors, self.dxl_pending = 0, []
            self.dxl_worker = DxlWorker(self.dxl, DXL_ID)
            self.dxl_worker.start()
            self.mcp.set_connected(True)
            self._log(f"MCP F/E: U2D2 연결 ({DXL_PORT}, {DXL_BAUD} bps, ID {DXL_ID}, model {model}), 위치 제어 OFF")
            return
        if self.mcp.cb_control.isChecked():
            self.mcp.cb_control.setChecked(False)  # → _on_mcp_control: 토크 OFF
        self._close_dxl()
        self.mcp.set_connected(False)
        self._log("MCP F/E: 연결 해제")

    def _close_dxl(self):
        if self.dxl_worker is not None:
            self.dxl_worker.stop_event.set()  # 스레드가 끝나면서 토크 OFF
            self.dxl_worker.join()  # 진행 중인 SDK 통신이 끝난 뒤 포트를 닫는다.
            self.dxl_worker = None
        if self.dxl is not None:
            self.dxl.close()
            self.dxl = None

    def _on_mcp_control(self, checked):
        if not checked:
            self.cmd.pop(self.mcp, None)
            self.dxl_worker.request(False)
            self._log("MCP F/E: 위치 제어 OFF (토크 OFF)")
            return
        current = self._mcp_deg()
        if current is None:
            self._log("MCP F/E: 상태 수신 없음, 위치 제어를 켤 수 없습니다.")
            self.mcp.force_off(self.mcp.cb_control)
            return
        # 영점이 틀리면 제약 끝까지 크게 움직일 수 있으므로 범위 밖에서는 켜지 않는다.
        if not self.mcp.low - 1.0 <= current <= self.mcp.high + 1.0:
            self._log(f"MCP F/E: 현재 {current:.1f} deg가 {self.mcp.low:g}~{self.mcp.high:g} 범위 밖"
                      " → 영점(Initialize 또는 DXL_ZERO_DEG) 확인 후 다시 켜세요.")
            self.mcp.force_off(self.mcp.cb_control)
            return
        self.mcp.set_ref(current)  # 현재 위치를 첫 목표로 잡아 켜는 순간 튀지 않게 한다.
        self._command(self.mcp, self.mcp.ref_deg)
        self._log(f"MCP F/E: 위치 제어 ON, 목표 {self.mcp.ref_deg:.1f} deg")

    def _on_tip_drag(self, x, y):
        """굽힘 평면에서 끈 손끝을 최소 구동 IK로 풀어 슬라이더에 넣는다. 위치 명령은 슬라이더가 보낸다."""
        pip, dip = (self.sections[i] for i in FINGER_SECTIONS)
        if self.mcp.cb_control.isChecked():
            q0_range, q0_ref = MCP_FE_LIMITS, self.mcp.ref_deg
        else:
            q0_ref = self._mcp_deg() or 0.0  # MCP 고정
            q0_range = (q0_ref, q0_ref)
        q0, q1, q2 = finger_ik_min_motion((x, y), (q0_ref, pip.ref_deg, dip.ref_deg), q0_range)
        if self.mcp.cb_control.isChecked():
            self.mcp.sld_ref.setValue(round(q0 * REF_SCALE))
        self._on_finger_drag(q1, q2)  # PIP/DIP 슬라이더 + 그림 갱신

    def _on_initialize(self):
        super()._on_initialize()
        if not self.mcp.cb_connect.isChecked():
            return
        sample = self._mcp_sample()
        if sample is None:
            self._log("MCP F/E: 상태 수신 없음, 영점 설정 생략")
            return
        self.dxl_zero = sample[1] / DXL_POS_PER_DEG
        if self.mcp.cb_control.isChecked():
            self.mcp.set_ref(0.0)
            self._command(self.mcp, 0.0)  # 새 영점 = 현재 위치이므로 제자리 유지
        self._log(f"MCP F/E: 현재 위치를 0 deg로 설정 (다이나믹셀 {self.dxl_zero:.2f} deg)")

    # ------------- 주기 처리 -------------
    def _poll(self):
        if self.dxl_worker is not None:
            self.dxl_pending += self.dxl_worker.take_errors()
            now = time.monotonic()
            if self.dxl_pending and now - self.dxl_log_time >= DXL_LOG_S:  # 끊겼을 때 로그가 넘치지 않게
                self.dxl_errors += len(self.dxl_pending)
                self._log(f"MCP F/E: 통신 오류 {len(self.dxl_pending)}회 (누적 {self.dxl_errors}): "
                          f"{self.dxl_pending[-1]}")
                self.dxl_pending, self.dxl_log_time = [], now
            sample = self._mcp_sample()
            self.mcp.set_actual(None if sample is None else self._raw_to_deg(sample[1]),
                                None if sample is None else sample[1])
        super()._poll()

    def _update_finger(self):
        aa, pip, dip = (self.sections[i] for i in (AA_SECTION, *FINGER_SECTIONS))
        mcp_now = self._mcp_deg()
        mcp_target = self.mcp.ref_deg if self.mcp.cb_control.isChecked() else None
        now = [self._section_deg(s) for s in (pip, dip)]
        target = [s.ref_deg if s.cb_control.isChecked() else None for s in (pip, dip)]
        self.finger.set_state([mcp_now, *now], [mcp_target, *target])
        self.finger3d.set_state([self._section_deg(aa), mcp_now, *now],
                                [aa.ref_deg if aa.cb_control.isChecked() else None, mcp_target, *target])

    def closeEvent(self, event):
        self.mcp.cb_connect.setChecked(False)  # 토크 OFF 후 포트 닫기
        super().closeEvent(event)


# =====================================================================
# 6. 실행
# =====================================================================
def main():
    app = QtWidgets.QApplication(sys.argv)
    window = OneFingerApp()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
