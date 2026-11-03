# -*- coding: utf-8 -*-
"""
손가락 위치 제어 GUI (그래프 없이 제어 전용).

- 채널(Driver ID) 0 / 1+2 / 3 을 세 섹션으로 나누어 제어한다. 모든 모터는 can0 하나를 공유한다.
- [CAN 연결]  체크: 해당 섹션 상태 수신 시작 (첫 섹션 연결 시 버스 open, 마지막 해제 시 close).
- [위치 제어] 체크: 위치 모드(0x05), 해제: 0 A(0x01) 전송으로 토크 OFF → 엔코더 값만 표시.
- 모터가 여럿인 섹션(1+2)은 M 체크박스로 위치 제어에 참여할 모터를 고른다 (위치 제어 OFF에서만 변경).
- 슬라이더 각도는 출력축 기준 0~90 deg. 모터축 명령 = 영점 오프셋 + 각도 * 기어비.
- Initialize: 연결된 모터의 현재 위치를 0 deg(영점 오프셋)로 저장한다. 전자저울 영점과 같다.
- 위치 Kp/Kd: app.py와 동일하게 mode 0x22/0x23, float32 BE 4 bytes.
- Cogging Comp: 0x31 명령으로 LUT 코깅 보상 ON/OFF (펌웨어 부팅 기본값 ON). 섹션 연결 시에도 현재 상태를 보낸다.
- Soft start: 위치 제어를 켤 때 Kp/Kd를 0에서 설정값까지 선형으로 올려 켜는 순간의 충격(D항 kick)을 없앤다.
- M2 elec offset: Slave(M2) 전기각 오프셋을 0x24로 쓰고 0xA4로 읽으며, Save(0x10)로 플래시에 저장한다.
- Smoothing ON: 위치 명령을 목표까지 최대 기울기 [출력축 deg/s]로 1차 선형 보간해 RAMP_MS 주기로 보낸다.
"""
import math
import struct
import sys
import threading
import time
from functools import partial

from PyQt5 import QtCore, QtGui, QtWidgets

from can_io import CANBusManager, CANReaderThread
from config import MODE_COGGING_TOGGLE, MODE_CURRENT, MODE_POSITION
from protocol import (build_ext_id, format_error_code, pack_command_payload,
                      pack_position_gain, parse_broadcast_frame)

# =====================================================================
# 1. 설정값
# =====================================================================
CAN_INTERFACE = "socketcan"
CAN_CHANNEL = "can0"
CAN_BITRATE = 1_000_000  # socketcan은 비트레이트를 OS(ip link)가 관리한다.
TX_BRS = False           # app.py의 socketcan 기본값과 동일 (Jetson TDC 미검증 시 BRS OFF)

MODE_SET_POSITION_KP = 0x22
MODE_SET_POSITION_KD = 0x23
MODE_SET_ELEC_OFFSET = 0x24   # Slave 전기각 오프셋, float32 BE [electrical deg] (RAM만 변경)
MODE_SAVE_CTRL_PARAMS = 0x10  # 제어 파라미터 블록 전체(게인, 오프셋 등)를 플래시에 저장
MODE_READ_FLAG = 0x80         # mode | 0x80 = 읽기 요청, 응답도 같은 EID

GEAR_MOTOR0 = 10.0
GEAR_FINGER = (66 / 20) ** 2
# (섹션 이름, Driver ID 목록, 기어비, 기본 Kp [A/deg], 기본 Kd [A·s/deg]).
# 한 섹션의 모터는 항상 같은 명령을 받는다. 게인은 모터축 deg 기준 (펌웨어 GEAR_RATIO=1).
SECTIONS = (
    ("Channel 0", (0,), GEAR_MOTOR0, 0.05, 0.003),
    ("Channel 1 + 2", (1, 2), GEAR_FINGER, 0.002, 0.0001),
    ("Channel 3", (3,), GEAR_FINGER, 0.001, 0.0001),
)

ELEC_OFFSET_MOTOR = 2                  # 전기각 오프셋을 편집할 Slave Driver ID
ELEC_OFFSET_LIMIT = 180.0              # 펌웨어가 [-180, 180]으로 정규화해 저장한다
READ_TIMEOUT_S = 0.5                   # 읽기 응답 대기 시간

REF_MIN_DEG, REF_MAX_DEG = 0.0, 90.0   # 출력축 위치 제약
REF_SCALE = 10                         # 슬라이더 1칸 = 0.1 deg
GAIN_RANGE = 10                        # 게인 슬라이더 범위 = 0 ~ 기본값 * 10
GAIN_STEPS = 1000                      # 게인 슬라이더 1칸 = 최대값 / 1000

# 손가락 기하 [mm]: 기준 자세(모든 관절 0 deg)의 굽힘 평면 좌표. 원점 = MCP F/E 축,
# X = 오른쪽(굽힘 방향), Y = 위쪽, Z = 화면 밖. F/E 축은 모두 Z 방향, A/A 축은 X 방향.
# 관절0 = MCP F/E (다이나믹셀, 아직 0 deg 고정), 관절1 = Channel 1+2 (PIP), 관절2 = Channel 3 (DIP).
AA_AXIS_Y = -17.48                     # MCP A/A 축 위치 (0, AA_AXIS_Y, 0)
FINGER_JOINTS = ((0.0, 0.0), (7.0, 51.36), (7.0, 51.36 + 35.93))  # MCP, PIP, DIP F/E 축
TIP_LENGTH, TIP_RADIUS = 28.0, 12.0    # 손끝 = DIP 축에서 +Y로 뻗는 원기둥(직경 24) + 같은 직경의 반구
CONTACT_OFFSET_X = 3.5                 # 물체가 닿았으면 하는 위치 = 반구 중심 + X (기준점 아님, 현재 미사용)
FINGER_TIP = (FINGER_JOINTS[-1][0], FINGER_JOINTS[-1][1] + TIP_LENGTH)  # 손끝 기준점 = 반구 중심
# 기준 자세 링크 벡터: MCP→PIP, PIP→DIP, DIP→손끝 기준점
FINGER_LINKS = tuple((x1 - x0, y1 - y0) for (x0, y0), (x1, y1)
                     in zip(FINGER_JOINTS, FINGER_JOINTS[1:] + (FINGER_TIP,)))
FINGER_SECTIONS = (1, 2)               # 관절1, 관절2를 구동하는 SECTIONS 인덱스

SLOPE_DEFAULT = 300.0                  # 위치 명령 최대 기울기 기본값 [출력축 deg/s]
SLOPE_MIN, SLOPE_MAX = 0.1, 1000.0

SOFTSTART_DEFAULT_S = 0.3              # 위치 제어 ON 시 게인 램프 시간 기본값 [s], 0 = 즉시
SOFTSTART_MAX_S = 5.0

RAMP_MS = 10       # Smoothing / Soft start 명령 전송 주기 (100 Hz)
POLL_MS = 50       # 상태 표시 주기
STALE_S = 0.5      # 이 시간 동안 수신이 없으면 위치를 모르는 것으로 본다
TX_DRAIN_S = 0.2   # 버스를 닫기 전 0 A 명령이 나가길 기다리는 최대 시간


# =====================================================================
# 2. 보조 함수
# =====================================================================
def clamp(value, low, high):
    return max(low, min(high, value))


def make_slider(minimum, maximum, value):
    slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
    slider.setRange(minimum, maximum)
    slider.setValue(value)
    slider.setMinimumWidth(360)
    return slider


def make_indicator(low, high):
    """위치 슬라이더와 같은 눈금(low~high deg, 0.1 deg)의 읽기 전용 막대."""
    bar = QtWidgets.QProgressBar()
    bar.setRange(round(low * REF_SCALE), round(high * REF_SCALE))
    bar.setTextVisible(False)
    bar.setFixedHeight(12)
    return bar


def rotate(vector, deg):
    """굽힘 평면 벡터를 deg만큼 +X 쪽(시계 방향)으로 돌린다."""
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return vector[0] * c + vector[1] * s, -vector[0] * s + vector[1] * c


def heading(vector):
    """+Y에서 +X 쪽(시계 방향)으로 잰 벡터 방향 [deg]."""
    return math.degrees(math.atan2(vector[0], vector[1]))


def finger_points(angles):
    """순기구학: 관절 각 [deg, 앞 링크 기준] → MCP, J1(PIP), J2(DIP), 손끝 좌표 [mm]. MCP F/E = 0."""
    points, absolute = [(0.0, 0.0)], 0.0
    for link, joint in zip(FINGER_LINKS, (0.0, *angles)):
        absolute += joint
        dx, dy = rotate(link, absolute)
        points.append((points[-1][0] + dx, points[-1][1] + dy))
    return points


def finger_workspace(n=30):
    """손끝 도달 가능 영역 경계 = 관절 범위(0~90)x(0~90) 사각형의 네 변을 따라간 손끝 궤적."""
    low, high = REF_MIN_DEG, REF_MAX_DEG
    ramp = [low + (high - low) * k / n for k in range(n + 1)]
    edges = ([(q, low) for q in ramp] + [(high, q) for q in ramp]
             + [(q, high) for q in reversed(ramp)] + [(low, q) for q in reversed(ramp)])
    return [finger_points(edge)[-1] for edge in edges]


def nearest_in_range(angle, low=REF_MIN_DEG, high=REF_MAX_DEG):
    """원 위에서 angle [deg]에 가장 가까운 [low, high] 안의 각도."""
    angle = (angle - low) % 360.0 + low  # [low, low + 360)
    if angle <= high:
        return angle
    # 범위 밖이면 high 쪽으로 가는 거리와 한 바퀴 돌아 low로 가는 거리를 비교한다.
    return high if angle - high < low + 360.0 - angle else low


def finger_ik(x, y):
    """역기구학: 손끝 목표 (x, y) [mm] → (q1, q2), 항상 0~90 deg. MCP F/E = 0.

    q1을 슬라이더 해상도(0.1 deg)로 훑고, 각 q1에서 손끝에 가장 가까운 q2를 직접 구한다.
    도달할 수 없는 목표는 자연히 가장 가까운 도달 가능 위치의 각도가 된다.
    """
    pip, middle, distal = FINGER_LINKS
    best = None
    for step in range(round(REF_MIN_DEG * REF_SCALE), round(REF_MAX_DEG * REF_SCALE) + 1):
        q1 = step / REF_SCALE
        mx, my = rotate(middle, q1)
        jx, jy = pip[0] + mx, pip[1] + my
        q2 = nearest_in_range(heading((x - jx, y - jy)) - heading(distal) - q1)
        tx, ty = rotate(distal, q1 + q2)
        dist = math.hypot(jx + tx - x, jy + ty - y)
        if best is None or dist < best[0]:
            best = (dist, q1, q2)
    return best[1], best[2]


def motor_status(data):
    """브로드캐스트 8 bytes → (모터축 다회전 위치 deg, 에러 코드, 전류 A).

    펌웨어 position_loop과 같은 rev * 360 + theta_m. byte 6(temp_C 필드)은 rev(int8)이다.
    """
    frame = parse_broadcast_frame(data)
    return frame["temp_C"] * 360.0 + frame["pos_deg"], frame["error"], frame["cur_A"]


# =====================================================================
# 3. 섹션 위젯: 입력만 시그널로 알리고 CAN 처리는 메인 창이 한다.
# =====================================================================
class MotorSection(QtWidgets.QGroupBox):
    connect_toggled = QtCore.pyqtSignal(bool)
    control_toggled = QtCore.pyqtSignal(bool)
    ref_changed = QtCore.pyqtSignal()
    gains_changed = QtCore.pyqtSignal()
    offset_read = QtCore.pyqtSignal()
    offset_write = QtCore.pyqtSignal(float)
    offset_save = QtCore.pyqtSignal()

    def __init__(self, name, motor_ids, gear, kp_default, kd_default,
                 limits=(REF_MIN_DEG, REF_MAX_DEG)):
        super().__init__(f"{name}   (gear {gear:.2f})")
        self.name, self.motor_ids, self.gear = name, motor_ids, gear
        self.low, self.high = limits  # 출력축 위치 제약 [deg]
        self.kp_max, self.kd_max = kp_default * GAIN_RANGE, kd_default * GAIN_RANGE

        self.cb_connect = QtWidgets.QCheckBox("CAN 연결")
        self.cb_control = QtWidgets.QCheckBox("위치 제어")
        # 모터가 여럿인 섹션: 위치 제어에 참여할 모터 선택. Driver ID -> checkbox
        self.cb_motors = {i: QtWidgets.QCheckBox(f"M{i}") for i in motor_ids} if len(motor_ids) > 1 else {}
        for cb in self.cb_motors.values():
            cb.setChecked(True)
        self.lbl_status = QtWidgets.QLabel()
        self.sld_ref = make_slider(round(self.low * REF_SCALE), round(self.high * REF_SCALE),
                                   round(clamp(0.0, self.low, self.high) * REF_SCALE))
        self.sld_kp = make_slider(0, GAIN_STEPS, GAIN_STEPS // GAIN_RANGE)  # 시작 위치 = 기본값
        self.sld_kd = make_slider(0, GAIN_STEPS, GAIN_STEPS // GAIN_RANGE)
        self.lbl_ref, self.lbl_kp, self.lbl_kd = (QtWidgets.QLabel() for _ in range(3))
        # 모터별 현재 각도 표시 (읽기 전용 바). Driver ID -> (bar, label)
        self.indicators = {i: (make_indicator(self.low, self.high), QtWidgets.QLabel())
                           for i in motor_ids}
        # Slave 전기각 오프셋 편집 (ELEC_OFFSET_MOTOR가 있는 섹션만)
        self.offset_id = ELEC_OFFSET_MOTOR if ELEC_OFFSET_MOTOR in motor_ids else None
        self.spin_offset = QtWidgets.QDoubleSpinBox()
        self.spin_offset.setRange(-ELEC_OFFSET_LIMIT, ELEC_OFFSET_LIMIT)
        self.spin_offset.setWrapping(True)
        self.spin_offset.setDecimals(1)
        self.spin_offset.setSingleStep(0.5)
        self.spin_offset.setSuffix(" deg")
        self.spin_offset.setKeyboardTracking(False)
        self.btn_offset_read, self.btn_offset_write, self.btn_offset_save = (
            QtWidgets.QPushButton(text) for text in ("Read", "Write", "Save"))
        self.btn_offset_save.setToolTip("M2 제어 파라미터 전체를 플래시에 저장 (위치 제어 OFF에서만)")
        self.lbl_offset = QtWidgets.QLabel()
        # 부모 없는 위젯이 지워지지 않도록 속성으로 붙잡아 둔다 (오프셋이 없는 섹션은 레이아웃에 안 넣음).
        self.offset_row = offset_row = QtWidgets.QWidget()
        offset_box = QtWidgets.QHBoxLayout(offset_row)
        offset_box.setContentsMargins(0, 0, 0, 0)
        for widget in (self.spin_offset, self.btn_offset_read, self.btn_offset_write,
                       self.btn_offset_save):
            offset_box.addWidget(widget)
        offset_box.addStretch()

        top = QtWidgets.QHBoxLayout()
        top.addWidget(self.cb_connect)
        top.addWidget(self.cb_control)
        for cb in self.cb_motors.values():
            top.addWidget(cb)
        top.addStretch()
        top.addWidget(self.lbl_status)
        grid = QtWidgets.QGridLayout(self)
        grid.addLayout(top, 0, 0, 1, 3)
        rows = [("Position", self.sld_ref, self.lbl_ref),
                *((f"M{i} actual", bar, label) for i, (bar, label) in self.indicators.items())]
        if self.offset_id is not None:
            rows.append((f"M{self.offset_id} elec offset", offset_row, self.lbl_offset))
        rows += [("P gain", self.sld_kp, self.lbl_kp),
                 ("D gain", self.sld_kd, self.lbl_kd)]
        for row, (text, slider, label) in enumerate(rows, start=1):
            label.setMinimumWidth(190)
            grid.addWidget(QtWidgets.QLabel(text), row, 0)
            grid.addWidget(slider, row, 1)
            grid.addWidget(label, row, 2)
        grid.setColumnStretch(1, 1)

        self.cb_connect.toggled.connect(self.connect_toggled)
        self.cb_control.toggled.connect(self.sld_ref.setEnabled)
        self.cb_control.toggled.connect(self._update_motor_select)
        self.cb_control.toggled.connect(self.control_toggled)
        self.sld_ref.valueChanged.connect(self._on_ref_slider)
        for slider in (self.sld_kp, self.sld_kd):
            slider.valueChanged.connect(self._on_gain_slider)
            slider.sliderReleased.connect(self.gains_changed)
        self.btn_offset_read.clicked.connect(self.offset_read)
        self.btn_offset_write.clicked.connect(lambda: self.offset_write.emit(self.spin_offset.value()))
        self.btn_offset_save.clicked.connect(self.offset_save)
        self._update_labels()
        self.set_connected(False)

    @property
    def ref_deg(self):
        return self.sld_ref.value() / REF_SCALE

    @property
    def active_ids(self):
        """위치 제어에 참여하는 Driver ID. 단일 모터 섹션은 항상 그 모터."""
        return tuple(i for i in self.motor_ids
                     if i not in self.cb_motors or self.cb_motors[i].isChecked())

    @property
    def kp(self):
        return self.sld_kp.value() * self.kp_max / GAIN_STEPS

    @property
    def kd(self):
        return self.sld_kd.value() * self.kd_max / GAIN_STEPS

    def set_ref(self, deg):
        """프로그램이 목표를 옮길 때 사용 (ref_changed 미발생)."""
        self.sld_ref.blockSignals(True)
        self.sld_ref.setValue(round(clamp(deg, self.low, self.high) * REF_SCALE))
        self.sld_ref.blockSignals(False)
        self._update_labels()

    def force_off(self, checkbox):
        """시그널 없이 체크 해제 (실패/오류 처리용)."""
        checkbox.blockSignals(True)
        checkbox.setChecked(False)
        checkbox.blockSignals(False)
        self.sld_ref.setEnabled(self.cb_control.isChecked())
        self._update_motor_select()

    def set_actual(self, motor_id, deg, current=None):
        """현재 각도/전류 표시. 바는 위치 제약으로 자르고, 숫자는 실제 값을 보여준다. None = 수신 없음."""
        bar, label = self.indicators[motor_id]
        if deg is None:
            bar.setValue(bar.minimum())
            label.setText("  --  deg     --  A")
        else:
            bar.setValue(round(clamp(deg, self.low, self.high) * REF_SCALE))
            label.setText(f"{deg:5.1f} deg  {current:6.3f} A")

    def set_offset(self, deg):
        """펌웨어에서 읽은 오프셋 표시. None = 응답 없음."""
        if deg is None:
            self.lbl_offset.setText("dev  --  deg")
            return
        self.spin_offset.setValue(deg)
        self.lbl_offset.setText(f"dev {deg:6.1f} deg")

    def set_connected(self, connected):
        for widget in (self.spin_offset, self.btn_offset_read, self.btn_offset_write,
                       self.btn_offset_save):
            widget.setEnabled(connected)
        self.cb_control.setEnabled(connected)
        self.sld_kp.setEnabled(connected)
        self.sld_kd.setEnabled(connected)
        self.sld_ref.setEnabled(connected and self.cb_control.isChecked())
        self._update_motor_select()
        if not connected:
            self.lbl_status.setText("disconnected")
            self.set_offset(None)
            for motor_id in self.motor_ids:
                self.set_actual(motor_id, None)

    def _update_motor_select(self):
        # 위치 제어 중에는 참여 모터를 바꾸지 못하게 잠근다.
        editable = self.cb_control.isEnabled() and not self.cb_control.isChecked()
        for cb in self.cb_motors.values():
            cb.setEnabled(editable)

    def _update_labels(self):
        self.lbl_ref.setText(f"{self.ref_deg:5.1f} deg")
        self.lbl_kp.setText(f"Kp {self.kp:.6f} A/deg")
        self.lbl_kd.setText(f"Kd {self.kd:.6f} A·s/deg")

    def _on_ref_slider(self):
        self._update_labels()
        self.ref_changed.emit()

    def _on_gain_slider(self):
        self._update_labels()
        # 드래그 중에는 놓을 때(sliderReleased) 한 번만 전송, 클릭/키보드 입력은 즉시 전송.
        if not (self.sld_kp.isSliderDown() or self.sld_kd.isSliderDown()):
            self.gains_changed.emit()


# =====================================================================
# 4. 손가락 그림: 현재 각도로 옆에서 본 손가락을 그린다.
# =====================================================================
class FingerView(QtWidgets.QWidget):
    """0 deg = 기준 자세. 관절 각은 앞 링크 기준 상대각이며 오른쪽(+X)으로 굽힌다.

    실선 = 현재(엔코더), 점선 = 목표, 연한 파란 영역 = 손끝 도달 가능 범위, 외곽선 = 손끝 형상.
    두 관절 모두 위치 제어 중일 때 목표 손끝(주황 원)을 드래그하면 target_dragged(q1, q2)를 보낸다.
    """
    target_dragged = QtCore.pyqtSignal(float, float)
    HANDLE_PX = 9

    def __init__(self):
        super().__init__()
        self.setMinimumSize(340, 460)
        self.angles = [None] * len(FINGER_SECTIONS)   # 현재 [deg], None = 수신 없음
        self.targets = [None] * len(FINGER_SECTIONS)  # 목표 [deg], None = 위치 제어 OFF
        self.dragging = False
        self.workspace = finger_workspace()
        # 그림 범위 [mm] (x0, x1, y0, y1): A/A 축, 기준 자세, 도달 영역 + 손끝 반지름
        xs, ys = zip((0.0, AA_AXIS_Y), *finger_points((0.0, 0.0)), *self.workspace)
        self.bounds = (min(xs) - TIP_RADIUS, max(xs) + TIP_RADIUS, min(ys), max(ys) + TIP_RADIUS)

    def set_state(self, angles, targets):
        if (angles, targets) != (self.angles, self.targets):
            self.angles, self.targets = angles, targets
            self.update()

    def _transform(self):
        """(원점 x, 원점 y, 배율): mm → 픽셀. 그림 범위(self.bounds)를 창 가운데에 맞춘다."""
        x0, x1, y0, y1 = self.bounds
        margin = 50
        scale = min((self.width() - 2 * margin) / (x1 - x0), (self.height() - 2 * margin) / (y1 - y0))
        return (self.width() - (x1 - x0) * scale) / 2 - x0 * scale, self.height() - margin + y0 * scale, scale

    def _to_screen(self, point):
        ox, oy, scale = self._transform()
        return QtCore.QPointF(ox + point[0] * scale, oy - point[1] * scale)

    def _can_drag(self):
        return None not in self.targets

    def _target_points(self):
        # 위치 제어 OFF인 관절은 현재 각(없으면 0)으로 채워 목표 자세를 잇는다.
        return finger_points([a if t is None else t
                              for t, a in zip(self.targets, (a or 0.0 for a in self.angles))])

    # ------------- 마우스: 목표 손끝 드래그 -------------
    def mousePressEvent(self, event):
        if self._can_drag() and event.button() == QtCore.Qt.LeftButton:
            delta = event.localPos() - self._to_screen(self._target_points()[-1])
            self.dragging = math.hypot(delta.x(), delta.y()) <= 2 * self.HANDLE_PX

    def mouseMoveEvent(self, event):
        if not (self.dragging and self._can_drag()):
            return
        ox, oy, scale = self._transform()
        pos = event.localPos()
        self.target_dragged.emit(*finger_ik((pos.x() - ox) / scale, (oy - pos.y()) / scale))

    def mouseReleaseEvent(self, event):
        self.dragging = False

    # ------------- 그리기 -------------
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
        points = [self._to_screen(p) for p in finger_points(current)]  # MCP, PIP, DIP, 반구 중심
        colors = []
        for i in range(len(FINGER_LINKS)):
            if i == 0:
                pen = QtGui.QPen(QtGui.QColor("#888888"), width)          # 고정 링크 (MCP F/E)
            elif self.angles[i - 1] is None:
                pen = QtGui.QPen(QtGui.QColor("#555555"), width, QtCore.Qt.DashLine)
            else:
                pen = QtGui.QPen(QtGui.QColor("#2ea043"), width)
            pen.setCapStyle(QtCore.Qt.RoundCap)
            painter.setPen(pen)
            painter.drawLine(points[i], points[i + 1])
            colors.append(pen.color())

        # 손끝 형상: DIP 축에서 시작하는 원기둥 + 반구. 원기둥 축은 기준 자세에서 +Y이므로
        # 관절 각의 합만큼 돌린다 (MCP F/E = 0).
        radius, length = TIP_RADIUS * scale, TIP_LENGTH * scale
        outline = QtGui.QPainterPath(QtCore.QPointF(-radius, 0))
        outline.lineTo(-radius, -length)
        outline.arcTo(QtCore.QRectF(-radius, -length - radius, 2 * radius, 2 * radius), 180, -180)
        outline.lineTo(radius, 0)
        outline.closeSubpath()
        painter.save()
        painter.translate(points[-2])
        painter.rotate(sum(current))
        painter.setPen(QtGui.QPen(colors[-1], 2))
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
            painter.drawPolyline(QtGui.QPolygonF(target[1:]))
            painter.setPen(QtGui.QPen(orange, 2))
            painter.setBrush(orange if self._can_drag() else QtCore.Qt.NoBrush)
            painter.drawEllipse(target[-1], self.HANDLE_PX, self.HANDLE_PX)

        def fmt(angle):
            return "  --  " if angle is None else f"{angle:5.1f}°"

        painter.setPen(QtGui.QColor("#d4d4d4"))
        for row, (angle, target) in enumerate(zip(self.angles, self.targets), start=1):
            painter.drawText(points[row] + QtCore.QPointF(width, -width), f"J{row}")
            painter.drawText(12, 22 * row, f"J{row} ({SECTIONS[FINGER_SECTIONS[row - 1]][0]}): "
                                           f"now {fmt(angle)}   target {fmt(target)}")
        hint = ("주황 원을 드래그해 목표 설정" if self._can_drag()
                else "드래그: 두 관절 모두 위치 제어 ON 필요")
        painter.drawText(12, self.height() - 12, hint)


# =====================================================================
# 5. 메인 창: CAN 연결, 명령 전송, 상태 표시
# =====================================================================
class PositionControlApp(QtWidgets.QMainWindow):
    SECTION_LIMITS = {}  # SECTIONS 인덱스 -> (최소, 최대) 출력축 deg. 없으면 REF_MIN_DEG~REF_MAX_DEG

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Finger Position Control")
        self.resize(1250, 720)
        self.manager = CANBusManager()
        self.reader = None
        self.reader_stop = threading.Event()
        self.offsets = {i: 0.0 for spec in SECTIONS for i in spec[1]}  # 모터축 영점 [deg]
        self.status = {}  # Driver ID -> (수신 시각, 모터축 위치 deg, 에러 코드, 전류 A)
        self.cmd = {}     # MotorSection -> 마지막으로 보낸 출력축 위치 명령 [deg]
        self.softstart = {}  # MotorSection -> 게인 램프 시작 시각 (진행 중인 섹션만)
        # 파라미터 읽기 응답: 수신 스레드가 eid가 같은 프레임의 data를 채운다.
        self.read_lock = threading.Lock()
        self.read_watch = {"eid": None, "data": None, "ts": None}
        self.read_pending = None  # (MotorSection, 응답 마감 시각)

        self._build_ui()
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._poll)
        self.timer.start(POLL_MS)
        self.ramp_timer = QtCore.QTimer(self)
        self.ramp_timer.setTimerType(QtCore.Qt.PreciseTimer)
        self.ramp_timer.timeout.connect(self._ramp)
        self.ramp_timer.start(RAMP_MS)

    # ------------- UI -------------
    def _build_ui(self):
        self.setStyleSheet("""
            QWidget { background-color: #1e1e1e; color: #d4d4d4; font-size: 10pt; }
            QGroupBox { border: 1px solid #333333; border-radius: 6px; margin-top: 1.2em;
                        padding-top: 10px; font-weight: bold; color: #569cd6; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }
            QPushButton { background-color: #0e639c; color: white; border-radius: 4px; padding: 8px 16px; }
            QPushButton:hover { background-color: #1177bb; }
            QPushButton:checked { background-color: #2ea043; }
            QPlainTextEdit { background-color: #252526; border: 1px solid #333333; }
            QProgressBar { background-color: #252526; border: 1px solid #333333; border-radius: 4px; }
            QProgressBar::chunk { background-color: #2ea043; border-radius: 3px; }
            QWidget:disabled { color: #666666; }
        """)
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        vbox = QtWidgets.QVBoxLayout(central)

        header = QtWidgets.QHBoxLayout()
        title = QtWidgets.QLabel("Finger Position Control")
        title.setStyleSheet("font-size: 18pt; font-weight: bold;")
        self.btn_init = QtWidgets.QPushButton("Initialize (Set Zero)")
        self.btn_init.clicked.connect(self._on_initialize)
        self.btn_cogging = QtWidgets.QPushButton("Cogging Comp ON")
        self.btn_cogging.setCheckable(True)
        self.btn_cogging.setChecked(True)  # 펌웨어 부팅 기본값과 같게 시작
        self.btn_cogging.toggled.connect(self._on_cogging)
        self.btn_smooth = QtWidgets.QPushButton("Smoothing OFF")
        self.btn_smooth.setCheckable(True)
        self.btn_smooth.toggled.connect(self._on_smooth)
        self.spin_slope = QtWidgets.QDoubleSpinBox()
        self.spin_slope.setRange(SLOPE_MIN, SLOPE_MAX)
        self.spin_slope.setDecimals(1)
        self.spin_slope.setValue(SLOPE_DEFAULT)
        self.spin_slope.setSuffix(" deg/s")
        self.spin_slope.setToolTip("위치 명령 최대 기울기 (출력축 기준)")
        self.spin_softstart = QtWidgets.QDoubleSpinBox()
        self.spin_softstart.setRange(0.0, SOFTSTART_MAX_S)
        self.spin_softstart.setDecimals(2)
        self.spin_softstart.setSingleStep(0.1)
        self.spin_softstart.setValue(SOFTSTART_DEFAULT_S)
        self.spin_softstart.setSuffix(" s")
        self.spin_softstart.setToolTip("위치 제어 ON 시 Kp/Kd를 0에서 설정값까지 올리는 시간 (0 = 즉시)")
        header.addWidget(title)
        header.addStretch()
        header.addWidget(self.btn_cogging)
        header.addSpacing(20)
        header.addWidget(self.btn_smooth)
        header.addWidget(QtWidgets.QLabel("Max slope"))
        header.addWidget(self.spin_slope)
        header.addSpacing(20)
        header.addWidget(QtWidgets.QLabel("Soft start"))
        header.addWidget(self.spin_softstart)
        header.addSpacing(20)
        header.addWidget(self.btn_init)
        vbox.addLayout(header)

        # 왼쪽: 섹션들, 오른쪽: 손가락 그림
        body, left = QtWidgets.QHBoxLayout(), QtWidgets.QVBoxLayout()
        self.sections = [MotorSection(*spec, limits=self.SECTION_LIMITS.get(index, (REF_MIN_DEG, REF_MAX_DEG)))
                         for index, spec in enumerate(SECTIONS)]
        for sec in self.sections:
            sec.connect_toggled.connect(partial(self._on_connect, sec))
            sec.control_toggled.connect(partial(self._on_control, sec))
            sec.ref_changed.connect(partial(self._send_ref, sec))
            sec.gains_changed.connect(partial(self._on_gains_changed, sec))
            sec.offset_read.connect(partial(self._request_offset_read, sec))
            sec.offset_write.connect(partial(self._on_offset_write, sec))
            sec.offset_save.connect(partial(self._on_offset_save, sec))
            left.addWidget(sec)
        left.addStretch()
        body.addLayout(left, 1)
        body.addWidget(self._build_finger())
        vbox.addLayout(body)

        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(300)
        self.log.setMaximumHeight(140)
        vbox.addWidget(self.log)

    def _build_finger(self):
        """오른쪽 손가락 그림 영역. self.finger(FingerView)를 만들어야 한다."""
        self.finger = FingerView()
        self.finger.target_dragged.connect(self._on_finger_drag)
        finger_box = QtWidgets.QGroupBox("Finger (side view)")
        QtWidgets.QVBoxLayout(finger_box).addWidget(self.finger)
        return finger_box

    def _log(self, msg):
        self.log.appendPlainText(f"[{time.strftime('%H:%M:%S')}] {msg}")

    # ------------- CAN 버스 -------------
    def _open_bus(self):
        if self.manager.bus is not None:
            return
        self.manager.connect(CAN_INTERFACE, CAN_CHANNEL, CAN_BITRATE, tx_brs=TX_BRS)
        self.status.clear()
        self.reader_stop.clear()
        self.reader = CANReaderThread(self.manager.bus, self.reader_stop,
                                      read_watch=self.read_watch, read_lock=self.read_lock,
                                      on_error=self.manager.writer.stop,
                                      on_warning=self.manager.writer.pause)
        self.reader.start()
        self._log(f"CAN open: {CAN_INTERFACE}/{CAN_CHANNEL}, TX {'FD+BRS' if TX_BRS else 'FD (BRS off)'}")
        if TX_BRS and self.manager.setup_warning:
            self._log(f"CAN setup warning: {self.manager.setup_warning}")

    def _close_bus(self):
        if self.manager.bus is None:
            return
        writer = self.manager.writer
        # 직전에 넣은 0 A 명령이 실제로 송신된 뒤 닫는다 (writer.stop은 대기 명령을 버린다).
        deadline = time.monotonic() + TX_DRAIN_S
        while (not writer.stop_event.is_set() and not writer.commands.empty()
               and time.monotonic() < deadline):
            time.sleep(0.005)
        time.sleep(0.01)  # 큐에서 꺼낸 마지막 프레임의 송신 완료 여유
        self.reader_stop.set()
        self.reader.join(timeout=1.0)
        self.manager.disconnect()
        self.reader = None
        self.status.clear()
        self._clear_read()
        self._log("CAN closed")

    def _send(self, mode_id, motor_id, payload):
        try:
            self.manager.send_ext(build_ext_id(mode_id, motor_id), payload)
            return True
        except Exception as exc:
            self._log(f"TX 실패 (ID {motor_id}, mode 0x{mode_id:02X}): {exc}")
            return False

    # ------------- 위치 계산 -------------
    def _output_deg(self, motor_id, gear):
        """출력축 위치 [deg] = (모터축 위치 - 영점) / 기어비. 수신이 끊겼으면 None."""
        sample = self.status.get(motor_id)
        if sample is None or time.monotonic() - sample[0] > STALE_S:
            return None
        return (sample[1] - self.offsets[motor_id]) / gear

    def _current_a(self, motor_id):
        """모터 전류 [A]. 수신이 끊겼으면 None."""
        sample = self.status.get(motor_id)
        if sample is None or time.monotonic() - sample[0] > STALE_S:
            return None
        return sample[3]

    def _section_deg(self, sec, motor_ids=None):
        """섹션의 현재 출력축 각도 (함께 도는 모터는 평균). 연결/수신이 없으면 None."""
        if not sec.cb_connect.isChecked():
            return None
        positions = [self._output_deg(i, sec.gear) for i in motor_ids or sec.motor_ids]
        if None in positions:
            return None
        return sum(positions) / len(positions)

    # ------------- 명령 전송 -------------
    def _command(self, sec, deg):
        """출력축 위치 명령 deg를 섹션의 모든 모터에 보내고 마지막 명령으로 기록한다."""
        target = clamp(deg, sec.low, sec.high)  # 위치 제약 최종 확인
        self.cmd[sec] = target
        # list로 모아 실패가 있어도 섹션의 모든 모터에 같은 명령을 시도한다.
        return all([self._send(MODE_POSITION, i, pack_command_payload(
            MODE_POSITION, self.offsets[i] + target * sec.gear)) for i in sec.active_ids])

    def _send_ref(self, sec):
        """슬라이더 목표 반영. Smoothing ON이면 _ramp가 기울기 제한을 지키며 따라간다."""
        if not sec.cb_control.isChecked():
            return False
        if self.btn_smooth.isChecked():
            return True
        return self._command(sec, sec.ref_deg)

    def _ramp(self):
        self._ramp_gains()
        self._ramp_command()

    def _ramp_gains(self):
        """소프트 스타트: 경과 시간 비율만큼 Kp/Kd를 0 → 설정값으로 선형 증가."""
        now = time.monotonic()
        for sec, start in list(self.softstart.items()):
            if not sec.cb_control.isChecked():
                del self.softstart[sec]
                continue
            duration = self.spin_softstart.value()
            scale = 1.0 if duration <= 0.0 else min(1.0, (now - start) / duration)
            self._send_gains(sec, scale)
            if scale >= 1.0:
                del self.softstart[sec]

    def _ramp_command(self):
        """1차 선형 보간: 마지막 명령을 목표 쪽으로 RAMP_MS마다 최대 기울기 * 주기만큼 옮긴다.

        타이머가 늦게 불리면 실제 기울기는 더 작아질 뿐 최대값을 넘지 않는다.
        """
        if not self.btn_smooth.isChecked():
            return
        max_step = self.spin_slope.value() * RAMP_MS / 1000.0
        for sec in self.sections:
            cmd = self.cmd.get(sec)
            if not sec.cb_control.isChecked() or cmd is None:
                continue
            error = clamp(sec.ref_deg, sec.low, sec.high) - cmd
            if error != 0.0:
                self._command(sec, cmd + clamp(error, -max_step, max_step))

    def _send_gains(self, sec, scale=1.0):
        """슬라이더 게인 * scale 전송. scale < 1은 소프트 스타트 중간값이라 로그를 남기지 않는다."""
        kp, kd = pack_position_gain(sec.kp * scale), pack_position_gain(sec.kd * scale)
        ok = all([self._send(MODE_SET_POSITION_KP, i, kp) and self._send(MODE_SET_POSITION_KD, i, kd)
                  for i in sec.active_ids])
        if ok and scale >= 1.0:
            self._log(f"{sec.name}: Kp={sec.kp:.6f}, Kd={sec.kd:.6f} 전송")
        return ok

    def _on_gains_changed(self, sec):
        # 소프트 스타트 중에는 램프가 매 주기 슬라이더 값을 읽으므로 끝날 때 반영된다.
        if sec not in self.softstart:
            self._send_gains(sec)

    def _send_cogging(self, sec):
        """코깅 보상 ON/OFF (mode 0x31, data[0] = 1/0). LUT는 지우지 않는다."""
        payload = bytes([1 if self.btn_cogging.isChecked() else 0]) + bytes(7)
        return all([self._send(MODE_COGGING_TOGGLE, i, payload) for i in sec.motor_ids])

    # ------------- M2 전기각 오프셋 -------------
    def _clear_read(self):
        self.read_pending = None
        with self.read_lock:
            for key in self.read_watch:
                self.read_watch[key] = None

    def _request_offset_read(self, sec):
        """0xA4 읽기 요청. 응답은 _check_offset_read가 _poll에서 처리한다."""
        eid = build_ext_id(MODE_SET_ELEC_OFFSET | MODE_READ_FLAG, sec.offset_id)
        with self.read_lock:
            self.read_watch.update(eid=eid, data=None, ts=None)
        if self._send(MODE_SET_ELEC_OFFSET | MODE_READ_FLAG, sec.offset_id, bytes(8)):
            self.read_pending = (sec, time.monotonic() + READ_TIMEOUT_S)
        else:
            self._clear_read()

    def _check_offset_read(self):
        if self.read_pending is None:
            return
        sec, deadline = self.read_pending
        with self.read_lock:
            data = self.read_watch["data"]
        if data is not None and len(data) >= 4:
            deg = struct.unpack(">f", data[:4])[0]
            self._clear_read()
            if math.isfinite(deg):
                sec.set_offset(deg)
                self._log(f"M{sec.offset_id}: elec offset = {deg:.2f} deg")
            else:
                self._log(f"M{sec.offset_id}: elec offset 응답이 유효하지 않습니다 ({data.hex(' ')})")
        elif time.monotonic() > deadline:
            self._clear_read()
            sec.set_offset(None)
            self._log(f"M{sec.offset_id}: elec offset 읽기 응답 없음 (펌웨어 0x24 지원 확인)")

    def _on_offset_write(self, sec, deg):
        """0x24 쓰기(RAM만) 후 바로 읽어 펌웨어에 적용된 값(정규화 포함)을 확인한다."""
        if self._send(MODE_SET_ELEC_OFFSET, sec.offset_id, struct.pack(">f", deg)):
            self._log(f"M{sec.offset_id}: elec offset {deg:.2f} deg 전송")
            self._request_offset_read(sec)

    def _on_offset_save(self, sec):
        # 플래시 erase 동안 펌웨어 인터럽트가 꺼져 제어 루프가 멈추므로 토크가 없을 때만 저장한다.
        if sec.cb_control.isChecked():
            self._log(f"M{sec.offset_id}: 위치 제어 OFF 상태에서만 플래시 저장할 수 있습니다.")
            return
        if self._send(MODE_SAVE_CTRL_PARAMS, sec.offset_id, bytes(8)):
            self._log(f"M{sec.offset_id}: 제어 파라미터 플래시 저장 요청 (게인 + elec offset)")

    def _release(self, sec):
        """0 A 전류 명령으로 토크를 끈다. 선택 안 된 모터도 포함해 섹션 전체에 보낸다."""
        for motor_id in sec.motor_ids:
            self._send(MODE_CURRENT, motor_id, pack_command_payload(MODE_CURRENT, 0.0))

    # ------------- 사용자 입력 처리 -------------
    def _on_connect(self, sec, checked):
        if checked:
            try:
                self._open_bus()
            except Exception as exc:
                self._log(f"CAN 연결 실패: {exc}")
                sec.force_off(sec.cb_connect)
                return
            sec.set_connected(True)
            self._log(f"{sec.name}: 연결 (위치 제어 OFF, 엔코더 값만 표시)")
            # 앱 재시작 등으로 펌웨어 상태를 모르므로 연결할 때 버튼 상태로 맞춘다.
            if self._send_cogging(sec):
                self._log(f"{sec.name}: Cogging Comp {'ON' if self.btn_cogging.isChecked() else 'OFF'}")
            if sec.offset_id is not None:
                self._request_offset_read(sec)
            return
        if sec.cb_control.isChecked():
            sec.cb_control.setChecked(False)  # → _on_control: 0 A로 토크 해제
        sec.set_connected(False)
        self._log(f"{sec.name}: 연결 해제")
        if not any(s.cb_connect.isChecked() for s in self.sections):
            self._close_bus()

    def _on_control(self, sec, checked):
        if not checked:
            self.cmd.pop(sec, None)
            self.softstart.pop(sec, None)
            self._release(sec)
            self._log(f"{sec.name}: 위치 제어 OFF (0 A)")
            return
        if not sec.active_ids:
            self._log(f"{sec.name}: 선택된 모터가 없어 위치 제어를 켤 수 없습니다.")
            sec.force_off(sec.cb_control)
            return
        current = self._section_deg(sec, sec.active_ids)
        if current is None:
            self._log(f"{sec.name}: 상태 수신 없음, 위치 제어를 켤 수 없습니다.")
            sec.force_off(sec.cb_control)
            return
        # 현재 위치를 첫 목표로 잡아 켜는 순간 튀지 않게 한다.
        sec.set_ref(current)
        if abs(sec.ref_deg - current) > 0.1:
            self._log(f"{sec.name}: 현재 {current:.1f} deg가 {sec.low:g}~{sec.high:g} 범위 밖"
                      f" → {sec.ref_deg:.1f} deg로 이동")
        # 소프트 스타트: 게인 0으로 위치 모드에 들어가면 첫 주기에 펌웨어 D항 상태(d[0])가
        # 현재 위치로 갱신되어 켜는 순간의 D항 kick이 없다. 이후 _ramp_gains가 게인을 올린다.
        soft = self.spin_softstart.value() > 0.0
        if not (self._send_gains(sec, 0.0 if soft else 1.0) and self._command(sec, sec.ref_deg)):
            sec.cb_control.setChecked(False)  # 일부만 전송됐을 수 있으므로 0 A로 되돌린다.
            return
        if soft:
            self.softstart[sec] = time.monotonic()
        motors = "+".join(f"M{i}" for i in sec.active_ids)
        self._log(f"{sec.name}: 위치 제어 ON ({motors}), 목표 {sec.ref_deg:.1f} deg"
                  + (f", soft start {self.spin_softstart.value():.2f} s" if soft else ""))

    def _on_cogging(self, checked):
        state = "ON" if checked else "OFF"
        self.btn_cogging.setText(f"Cogging Comp {state}")
        connected = [s for s in self.sections if s.cb_connect.isChecked()]
        if not connected:
            self._log(f"Cogging Comp {state}: 연결된 모터 없음, 다음 연결 시 적용")
            return
        for sec in connected:
            if self._send_cogging(sec):
                self._log(f"{sec.name}: Cogging Comp {state}")

    def _on_smooth(self, checked):
        self.btn_smooth.setText("Smoothing ON" if checked else "Smoothing OFF")
        if not checked:
            # 보간 중에 끄면 남은 목표로 바로 튀므로, 목표를 지금까지 보낸 명령 위치로 맞춘다.
            for sec, cmd in self.cmd.items():
                if sec.cb_control.isChecked() and abs(sec.ref_deg - cmd) > 1e-9:
                    sec.set_ref(cmd)
                    self._command(sec, sec.ref_deg)
        self._log(f"Smoothing {'ON' if checked else 'OFF'}"
                  + (f" (max {self.spin_slope.value():.1f} deg/s)" if checked else ""))

    def _on_finger_drag(self, *angles):
        """그림에서 끈 목표 손끝의 IK 각도를 슬라이더에 넣는다. 위치 명령은 슬라이더가 보낸다."""
        for index, angle in zip(FINGER_SECTIONS, angles):
            sec = self.sections[index]
            if sec.cb_control.isChecked():
                sec.sld_ref.setValue(round(clamp(angle, sec.low, sec.high) * REF_SCALE))
        self._update_finger()

    def _on_initialize(self):
        connected = [s for s in self.sections if s.cb_connect.isChecked()]
        if not connected:
            self._log("Initialize: 연결된 모터가 없습니다.")
            return
        for sec in connected:
            # 1·2번처럼 함께 도는 모터는 모두 수신될 때만 같이 영점을 잡는다.
            if any(self._output_deg(i, sec.gear) is None for i in sec.motor_ids):
                self._log(f"{sec.name}: 상태 수신 없음, 영점 설정 생략")
                continue
            for i in sec.motor_ids:
                self.offsets[i] = self.status[i][1]
            if sec.cb_control.isChecked():
                sec.set_ref(0.0)
                self._command(sec, 0.0)  # 새 영점 = 현재 위치이므로 제자리 유지 (보간 없이 즉시)
            self._log(f"{sec.name}: 현재 위치를 0 deg로 설정")

    # ------------- 주기 처리 -------------
    def _poll(self):
        reader, writer = self.reader, self.manager.writer
        if reader is None:
            self._update_finger()
            return
        for kind, notice in reader.take_notices():
            self._log(f"CAN {kind}: {notice}")
        _, _, errors = writer.take_stats()
        if errors:
            self._log(f"TX error: {errors[-1]}")
        if reader.last_error or writer.stop_event.is_set():
            self._log(f"CAN 오류로 전체 연결 해제: {reader.last_error or 'TX stopped'}")
            self._disconnect_all()
            return

        for motor_id in self.offsets:
            sample = reader.take_latest_status(motor_id)
            if sample is not None:
                stamp, data = sample
                self.status[motor_id] = (stamp, *motor_status(data))
        for sec in self.sections:
            if sec.cb_connect.isChecked():
                sec.lbl_status.setText(self._status_text(sec))
                for i in sec.motor_ids:
                    sec.set_actual(i, self._output_deg(i, sec.gear), self._current_a(i))
        self._check_offset_read()
        self._update_finger()

    def _update_finger(self):
        secs = [self.sections[i] for i in FINGER_SECTIONS]
        self.finger.set_state([self._section_deg(s) for s in secs],
                              [s.ref_deg if s.cb_control.isChecked() else None for s in secs])

    def _status_text(self, sec):
        parts = []
        for i in sec.motor_ids:
            deg = self._output_deg(i, sec.gear)
            if deg is None:
                parts.append(f"M{i}: no data")
            else:
                parts.append(f"M{i}: {deg:6.1f} deg  {format_error_code(self.status[i][2])}")
        return "    ".join(parts)

    def _disconnect_all(self):
        """버스 오류 시: 송신이 불가능하므로 UI만 정리하고 버스를 닫는다."""
        for sec in self.sections:
            sec.force_off(sec.cb_control)
            sec.force_off(sec.cb_connect)
            sec.set_connected(False)
        self._close_bus()

    def closeEvent(self, event):
        for sec in self.sections:
            sec.cb_connect.setChecked(False)  # 위치 제어 OFF(0 A) 후 마지막 섹션에서 버스 종료
        super().closeEvent(event)


# =====================================================================
# 6. 실행
# =====================================================================
def main():
    app = QtWidgets.QApplication(sys.argv)
    window = PositionControlApp()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
