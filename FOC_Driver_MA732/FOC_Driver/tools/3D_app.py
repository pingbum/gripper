# -*- coding: utf-8 -*-
"""
손가락 위치 제어 GUI - 3D 보기 (position_control_app.py 확장).

- 관절: MCP A/A = Channel 0, MCP F/E = 다이나믹셀 (미연결, 0 deg 고정),
        PIP F/E = Channel 1 + 2, DIP F/E = Channel 3.
- CAN 연결, 위치 제어, 게인, Smoothing, Soft start, Initialize는 position_control_app.py와 같다.
- 굽힘 평면 뷰: 기존 측면 그림. 주황 손끝을 드래그하면 PIP/DIP 목표가 정해진다 (A/A와 무관).
- 3D 뷰 (보기 전용): 현재 자세(초록), 목표 자세(주황), 손끝 도달 가능 영역(파란 선).
- 도달 영역(두 뷰 모두)은 MCP F/E 범위까지 포함한다. 드래그 IK는 아직 PIP/DIP만 쓴다 (MCP F/E = 0).
  왼쪽 드래그 = 회전, 휠 = 줌, 가운데 드래그 = 이동. A/A 목표는 Channel 0 슬라이더로 정한다.
- 손가락 기하 [mm]는 position_control_app.py의 AA_AXIS_Y, FINGER_JOINTS, TIP_*를 함께 쓴다.
  좌표: X = 굽힘 방향, Y = 손가락 방향(위), Z = 화면 밖. GL은 z가 위쪽이라 (X, -Z, Y)로 그린다.

실행: python3 3D_app.py
"""
import math
import sys
from functools import lru_cache

import numpy as np
import pyqtgraph.opengl as gl
from PyQt5 import QtCore, QtGui, QtWidgets

from position_control_app import (AA_AXIS_Y, FINGER_SECTIONS, FINGER_TIP, GAIN_STEPS, TIP_LENGTH, TIP_RADIUS,
                                  PositionControlApp, finger_points, finger_workspace, rotate)

# =====================================================================
# 1. 설정값
# =====================================================================
AA_SECTION = 0                 # MCP A/A를 구동하는 SECTIONS 인덱스 (Channel 0)
AA_LIMITS = (-17.0, 17.0)      # MCP A/A 출력축 위치 제약 [deg]
AA_SIGN = 1.0                  # A/A + 방향 = 손가락이 +Z(화면 밖)로 기운다. 반대면 -1.0
JOINT_NAMES = {AA_SECTION: "MCP A/A", FINGER_SECTIONS[0]: "PIP F/E", FINGER_SECTIONS[1]: "DIP F/E"}

# 섹션별 게인 슬라이더 최대값 (Kp [A/deg], Kd [A·s/deg]). 시작 위치 = 최대값 * GAIN_INIT_RATIO
GAIN_MAX = {AA_SECTION: (0.005, 0.001), FINGER_SECTIONS[0]: (0.01, 0.0005), FINGER_SECTIONS[1]: (0.005, 0.0005)}
GAIN_INIT_RATIO = 0.2
MCP_FE_LIMITS = (0.0, 90.0)    # MCP F/E (다이나믹셀) 범위 [deg] (임시값, 하드웨어 확인 필요)
MCP_FE_STEP = 1.0              # 도달 영역 합집합을 만들 MCP F/E 간격 [deg]
WORKSPACE_AA_STEPS = 5        # 도달 영역 경계를 그릴 A/A 각도 개수

# 카메라 프리셋: (elevation, azimuth) [deg]. Side = 굽힘 평면 정면 (X 오른쪽, Y 위, Z 화면 밖)
VIEWS = {"Side": (0, -90), "Front": (0, 0), "Top": (90, -90), "Iso": (25, -60)}

GREEN = (0.18, 0.63, 0.26, 1.0)
GRAY = (0.53, 0.53, 0.53, 1.0)
DARK = (0.33, 0.33, 0.33, 1.0)
ORANGE = (0.88, 0.63, 0.19, 1.0)
JOINT = (0.83, 0.83, 0.83, 1.0)
WORKSPACE = (0.34, 0.61, 0.84, 0.35)


# =====================================================================
# 2. 기구학
# =====================================================================
def to_gl(points, aa):
    """굽힘 평면 점 (X, Y) [mm]를 A/A 축(X 방향, Y = AA_AXIS_Y) 둘레로 aa [deg] 돌려 GL 좌표로."""
    a = math.radians(AA_SIGN * aa)
    return np.array([(x, -(y - AA_AXIS_Y) * math.sin(a), AA_AXIS_Y + (y - AA_AXIS_Y) * math.cos(a))
                     for x, y in points])


def finger_points_3d(aa, angles):
    """순기구학: A/A [deg], (PIP, DIP) [deg] → A/A, MCP, PIP, DIP, 반구 중심 GL 좌표 (5 x 3) [mm]."""
    return to_gl([(0.0, AA_AXIS_Y), *finger_points(angles)], aa)


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
# 3. 3D 손가락 그림
# =====================================================================
class FingerView3D(QtWidgets.QWidget):
    """angles / targets = [A/A, PIP, DIP] [deg]. None = 수신 없음 / 위치 제어 OFF."""

    def __init__(self):
        super().__init__()
        self.setMinimumSize(460, 460)
        self.angles = [None] * 3
        self.targets = [None] * 3

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
        for item in (self.links, self.joints, self.tip, self.target,
                     self.target_tip, *self.labels):
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

        # 링크별 색: A/A→MCP = A/A 수신 여부, MCP→PIP = MCP F/E 미연결(회색), PIP→DIP / DIP→반구 중심 = 각 관절
        known = [GREEN if a is not None else DARK for a in self.angles]
        colors = [known[0], GRAY, known[1], known[2]]
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

        self.lbl_state.setText(f"A/A now {fmt(self.angles[0])}  target {fmt(self.targets[0])}"
                               f"    MCP F/E: 미연결 (0°)")


# =====================================================================
# 4. 메인 창: position_control_app의 창에 3D 뷰와 A/A 범위를 더한다.
# =====================================================================
class Finger3DApp(PositionControlApp):
    SECTION_LIMITS = {AA_SECTION: AA_LIMITS}

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Finger Position Control 3D")
        self.resize(1750, 820)
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

    def _build_finger(self):
        plane_box = super()._build_finger()
        plane_box.setTitle("Finger (bending plane)")
        # 굽힘 평면 뷰의 도달 영역도 MCP F/E 포함으로 바꾸고 그림 범위를 다시 잡는다.
        workspace = plane_workspace()
        xs, ys = zip((0.0, AA_AXIS_Y), *finger_points((0.0, 0.0)), *workspace)
        self.finger.workspace = workspace
        self.finger.bounds = (min(xs) - TIP_RADIUS, max(xs) + TIP_RADIUS, min(ys), max(ys) + TIP_RADIUS)
        self.finger3d = FingerView3D()
        box3d = QtWidgets.QGroupBox("Finger (3D)")
        QtWidgets.QVBoxLayout(box3d).addWidget(self.finger3d)
        splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        splitter.addWidget(plane_box)
        splitter.addWidget(box3d)
        splitter.setStretchFactor(1, 1)
        return splitter

    def _update_finger(self):
        super()._update_finger()
        secs = [self.sections[i] for i in (AA_SECTION, *FINGER_SECTIONS)]
        self.finger3d.set_state([self._section_deg(s) for s in secs],
                                [s.ref_deg if s.cb_control.isChecked() else None for s in secs])


# =====================================================================
# 5. 실행
# =====================================================================
def main():
    app = QtWidgets.QApplication(sys.argv)
    window = Finger3DApp()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
