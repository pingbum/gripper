# -*- coding: utf-8 -*-
"""
Master/Slave 백래시 테스트용 손가락 제어 GUI - one_finger.py + 위치 명령 feedforward 전류.

- one_finger.py(one_finger_smooth)의 기능을 그대로 쓰고, PIP F/E(Channel 1 + 2)의
  M1(Master), M2(Slave)에 각각 따로 입력한 i_ff [A]를 위치 명령 4-5 byte에 실어 보낸다.
- 위치 명령 (mode 0x05, 항상 8 bytes): [0..3] s32 BE 0.01 deg, [4..5] s16 BE i_ff mA, [6..7] 0.
  i_ff는 모터축 mechanical sign 전류 (상태 표시 전류, mode 0x01과 같은 부호).
  펌웨어가 PD + i_ff + cogging을 더한 뒤 ±CURRENT_MAX로 한 번만 제한한다.
- i_ff는 계단식으로 즉시 적용한다. 아래 조건을 모두 만족할 때만 입력값을, 아니면 0을 보낸다.
  FF ON  /  PIP 위치 제어 ON  /  Soft start 완료  /  M1, M2 모두 위치 제어에 참여
- 위치가 그대로여도 i_ff가 바뀌면 마지막 위치 명령을 새 i_ff로 다시 보낸다.
- 다른 관절(M0, M3)은 항상 i_ff = 0. 위치 제어 OFF는 기존처럼 0 A 전류 명령.

실행: python3 feed_forward.py
"""
import sys

from PyQt5 import QtWidgets

from config import MODE_POSITION
from one_finger import OneFingerApp
from position_control_app import FINGER_SECTIONS, clamp
from protocol import pack_position_payload

# =====================================================================
# 1. 설정값
# =====================================================================
FF_SECTION = FINGER_SECTIONS[0]        # PIP F/E = Channel 1 + 2 (SECTIONS 인덱스)
FF_MOTORS = {1: "Master", 2: "Slave"}  # i_ff를 주는 Driver ID -> 이름
FF_LIMIT_A = 0.6                       # i_ff 입력 범위 ±[A] (펌웨어 CURRENT_MAX = 1.2 A)
FF_STEP_A = 0.01                       # 스핀박스 화살표 1칸 [A]


# =====================================================================
# 2. 메인 창: one_finger의 창에 PIP Master/Slave feedforward 전류를 더한다.
# =====================================================================
class FeedForwardApp(OneFingerApp):
    def __init__(self):
        self.ff_hold = set()                         # 위치 제어 ON 처리 중인 섹션 (soft start 등록 전)
        self.ff_sent = {i: 0.0 for i in FF_MOTORS}   # 펌웨어에 마지막으로 보낸 i_ff [A]
        self.ff_reason = None                        # 마지막으로 로그에 남긴 i_ff 차단 이유
        super().__init__()
        self.setWindowTitle("One Finger Control - Feedforward")
        self.ff_sec = self.sections[FF_SECTION]
        if set(self.ff_sec.motor_ids) != set(FF_MOTORS):
            raise RuntimeError(f"FF_MOTORS {tuple(FF_MOTORS)} != {self.ff_sec.name} {self.ff_sec.motor_ids}")
        self._build_ff()
        self._update_ff_label()

    def _build_ff(self):
        """PIP 섹션 아래에 [FF ON/OFF] + M1/M2 i_ff 입력 + 적용값 표시 줄을 넣는다."""
        self.btn_ff = QtWidgets.QPushButton("FF OFF")
        self.btn_ff.setCheckable(True)
        self.btn_ff.setToolTip("ON: 위치 제어 중(Soft start 완료, M1+M2 선택)일 때 입력한 i_ff를 위치 명령에 싣는다. "
                               "OFF: i_ff = 0 (입력값은 유지)")
        self.btn_ff.toggled.connect(self._on_ff_toggled)
        self.spin_ff = {}
        # 부모 없는 위젯이 지워지지 않도록 속성으로 붙잡아 둔다.
        self.ff_row = row = QtWidgets.QWidget()
        box = QtWidgets.QHBoxLayout(row)
        box.setContentsMargins(0, 0, 0, 0)
        box.addWidget(self.btn_ff)
        for motor_id, name in FF_MOTORS.items():
            spin = QtWidgets.QDoubleSpinBox()
            spin.setRange(-FF_LIMIT_A, FF_LIMIT_A)
            spin.setDecimals(3)
            spin.setSingleStep(FF_STEP_A)
            spin.setSuffix(" A")
            spin.setKeyboardTracking(False)  # 입력 중간값이 계단으로 나가지 않게 Enter/포커스 이동 때만 반영
            spin.setToolTip("모터축 mechanical sign 전류 (표시 전류와 같은 부호)")
            spin.valueChanged.connect(self._refresh_ff)
            box.addWidget(QtWidgets.QLabel(f"M{motor_id} {name}"))
            box.addWidget(spin)
            self.spin_ff[motor_id] = spin
        box.addStretch()
        self.lbl_ff = QtWidgets.QLabel()
        self.lbl_ff.setMinimumWidth(190)
        grid = self.ff_sec.layout()
        r = grid.rowCount()
        grid.addWidget(QtWidgets.QLabel("i_ff"), r, 0)
        grid.addWidget(row, r, 1)
        grid.addWidget(self.lbl_ff, r, 2)

    # ------------- i_ff 결정 -------------
    def _ff_block_reason(self):
        """입력한 i_ff를 보낼 수 없는 이유. None = 적용 가능."""
        sec = self.ff_sec
        if not self.btn_ff.isChecked():
            return "FF OFF"
        if not sec.cb_control.isChecked():
            return "위치 제어 OFF"
        if sec in self.ff_hold or sec in self.softstart:
            return "Soft start 중"
        if set(sec.active_ids) != set(FF_MOTORS):
            return "M1, M2 모두 선택 필요"
        return None

    def _ff_target(self, sec, motor_id):
        """이번 위치 명령에 실을 i_ff [A]."""
        if sec is not self.ff_sec or motor_id not in FF_MOTORS or self._ff_block_reason():
            return 0.0
        return self.spin_ff[motor_id].value()

    # ------------- 명령: 위치 + i_ff -------------
    def _command(self, sec, deg):
        """position_control_app._command와 같고, 4-5 byte에 모터별 i_ff를 싣는다."""
        if sec is self.mcp:
            return super()._command(sec, deg)
        target = clamp(deg, sec.low, sec.high)  # 위치 제약 최종 확인
        self.cmd[sec] = target
        before = dict(self.ff_sent)
        results = []  # 실패가 있어도 섹션의 모든 모터에 같은 위치 명령을 시도한다.
        for i in sec.active_ids:
            i_ff = self._ff_target(sec, i)
            ok = self._send(MODE_POSITION, i,
                            pack_position_payload(self.offsets[i] + target * sec.gear, i_ff))
            if ok and i in self.ff_sent:
                self.ff_sent[i] = i_ff
            results.append(ok)
        if self.ff_sent != before:
            self._log(f"{sec.name}: i_ff " + ", ".join(f"M{i} {a:+.3f} A" for i, a in self.ff_sent.items()))
        self._update_ff_label()
        return all(results)

    def _refresh_ff(self, *_):
        """i_ff 목표가 마지막으로 보낸 값과 다르면 마지막 위치 명령을 새 i_ff로 다시 보낸다."""
        sec = self.ff_sec
        cmd = self.cmd.get(sec)
        if not sec.cb_control.isChecked() or cmd is None:
            # 위치 모드가 아니면 펌웨어가 i_ff를 쓰지 않는다 (OFF = 0 A 전류 명령).
            for i in self.ff_sent:
                self.ff_sent[i] = 0.0
        elif any(self.ff_sent[i] != self._ff_target(sec, i) for i in FF_MOTORS):
            self._command(sec, cmd)
        reason = self._ff_block_reason()
        if self.btn_ff.isChecked() and reason != self.ff_reason and reason == "M1, M2 모두 선택 필요":
            self._log(f"{sec.name}: {reason} → i_ff = 0")
        self.ff_reason = reason
        self._update_ff_label()

    def _update_ff_label(self):
        reason = self._ff_block_reason()
        values = "  ".join(f"M{i} {a:+.3f}" for i, a in self.ff_sent.items())
        self.lbl_ff.setText(f"{values} A  ({reason or '적용 중'})")

    # ------------- 사용자 입력 / 주기 처리 -------------
    def _on_ff_toggled(self, checked):
        self.btn_ff.setText("FF ON" if checked else "FF OFF")
        self._log(f"Feedforward {'ON' if checked else 'OFF'}")
        self._refresh_ff()

    def _on_control(self, sec, checked):
        # 부모는 첫 위치 명령을 soft start 등록 전에 보내므로, 그 동안 i_ff를 막는다.
        self.ff_hold.add(sec)
        try:
            super()._on_control(sec, checked)
        finally:
            self.ff_hold.discard(sec)
        if sec is self.ff_sec:
            self._refresh_ff()  # Soft start 0 s면 여기서 바로 i_ff 적용

    def _ramp(self):
        super()._ramp()     # soft start가 끝나면 softstart에서 빠진다
        self._refresh_ff()  # → 같은 주기에 i_ff 적용


# =====================================================================
# 3. 실행
# =====================================================================
def main():
    app = QtWidgets.QApplication(sys.argv)
    window = FeedForwardApp()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
