# -*- coding: utf-8 -*-
"""one_finger의 GUI/CAN/IK/ROM Guard를 사용하되 DXL은 전류 기반 위치 제어(mode 5).

실행: python3 tools/dxl_current.py
DXL 연결 시 토크 OFF에서 모드와 설정을 적용한다. 위치 제어를 켜야 토크가 켜진다.
초기 전류 상한 100 mA, Position P/I/D = 800/0/500 (raw). GUI에서 설정 적용 가능.
one_finger.py의 mode 4 동작은 변경하지 않는다.
"""
import sys
import threading
import time
from dataclasses import dataclass

from PyQt5 import QtWidgets

from one_finger import (DXL_ADDR_DRIVE_MODE, DXL_ADDR_GOAL_POSITION, DXL_ADDR_OPERATING_MODE,
                        DXL_ADDR_PRESENT_POSITION, DXL_ADDR_PROFILE_ACCELERATION,
                        DXL_ADDR_PROFILE_VELOCITY, DXL_ADDR_TORQUE_ENABLE, DXL_BAUD, DXL_GEAR,
                        DXL_ID, DXL_PERIOD_S, DXL_PORT, DXL_PROFILE_ACCELERATION,
                        DXL_PROFILE_VELOCITY, STALE_S, DxlError, OneFingerApp,
                        U2D2Dynamixel, clamp, dxl_signed32)

DXL_MODE_CURRENT_POSITION = 5
DXL_MODEL_NUMBER = 1710       # XM335-T323-T: 전류 단위 약 1 mA/count
DXL_ADDR_MODEL_NUMBER = 0
DXL_ADDR_CURRENT_LIMIT = 38
DXL_ADDR_POSITION_D = 80
DXL_ADDR_POSITION_I = 82
DXL_ADDR_POSITION_P = 84
DXL_ADDR_GOAL_CURRENT = 102
DXL_ADDR_PRESENT_CURRENT = 126
DXL_MAX_CURRENT_MA = 910
DXL_MAX_GAIN = 16383
DXL_DRIVE_PROFILE_AUTO_TORQUE = 0x0C  # bit 2: 시간 기반 프로파일, bit 3: Goal로 자동 토크 ON


def dxl_signed16(value):
    return value - (1 << 16) if value & 0x8000 else value


@dataclass(frozen=True)
class DxlCurrentSettings:
    current_ma: int = 100
    p: int = 800
    i: int = 0
    d: int = 500

    def validate(self, current_limit=DXL_MAX_CURRENT_MA):
        for name, value, maximum in (("전류 상한", self.current_ma, current_limit),
                                     ("P", self.p, DXL_MAX_GAIN), ("I", self.i, DXL_MAX_GAIN),
                                     ("D", self.d, DXL_MAX_GAIN)):
            if type(value) is not int or not 0 <= value <= maximum:
                raise ValueError(f"{name}: 0~{maximum} 범위의 정수가 필요합니다 ({value})")


class U2D2CurrentDynamixel(U2D2Dynamixel):
    def w2(self, i, a, v):
        if type(v) is not int or not 0 <= v <= 0xFFFF:
            raise ValueError(f"write2 범위 오류: {v}")
        self._call(self.packet.write2ByteTxRx, f"write2 ID {i}@{a}", i, a, v)


class CurrentPositionWorker(threading.Thread):
    """포트 접근은 이 스레드에서만 수행한다. 오류는 래치하여 재연결 전 자동 토크 ON을 막는다."""

    def __init__(self, dxl, dxl_id, settings=None):
        super().__init__(daemon=True)
        self.dxl, self.id = dxl, dxl_id
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.torque, self.goal = False, None
        self.sample = None
        self.errors = []
        self._torque_on, self._goal_sent = False, None
        self.settings = settings or DxlCurrentSettings()
        self.settings.validate()
        self.applied_settings = None
        self.current_limit = None
        self.current_sample = None  # (수신 시각, signed 전류 [mA])
        self.ready, self.fault = False, None
        self.bounds = None
        self.epoch = 0
        self._active_epoch = None
        self.activation = None     # (요청 세대, 토크 ON 시점의 위치 raw)

    def request(self, torque, goal=None, bounds=None):
        with self.lock:
            if not torque or torque != self.torque:
                self.epoch += 1
                self.activation = None
            self.torque = torque
            if goal is not None:
                self.goal = goal
            if bounds is not None:
                self.bounds = bounds

    def take_errors(self):
        with self.lock:
            errors, self.errors = self.errors, []
        return errors

    def begin(self, bounds, settings):
        with self.lock:
            if not self.ready or self.fault:
                raise DxlError("mode 5 준비가 완료되지 않았습니다. DXL을 재연결하세요.")
            settings.validate(self.current_limit)
            self.epoch += 1
            self.torque, self.goal = True, None
            self.bounds, self.settings = bounds, settings
            self.activation = None

    def request_settings(self, settings):
        with self.lock:
            if not self.ready or self.fault:
                raise DxlError("mode 5 준비가 완료되지 않았습니다. DXL을 재연결하세요.")
            settings.validate(self.current_limit)
            self.settings = settings

    def snapshot(self):
        with self.lock:
            return dict(ready=self.ready, fault=self.fault, active=self._torque_on is True,
                        current_limit=self.current_limit, current_sample=self.current_sample,
                        applied=self.applied_settings, activation=self.activation)

    @staticmethod
    def _expect(name, value, expected):
        if value != expected:
            raise DxlError(f"{name} 확인 실패: 요청 {expected}, 수신 {value}")

    def _write_settings(self, settings):
        settings.validate(self.current_limit)
        dxl, i = self.dxl, self.id
        gains = [(DXL_ADDR_POSITION_P, settings.p), (DXL_ADDR_POSITION_I, settings.i),
                 (DXL_ADDR_POSITION_D, settings.d)]
        current = (DXL_ADDR_GOAL_CURRENT, settings.current_ma)
        # 동작 중 상한을 줄일 때는 전류부터, 늘릴 때는 게인부터 적용한다.
        increasing = self.applied_settings and settings.current_ma > self.applied_settings.current_ma
        values = gains + [current] if increasing else [current] + gains
        for address, value in values:
            dxl.w2(i, address, value)
        for address, value in values:
            self._expect(f"레지스터 {address}", dxl.r2(i, address), value)
        with self.lock:
            self.applied_settings = settings

    def _prepare(self):
        dxl, i = self.dxl, self.id
        dxl.torque(i, False)
        self._expect("Torque OFF", dxl.r1(i, DXL_ADDR_TORQUE_ENABLE), 0)
        self._torque_on = False
        self._expect("XM335 모델", dxl.r2(i, DXL_ADDR_MODEL_NUMBER), DXL_MODEL_NUMBER)
        drive = dxl.r1(i, DXL_ADDR_DRIVE_MODE)
        desired_drive = drive & ~DXL_DRIVE_PROFILE_AUTO_TORQUE
        if drive != desired_drive:
            dxl.w1(i, DXL_ADDR_DRIVE_MODE, desired_drive)
        if dxl.r1(i, DXL_ADDR_OPERATING_MODE) != DXL_MODE_CURRENT_POSITION:
            dxl.w1(i, DXL_ADDR_OPERATING_MODE, DXL_MODE_CURRENT_POSITION)
        self._expect("Drive Mode", dxl.r1(i, DXL_ADDR_DRIVE_MODE), desired_drive)
        self._expect("Operating Mode", dxl.r1(i, DXL_ADDR_OPERATING_MODE), DXL_MODE_CURRENT_POSITION)
        limit = dxl.r2(i, DXL_ADDR_CURRENT_LIMIT)
        if not 0 <= limit <= DXL_MAX_CURRENT_MA:
            raise DxlError(f"XM335 Current Limit 범위 오류: {limit}")
        with self.lock:
            self.current_limit = limit
            settings = self.settings
        # 모드 변경은 PID/전류/프로파일을 초기화하므로 그 뒤에 RAM 설정을 쓴다.
        self._write_settings(settings)
        for address, value in ((DXL_ADDR_PROFILE_ACCELERATION, DXL_PROFILE_ACCELERATION),
                               (DXL_ADDR_PROFILE_VELOCITY, DXL_PROFILE_VELOCITY)):
            dxl.w4(i, address, value)
            self._expect(f"프로파일 {address}", dxl.r4(i, address), value)
        self._read_sample()  # 모드 전환 전의 위치 캐시는 사용하지 않는다.
        with self.lock:
            self.ready = True

    def _activate(self, epoch, bounds):
        dxl, i = self.dxl, self.id
        dxl.torque(i, False)
        self._expect("Torque OFF", dxl.r1(i, DXL_ADDR_TORQUE_ENABLE), 0)
        self._expect("Operating Mode", dxl.r1(i, DXL_ADDR_OPERATING_MODE), DXL_MODE_CURRENT_POSITION)
        drive = dxl.r1(i, DXL_ADDR_DRIVE_MODE)
        if drive & DXL_DRIVE_PROFILE_AUTO_TORQUE:
            raise DxlError("Drive Mode가 변경됐습니다. DXL을 재연결하세요.")
        raw = dxl_signed32(dxl.r4(i, DXL_ADDR_PRESENT_POSITION))
        if bounds is None or not bounds[0] <= raw <= bounds[1]:
            raise DxlError("현재 위치가 MCP F/E 범위 밖입니다. 영점(Initialize)을 확인하세요.")
        dxl.w4(i, DXL_ADDR_GOAL_POSITION, raw)
        self._expect("첫 목표 위치", dxl_signed32(dxl.r4(i, DXL_ADDR_GOAL_POSITION)), raw)
        # 설정 도중 OFF/재시작 요청이 오면 이전 요청으로 토크를 켜지 않는다.
        with self.lock:
            if self.epoch != epoch or not self.torque or self.stop_event.is_set():
                return
            dxl.torque(i, True)
            self._expect("Torque ON", dxl.r1(i, DXL_ADDR_TORQUE_ENABLE), 1)
            self._torque_on, self._goal_sent = True, raw
            self._active_epoch = epoch
            self.goal = raw
            self.activation = (epoch, raw)

    def _step(self):
        with self.lock:
            torque, goal, bounds, epoch = self.torque, self.goal, self.bounds, self.epoch
            settings = self.settings
        # GUI에서 OFF→ON이 한 통신 주기 안에 일어나도 새 요청은 현재 위치에서 다시 시작한다.
        if self._torque_on and (not torque or self._active_epoch != epoch):
            self.dxl.torque(self.id, False)
            with self.lock:
                self._torque_on, self._active_epoch = False, None
        if settings != self.applied_settings:
            self._write_settings(settings)
        if torque:
            if not self._torque_on:
                self._activate(epoch, bounds)
            elif goal is not None and goal != self._goal_sent:
                if bounds is None or not bounds[0] <= goal <= bounds[1]:
                    raise DxlError("목표 위치가 MCP F/E 범위를 벗어났습니다.")
                self.dxl.w4(self.id, DXL_ADDR_GOAL_POSITION, goal)
                self._goal_sent = goal

    def _read_sample(self):
        current = dxl_signed16(self.dxl.r2(self.id, DXL_ADDR_PRESENT_CURRENT))
        raw = dxl_signed32(self.dxl.r4(self.id, DXL_ADDR_PRESENT_POSITION))
        stamp = time.monotonic()
        with self.lock:
            self.sample = (stamp, raw)
            self.current_sample = (stamp, current)

    def run(self):
        try:
            self._prepare()
            while not self.stop_event.is_set():
                start = time.monotonic()
                self._step()
                self._read_sample()
                self.stop_event.wait(max(0.0, DXL_PERIOD_S - (time.monotonic() - start)))
        except Exception as exc:
            with self.lock:
                self.ready, self.torque = False, False
                self.fault = str(exc) or type(exc).__name__
                self.errors.append(f"mode 5 제어 중단: {exc} (DXL 재연결 필요)")
                self.sample, self.current_sample = None, None
        finally:
            try:
                self.dxl.torque(self.id, False)
            except Exception as exc:
                with self.lock:
                    self.errors.append(f"토크 OFF 전송 실패: {exc}")
            with self.lock:
                self.ready, self.torque, self._torque_on = False, False, False
                self.activation, self._active_epoch = None, None


class DxlCurrentApp(OneFingerApp):
    def __init__(self):
        self._seen_activation = None
        self._seen_fault = None
        super().__init__()
        self.setWindowTitle("One Finger Control - DXL Current-based Position (mode 5)")
        self.mcp.setTitle(f"MCP F/E - Dynamixel ID {DXL_ID}  (mode 5, gear {DXL_GEAR:.2f})")
        defaults = DxlCurrentSettings()
        self.spin_current = self._spin(DXL_MAX_CURRENT_MA, defaults.current_ma, 10)
        self.spin_current.setSuffix(" mA")
        self.spin_current.setToolTip("DXL 출력 전류 상한. 0 mA에서는 위치 유지 토크가 나오지 않습니다.")
        self.spin_p = self._spin(DXL_MAX_GAIN, defaults.p, 50)
        self.spin_i = self._spin(DXL_MAX_GAIN, defaults.i, 1)
        self.spin_d = self._spin(DXL_MAX_GAIN, defaults.d, 50)
        gains = QtWidgets.QWidget()
        gain_layout = QtWidgets.QHBoxLayout(gains)
        gain_layout.setContentsMargins(0, 0, 0, 0)
        for name, spin in (("P", self.spin_p), ("I", self.spin_i), ("D", self.spin_d)):
            gain_layout.addWidget(QtWidgets.QLabel(name))
            gain_layout.addWidget(spin)
        self.btn_apply_dxl = QtWidgets.QPushButton("DXL 설정 적용")
        self.btn_apply_dxl.setEnabled(False)
        self.btn_apply_dxl.clicked.connect(self._apply_dxl_settings)
        self.lbl_current = QtWidgets.QLabel("Actual: -- mA")
        self.lbl_dxl_mode = QtWidgets.QLabel("mode 5 · disconnected")
        self.lbl_dxl_mode.setWordWrap(True)
        grid = self.mcp.layout()
        grid.addWidget(QtWidgets.QLabel("Current limit"), 3, 0)
        grid.addWidget(self.spin_current, 3, 1)
        grid.addWidget(self.lbl_current, 3, 2)
        grid.addWidget(QtWidgets.QLabel("Position PID (raw)"), 4, 0)
        grid.addWidget(gains, 4, 1)
        grid.addWidget(self.btn_apply_dxl, 4, 2)
        grid.addWidget(self.lbl_dxl_mode, 5, 0, 1, 3)

    @staticmethod
    def _spin(maximum, value, step):
        spin = QtWidgets.QSpinBox()
        spin.setRange(0, maximum)
        spin.setValue(value)
        spin.setSingleStep(step)
        spin.setKeyboardTracking(False)
        return spin

    def _dxl_settings(self):
        return DxlCurrentSettings(self.spin_current.value(), self.spin_p.value(),
                                  self.spin_i.value(), self.spin_d.value())

    def _dxl_bounds(self):
        return tuple(sorted((self._deg_to_raw(self.mcp.low), self._deg_to_raw(self.mcp.high))))

    def _on_mcp_connect(self, checked):
        if not checked:
            super()._on_mcp_connect(False)
            self.btn_apply_dxl.setEnabled(False)
            self.lbl_current.setText("Actual: -- mA")
            self.lbl_dxl_mode.setText("mode 5 · disconnected")
            return
        try:
            self.dxl = U2D2CurrentDynamixel(DXL_PORT, DXL_BAUD)
            model = self.dxl.ping(DXL_ID)
            if model != DXL_MODEL_NUMBER:
                raise DxlError(f"XM335-T323-T가 아닙니다: model {model}, 기대값 {DXL_MODEL_NUMBER}")
            self.dxl_worker = CurrentPositionWorker(self.dxl, DXL_ID, self._dxl_settings())
            self.dxl_errors, self.dxl_pending = 0, []
            self._seen_activation, self._seen_fault = None, None
            self.dxl_worker.start()
            self.mcp.set_connected(True)
            self.mcp.cb_control.setEnabled(False)
            self.lbl_dxl_mode.setText("mode 5 설정 중 (토크 OFF)")
            self._log(f"MCP F/E: U2D2 연결 ({DXL_PORT}, {DXL_BAUD} bps), mode 5 설정 중")
        except Exception as exc:
            self._close_dxl()
            self.mcp.force_off(self.mcp.cb_connect)
            self.mcp.set_connected(False)
            self._log(f"MCP F/E: 다이나믹셀 연결 실패: {exc}")

    def _apply_dxl_settings(self):
        if self.dxl_worker is None or not self.dxl_worker.snapshot()["ready"]:
            return
        try:
            self.dxl_worker.request_settings(self._dxl_settings())
            self._log("MCP F/E: DXL 전류 상한/PID 적용 요청")
        except (DxlError, ValueError) as exc:
            self._log(f"MCP F/E: 설정 오류: {exc}")

    def _on_mcp_control(self, checked):
        worker = self.dxl_worker
        if not checked:
            self.cmd.pop(self.mcp, None)
            if worker is not None:
                worker.request(False)
            self._log("MCP F/E: 위치 제어 OFF (토크 OFF)")
            return
        current = self._mcp_deg()
        if (worker is None or not worker.snapshot()["ready"] or current is None
                or not self.mcp.low <= current <= self.mcp.high):
            self.mcp.force_off(self.mcp.cb_control)
            self._log("MCP F/E: mode 5 설정/상태 수신/위치 범위를 확인하세요. 영점은 Initialize로 설정합니다.")
            return
        self.mcp.set_ref(current)
        self.cmd.pop(self.mcp, None)  # worker가 토크 ON 위치를 알려줄 때까지 보간/IK 명령은 보내지 않는다.
        self.mcp.sld_ref.setEnabled(False)
        try:
            worker.begin(self._dxl_bounds(), self._dxl_settings())
            self._log("MCP F/E: 현재 위치에서 mode 5 위치 제어 시작 요청")
        except (DxlError, ValueError) as exc:
            self.mcp.force_off(self.mcp.cb_control)
            self._log(f"MCP F/E: 설정 오류: {exc}")

    def _command(self, sec, deg):
        if sec is not self.mcp:
            return super()._command(sec, deg)
        if self.cmd.get(sec) is None or self.dxl_worker is None:
            return False
        state = self.dxl_worker.snapshot()
        if not state["ready"] or not state["active"]:
            return False
        target = clamp(deg, sec.low, sec.high)
        self.cmd[sec] = target
        self.dxl_worker.request(True, self._deg_to_raw(target), self._dxl_bounds())
        return True

    def _on_initialize(self):
        if self.mcp.cb_control.isChecked() and self.cmd.get(self.mcp) is None:
            self.mcp.cb_control.setChecked(False)  # 초기화 중인 제어 시작 요청을 취소한다.
        super()._on_initialize()

    def _poll(self):
        super()._poll()
        if self.dxl_worker is None:
            return
        state = self.dxl_worker.snapshot()
        self.mcp.cb_control.setEnabled(state["ready"])
        self.btn_apply_dxl.setEnabled(state["ready"])
        if state["current_limit"] is not None:
            self.spin_current.setMaximum(state["current_limit"])
        if state["fault"]:
            if state["fault"] != self._seen_fault:
                self._seen_fault = state["fault"]
                self.mcp.cb_control.setChecked(False)
                self._log(f"MCP F/E: mode 5 중단: {state['fault']} → DXL 재연결 필요")
            self.lbl_dxl_mode.setText(f"mode 5 오류: {state['fault']}")
        elif state["ready"]:
            activation = state["activation"]
            if (state["active"] and self.mcp.cb_control.isChecked() and activation is not None
                    and activation != self._seen_activation):
                self._seen_activation = activation
                current = self._raw_to_deg(activation[1])
                self.mcp.set_ref(current)
                self.cmd[self.mcp] = current
                self.mcp.sld_ref.setEnabled(True)
            applied = state["applied"]
            active = "ON" if state["active"] else "OFF"
            self.lbl_dxl_mode.setText(f"mode 5 · 토크 {active} · 적용 {applied.current_ma} mA · "
                                      f"P/I/D {applied.p}/{applied.i}/{applied.d} · "
                                      f"모터 상한 {state['current_limit']} mA")
        sample = state["current_sample"]
        text = "--" if sample is None or time.monotonic() - sample[0] > STALE_S else str(sample[1])
        self.lbl_current.setText(f"Actual: {text} mA")


def main():
    app = QtWidgets.QApplication(sys.argv)
    window = DxlCurrentApp()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
