# -*- coding: utf-8 -*-
"""Standalone open-loop voltage control: python tools/open_loop.py.

The monitor does not import this module. Only mode 0x04 commands are sent,
using Extended CAN FD / BRS OFF. Mechanical RPM display assumes seven pole
pairs; this does not write the firmware's pole-pair setting.
"""
import math
import queue
import socket
import struct
import sys
import threading
import time

import can
from PyQt5 import QtCore, QtWidgets
from serial.tools import list_ports

from can_io import CANBusManager, CANReaderThread, connection_options
from can_workers import CANWriterThread
from config import DEFAULT_BITRATE, DEFAULT_CHANNEL, DEFAULT_INTERFACE, TX_TIMEOUT_S
from protocol import build_ext_id, parse_broadcast_frame


MODE_OPEN_LOOP = 0x04
MOTOR_POLE_PAIRS = 7


def pack_open_loop_command(electrical_rpm, voltage_v):
    """Signed eRPM and non-negative mV, exactly eight big-endian bytes."""
    if not math.isfinite(electrical_rpm) or not -600 <= electrical_rpm <= 600:
        raise ValueError("Electrical RPM must be between -600 and +600")
    if not math.isfinite(voltage_v) or not 0 <= voltage_v <= 2.24:
        raise ValueError("Voltage must be between 0 and 2.24 V")
    return struct.pack(">ii", round(electrical_rpm), round(voltage_v * 1000.0))


class OpenLoopWriter(CANWriterThread):
    """Keep stop serialization and queue invalidation local to this program."""
    def __init__(self, bus):
        super().__init__(bus, bitrate_switch=False)
        self.send_lock = threading.Lock()

    def _send(self, message, timeout, tx_generation):
        with self.send_lock:
            return super()._send(message, timeout, tx_generation)

    def send_immediate(self, eid, payload, timeout=TX_TIMEOUT_S):
        # Called by a background action, never by the GUI thread. A packet
        # already in the backend can finish, then the stop precedes new work.
        # Attempt a stop even if ordinary TX is paused/stopped.
        with self.send_lock:
            with self.lock:
                self.factory = None
                self.generation += 1
                self.tx_generation += 1
                self.latest_reference = None
                while True:
                    try:
                        self.commands.get_nowait()
                    except queue.Empty:
                        break
            self.wake.set()
            try:
                self.bus.send(self._message(eid, payload), timeout=timeout)
            except Exception as exc:
                with self.lock:
                    self.errors.append(str(exc))
                self.stop()
                raise
            with self.lock:
                self.sent += 1


class OpenLoopBusManager(CANBusManager):
    def __init__(self):
        super().__init__()
        self.open_loop_motors = set()

    def connect(self, interface, channel, bitrate=DEFAULT_BITRATE):
        if self.bus is not None:
            return
        bus = can.Bus(**connection_options(interface, channel, bitrate))
        try:
            writer = OpenLoopWriter(bus)
            writer.start()
        except Exception:
            bus.shutdown()
            raise
        self.bus, self.writer = bus, writer
        self.interface, self.channel = interface, channel.strip()

    def send_open_loop(self, motor_id, electrical_rpm, voltage_v):
        self._check_motor(motor_id)
        payload = pack_open_loop_command(electrical_rpm, voltage_v)
        if round(voltage_v * 1000.0) == 0:
            self.stop_open_loop(motor_id)
            return
        self.writer.enqueue(build_ext_id(MODE_OPEN_LOOP, motor_id), payload)
        # Track queued commands too, so disconnect cannot miss a pending start.
        self.open_loop_motors.add(motor_id)

    def _check_motor(self, motor_id):
        if not isinstance(motor_id, int) or not 0 <= motor_id <= 255:
            raise ValueError("Motor ID must be an integer in 0..255")
        if self.bus is None or self.writer is None:
            raise RuntimeError("CAN bus not connected")

    def stop_open_loop(self, motor_id):
        self._check_motor(motor_id)
        self.writer.send_immediate(build_ext_id(MODE_OPEN_LOOP, motor_id),
                                   pack_open_loop_command(0, 0))
        self.open_loop_motors.discard(motor_id)

    def stop_open_loop_motors(self):
        errors = []
        for motor_id in sorted(self.open_loop_motors):
            try:
                self.stop_open_loop(motor_id)
            except Exception as exc:
                errors.append(f"Motor {motor_id}: {exc}")
        if errors:
            raise RuntimeError("Open-loop stop failed: " + "; ".join(errors))


class OpenLoopWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("MA732 — Open-loop Voltage Control")
        self.resize(760, 570)
        self.manager = OpenLoopBusManager()
        self.reader = None
        self.reader_stop = threading.Event()
        self._action_thread = None
        self._action_kind = None
        self._action_error = None
        self._closing = False
        self._last_sample_ts = 0.0
        self.settings = QtCore.QSettings("MA732", "OpenLoopControl")
        self._build_ui()
        self.interface.setCurrentText(DEFAULT_INTERFACE)
        self.interface.currentTextChanged.connect(self._load_channel)
        self._load_channel()
        self._sync_controls()
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._poll)
        self.timer.start(100)

    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)
        connection = QtWidgets.QGroupBox("CAN 연결 — Extended FD / BRS OFF")
        form = QtWidgets.QGridLayout(connection)
        self.interface = QtWidgets.QComboBox()
        self.interface.addItems(["socketcan", "slcan"])
        self.channel = QtWidgets.QComboBox()
        self.channel.setEditable(True)
        self.refresh = QtWidgets.QPushButton("포트 새로고침")
        self.refresh.clicked.connect(self._refresh_channels)
        self.connect_button = QtWidgets.QPushButton("Connect")
        self.connect_button.clicked.connect(self._on_connect)
        self.disconnect_button = QtWidgets.QPushButton("Disconnect / Stop all")
        self.disconnect_button.clicked.connect(self._on_disconnect)
        form.addWidget(QtWidgets.QLabel("Interface"), 0, 0)
        form.addWidget(self.interface, 0, 1)
        form.addWidget(QtWidgets.QLabel("Channel"), 1, 0)
        form.addWidget(self.channel, 1, 1)
        form.addWidget(self.refresh, 1, 2)
        form.addWidget(self.connect_button, 2, 1)
        form.addWidget(self.disconnect_button, 2, 2)
        self.hint = QtWidgets.QLabel()
        self.hint.setWordWrap(True)
        form.addWidget(self.hint, 3, 0, 1, 3)
        layout.addWidget(connection)

        control = QtWidgets.QGroupBox("Open-loop rotation")
        grid = QtWidgets.QGridLayout(control)
        self.motor_id = QtWidgets.QSpinBox()
        self.motor_id.setRange(0, 255)
        self.electrical_rpm = QtWidgets.QDoubleSpinBox()
        self.electrical_rpm.setRange(-600, 600)
        self.electrical_rpm.setDecimals(0)
        self.electrical_rpm.setValue(55)
        self.voltage = QtWidgets.QDoubleSpinBox()
        self.voltage.setRange(0, 2.24)
        self.voltage.setDecimals(3)
        self.voltage.setSingleStep(0.05)
        self.voltage.setValue(0.1)
        self.mechanical_rpm = QtWidgets.QLabel()
        self.electrical_rpm.valueChanged.connect(self._update_mechanical_rpm)
        self.motor_id.valueChanged.connect(self._reset_status)
        self.start_button = QtWidgets.QPushButton("Start / Update")
        self.start_button.clicked.connect(self._on_start)
        self.stop_button = QtWidgets.QPushButton("Stop (0 V)")
        self.stop_button.clicked.connect(self._on_stop)
        for row, (label, widget) in enumerate([
            ("Motor ID", self.motor_id),
            ("Electrical RPM (± 방향)", self.electrical_rpm),
            ("Voltage (V)", self.voltage),
        ]):
            grid.addWidget(QtWidgets.QLabel(label), row, 0)
            grid.addWidget(widget, row, 1)
        grid.addWidget(self.mechanical_rpm, 3, 0, 1, 2)
        grid.addWidget(self.start_button, 4, 0)
        grid.addWidget(self.stop_button, 4, 1)
        self._update_mechanical_rpm()
        layout.addWidget(control)
        note = QtWidgets.QLabel(
            "전류 PI 제어 없음 · 낮은 전압부터 시험하세요. 입력값 변경만으로는 송신하지 않습니다.\n"
            "통신이 끊겨도 마지막 명령으로 계속 구동합니다. 0 eRPM + 전압 > 0은 고정 전기각 인가입니다.\n"
            "Disconnect / 창 종료 시 이 프로그램이 명령을 보낸 모든 모터에 0 V 정지를 시도합니다.")
        note.setWordWrap(True)
        layout.addWidget(note)
        self.status = QtWidgets.QLabel("Disconnected")
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        self.motor_status = QtWidgets.QLabel("선택한 모터의 상태 수신 대기")
        self.motor_status.setWordWrap(True)
        layout.addWidget(self.motor_status)
        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(100)
        layout.addWidget(self.log, 1)

    def _load_channel(self):
        interface = self.interface.currentText()
        default = DEFAULT_CHANNEL if interface == DEFAULT_INTERFACE else ""
        self.channel.setEditText(str(self.settings.value(f"{sys.platform}/{interface}/channel", default)))
        self._refresh_channels()
        self.hint.setText("SocketCAN은 OS에서 FD 1M/5M 설정 후 사용하세요." if interface == "socketcan" else
                          "CANable 2.0 호환 FD SLCAN / 1M/5M. 같은 시리얼 어댑터를 쓰는 app.py / LUT.py는 먼저 닫으세요.")

    def _refresh_channels(self):
        selected = self.channel.currentText()
        try:
            if self.interface.currentText() == "slcan":
                channels = [port.device for port in list_ports.comports()]
            elif sys.platform.startswith("linux"):
                channels = [name for _, name in socket.if_nameindex()
                            if name.startswith(("can", "vcan", "slcan"))]
            else:
                channels = []
            self.channel.clear()
            self.channel.addItems(sorted(channels))
            self.channel.setEditText(selected or (channels[0] if channels else ""))
        except Exception as exc:
            self.log.appendPlainText(f"포트 조회 실패: {exc}")

    def _update_mechanical_rpm(self, *_):
        self.mechanical_rpm.setText(
            f"Mechanical RPM ≈ {self.electrical_rpm.value() / MOTOR_POLE_PAIRS:.2f} "
            f"({MOTOR_POLE_PAIRS} pole pairs, 표시용)")

    def _reset_status(self, *_):
        self._last_sample_ts = 0.0
        self.motor_status.setText("선택한 모터의 상태 수신 대기")

    def _sync_controls(self):
        busy = self._action_thread is not None or self._closing
        connected = self.manager.bus is not None
        writer = self.manager.writer
        ready = connected and writer is not None and not (
            writer.stop_event.is_set() or writer.pause_event.is_set())
        for widget in (self.interface, self.channel, self.refresh):
            widget.setEnabled(not connected and not busy)
        self.connect_button.setEnabled(not connected and not busy)
        self.disconnect_button.setEnabled(connected and not busy)
        self.start_button.setEnabled(ready and not busy)
        self.stop_button.setEnabled(connected and not busy)
        for widget in (self.motor_id, self.electrical_rpm, self.voltage):
            widget.setEnabled(not busy)

    def _begin_action(self, kind, action):
        if self._action_thread is not None:
            return
        self._action_kind, self._action_error = kind, None

        def run():
            try:
                action()
            except Exception as exc:
                self._action_error = str(exc)

        self._action_thread = threading.Thread(target=run, name=f"open-loop-{kind}", daemon=True)
        self._sync_controls()
        self.status.setText(f"{kind} 진행 중…")
        self._action_thread.start()

    def _on_connect(self):
        if self.manager.bus is not None or self._action_thread is not None or self._closing:
            return
        interface, channel = self.interface.currentText(), self.channel.currentText().strip()
        if not channel:
            self.status.setText("실제 CAN 채널/시리얼 포트를 입력하세요.")
            return
        self.settings.setValue(f"{sys.platform}/{interface}/channel", channel)

        def connect():
            try:
                self.manager.connect(interface, channel)
                self.reader_stop.clear()
                self.reader = CANReaderThread(self.manager.bus, self.reader_stop,
                                               on_error=self.manager.writer.stop,
                                               on_warning=self.manager.writer.pause)
                self.reader.start()
            except Exception:
                self._disconnect()
                raise

        self._reset_status()
        self._begin_action("Connect", connect)

    def _on_start(self):
        if self._action_thread is not None or self._closing:
            return
        if round(self.voltage.value() * 1000.0) == 0:
            self._on_stop()
            return
        motor, rpm, voltage = self.motor_id.value(), self.electrical_rpm.value(), self.voltage.value()
        try:
            self.manager.send_open_loop(motor, rpm, voltage)
        except Exception as exc:
            self.log.appendPlainText(f"송신 실패: {exc}")
        else:
            self.log.appendPlainText(f"Open-loop queued: motor={motor}, {rpm:g} eRPM, {voltage:.3f} V, FD / BRS OFF")

    def _on_stop(self):
        if self.manager.bus is None or self._action_thread is not None or self._closing:
            return
        motor = self.motor_id.value()
        self._begin_action(f"Stop motor {motor}", lambda: self.manager.stop_open_loop(motor))

    def _disconnect(self):
        stop_error = None
        try:
            self.manager.stop_open_loop_motors()
        except Exception as exc:
            stop_error = str(exc)
        finally:
            self.reader_stop.set()
            if self.manager.writer is not None:
                self.manager.writer.stop()
            if self.reader is not None and self.reader.ident is not None:
                self.reader.join(timeout=1.0)
            self.manager.disconnect()
            self.reader = None
        if stop_error:
            raise RuntimeError(stop_error + "; 모터 정지 확인 필요")

    def _on_disconnect(self):
        if self.manager.bus is not None and self._action_thread is None:
            self._begin_action("Disconnect / Stop all", self._disconnect)

    def _poll(self):
        if self._action_thread is not None:
            if self._action_thread.is_alive():
                return
            self._action_thread = None
            if self._action_error:
                message = f"{self._action_kind} 실패: {self._action_error}"
            else:
                message = f"{self._action_kind} 완료"
                if self._action_kind.startswith("Stop"):
                    message += " — 0 V 전송 (펌웨어 정지 확인 응답 없음)"
            self.status.setText(message)
            self.log.appendPlainText(message)
            self._sync_controls()
        if self._closing:
            if self.manager.bus is not None:
                self._on_disconnect()
            elif self._action_thread is None:
                QtCore.QTimer.singleShot(0, self.close)
            return
        if self.manager.bus is None or self._action_thread is not None:
            return
        self._sync_controls()
        _, _, errors = self.manager.writer.take_stats()
        for error in errors:
            self.log.appendPlainText(f"CAN TX 실패: {error}")
        if errors or self.reader.last_error:
            if self.reader.last_error:
                self.log.appendPlainText(self.reader.last_error)
            self._on_disconnect()
            return
        for _, notice in self.reader.take_notices():
            self.log.appendPlainText(notice)
        sample = self.reader.take_latest_status(self.motor_id.value())
        if sample is not None:
            self._last_sample_ts, payload = sample
            state = parse_broadcast_frame(payload)
            self.motor_status.setText(
                f"Motor {self.motor_id.value()}: {state['pos_deg']:.1f} deg | "
                f"{state['spd_erpm']:g} eRPM / {state['spd_erpm'] / MOTOR_POLE_PAIRS:.2f} RPM | "
                f"{state['cur_A']:.3f} A | {state['temp_C']} °C | error 0x{state['error']:02X}")
        elif self._last_sample_ts and time.monotonic() - self._last_sample_ts > 0.5:
            self.motor_status.setText("선택한 모터의 상태 수신 지연 (stale)")

    def closeEvent(self, event):
        self._closing = True
        if self._action_thread is not None or self.manager.bus is not None:
            self._on_disconnect()
            self._sync_controls()
            event.ignore()
        else:
            self.timer.stop()
            event.accept()


def main():
    app = QtWidgets.QApplication(sys.argv)
    window = OpenLoopWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
