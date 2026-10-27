# -*- coding: utf-8 -*-
"""Standalone cogging measurement and sweep/LUT download GUI.

Run: python tools/cogging_LUT.py. A single background session owns CAN RX/TX.
The monitor does not import this GUI or add its collection/rendering workload.
"""
import queue
import socket
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import can
import numpy as np
import pyqtgraph as pg
from PyQt5 import QtCore, QtWidgets
from serial.tools import list_ports

from can_io import connection_options
from config import DEFAULT_BITRATE, DEFAULT_CHANNEL, DEFAULT_INTERFACE
from cogging_reader import (CoggingCancelled, CoggingReader, SELECTOR_FILES,
                            write_cogging_files)


SERIES_LABELS = {0: "CW 1", 1: "CW 2", 2: "CW 3", 0x40: "CCW 1", 0x41: "CCW 2",
                 0x42: "CCW 3", 0x80: "Final LUT"}


class SessionCancellation:
    def __init__(self, session):
        self.session = session

    def is_set(self):
        return any(event.is_set() for event in
                   (self.session.stop_event, self.session.abort_event, self.session.cancel_event))


class CoggingSession(QtCore.QThread):
    connected_changed = QtCore.pyqtSignal(bool)
    status_received = QtCore.pyqtSignal(object)
    activity_changed = QtCore.pyqtSignal(bool)
    busy_changed = QtCore.pyqtSignal(str)
    progress = QtCore.pyqtSignal(int, int)
    capture_received = QtCore.pyqtSignal(object)
    notice = QtCore.pyqtSignal(str)

    def __init__(self, interface, channel, driver_id, timeout=0.35, retries=5, parent=None):
        super().__init__(parent)
        self.interface, self.channel, self.driver_id = interface, channel, driver_id
        self.timeout, self.retries = timeout, retries
        self.commands = queue.Queue(maxsize=8)
        self.stop_event = threading.Event()
        self.abort_event = threading.Event()
        self.cancel_event = threading.Event()
        self.error = None
        self.last_status = None
        self.monitoring = False
        self.may_be_running = False
        self.auto_download = False
        self._saw_running = False
        self._last_progress = 0.0

    def enqueue(self, kind, argument=None):
        if self.stop_event.is_set():
            raise RuntimeError("CAN session is closing")
        if self.abort_event.is_set():
            raise RuntimeError("Measurement abort is pending")
        if kind in ("download", "final"):
            self.cancel_event.clear()
        try:
            self.commands.put_nowait((kind, argument))
        except queue.Full:
            raise RuntimeError("Command queue full") from None

    def abort(self):
        # Interrupt a receive/download, then send 0x30/0 before any new work.
        self.abort_event.set()
        while True:
            try:
                self.commands.get_nowait()
            except queue.Empty:
                break

    def close_session(self):
        self.stop_event.set()

    def _publish_status(self, status):
        self.last_status = status
        self.status_received.emit(status)
        if status.state == 1:
            self._saw_running = True
        if self.monitoring and (status.state == 3 or (self._saw_running and status.state != 1)):
            self.monitoring = self.may_be_running = False
            if (self.auto_download and status.state == 2 and status.has_raw
                    and status.lut_valid and not status.processing and not self.stop_event.is_set()):
                self.enqueue("download")
        self.activity_changed.emit(self.monitoring or status.state == 1)

    def _progress(self, done, total):
        now = time.monotonic()
        if done == total or now - self._last_progress >= 0.1:
            self._last_progress = now
            self.progress.emit(done, total)

    def _execute(self, reader, kind, argument):
        if kind == "status":
            self._publish_status(reader.read_status())
        elif kind == "start":
            status = reader.read_status()
            self._publish_status(status)
            if status.state == 1 or status.processing:
                raise RuntimeError("측정/후처리 중입니다. 먼저 중단하거나 완료를 기다리세요.")
            rpm, self.auto_download = argument
            self.may_be_running = True  # Track an attempted start conservatively.
            reader.start_measurement(rpm)
            self.monitoring, self._saw_running = True, False
            self.activity_changed.emit(True)
            self.notice.emit(f"측정 시작 요청: {rpm:g} mechanical RPM. 상태를 읽어 확인합니다.")
            self._publish_status(reader.read_status())
        elif kind == "compensation":
            reader.set_compensation(argument)
            self.notice.emit(f"보상 {'ON' if argument else 'OFF'} 요청 전송 (RAM 설정 / ACK 없음)")
            self._publish_status(reader.read_status())
        elif kind in ("download", "final"):
            result = reader.download(kind == "final", self._publish_status, self._progress)
            self.capture_received.emit(result)
            self.notice.emit(f"수집 완료: {len(result.series)}개 데이터 / {sum(map(len, result.series.values()))} bins")

    def run(self):
        bus = reader = None
        try:
            if self.stop_event.is_set():
                return
            bus = can.Bus(**connection_options(self.interface, self.channel, DEFAULT_BITRATE))
            reader = CoggingReader(bus, self.driver_id, self.timeout, self.retries,
                                    SessionCancellation(self),
                                    lambda label, retry, limit: self.notice.emit(
                                        f"{label}: 타임아웃, 재요청 {retry}/{limit}"))
            self.connected_changed.emit(True)
            next_status = time.monotonic()
            while not self.stop_event.is_set():
                if self.abort_event.is_set():
                    self.busy_changed.emit("abort")
                    self.abort_event.clear()
                    self.cancel_event.clear()
                    reader.abort_measurement()
                    self.monitoring = self.may_be_running = False
                    self.activity_changed.emit(False)
                    self.notice.emit("중단 요청 전송: 0x30 / 0 RPM. 전류 지령 0을 요청했습니다.")
                    self.busy_changed.emit("")
                    next_status = time.monotonic()
                try:
                    kind, argument = self.commands.get(timeout=0.05)
                except queue.Empty:
                    if time.monotonic() < next_status:
                        continue
                    kind, argument = "status", None
                if self.stop_event.is_set() or self.abort_event.is_set():
                    continue
                self.busy_changed.emit(kind)
                try:
                    self._execute(reader, kind, argument)
                except CoggingCancelled:
                    self.cancel_event.clear()
                    self.notice.emit("수집/상태 요청 취소 — 부분 데이터는 저장하지 않습니다.")
                except (TimeoutError, ValueError, RuntimeError, OSError) as exc:
                    self.notice.emit(f"{kind} 실패: {exc}")
                finally:
                    self.busy_changed.emit("")
                next_status = time.monotonic() + 1.0
        except Exception as exc:
            self.error = str(exc)
            self.notice.emit(f"CAN 연결/통신 실패: {exc}")
        finally:
            if reader is not None and (self.may_be_running or
                                       (self.last_status is not None and self.last_status.state == 1)):
                try:
                    reader.abort_measurement()
                    self.notice.emit("연결 해제 전 측정 중단 요청 전송")
                except Exception as exc:
                    self.notice.emit(f"측정 중단 전송 실패: {exc}; 모터 정지 확인 필요")
            if bus is not None:
                try:
                    bus.shutdown()
                except Exception as exc:
                    self.error = self.error or str(exc)
            self.connected_changed.emit(False)


class CoggingSaveWorker(QtCore.QThread):
    def __init__(self, directory, capture, parent=None):
        super().__init__(parent)
        self.directory, self.capture = directory, capture
        self.error = None

    def run(self):
        try:
            write_cogging_files(self.directory, self.capture)
        except Exception as exc:
            self.error = str(exc)


class CoggingWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("MA732 — Cogging Sweeps / LUT")
        self.resize(1120, 880)
        self.worker = None
        self.save_worker = None
        self.capture = None
        self.last_status = None
        self.connected = False
        self.measuring = False
        self.job = ""
        self._closing = False
        self.settings = QtCore.QSettings("MA732", "CoggingLUT")
        self._build_ui()
        self.interface.setCurrentText(DEFAULT_INTERFACE)
        self.interface.currentTextChanged.connect(self._load_channel)
        self._load_channel()
        self._sync_controls()

    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)
        config = QtWidgets.QGroupBox("CAN — Extended FD / BRS OFF 송신·진단 응답")
        grid = QtWidgets.QGridLayout(config)
        self.interface = QtWidgets.QComboBox()
        self.interface.addItems(["socketcan", "slcan"])
        self.channel = QtWidgets.QComboBox()
        self.channel.setEditable(True)
        self.driver_id = QtWidgets.QSpinBox()
        self.driver_id.setRange(0, 255)
        self.refresh = QtWidgets.QPushButton("포트 새로고침")
        self.refresh.clicked.connect(self._refresh_channels)
        self.timeout_ms = QtWidgets.QSpinBox()
        self.timeout_ms.setRange(10, 5000)
        self.timeout_ms.setValue(350)
        self.timeout_ms.setSuffix(" ms")
        self.retries = QtWidgets.QSpinBox()
        self.retries.setRange(0, 10)
        self.retries.setValue(5)
        self.connect_button = QtWidgets.QPushButton("Connect")
        self.connect_button.clicked.connect(self._connect)
        self.disconnect_button = QtWidgets.QPushButton("Disconnect / Abort")
        self.disconnect_button.clicked.connect(self._disconnect)
        for column, (label, widget) in enumerate([
            ("Interface", self.interface), ("Channel", self.channel), ("Driver ID", self.driver_id),
            ("응답 timeout", self.timeout_ms), ("추가 재요청", self.retries),
        ]):
            grid.addWidget(QtWidgets.QLabel(label), 0, column)
            grid.addWidget(widget, 1, column)
        grid.addWidget(self.refresh, 1, 5)
        grid.addWidget(self.connect_button, 2, 0, 1, 3)
        grid.addWidget(self.disconnect_button, 2, 3, 1, 3)
        self.hint = QtWidgets.QLabel()
        self.hint.setWordWrap(True)
        grid.addWidget(self.hint, 3, 0, 1, 6)
        layout.addWidget(config)

        actions = QtWidgets.QHBoxLayout()
        self.rpm = QtWidgets.QDoubleSpinBox()
        self.rpm.setRange(1.0, 20.0)
        self.rpm.setDecimals(2)
        self.rpm.setValue(5)
        self.rpm.setSuffix(" mechanical RPM")
        actions.addWidget(self.rpm)
        self.start_button = QtWidgets.QPushButton("측정 시작 (CW → CCW)")
        self.start_button.clicked.connect(self._start_measurement)
        self.abort_button = QtWidgets.QPushButton("측정 중단 / 0 A")
        self.abort_button.clicked.connect(self._abort)
        self.comp_off = QtWidgets.QPushButton("보상 OFF 요청")
        self.comp_off.clicked.connect(lambda: self._enqueue("compensation", False))
        self.comp_on = QtWidgets.QPushButton("보상 ON 요청")
        self.comp_on.clicked.connect(lambda: self._enqueue("compensation", True))
        for widget in (self.start_button, self.abort_button, self.comp_off, self.comp_on):
            actions.addWidget(widget)
        layout.addLayout(actions)
        self.auto_download = QtWidgets.QCheckBox("측정 완료 후 원시 6회차 + 최종 LUT 자동 수집")
        self.auto_download.setChecked(True)
        layout.addWidget(self.auto_download)
        reads = QtWidgets.QHBoxLayout()
        self.status_button = QtWidgets.QPushButton("상태 읽기")
        self.status_button.clicked.connect(lambda: self._enqueue("status"))
        self.download_button = QtWidgets.QPushButton("전체 7개 수집 (10080 bins)")
        self.download_button.clicked.connect(lambda: self._begin_download(False))
        self.final_button = QtWidgets.QPushButton("최종 LUT만 수집")
        self.final_button.clicked.connect(lambda: self._begin_download(True))
        self.cancel_button = QtWidgets.QPushButton("수집 취소")
        self.cancel_button.clicked.connect(self._cancel_download)
        self.save_button = QtWidgets.QPushButton("TXT 파일 저장")
        self.save_button.clicked.connect(self._save)
        for widget in (self.status_button, self.download_button, self.final_button,
                       self.cancel_button, self.save_button):
            reads.addWidget(widget)
        layout.addLayout(reads)
        self.status_label = QtWidgets.QLabel("연결 대기")
        self.status_label.setWordWrap(True)
        layout.addWidget(self.status_label)
        self.progress = QtWidgets.QProgressBar()
        self.progress.setRange(0, 10080)
        self.progress.setValue(0)
        layout.addWidget(self.progress)

        toggles = QtWidgets.QHBoxLayout()
        self.curves, self.checks = {}, {}
        self.plot = pg.PlotWidget(background="#17202b")
        self.plot.setMinimumHeight(240)
        self.plot.setLabel("bottom", "Mechanical angle", units="deg")
        self.plot.setLabel("left", "Current", units="A")
        self.plot.showGrid(x=True, y=True, alpha=0.25)
        self.plot.addLegend()
        colors = ("#69d2e7", "#56b4e9", "#3388cc", "#ffb347", "#ed8b42", "#cc6633", "#d3f26a")
        for (selector, label), color in zip(SERIES_LABELS.items(), colors):
            self.curves[selector] = self.plot.plot(name=label, pen=pg.mkPen(color, width=1.5), connect="finite")
            check = QtWidgets.QCheckBox(label)
            check.setChecked(True)
            check.toggled.connect(lambda visible, key=selector: self.curves[key].setVisible(visible))
            self.checks[selector] = check
            toggles.addWidget(check)
        layout.addLayout(toggles)
        layout.addWidget(self.plot, 1)
        self.summary = QtWidgets.QLabel("CW/CCW 원시값: 기계 방향 기준 iq_ref [A] · Final LUT: 보상 전류 [A]")
        self.summary.setWordWrap(True)
        layout.addWidget(self.summary)
        note = QtWidgets.QLabel(
            "자동 순서: CW 안정화 1 + 기록 3회전 → 정지(≤0.1 RPM) → CCW 안정화 1 + 기록 3회전 → 0 A 정지 / LUT 계산·Flash 자동 저장.\n"
            "속도 PI를 먼저 튜닝하고, 작은 외부 부하와 전체 회전이 가능한 상태에서 시험하세요. 회전 정지/진동 시 측정 중단을 누르세요.\n"
            "원시 6회차는 RAM에만 유지됩니다. 수집 중 리셋·재측정이나 다른 운동 명령을 보내면 처음부터 다시 수집하세요.")
        note.setWordWrap(True)
        layout.addWidget(note)
        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(150)
        self.log.setMaximumHeight(90)
        layout.addWidget(self.log)

    def _load_channel(self):
        interface = self.interface.currentText()
        default = DEFAULT_CHANNEL if interface == DEFAULT_INTERFACE else ""
        self.channel.setEditText(str(self.settings.value(f"{sys.platform}/{interface}/channel", default)))
        self._refresh_channels()
        self.hint.setText("SocketCAN은 OS에서 FD 1M/5M 설정 후 사용하세요. 기존 BRS ON 상태/파라미터도 수신 가능해야 합니다." if interface == "socketcan" else
                          "CANable 2.0 호환 FD SLCAN / 1M/5M. 같은 시리얼 어댑터를 사용하는 다른 GUI는 먼저 닫으세요.")

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

    def _sync_controls(self):
        ready = self.connected and not self._closing
        idle = ready and not self.job
        for widget in (self.interface, self.channel, self.driver_id, self.refresh,
                       self.timeout_ms, self.retries):
            widget.setEnabled(self.worker is None and not self._closing)
        self.connect_button.setEnabled(self.worker is None and not self._closing)
        self.disconnect_button.setEnabled(ready)
        self.abort_button.setEnabled(ready)
        status = self.last_status
        available = status is not None and status.state in (0, 2) and not status.processing
        self.start_button.setEnabled(idle and not self.measuring and (status is None or not status.processing))
        self.rpm.setEnabled(idle and not self.measuring)
        self.auto_download.setEnabled(idle and not self.measuring)
        for widget in (self.comp_off, self.comp_on, self.status_button):
            widget.setEnabled(idle)
        self.download_button.setEnabled(idle and not self.measuring and available
                                        and status.state == 2 and status.has_raw and status.lut_valid)
        self.final_button.setEnabled(idle and not self.measuring and available and status.lut_valid)
        self.cancel_button.setEnabled(ready and self.job in ("download", "final"))
        self.save_button.setEnabled(not self.job and not self._closing
                                    and self.capture is not None and self.save_worker is None)

    def _connect(self):
        if self.worker is not None:
            return
        channel = self.channel.currentText().strip()
        if not channel:
            self.status_label.setText("실제 CAN 채널/시리얼 포트를 입력하세요.")
            return
        interface = self.interface.currentText()
        self.settings.setValue(f"{sys.platform}/{interface}/channel", channel)
        self.capture, self.last_status = None, None
        self._clear_plot()
        self.worker = CoggingSession(interface, channel, self.driver_id.value(),
                                     self.timeout_ms.value() / 1000.0, self.retries.value(), self)
        self.worker.connected_changed.connect(self._connected_changed)
        self.worker.status_received.connect(self._status_received)
        self.worker.activity_changed.connect(self._activity_changed)
        self.worker.busy_changed.connect(self._busy_changed)
        self.worker.progress.connect(self._progress)
        self.worker.capture_received.connect(self._capture_received)
        self.worker.notice.connect(self._notice)
        self.worker.finished.connect(self._finished)
        self.status_label.setText("CAN 연결 중…")
        self._sync_controls()
        self.worker.start()

    def _connected_changed(self, connected):
        self.connected = connected
        self._sync_controls()

    def _activity_changed(self, active):
        self.measuring = active
        self._sync_controls()

    def _busy_changed(self, kind):
        self.job = kind
        self._sync_controls()

    def _status_received(self, status):
        self.last_status = status
        names = ("idle / 원시 데이터 없음", "측정 또는 정지·후처리 대기", "다운로드 가능", "측정 실패")
        flags = []
        for enabled, label in ((status.has_raw, "원시 데이터 있음"), (status.lut_valid, "LUT 유효"),
                               (status.processing, "후처리/저장 진행"), (status.flash_saved, "Flash 저장 성공"),
                               (status.flash_failed, "Flash 저장 실패 — RAM 수집 가능")):
            if enabled:
                flags.append(label)
        self.status_label.setText(f"Driver {self.driver_id.value()}: state {status.state} — {names[status.state]}\n"
                                  f"{status.bin_count} bins / {status.sweeps} sweeps / v{status.version} / flags 0x{status.flags:02X}: " +
                                  (", ".join(flags) if flags else "없음"))
        self._sync_controls()

    def _enqueue(self, kind, argument=None):
        if self.worker is None or not self.connected:
            return
        try:
            self.worker.enqueue(kind, argument)
            self._busy_changed(kind)
        except Exception as exc:
            self._notice(f"요청 실패: {exc}")

    def _start_measurement(self):
        self.capture = None
        self._clear_plot()
        self._enqueue("start", (self.rpm.value(), self.auto_download.isChecked()))

    def _begin_download(self, final_only):
        self.capture = None
        self._clear_plot()
        self._enqueue("final" if final_only else "download")

    def _clear_plot(self):
        for curve in self.curves.values():
            curve.setData([], [])
        self.progress.setValue(0)

    def _abort(self):
        if self.worker is not None:
            self.worker.abort()
            self._notice("측정 중단 요청 대기 — 진행 중인 수집/상태 읽기를 취소합니다.")

    def _cancel_download(self):
        if self.worker is not None:
            self.worker.cancel_event.set()
            self.cancel_button.setEnabled(False)

    def _disconnect(self):
        if self.worker is not None:
            self.worker.close_session()
            self.connected = False
            self._sync_controls()
            self.status_label.setText("측정 중단 요청 및 CAN 연결 해제 중…")

    def _progress(self, done, total):
        self.progress.setRange(0, total)
        self.progress.setValue(done)

    def _capture_received(self, capture):
        self.capture = capture
        self._clear_plot()
        abnormal = 0
        for selector, values in capture.series.items():
            array = np.asarray(values, dtype=float)
            abnormal += int((~np.isfinite(array)).sum())
            self.curves[selector].setData(np.arange(len(values)) * 0.25, array, connect="finite")
        self.plot.enableAutoRange()
        total = sum(map(len, capture.series.values()))
        self.progress.setRange(0, total)
        self.progress.setValue(total)
        self.summary.setText(f"Driver {capture.driver_id}: {len(capture.series)}개 / {total} bins 수집 완료. "
                             f"NaN/Inf {abnormal}개 보존. 원시 회차=기계 방향 iq_ref [A], 최종 LUT=보상 전류 [A].")
        self._sync_controls()

    def _save(self):
        if self.capture is None:
            return
        parent = QtWidgets.QFileDialog.getExistingDirectory(self, "저장할 상위 폴더", str(Path.home()))
        if not parent:
            return
        directory = Path(parent) / (f"cogging_driver{self.capture.driver_id}_" +
                                     datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
        self._start_save(directory)

    def _start_save(self, directory):
        if self.capture is None or self.save_worker is not None:
            return
        self.save_worker = CoggingSaveWorker(directory, self.capture, self)
        self.save_worker.finished.connect(self._save_finished)
        self._sync_controls()
        self.save_worker.start()

    def _save_finished(self):
        worker, self.save_worker = self.save_worker, None
        self._notice(f"저장 실패: {worker.error}" if worker.error else
                     f"저장 완료: {worker.directory} ({len(worker.capture.series)}개 파일)")
        worker.deleteLater()
        self._sync_controls()
        if self._closing and self.worker is None:
            QtCore.QTimer.singleShot(0, self.close)

    def _notice(self, message):
        self.log.appendPlainText(message)
        if "실패" in message or "취소" in message:
            self.summary.setText(message)

    def _finished(self):
        worker, self.worker = self.worker, None
        self.connected, self.measuring, self.job = False, False, ""
        self.status_label.setText("CAN 연결 해제됨" + (f": {worker.error}" if worker.error else ""))
        worker.deleteLater()
        self._sync_controls()
        if self._closing and self.save_worker is None:
            QtCore.QTimer.singleShot(0, self.close)

    def closeEvent(self, event):
        if self.worker is not None or self.save_worker is not None:
            self._closing = True
            self._disconnect()
            self._sync_controls()
            event.ignore()
        else:
            event.accept()


def main():
    app = QtWidgets.QApplication(sys.argv)
    window = CoggingWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
