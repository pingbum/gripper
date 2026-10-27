# -*- coding: utf-8 -*-
"""Standalone encoder LUT inspector. Run: python tools/LUT.py

Only read modes 0xC0/0xC1 are sent, always Extended CAN FD / BRS OFF.
Connection, collection and shutdown run outside the GUI thread. The plot is
updated once per completed capture, not once per CAN frame.
"""
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
from lut_reader import LUTCancelled, LUTCapture, LUTReader, write_lut_csv


class LUTReadWorker(QtCore.QThread):
    metadata_received = QtCore.pyqtSignal(object)
    progress = QtCore.pyqtSignal(int, int)
    notice = QtCore.pyqtSignal(str)

    def __init__(self, interface, channel, driver_id, timeout, retries,
                 metadata_only=False, parent=None):
        super().__init__(parent)
        self.interface = interface
        self.channel = channel
        self.driver_id = driver_id
        self.timeout = timeout
        self.retries = retries
        self.metadata_only = metadata_only
        self.stop_event = threading.Event()
        self.result = None
        self.error = None
        self.cancelled = False
        self._last_progress = 0.0

    def cancel(self):
        self.stop_event.set()

    def _progress(self, done, total):
        now = time.monotonic()
        if done == total or now - self._last_progress >= 0.1:
            self._last_progress = now
            self.progress.emit(done, total)

    def run(self):
        bus = None
        try:
            if self.stop_event.is_set():
                raise LUTCancelled()
            options = connection_options(self.interface, self.channel, DEFAULT_BITRATE)
            bus = can.Bus(**options)
            reader = LUTReader(bus, self.driver_id, self.timeout, self.retries,
                               self.stop_event, self._retry)
            if self.metadata_only:
                self.result = reader.read_metadata()
                self.metadata_received.emit(self.result)
            else:
                self.result = reader.read_all(self.metadata_received.emit, self._progress)
        except LUTCancelled:
            self.cancelled = True
        except Exception as exc:
            self.error = str(exc)
        finally:
            if bus is not None:
                try:
                    bus.shutdown()
                except Exception as exc:
                    if self.error is None:
                        self.error = f"CAN shutdown failed: {exc}"
            if self.stop_event.is_set():
                self.cancelled = True
            if self.cancelled or self.error is not None:
                self.result = None

    def _retry(self, label, retry, maximum):
        self.notice.emit(f"{label}: 타임아웃, 재요청 {retry}/{maximum}")


class LUTWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("MA732 — Encoder LUT Inspector")
        self.resize(1040, 850)
        self.worker = None
        self.capture = None
        self._closing = False
        self._started = 0.0
        self.settings = QtCore.QSettings("MA732", "LUTInspector")
        self._build_ui()
        self.interface.setCurrentText(DEFAULT_INTERFACE)
        self.driver_id.setValue(self.settings.value("driver_id", 0, type=int))
        self.interface.currentTextChanged.connect(self._load_channel)
        self._load_channel()

    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)
        group = QtWidgets.QGroupBox("CAN 연결 / 읽기 설정")
        form = QtWidgets.QGridLayout(group)
        self.interface = QtWidgets.QComboBox()
        self.interface.addItems(["socketcan", "slcan"])
        self.channel = QtWidgets.QComboBox()
        self.channel.setEditable(True)
        self.refresh = QtWidgets.QPushButton("포트 새로고침")
        self.refresh.clicked.connect(self._refresh_channels)
        self.driver_id = QtWidgets.QSpinBox()
        self.driver_id.setRange(0, 255)
        self.timeout_ms = QtWidgets.QSpinBox()
        self.timeout_ms.setRange(10, 5000)
        self.timeout_ms.setValue(100)
        self.timeout_ms.setSuffix(" ms")
        self.retries = QtWidgets.QSpinBox()
        self.retries.setRange(0, 10)
        self.retries.setValue(3)
        for column, (label, widget) in enumerate([
            ("Interface", self.interface), ("Channel", self.channel),
            ("Driver ID", self.driver_id), ("응답 타임아웃", self.timeout_ms),
            ("추가 재요청 횟수", self.retries),
        ]):
            form.addWidget(QtWidgets.QLabel(label), 0, column)
            form.addWidget(widget, 1, column)
        form.addWidget(self.refresh, 1, 5)
        self.connection_hint = QtWidgets.QLabel()
        self.connection_hint.setWordWrap(True)
        form.addWidget(self.connection_hint, 2, 0, 1, 6)
        layout.addWidget(group)

        actions = QtWidgets.QHBoxLayout()
        self.read_metadata = QtWidgets.QPushButton("메타데이터 읽기")
        self.read_metadata.clicked.connect(lambda: self._start_read(metadata_only=True))
        self.read_full = QtWidgets.QPushButton("전체 LUT 수집")
        self.read_full.clicked.connect(lambda: self._start_read(metadata_only=False))
        self.cancel = QtWidgets.QPushButton("수집 취소")
        self.cancel.clicked.connect(self._cancel_read)
        self.cancel.setEnabled(False)
        self.save = QtWidgets.QPushButton("CSV 저장")
        self.save.clicked.connect(self._save_csv)
        self.save.setEnabled(False)
        for widget in (self.read_metadata, self.read_full, self.cancel, self.save):
            actions.addWidget(widget)
        layout.addLayout(actions)
        self.progress = QtWidgets.QProgressBar()
        self.progress.setRange(0, 1)
        self.progress.setValue(0)
        layout.addWidget(self.progress)
        self.status = QtWidgets.QLabel("대기 중 — 읽기 버튼을 누르면 연결하여 수집 후 자동 해제합니다.")
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        self.metadata_label = QtWidgets.QLabel("Samples: —    direction: —")
        layout.addWidget(self.metadata_label)

        self.plot = pg.PlotWidget(background="#17202b")
        self.plot.setMinimumHeight(260)
        self.plot.setLabel("bottom", "Mechanical angle", units="deg")
        self.plot.setLabel("left", "Electrical offset", units="deg")
        self.plot.showGrid(x=True, y=True, alpha=0.25)
        self.curve = self.plot.plot(pen=pg.mkPen("#62d7d0", width=1.5), connect="finite")
        layout.addWidget(self.plot, 1)
        self.point_label = QtWidgets.QLabel("그래프 위에 마우스를 올리면 해당 인덱스의 원래 값을 표시합니다.")
        self.mouse_proxy = pg.SignalProxy(self.plot.scene().sigMouseMoved,
                                         rateLimit=20, slot=self._show_point)
        layout.addWidget(self.point_label)
        self.stats = QtWidgets.QLabel("LUT 수집 완료 후 통계가 표시됩니다.")
        self.stats.setWordWrap(True)
        layout.addWidget(self.stats)
        note = QtWidgets.QLabel(
            "현재 FOC에 적용된 필터링 후 LUT입니다. 샘플 수는 보정 성공 여부를 뜻하지 않습니다. "
            "보정 로딩 실패 시 전체 0일 수 있습니다. NaN/Inf도 CSV에 그대로 보존합니다.\n"
            "수집 중 리셋/재캘리브레이션을 했다면 취소하고 처음부터 다시 수집하세요.")
        note.setWordWrap(True)
        layout.addWidget(note)
        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(100)
        self.log.setMaximumHeight(100)
        layout.addWidget(self.log)

    def _load_channel(self):
        interface = self.interface.currentText()
        default = DEFAULT_CHANNEL if interface == DEFAULT_INTERFACE else ""
        value = str(self.settings.value(f"{sys.platform}/{interface}/channel", default))
        self.channel.setEditText(value)
        self._refresh_channels()
        if interface == "socketcan":
            hint = "Linux SocketCAN: OS에서 FD 1M/5M 설정 후 사용합니다. 프로그램은 CAN 설정을 변경하지 않습니다."
        else:
            hint = "CANable 2.0 호환 FD SLCAN 펌웨어 / 1M/5M. 시리얼 어댑터를 사용하는 app.py를 먼저 닫으세요."
        self.connection_hint.setText(hint + " 요청/응답: Extended FD, BRS OFF.")

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

    def _set_busy(self, busy):
        for widget in (self.interface, self.channel, self.refresh, self.driver_id,
                       self.timeout_ms, self.retries, self.read_metadata, self.read_full):
            widget.setEnabled(not busy)
        self.cancel.setEnabled(busy)
        self.save.setEnabled(not busy and self.capture is not None)

    def _start_read(self, metadata_only=False):
        if self.worker is not None:
            return
        channel = self.channel.currentText().strip()
        if not channel:
            self.status.setText("Channel에 can0 또는 실제 시리얼 포트를 입력하세요.")
            return
        self.capture = None
        self.curve.setData([], [])
        self.metadata_label.setText("Samples: —    direction: —")
        self.stats.setText("메타데이터를 읽는 중입니다.")
        self.point_label.setText("완료된 LUT가 없습니다.")
        self.progress.setRange(0, 0)
        self._started = time.monotonic()
        interface = self.interface.currentText()
        self.settings.setValue(f"{sys.platform}/{interface}/channel", channel)
        self.settings.setValue("driver_id", self.driver_id.value())
        self.worker = LUTReadWorker(interface, channel, self.driver_id.value(),
                                    self.timeout_ms.value() / 1000.0, self.retries.value(),
                                    metadata_only, self)
        self.worker.metadata_received.connect(self._on_metadata)
        self.worker.progress.connect(self._on_progress)
        self.worker.notice.connect(self.log.appendPlainText)
        self.worker.finished.connect(self._read_finished)
        self._set_busy(True)
        self.status.setText(f"{interface} / {channel} / Driver ID {self.driver_id.value()}: 연결 및 읽기 중…")
        self.log.appendPlainText(self.status.text())
        self.worker.start()

    def _on_metadata(self, metadata):
        self.metadata_label.setText(f"Samples: {metadata.sample_count}    direction: {metadata.direction:+g}")
        self.progress.setRange(0, metadata.sample_count)
        self.progress.setValue(0)

    def _on_progress(self, done, total):
        self.progress.setValue(done)
        self.status.setText(f"LUT 수집 중: {done}/{total} ({time.monotonic() - self._started:.1f} s)")

    def _cancel_read(self):
        if self.worker is not None:
            self.worker.cancel()
            self.cancel.setEnabled(False)
            self.status.setText("취소 및 CAN 연결 해제 중…")

    def _read_finished(self):
        worker = self.worker
        self.worker = None
        elapsed = time.monotonic() - self._started
        if worker.cancelled or worker.error is not None:
            self.progress.setRange(0, 1)
            self.progress.setValue(0)
            self.status.setText("수집 취소 — 처음부터 다시 수집하세요." if worker.cancelled else
                                f"수집 실패: {worker.error}")
            self.stats.setText("완료된 LUT가 없습니다. 부분 데이터는 CSV로 저장하지 않습니다.")
        elif isinstance(worker.result, LUTCapture):
            self.capture = worker.result
            self._display_capture()
            self.progress.setValue(self.capture.metadata.sample_count)
            self.status.setText(f"Driver ID {self.capture.driver_id}: 전체 LUT 수집 완료 ({elapsed:.2f} s). CAN 연결 해제됨.")
        else:
            self.progress.setValue(worker.result.sample_count)
            self.status.setText(f"메타데이터 읽기 완료 ({elapsed:.2f} s). CAN 연결 해제됨.")
            self.stats.setText("전체 LUT 수집 버튼을 누르면 그래프와 CSV 저장을 사용할 수 있습니다.")
        self.log.appendPlainText(self.status.text())
        self._set_busy(False)
        worker.deleteLater()
        if self._closing:
            QtCore.QTimer.singleShot(0, self.close)

    def _display_capture(self):
        values = np.asarray(self.capture.offsets, dtype=float)
        count = self.capture.metadata.sample_count
        angles = np.arange(count, dtype=float) * (360.0 / count)
        self.curve.setData(angles, values, connect="finite")
        self.plot.enableAutoRange()
        finite = values[np.isfinite(values)]
        summary = (f"유한값 {finite.size}/{count} | NaN {np.isnan(values).sum()} | "
                   f"+Inf {np.isposinf(values).sum()} | -Inf {np.isneginf(values).sum()}")
        if finite.size:
            summary += (f"\n유한값 기준: min {finite.min():.6g}, max {finite.max():.6g}, "
                        f"mean {finite.mean():.6g}, peak-to-peak {np.ptp(finite):.6g} deg (전기각)")
        if np.all(values == 0):
            summary += "\n전체 값이 0입니다. 메타데이터만으로 보정 성공/로딩 실패를 판단할 수 없습니다."
        self.stats.setText(summary)
        self.point_label.setText("그래프 위에 마우스를 올리면 인덱스 / 기계각 / 전기각 보정값을 표시합니다.")

    def _show_point(self, event):
        if self.capture is None or not self.plot.sceneBoundingRect().contains(event[0]):
            return
        angle = self.plot.getViewBox().mapSceneToView(event[0]).x()
        count = self.capture.metadata.sample_count
        index = min(count - 1, max(0, round(angle * count / 360.0)))
        self.point_label.setText(f"index {index} | mechanical {index * 360.0 / count:.6f} deg | "
                                 f"offset {self.capture.offsets[index]:.9g} deg (전기각)")

    def _save_csv(self):
        if self.capture is None:
            return
        default = Path.home() / (f"lut_driver{self.capture.driver_id}_" +
                                 datetime.now().strftime("%Y%m%d_%H%M%S") + ".csv")
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "LUT CSV 저장", str(default), "CSV (*.csv)")
        if not path:
            return
        if not path.lower().endswith(".csv"):
            path += ".csv"
        try:
            write_lut_csv(path, self.capture)
        except Exception as exc:
            self.status.setText(f"CSV 저장 실패 (기존 파일은 덮어쓰지 않습니다): {exc}")
        else:
            self.status.setText(f"CSV 저장 완료: {path} ({len(self.capture.offsets)} samples)")
        self.log.appendPlainText(self.status.text())

    def closeEvent(self, event):
        if self.worker is not None:
            self._closing = True
            self._cancel_read()
            event.ignore()
        else:
            event.accept()


def main():
    app = QtWidgets.QApplication(sys.argv)
    window = LUTWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
