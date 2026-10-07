"""Platform settings and asynchronous capture/connection cleanup."""
import socket
import sys
from datetime import datetime
from pathlib import Path

from PyQt5 import QtCore, QtWidgets
from serial.tools import list_ports

from can_workers import CaptureWriter


class ConnectionHandlersMixin:
    def _connection_settings(self):
        return QtCore.QSettings("MA732", "CANMonitor")

    def _on_interface_changed(self):
        iface = self.iface_combo.currentText()
        settings = self._connection_settings()
        key = f"connection/{sys.platform}/{iface}"
        default = "can0" if iface == "socketcan" else ("0" if iface == "gs_usb" else "")
        self.channel_edit.setText(str(settings.value(key + "/channel", default)))
        self.bitrate_edit.setText(str(settings.value(key + "/bitrate", 1000000)))
        self.bitrate_edit.setEnabled(iface != "socketcan")
        if iface == "socketcan":
            self.connection_hint.setText("Linux에서 ip link/slcand로 bitrate와 인터페이스를 먼저 설정하세요.")
        elif iface == "slcan":
            self.connection_hint.clear()
        else:
            self.connection_hint.setText("candleLight/gs_usb 장치 인덱스 (보통 0). gs-usb/pyusb가 필요합니다.")
        self.connection_hint.setVisible(bool(self.connection_hint.text()))
        self._refresh_channels()

    def _refresh_channels(self):
        selected = self.channel_edit.text()
        iface = self.iface_combo.currentText()
        try:
            if iface == "slcan":
                channels = [port.device for port in list_ports.comports()]
            elif iface == "socketcan" and sys.platform.startswith("linux"):
                channels = [name for _, name in socket.if_nameindex()
                            if name.startswith(("can", "vcan", "slcan"))]
            elif iface == "gs_usb":
                channels = ["0"]
            else:
                channels = []
            self.channel_combo.clear()
            self.channel_combo.addItems(sorted(channels))
            self.channel_edit.setText(selected or (channels[0] if channels else ""))
        except Exception as exc:
            self._log(f"Channel list: {exc}")

    def _save_connection_settings(self):
        settings = self._connection_settings()
        iface = self.iface_combo.currentText()
        key = f"connection/{sys.platform}/{iface}"
        settings.setValue(key + "/channel", self.channel_edit.text().strip())
        settings.setValue(key + "/bitrate", self.bitrate_edit.text().strip())

    def _set_connection_controls(self, connected):
        self.btn_connect.setEnabled(not connected)
        self.btn_disconnect.setEnabled(connected)
        self.iface_combo.setEnabled(not connected)
        self.channel_combo.setEnabled(not connected)
        self.btn_refresh_channels.setEnabled(not connected)
        self.bitrate_edit.setEnabled(not connected and self.iface_combo.currentText() != "socketcan")
        self.btn_record.setEnabled(connected and self.capture is None)

    def _on_record_toggle(self):
        if self.capture is not None:
            self.capture.stop()
            self.btn_record.setEnabled(False)
            self.btn_record.setText("Saving CSV...")
            return
        if self.reader_thread is None:
            return
        default = str(Path.home() / ("can_rx_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".csv"))
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Record raw RX (all IDs)", default, "CSV (*.csv)")
        if not path:
            return
        if Path(path).exists():
            self._log("CSV 파일이 이미 있습니다. 새 파일명을 선택하세요.")
            return
        self.capture = CaptureWriter(path)
        self.reader_thread.recorder = self.capture
        self.capture.start()
        self.btn_record.setText("Stop Recording")
        self._log(f"RX recording: {path}")

    def _poll_capture(self):
        if self.capture is None:
            return
        written, dropped, error = self.capture.snapshot()
        self.lbl_record.setText(f"CSV: {written} frames / dropped {dropped}")
        if self.capture.done.is_set():
            if self.reader_thread is not None:
                self.reader_thread.recorder = None
            self._log(f"CSV finished: {written} frames, dropped {dropped}" + (f", ERROR: {error}" if error else ""))
            self.capture = None
            self.btn_record.setText("Record RX CSV")
            self.btn_record.setEnabled(self.manager.bus is not None and self._disconnect_thread is None)

    def _poll_background(self):
        self._poll_capture()
        worker = self._disconnect_thread
        if worker is not None and not worker.is_alive():
            self._disconnect_thread = None
            self.reader_thread = None
            self._set_connection_controls(False)
            self._reset_monitor()
            self._log("Disconnected" + (f": {self._disconnect_error}" if self._disconnect_error else ""))
        if self._closing and self.manager.bus is None and self._disconnect_thread is None and self.capture is None:
            QtCore.QTimer.singleShot(0, self.close)

    def _reset_monitor(self):
        for buffer in (self.t_buf, self.pos_buf, self.spd_buf, self.cur_buf, self.tmp_buf):
            buffer.clear()
        for curve in (self.curve_pos, self.curve_spd, self.curve_cur, self.curve_tmp):
            curve.setData([], [])
        self.last_error_code = None
        self._last_sample_ts = 0.0
        self.lbl_error.setText("Error: waiting for selected ID")
