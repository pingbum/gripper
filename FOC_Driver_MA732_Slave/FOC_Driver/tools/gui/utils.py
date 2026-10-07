# -*- coding: utf-8 -*-
import struct
import time

from .deps import format_error_code, build_ext_id


class UtilsMixin:
    def _log(self, msg: str):
        self.log.appendPlainText(msg)
        self.log.verticalScrollBar().setValue(self.log.verticalScrollBar().maximum())

    def _update_motor_list(self):
        if not hasattr(self, "combo_motor"):
            return
        try:
            with self.scan_lock:
                ids = sorted(self.scan_hits)
            existing = {self.combo_motor.itemData(i) for i in range(self.combo_motor.count())}
            for motor_id in ids:
                if motor_id in existing:
                    continue
                self.combo_motor.addItem(f"Motor {motor_id}", motor_id)
            if self.combo_motor.count() > 1 and self.combo_motor.currentIndex() == 0:
                self.combo_motor.setCurrentIndex(1)
            if hasattr(self, "_scan_in_progress") and self._scan_in_progress:
                self.combo_motor.setEditable(True)
                self.combo_motor.setEditText("Scanning...")
                line_edit = self.combo_motor.lineEdit()
                if line_edit is not None:
                    line_edit.setReadOnly(True)
            else:
                self.combo_motor.setEditable(False)
        except Exception as e:
            self._log(f"Motor list update error: {e}")

    def _update_rate_label(self, force: bool = False):
        now = time.monotonic()
        dt = now - self._last_rate_ts
        if not force and dt < 1.0:
            return
        if dt <= 0:
            return
        if self.reader_thread and hasattr(self.reader_thread, "get_and_reset_rx_count"):
            rx_count = self.reader_thread.get_and_reset_rx_count()
        else:
            rx_count = self._rx_count
            self._rx_count = 0
        rx_hz = rx_count / dt
        tx_count, skipped, errors = self.manager.writer.take_stats() if self.manager.writer else (0, 0, [])
        tx_hz = tx_count / dt
        paused = self.manager.writer is not None and self.manager.writer.pause_event.is_set()
        count = self.reader_thread.warning_count if self.reader_thread else 0
        if paused:
            self.lbl_tx_health.setText(f"TX paused / RX continues / warnings: {count}")
        elif self.manager.writer is not None:
            self.lbl_tx_health.setText(f"TX ready / CAN warnings: {count} / skipped: {skipped}")
        else:
            self.lbl_tx_health.setText(f"TX skipped: {skipped} (last {dt:.1f}s)")
        self.btn_resume_tx.setEnabled(
            paused and self.manager.interface == "socketcan"
            and self._disconnect_thread is None and self._tx_resume_thread is None)
        if errors:
            self._log(f"TX stopped: {errors[-1]} ({len(errors)} errors)")
            if self.manager.writer.stop_event.is_set():
                self._on_disconnect()
        self.lbl_rate.setText(f"Rate: RX {rx_hz:.1f} Hz / TX {tx_hz:.1f} Hz")
        self._last_rate_ts = now

    def _set_error_code(self, error_code: int):
        code = int(error_code) & 0xFF
        if self.last_error_code == code:
            return
        first_update = self.last_error_code is None
        self.last_error_code = code
        text = format_error_code(code)
        self.lbl_error.setText(f"Error: {text}")
        if not first_update:
            if code == 0:
                self._log("Error cleared")
            else:
                self._log(f"Error update: {text}")

    def _pack_value_32(self, value_str: str, value_type: str) -> bytes:
        if value_type == "float32":
            v = float(value_str)
            return struct.pack(">f", v)
        if value_type == "uint32":
            v = int(value_str, 0)
            if v < 0 or v > 0xFFFFFFFF:
                raise ValueError("uint32 out of range")
            return struct.pack(">I", v)
        v = int(value_str, 0)
        if v < -0x80000000 or v > 0x7FFFFFFF:
            raise ValueError("int32 out of range")
        return struct.pack(">i", v)

    def _decode_read_value(self, data: bytes, value_type: str, func_id: int) -> str:
        if func_id == 0x07:
            if not data:
                return "N/A"
            return f"{data[0]}"
        if len(data) < 4:
            return "N/A"
        data4 = data[:4]
        if value_type == "float32":
            v = struct.unpack(">f", data4)[0]
            return f"{v:.6f}"
        if value_type == "uint32":
            v = struct.unpack(">I", data4)[0]
            return f"{v}"
        v = struct.unpack(">i", data4)[0]
        return f"{v}"

    def _on_raw_send(self):
        if self.manager.bus is None:
            self._log("버스 미연결")
            return
        motor_id = self.spin_raw_motor.value()
        func_id = self.spin_raw_func.value()
        value_str = self.edit_raw_value.text().strip()
        value_type = self.combo_raw_type.currentText()
        try:
            payload = self._pack_value_32(value_str, value_type)
            eid = build_ext_id(func_id, motor_id)
            self.manager.send_ext(eid, payload)
            self._log(f"Raw Send queued: EID=0x{eid:08X}, type={value_type}, value={value_str}")
        except Exception as e:
            self._log(f"Raw Send 실패: {e}")

    def _on_read_request(self):
        if self.manager.bus is None:
            self._log("버스 미연결")
            return
        if self.read_pending:
            self._log("읽기 진행중")
            return
        motor_id = self.spin_read_motor.value()
        func_id = self.spin_read_func.value()
        self.read_type = self.combo_read_type.currentText()
        base_func_id = func_id & 0x7F
        req_func_id = base_func_id | 0x80
        resp_func_id = req_func_id
        resp_eid = build_ext_id(resp_func_id, motor_id)
        req_eid = build_ext_id(req_func_id, motor_id)
        with self.read_lock:
            self.read_watch["eid"] = resp_eid
            self.read_watch["data"] = None
            self.read_watch["ts"] = None
            self.read_watch["func_id"] = base_func_id
            self.read_watch["tag"] = "manual"
        self.read_pending = True
        self.read_deadline = time.monotonic() + 0.5
        self.lbl_read_value.setText("Read: (waiting)")
        try:
            self.manager.send_ext(req_eid, b"\x00" * 8)
            self._log(f"Read 요청: EID=0x{req_eid:08X}, type={self.read_type}")
        except Exception as e:
            self._log(f"Read 실패: {e}")

    def _start_param_read(self):
        if self.manager.bus is None:
            return
        if self.read_pending:
            return
        self.param_read_queue = [(0x11, 2), (0x20, 2), (0x21, 2)]
        self._request_next_param_read()

    def _request_next_param_read(self):
        if not self.param_read_queue or self.read_pending:
            return
        motor_id = self.spin_driver.value()
        base_func_id, retries = self.param_read_queue.pop(0)
        req_func_id = base_func_id | 0x80
        resp_eid = build_ext_id(req_func_id, motor_id)
        with self.read_lock:
            self.read_watch["eid"] = resp_eid
            self.read_watch["data"] = None
            self.read_watch["ts"] = None
            self.read_watch["func_id"] = base_func_id
            self.read_watch["tag"] = "param"
        self.read_retry_left = retries
        self.read_param_current = base_func_id
        self.read_pending = True
        self.read_deadline = time.monotonic() + 0.5
        try:
            self.manager.send_ext(build_ext_id(req_func_id, motor_id), b"\x00" * 8)
        except Exception as exc:
            self.read_pending = False
            self._log(f"Read Params 요청 실패: {exc}")

    def _apply_param_read(self, func_id: int, data: bytes):
        if len(data) < 4:
            return
        value = struct.unpack(">f", data[:4])[0]
        if func_id == 0x11:
            self.spin_ctrl_bw.setValue(value)
        elif func_id == 0x20:
            self.spin_ctrl_kp.setValue(value)
        elif func_id == 0x21:
            self.spin_ctrl_ki.setValue(value)

    def _poll_read_response(self):
        if not self.read_pending:
            return
        with self.read_lock:
            data = self.read_watch.get("data")
            func_id = self.read_watch.get("func_id") or 0
            tag = self.read_watch.get("tag")
        if data:
            text = self._decode_read_value(data, self.read_type, int(func_id))
            if tag == "manual":
                self.lbl_read_value.setText(f"Read: {text}")
            elif tag == "param":
                self._apply_param_read(int(func_id), data)
            self.read_pending = False
            with self.read_lock:
                self.read_watch["eid"] = None
                self.read_watch["data"] = None
                self.read_watch["ts"] = None
                self.read_watch["func_id"] = None
                self.read_watch["tag"] = None
            if tag == "param":
                self._request_next_param_read()
            return
        if time.monotonic() > self.read_deadline:
            if tag == "manual":
                self.lbl_read_value.setText("Read: timeout")
            self.read_pending = False
            with self.read_lock:
                self.read_watch["eid"] = None
                self.read_watch["data"] = None
                self.read_watch["ts"] = None
                self.read_watch["func_id"] = None
                self.read_watch["tag"] = None
            if tag == "param":
                if self.read_retry_left > 0 and self.read_param_current is not None:
                    self.param_read_queue.insert(0, (self.read_param_current, self.read_retry_left - 1))
                self._request_next_param_read()

    def _on_mode_changed(self):
        mode_id = self.combo_mode.currentData()
        self._set_ref_axis_target(mode_id)
        self._reset_ref_amp_for_mode(mode_id)

    def _on_tx_value_changed(self):
        mode_id = self.combo_mode.currentData()
        self._reset_ref_amp_for_mode(mode_id)

    def _on_motor_select(self):
        data = self.combo_motor.currentData()
        if data is None:
            return
        try:
            motor_id = int(data)
        except Exception:
            return
        self.spin_driver.setValue(motor_id)
        self.spin_listen.setValue(motor_id)
        self.spin_raw_motor.setValue(motor_id)
        if hasattr(self, "spin_read_motor"):
            self.spin_read_motor.setValue(motor_id)

    def _on_scan_click(self):
        if self.manager.bus is None:
            self._log("버스 미연결")
            return
        if hasattr(self, "combo_motor"):
            self.combo_motor.clear()
            self.combo_motor.addItem("Select Motor")
        if hasattr(self, "scan_hits"):
            with self.scan_lock:
                self.scan_hits.clear()
        if hasattr(self, "_start_scan"):
            self._start_scan()
        if hasattr(self, "scan_timer"):
            self.scan_timer.start(200)

    def _on_ctrl_send(self):
        if self.manager.bus is None:
            self._log("버스 미연결")
            return
        motor_id = self.spin_driver.value()
        try:
            bw = float(self.spin_ctrl_bw.value())
            kp = float(self.spin_ctrl_kp.value())
            ki = float(self.spin_ctrl_ki.value())
        except Exception:
            self._log("Control Params 값 오류")
            return
        try:
            payload_bw = struct.pack(">f", bw)
            payload_kp = struct.pack(">f", kp)
            payload_ki = struct.pack(">f", ki)
            self.manager.send_ext(build_ext_id(0x11, motor_id), payload_bw)
            self.manager.send_ext(build_ext_id(0x20, motor_id), payload_kp)
            self.manager.send_ext(build_ext_id(0x21, motor_id), payload_ki)
            self._log(f"Ctrl Params queued: BW={bw}, KP={kp}, KI={ki}")
        except Exception as e:
            self._log(f"Ctrl Params 실패: {e}")

    def _on_flash_update(self):
        if self.manager.bus is None:
            self._log("버스 미연결")
            return
        motor_id = self.spin_driver.value()
        try:
            self.manager.send_ext(build_ext_id(0x10, motor_id), b"\x00" * 8)
            self._log("Flash update queued")
        except Exception as e:
            self._log(f"Flash update 실패: {e}")
