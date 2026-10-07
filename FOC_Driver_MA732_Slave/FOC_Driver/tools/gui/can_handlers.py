# -*- coding: utf-8 -*-
import time
import threading
from PyQt5 import QtCore

from .deps import (
    MODE_CURRENT, MODE_VELOCITY, MODE_CALIBRATION, MODE_COGGING_COMPENSATION,
    MODE_COGGING_TOGGLE,
    PLOT_MAX_HZ
)
from .deps import (
    build_ext_id, pack_command_payload, parse_broadcast_frame, pack_calibration_payload
)
from .deps import CANReaderThread
from can_workers import ReferenceCommand


class CANHandlersMixin:
    def _start_scan(self):
        if self.scan_thread and self.scan_thread.is_alive():
            return
        self.scan_stop.clear()
        self._scan_in_progress = True
        self._update_motor_list()

        def worker():
            for motor_id in range(256):
                if self.scan_stop.is_set():
                    break
                try:
                    eid = build_ext_id(0x87, motor_id)
                    self.manager.send_ext(eid, b"\x00" * 8)
                except Exception:
                    break
                self.scan_stop.wait(0.003)
            self._scan_in_progress = False

        self.scan_thread = threading.Thread(target=worker, daemon=True)
        self.scan_thread.start()
    # ------------- 연결/해제 -------------
    def _on_connect(self):
        if self.manager.bus is not None or self._disconnect_thread is not None:
            return
        iface = self.iface_combo.currentText().strip()
        channel = self.channel_edit.text().strip()
        try:
            bitrate = int(self.bitrate_edit.text().strip())
            if bitrate <= 0:
                raise ValueError()
        except ValueError:
            self._log("CAN bitrate는 양의 정수여야 합니다.")
            return

        try:
            self.manager.connect(iface, channel, bitrate)
            self._save_connection_settings()
            self._reset_monitor()
            self._last_rate_ts = time.monotonic()
            with self.scan_lock:
                self.scan_hits.clear()
            self.combo_motor.clear()
            self.combo_motor.addItem("Select Motor")
            # 수신 스레드
            self.reader_stop.clear()
            self.reader_thread = CANReaderThread(
                self.manager.bus,
                self.reader_stop,
                self.scan_hits,
                self.scan_lock,
                0x87,
                self.read_watch,
                self.read_lock,
            )
            self.reader_thread.start()

            self._set_connection_controls(True)
            setting = "bitrate=OS-managed" if iface == "socketcan" else f"bitrate={bitrate}"
            self._log(f"Connected: iface={iface}, channel={channel}, {setting}")
            if hasattr(self, "scan_timer"):
                self._start_scan()
                self.scan_timer.start(200)
            if hasattr(self, "_start_param_read"):
                QtCore.QTimer.singleShot(300, self._start_param_read)
        except Exception as e:
            self._log(f"Connect 실패: {e}")
            if self.manager.bus is not None:
                self._on_disconnect()

    def _on_disconnect(self):
        if self._disconnect_thread is not None:
            return
        self._stop_reference()
        self.scan_stop.set()
        self.scan_timer.stop()
        self.reader_stop.set()
        if self.manager.writer:
            self.manager.writer.stop()
        if self.capture:
            self.capture.stop()
        self.read_pending = False
        self.param_read_queue.clear()
        with self.read_lock:
            for key in self.read_watch:
                self.read_watch[key] = None
        if self.manager.bus is None:
            return
        self.btn_connect.setEnabled(False)
        self.btn_disconnect.setEnabled(False)
        self.btn_record.setEnabled(False)
        self._disconnect_error = None
        reader, scan = self.reader_thread, self.scan_thread

        def cleanup():
            try:
                if scan:
                    scan.join(timeout=1.0)
                if reader:
                    reader.join(timeout=1.0)
                self.manager.disconnect()
            except Exception as exc:
                self._disconnect_error = str(exc)

        self._disconnect_thread = threading.Thread(target=cleanup, name="can-disconnect", daemon=True)
        self._disconnect_thread.start()
        self.lbl_rate.setText("Disconnected / stopping workers...")

    # ------------- 쓰기 -------------
    def _on_write(self):
        if self.manager.bus is None:
            self._log("버스 미연결")
            return
        driver_id = self.spin_driver.value()
        mode_id = self.combo_mode.currentData()
        try:
            value = float(self.edit_value.text().strip())
        except ValueError:
            self._log("Value 숫자 아님")
            return

        try:
            eid = build_ext_id(mode_id, driver_id)
            payload = pack_command_payload(mode_id, value)
            self.manager.send_ext(eid, payload)
            self._log(f"Write queued: EID=0x{eid:08X}, mode={mode_id}, driver={driver_id}, value={value}")
        except Exception as e:
            self._log(f"Write 실패: {e}")

    # ------------- 수신/그래프 -------------
    def _drain_and_update(self):
        self._poll_background()
        if self._disconnect_thread is not None:
            return
        if self.reader_thread and self.reader_thread.last_error:
            self._log(f"RX error: {self.reader_thread.last_error}")
            self._on_disconnect()
            return
        self._collect_reference()
        listen_id = self.spin_listen.value()
        if listen_id != self._last_listen_id:
            self._reset_monitor()
            self._last_listen_id = listen_id
        any_parsed = False
        did_plot_update = False
        last_parsed = None
        last_ts = None
        last_error = None
        sample = self.reader_thread.take_latest_status(listen_id) if self.reader_thread else None
        if sample is not None:
            ts, data = sample
            try:
                last_parsed = parse_broadcast_frame(data)
            except Exception:
                pass
            else:
                last_ts = ts
                last_error = last_parsed["error"]

        self._update_rate_label()
        if self._disconnect_thread is not None:
            return
        self._poll_read_response()

        if last_parsed is not None:
            self.t_buf.append(last_ts)
            self.pos_buf.append(last_parsed["pos_deg"])
            self.spd_buf.append(last_parsed["spd_erpm"])
            self.cur_buf.append(last_parsed["cur_A"])
            self.tmp_buf.append(last_parsed["temp_C"])
            self._last_sample_ts = last_ts
            if last_error is not None:
                self._set_error_code(last_error)
            any_parsed = True

        if any_parsed:
            self._prune_buffers()
            if self.t_buf:
                t0 = self.t_buf[0]
                xs = [t - t0 for t in self.t_buf]
            else:
                xs = []

            t_now = time.monotonic()
            if (t_now - self._last_plot_ts) >= self._plot_min_dt:
                self._last_plot_ts = t_now
                max_points = max(20, int(float(self.spin_timewin.value()) * float(PLOT_MAX_HZ)))
                if self.cb_pos.isChecked():
                    x_d, y_d = self._decimate_minmax(xs, list(self.pos_buf), max_points)
                    self.curve_pos.setData(x_d, y_d)
                else:
                    self.curve_pos.setData([], [])
                if self.cb_spd.isChecked():
                    x_d, y_d = self._decimate_minmax(xs, list(self.spd_buf), max_points)
                    self.curve_spd.setData(x_d, y_d)
                else:
                    self.curve_spd.setData([], [])
                if self.cb_cur.isChecked():
                    x_d, y_d = self._decimate_minmax(xs, list(self.cur_buf), max_points)
                    self.curve_cur.setData(x_d, y_d)
                else:
                    self.curve_cur.setData([], [])
                if self.cb_tmp.isChecked():
                    x_d, y_d = self._decimate_minmax(xs, list(self.tmp_buf), max_points)
                    self.curve_tmp.setData(x_d, y_d)
                else:
                    self.curve_tmp.setData([], [])
                did_plot_update = True

                # === 추가: X축 이동 윈도우 ===
                if self.cb_follow.isChecked() and self.t_buf:
                    t0 = self.t_buf[0]
                    t_end = self.t_buf[-1] - t0                 # 마지막 시점(초)
                    win = float(self.spin_timewin.value())
                    x_start = max(t_end - win, 0.0)
                    x_end = max(t_end, win)
                    self.vb_pos.setXRange(x_start, x_end, padding=0)

        # Visibility/axis layout changes only when a checkbox or target changes.
        visibility = tuple(cb.isChecked() for cb in (
            self.cb_pos, self.cb_spd, self.cb_cur, self.cb_tmp, self.cb_ref_show)) + (self.ref_view,)
        if visibility != self._last_visibility:
            for curve, visible in zip((self.curve_pos, self.curve_spd, self.curve_cur,
                                       self.curve_tmp, self.curve_ref), visibility):
                curve.setVisible(visible)
            self._last_visibility = visibility
            self._update_axes_visibility()

        if self._last_sample_ts and time.monotonic() - self._last_sample_ts > 0.5:
            self.lbl_error.setText("Error: stale / selected ID not receiving")
            self.last_error_code = None

        # Reference samples are taken from successful worker transmissions at GUI rate.
        if self.cb_ref_show.isChecked() and self.t_ref:
            self._prune_buffers()
            t0 = self.t_buf[0] if self.t_buf else self.t_ref[0]
            t_now = time.monotonic()
            if did_plot_update or (t_now - self._last_plot_ts) >= self._plot_min_dt:
                self._last_plot_ts = t_now
                xs_ref = [tr - t0 for tr in self.t_ref]
                max_points = max(20, int(float(self.spin_timewin.value()) * float(PLOT_MAX_HZ)))
                x_d, y_d = self._decimate_minmax(xs_ref, list(self.y_ref), max_points)
                self.curve_ref.setData(x_d, y_d)

    def _update_axes_visibility(self):
        show_spd_axis = self.cb_spd.isChecked() or (
            self.cb_ref_show.isChecked() and self.ref_view == self.vb_spd
        )
        show_cur_axis = self.cb_cur.isChecked() or (
            self.cb_ref_show.isChecked() and self.ref_view == self.vb_cur
        )
        self.axis_spd.setVisible(show_spd_axis)
        self.axis_cur.setVisible(show_cur_axis)
        self.axis_tmp.setVisible(self.cb_tmp.isChecked())
        self.axis_ref.setVisible(self.cb_ref_show.isChecked() and self.ref_view == self.vb_ref)

    def _on_calib(self):
        """캘리브레이션 트리거(Write-only). mode_id=6, 8바이트 0 패딩."""
        if self.manager.bus is None:
            self._log("버스 미연결")
            return
        driver_id = self.spin_driver.value()
        try:
            eid = build_ext_id(MODE_CALIBRATION, driver_id)
            payload = pack_calibration_payload()
            self.manager.send_ext(eid, payload)
            self._log(f"Calibration queued: EID=0x{eid:08X}, driver={driver_id}")
        except Exception as e:
            self._log(f"Calibration 실패: {e}")

    def _on_cogging(self):
        """Start/stop the autonomous bidirectional cogging measurement."""
        if self.manager.bus is None:
            self._log("버스 미연결")
            return
        driver_id = self.spin_driver.value()
        start = self.btn_cogging.text().startswith("Start")
        rpm = float(self.spin_cogging_rpm.value()) if start else 0.0
        raw = int(round(rpm * 100.0))
        try:
            eid = build_ext_id(MODE_COGGING_COMPENSATION, driver_id)
            payload = raw.to_bytes(4, byteorder="big", signed=True) + b"\x00" * 4
            self.manager.send_ext(eid, payload)
            self.btn_cogging.setText("Stop Cogging" if start else "Start Cogging")
            self.spin_cogging_rpm.setEnabled(not start)
            if start:
                seconds = 8.0 * 60.0 / rpm
                self._log(f"Cogging measurement started: {rpm:.1f} mechanical RPM, about {seconds:.0f}s")
            else:
                self._log("Cogging measurement stop sent")
        except Exception as e:
            self._log(f"Cogging command failed: {e}")

    def _on_cogging_toggle(self, checked: bool):
        """Enable or disable LUT feed-forward without clearing the LUT."""
        if self.manager.bus is None:
            self.btn_cogging_comp.blockSignals(True)
            self.btn_cogging_comp.setChecked(not checked)
            self.btn_cogging_comp.blockSignals(False)
            self._log("버스 미연결")
            return
        driver_id = self.spin_driver.value()
        try:
            eid = build_ext_id(MODE_COGGING_TOGGLE, driver_id)
            payload = bytes([1 if checked else 0]) + b"\x00" * 7
            self.manager.send_ext(eid, payload)
            self.btn_cogging_comp.setText(
                "Cogging Comp: ON" if checked else "Cogging Comp: OFF"
            )
            self._log(f"Cogging compensation {'enabled' if checked else 'disabled'}")
        except Exception as e:
            self.btn_cogging_comp.blockSignals(True)
            self.btn_cogging_comp.setChecked(not checked)
            self.btn_cogging_comp.blockSignals(False)
            self._log(f"Cogging toggle failed: {e}")

    def _on_ref_toggle(self):
        if self.ref_running:
            self._stop_reference()
            self._log("Ref 정지 (주기 송신 중단)")
            return
        if self.manager.writer is None or self._disconnect_thread is not None:
            self._log("참조 시작 실패: 버스 미연결")
            return
        try:
            command = self._reference_command()
            self.manager.writer.set_periodic(command, self.spin_tx_hz.value())
        except Exception as e:
            self._log(f"Ref 시작 실패: {e}")
            return
        self.t_ref.clear()
        self.y_ref.clear()
        self.curve_ref.setData([], [])
        self.ref_running = True
        self.btn_ref_toggle.setText("Stop Ref")
        self.spin_tx_hz.setEnabled(False)
        self._log(f"Ref 시작: {command.shape}, target {self.spin_tx_hz.value()} Hz")

    def _reference_command(self):
        # Only the GUI reads widgets. The worker receives an immutable snapshot.
        return ReferenceCommand(self.spin_driver.value(), self.combo_mode.currentData(),
                                self.combo_shape.currentText(), self.spin_amp.value(),
                                self.spin_freq.value())

    def _on_ref_parameters_changed(self, *_):
        if self.ref_running and self.manager.writer:
            try:
                self.manager.writer.update_periodic(self._reference_command())
            except Exception as exc:
                self._stop_reference()
                self._log(f"Ref parameter error: {exc}")

    def _stop_reference(self):
        if self.manager.writer:
            self.manager.writer.stop_periodic()
        self.ref_running = False
        self.btn_ref_toggle.setText("Start Ref")
        self.spin_tx_hz.setEnabled(True)

    def _collect_reference(self):
        if self.manager.writer is None:
            return
        sample = self.manager.writer.take_reference()
        if sample is not None:
            stamp, value = sample
            self.t_ref.append(stamp)
            self.y_ref.append(value)
        if self.ref_running and not self.manager.writer.periodic_active():
            self._stop_reference()
