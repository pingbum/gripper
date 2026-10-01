# -*- coding: utf-8 -*-
import time
import threading
from PyQt5 import QtCore

from .deps import (
    REF_SEND_HZ,
    MODE_CURRENT, MODE_VELOCITY, MODE_CALIBRATION, MODE_COGGING_COMPENSATION,
    MODE_COGGING_TOGGLE,
    PLOT_MAX_HZ
)
from .deps import (
    build_ext_id, pack_command_payload, parse_broadcast_frame, pack_calibration_payload
)
from .deps import sine, square, triangle
from .deps import CANReaderThread


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
                time.sleep(0.003)
            self._scan_in_progress = False

        self.scan_thread = threading.Thread(target=worker, daemon=True)
        self.scan_thread.start()
    # ------------- 연결/해제 -------------
    def _on_connect(self):
        if self.manager.bus is not None:
            return
        iface = self.iface_combo.currentText().strip()
        channel = self.channel_edit.text().strip()
        try:
            bitrate = int(self.bitrate_edit.text().strip())
        except ValueError:
            self._log("Bitrate 숫자 아님")
            return

        try:
            self.manager.connect(iface, channel, bitrate)
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

            self.btn_connect.setEnabled(False)
            self.btn_disconnect.setEnabled(True)
            self._log(f"Connected: iface={iface}, channel={channel}, bitrate={bitrate}")
            if hasattr(self, "scan_hits"):
                self.scan_hits.clear()
            if hasattr(self, "scan_timer"):
                self._start_scan()
                self.scan_timer.start(200)
            if hasattr(self, "_start_param_read"):
                QtCore.QTimer.singleShot(300, self._start_param_read)
        except Exception as e:
            self._log(f"Connect 실패: {e}")

    def _on_disconnect(self):
        if self.manager.bus is None:
            return
        try:
            self.reader_stop.set()
            if self.reader_thread:
                self.reader_thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            self.manager.disconnect()
        finally:
            if hasattr(self, "scan_stop"):
                self.scan_stop.set()
            if hasattr(self, "scan_timer"):
                self.scan_timer.stop()
            self.reader_thread = None
            self.btn_connect.setEnabled(True)
            self.btn_disconnect.setEnabled(False)
            self._set_error_code(0)
            self._rx_count = 0
            self._tx_count = 0
            self._last_rate_ts = time.time()
            self._update_rate_label(force=True)
            self._log("Disconnected")

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
            self._tx_count += 1
            self._log(f"Write OK: EID=0x{eid:08X}, mode={mode_id}, driver={driver_id}, value={value}")
        except Exception as e:
            self._log(f"Write 실패: {e}")

    # ------------- 수신/그래프 -------------
    def _drain_and_update(self):
        listen_id = self.spin_listen.value()
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
        self._poll_read_response()

        if last_parsed is not None:
            self.t_buf.append(last_ts)
            self.pos_buf.append(last_parsed["pos_deg"])
            self.spd_buf.append(last_parsed["spd_erpm"])
            self.cur_buf.append(last_parsed["cur_A"])
            self.tmp_buf.append(last_parsed["temp_C"])
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

            t_now = time.time()
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

        # Visible 토글
        self.curve_pos.setVisible(self.cb_pos.isChecked())
        self.curve_spd.setVisible(self.cb_spd.isChecked())
        self.curve_cur.setVisible(self.cb_cur.isChecked())
        self.curve_tmp.setVisible(self.cb_tmp.isChecked())
        self.curve_ref.setVisible(self.cb_ref_show.isChecked())
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

        # 참조 곡선 갱신
        if self.cb_ref_show.isChecked() and self.t_ref:
            if self.t_buf:
                t0 = self.t_buf[0]
            else:
                t0 = self.t_ref[0]
            xs_ref = [tr - t0 for tr in self.t_ref]
            if did_plot_update:
                max_points = max(20, int(float(self.spin_timewin.value()) * float(PLOT_MAX_HZ)))
                x_d, y_d = self._decimate_minmax(xs_ref, list(self.y_ref), max_points)
                self.curve_ref.setData(x_d, y_d)
            else:
                t_now = time.time()
                if (t_now - self._last_plot_ts) >= self._plot_min_dt:
                    self._last_plot_ts = t_now
                    max_points = max(20, int(float(self.spin_timewin.value()) * float(PLOT_MAX_HZ)))
                    x_d, y_d = self._decimate_minmax(xs_ref, list(self.y_ref), max_points)
                    self.curve_ref.setData(x_d, y_d)
        else:
            self.curve_ref.setData([], [])

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
            self._tx_count += 1
            self._log(f"Calibration sent: EID=0x{eid:08X}, driver={driver_id}")
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
            self._tx_count += 1
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
            self._tx_count += 1
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
        """참조파 시작/정지 토글"""
        if not self.ref_running:
            # 시작
            if self.manager.bus is None:
                self._log("참조 시작 실패: 버스 미연결")
                return
            # 버퍼 리셋 및 기준 시간 기록
            self.t_ref.clear()
            self.y_ref.clear()
            self.ref_t0 = time.time()
            self.ref_running = True
            self.btn_ref_toggle.setText("Stop Ref")
            # 안내
            mode_id = self.combo_mode.currentData()
            shape = self.combo_shape.currentText()
            amp = float(self.spin_amp.value()); freq = float(self.spin_freq.value())
            unit = "A" if mode_id == MODE_CURRENT else ("eRPM" if mode_id == MODE_VELOCITY else "")
            self._log(f"Ref 시작: shape={shape}, amp={amp}{unit}, f={freq}Hz, mode={mode_id}")
        else:
            # 정지
            self.ref_running = False
            self.btn_ref_toggle.setText("Start Ref")
            self._log("Ref 정지")

    def _on_ref_tick(self):
        """REF_SEND_HZ 주기로 참조 값을 계산·전송하고, 버퍼를 쌓는다."""
        if not self.ref_running:
            return
        if self.manager.bus is None:
            return

        # 시간/파형 파라미터
        t_now = time.time()
        t = t_now - (self.ref_t0 or t_now)
        amp = float(self.spin_amp.value())
        freq = float(self.spin_freq.value())
        shape = self.combo_shape.currentText()

        # 파형 계산
        if shape == "Sine":
            val = sine(t, amp, freq)
        elif shape == "Triangle":
            val = triangle(t, amp, freq)
        else:  # "Square"
            val = square(t, amp, freq)

        # 송신: 현재 선택된 Mode/Driver 사용
        driver_id = self.spin_driver.value()
        mode_id = self.combo_mode.currentData()
        try:
            eid = build_ext_id(mode_id, driver_id)
            payload = pack_command_payload(mode_id, val)  # 단위: A 또는 eRPM
            self.manager.send_ext(eid, payload)
            self._tx_count += 1
        except Exception as e:
            # 송신 실패시 로그만 남기고 계속 시도
            self._log(f"Ref 송신 오류: {e}")
            return

        # 참조 버퍼 기록(그래프 표시용)
        if (t_now - self._last_ref_plot_ts) >= self._ref_plot_min_dt:
            self._last_ref_plot_ts = t_now
            self.t_ref.append(t_now)
            self.y_ref.append(val)
