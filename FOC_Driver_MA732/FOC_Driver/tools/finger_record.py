# -*- coding: utf-8 -*-
"""
손가락 위치 제어 + 동기화 녹화 GUI (one_finger.py 확장).

Record를 누르면 아래를 하나의 Jetson 시계(time.monotonic_ns, t=0 = Record 누른 시점)로 저장한다.
  - 관절 명령값: 드래그/슬라이더 목표(ref)와 실제 전송한 명령(cmd)   → command.csv
  - 실제 관절 각도: 제어 주기(100 Hz)마다 CAN 최신 상태 + 다이나믹셀 읽기값 → angle.csv
  - 안드로이드 폰 카메라 영상 (scrcpy-server, adb USB, 1080p 30 fps)    → video.mp4 + video_frames.csv
  - 앱 화면 (X11 캡처, 하드웨어 인코더)                              → screen.mp4 + screen_frames.csv
  - 이벤트 (녹화 시작/정지, 동기화 마커, Initialize)                   → events.csv
  - 설정/시계/카메라/제어 주기 정보                                   → meta.json

제어 루프를 화면과 분리 (one_finger.py와 다른 점):
  - Smoothing 램프와 위치 명령 전송은 Qt와 무관한 제어 스레드가 RAMP_MS마다 한다.
    화면(3D 뷰, 미리보기, 화면 캡처)이 바빠 Qt 메인 스레드가 밀려도 명령 주기는 유지된다.
  - GUI는 목표(슬라이더/드래그)와 위치 제어/Smoothing 상태를 ctl_state에 넘기기만 한다.
    명령 전송과 self.cmd 변경은 모두 ctl_lock 안에서 한다 (GUI가 직접 보내는 경우 포함).
  - Soft start(게인 램프)와 ROM Guard(Kd 조정)는 슬라이더를 다루므로 기존처럼 GUI 타이머에 남긴다.
  - 영상 수신/디코딩/인코딩은 GStreamer와 별도 스레드, CSV 쓰기는 큐 + 쓰기 스레드,
    녹화 시작/정지(파일 마무리 대기)는 백그라운드 스레드에서 한다.
  - 제어 주기와 GUI 작업 시간 통계를 녹화 전/중으로 나눠 meta.json에 남긴다.

Sync Marker: 고른 관절에 빠른 스텝 명령(SYNC_STEP_DEG)을 넣고 SYNC_HOLD_S 뒤 되돌린다.
영상에서 움직임이 시작된 프레임과 angle.csv 변화 시점을 비교해 고정 지연(video_offset_ms)을 잰다.

실행: python3 finger_record.py
"""
import csv
import json
import os
import queue
import subprocess
import sys
import threading
import time
from collections import deque
from datetime import datetime

from PyQt5 import QtCore, QtGui, QtWidgets

from config import MODE_POSITION
from one_finger import AA_SECTION, DXL_PERIOD_S, OneFingerApp
from phone_camera import (CAMERA_FPS, CAMERA_ID, CAMERA_SIZE, SERVER_VERSION, VIDEO_BIT_RATE,
                          PhoneCamera)
from position_control_app import FINGER_SECTIONS, RAMP_MS, STALE_S, clamp, motor_status
from protocol import pack_command_payload
from screen_recorder import SCREEN_FPS, ScreenRecorder

# =====================================================================
# 1. 설정값
# =====================================================================
HERE = os.path.dirname(os.path.abspath(__file__))
RECORD_ROOT = os.path.join(HERE, "recordings")
APP_VERSION = "finger_record 1.1"

PREVIEW_MS = 66           # 카메라 미리보기/REC 표시 갱신 주기 (15 Hz, GUI 부하 감소)
STATS_N = 3000            # 통계에 쓰는 최근 값 개수 (제어 주기 10 ms → 30 s)
CSV_QUEUE_SIZE = 200000
GIL_SWITCH_S = 0.002      # 파이썬 스레드 전환 간격 (기본 5 ms → 제어 스레드가 GIL을 더 빨리 얻는다)

SYNC_STEP_DEG = 10.0      # 동기화 마커 스텝 크기 [출력축 deg] (범위 가운데 쪽으로)
SYNC_HOLD_S = 0.5         # 스텝 유지 시간 후 원래 위치로 복귀
SYNC_DEFAULT_JOINT = "PIP F/E"

# 관절 표시 이름 → CSV 열 이름
JOINT_COLUMNS = {"MCP A/A": "MCP_AA", "MCP F/E": "MCP_FE", "PIP F/E": "PIP", "DIP F/E": "DIP"}


def git_version():
    try:
        return subprocess.run(["git", "describe", "--always", "--dirty"], cwd=HERE, capture_output=True,
                              text=True, timeout=3).stdout.strip() or None
    except Exception:
        return None


def stats_ms(values_ns):
    if not values_ns:
        return None
    ms = sorted(v / 1e6 for v in values_ns)
    n = len(ms)
    return {"samples": n, "mean_ms": round(sum(ms) / n, 3), "p50_ms": round(ms[n // 2], 3),
            "p99_ms": round(ms[min(n - 1, int(n * 0.99))], 3), "max_ms": round(ms[-1], 3)}


def is_gui_thread():
    return threading.current_thread() is threading.main_thread()


# =====================================================================
# 2. 기록 보조 클래스 (Qt 없음)
# =====================================================================
class CsvWriter(threading.Thread):
    """큐로 받은 행을 별도 스레드에서 쓴다. 디스크가 느려도 호출 스레드를 막지 않는다 (넘치면 drop 집계)."""

    def __init__(self, path, header):
        super().__init__(name=f"csv-{os.path.basename(path)}", daemon=True)
        self.path, self.header = path, header
        self.queue = queue.Queue(maxsize=CSV_QUEUE_SIZE)
        self.closing = threading.Event()
        self.written = self.dropped = 0
        self.error = None
        self.start()

    def put(self, row):
        try:
            self.queue.put_nowait(row)
        except queue.Full:
            self.dropped += 1

    def close(self):
        self.closing.set()
        self.join(timeout=10.0)

    def run(self):
        try:
            with open(self.path, "x", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                w.writerow(self.header)
                while not (self.closing.is_set() and self.queue.empty()):
                    try:
                        row = self.queue.get(timeout=0.1)
                    except queue.Empty:
                        continue
                    w.writerow(row)
                    self.written += 1
        except Exception as exc:
            self.error = str(exc)


class CanTap:
    """CANReaderThread.recorder 자리에 붙어 드라이버별 최신 상태 프레임과 수신 시각을 보관한다.
    앱의 take_latest_status(소비형)와 겹치지 않게 따로 보관한다. 수신 스레드에서 불린다."""

    def __init__(self):
        self.latest = {}  # Driver ID -> (monotonic ns, data)
        self.counts = {}

    def offer(self, msg):
        if msg.is_extended_id and 0 <= msg.arbitration_id <= 0xFF and len(msg.data) == 8 \
                and not msg.is_error_frame:
            self.latest[msg.arbitration_id] = (time.monotonic_ns(), bytes(msg.data))
            self.counts[msg.arbitration_id] = self.counts.get(msg.arbitration_id, 0) + 1


class Timing:
    """GUI 작업 시간/주기 기록 (최근 STATS_N개 + 녹화 중 전체)."""

    def __init__(self):
        self.recent = deque(maxlen=STATS_N)
        self.session = None

    def add(self, ns):
        self.recent.append(ns)
        if self.session is not None:
            self.session.append(ns)

    def begin(self):
        before = stats_ms(list(self.recent))
        self.session = []
        return before

    def end(self):
        values, self.session = self.session, None
        return stats_ms(values or [])


# =====================================================================
# 3. 메인 창
# =====================================================================
class FingerRecordApp(OneFingerApp):
    def __init__(self):
        self.can_tap = CanTap()
        self.camera = PhoneCamera()
        self.session = None          # 녹화 중인 세션 dict
        self.finishing = None        # 마무리 중인 백그라운드 스레드
        self.finish_result = None
        self.cam_state = None
        # 제어 스레드와 공유하는 상태 (ctl_lock 안에서만 읽고 쓴다)
        self.ctl_lock = threading.RLock()
        self.ctl_state = {}          # 섹션 -> {"enabled", "ref", "active"}
        self.ctl_smooth = (False, 0.0)
        self.ctl_stop = threading.Event()
        self.log_queue = deque()     # 다른 스레드의 로그 (GUI가 옮겨 적는다)
        self.timing = {name: Timing() for name in ("control_period", "gui_ramp_period", "gui_poll", "gui_preview")}
        self.gui_ramp_last = None
        super().__init__()
        self.setWindowTitle("One Finger Control + Record")
        self.resize(1750, 1000)
        with self.ctl_lock:
            self._sync_ctl()

        self.ctl_thread = threading.Thread(target=self._ctl_loop, name="control", daemon=True)
        self.ctl_thread.start()
        self.preview_timer = QtCore.QTimer(self)
        self.preview_timer.timeout.connect(self._update_preview)
        self.preview_timer.start(PREVIEW_MS)
        self.camera.start()  # 폰이 연결돼 있으면 바로 미리보기

    # ------------- UI -------------
    def _build_finger(self):
        finger = super()._build_finger()
        rec_box = QtWidgets.QGroupBox("Camera / Record")
        grid = QtWidgets.QGridLayout(rec_box)

        self.preview = QtWidgets.QLabel("카메라 연결 대기")
        self.preview.setAlignment(QtCore.Qt.AlignCenter)
        self.preview.setMinimumSize(320, 180)
        self.preview.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Ignored)
        self.preview.setStyleSheet("background-color: #000000; color: #888888;")

        self.btn_camera = QtWidgets.QPushButton("Camera ON")
        self.btn_camera.setCheckable(True)
        self.btn_camera.setChecked(True)
        self.btn_camera.toggled.connect(self._on_camera)
        self.btn_record = QtWidgets.QPushButton("●  Record")
        self.btn_record.setCheckable(True)
        self.btn_record.setStyleSheet("QPushButton { background-color: #6e1b1b; font-weight: bold; }"
                                      "QPushButton:checked { background-color: #d11a1a; }")
        self.btn_record.toggled.connect(self._on_record)
        self.lbl_rec = QtWidgets.QLabel("")
        self.lbl_rec.setMinimumWidth(150)
        self.lbl_cam = QtWidgets.QLabel("")
        self.combo_sync = QtWidgets.QComboBox()
        self.combo_sync.addItems(list(JOINT_COLUMNS))
        self.combo_sync.setCurrentText(SYNC_DEFAULT_JOINT)
        self.btn_sync = QtWidgets.QPushButton("Sync Marker")
        self.btn_sync.setToolTip(f"선택 관절에 {SYNC_STEP_DEG:g} deg 스텝 후 {SYNC_HOLD_S:g} s 뒤 복귀 "
                                 "(위치 제어 ON 필요). 영상-각도 시간 오차 측정용")
        self.btn_sync.clicked.connect(self._on_sync_marker)

        row = QtWidgets.QHBoxLayout()
        for widget in (self.btn_camera, self.btn_record, self.lbl_rec):
            row.addWidget(widget)
        row.addStretch()
        row.addWidget(QtWidgets.QLabel("Sync"))
        row.addWidget(self.combo_sync)
        row.addWidget(self.btn_sync)
        grid.addLayout(row, 0, 0)
        grid.addWidget(self.preview, 1, 0)
        grid.addWidget(self.lbl_cam, 2, 0)
        grid.setRowStretch(1, 1)

        splitter = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        splitter.addWidget(finger)
        splitter.addWidget(rec_box)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)
        return splitter

    def _joint_sections(self):
        """(표시 이름, 섹션) - 관절 순서: MCP A/A, MCP F/E, PIP, DIP."""
        return (("MCP A/A", self.sections[AA_SECTION]), ("MCP F/E", self.mcp),
                ("PIP F/E", self.sections[FINGER_SECTIONS[0]]), ("DIP F/E", self.sections[FINGER_SECTIONS[1]]))

    def _log(self, msg):
        """다른 스레드에서 부르면 큐에 넣고 GUI 타이머가 옮겨 적는다 (Qt 위젯은 GUI 스레드에서만)."""
        if is_gui_thread():
            super()._log(msg)
        else:
            self.log_queue.append(f"[{time.strftime('%H:%M:%S')}] {msg}")

    def _flush_logs(self):
        while self.log_queue:
            self.log.appendPlainText(self.log_queue.popleft())

    # ------------- 제어 스레드와 공유하는 상태 -------------
    def _sync_ctl(self):
        """GUI 위젯 상태 → ctl_state. GUI 스레드에서 ctl_lock을 잡고 부른다."""
        for _, sec in self._joint_sections():
            self.ctl_state[sec] = {
                "enabled": sec.cb_control.isChecked(),
                "ref": clamp(sec.ref_deg, sec.low, sec.high),
                "active": tuple(sec.active_ids) if hasattr(sec, "active_ids") else (),
            }
        self.ctl_smooth = (self.btn_smooth.isChecked(), self.spin_slope.value())

    def _command(self, sec, deg):
        """출력축 위치 명령 전송 (GUI/제어 스레드 공용). self.cmd와 전송은 ctl_lock 안에서만 바꾼다."""
        with self.ctl_lock:
            if is_gui_thread():
                self._sync_ctl()  # set_ref 직후 등 최신 목표/선택 모터를 반영
            target = clamp(deg, sec.low, sec.high)  # 위치 제약 최종 확인
            self.cmd[sec] = target
            if sec is self.mcp:
                worker = self.dxl_worker
                ok = worker is not None
                if ok:
                    worker.request(True, self._deg_to_raw(target))
            else:
                ok = all([self._send(MODE_POSITION, i, pack_command_payload(
                    MODE_POSITION, self.offsets[i] + target * sec.gear)) for i in self.ctl_state[sec]["active"]])
            self._log_command("cmd", sec)
        return ok

    def _ramp_command(self):
        """GUI 타이머에서는 하지 않는다. Smoothing 램프는 제어 스레드(_ctl_step)가 한다."""

    def _ctl_step(self):
        """1차 선형 보간 (one_finger와 같은 규칙): 마지막 명령을 목표 쪽으로 최대 기울기 * 주기만큼 옮긴다."""
        with self.ctl_lock:
            smooth, slope = self.ctl_smooth
            if not smooth:
                return
            max_step = slope * RAMP_MS / 1000.0
            for sec, state in self.ctl_state.items():
                cmd = self.cmd.get(sec)
                if not state["enabled"] or cmd is None:
                    continue
                error = state["ref"] - cmd
                if error != 0.0:
                    self._command(sec, cmd + clamp(error, -max_step, max_step))

    def _ctl_loop(self):
        """제어 스레드: RAMP_MS마다 램프 명령 + (녹화 중) 각도 샘플링."""
        period = RAMP_MS / 1000.0
        next_t = time.monotonic()
        last = None
        while not self.ctl_stop.is_set():
            now = time.monotonic_ns()
            if last is not None:
                self.timing["control_period"].add(now - last)
            last = now
            try:
                self._ctl_step()
                session = self.session
                if session is not None:
                    session["angle"].put(self.angle_row(session["t0"]))
            except Exception as exc:
                self._log(f"제어 스레드 오류: {exc!r}")
            next_t += period
            delay = next_t - time.monotonic()
            if delay > 0:
                self.ctl_stop.wait(delay)
            else:
                next_t = time.monotonic()  # 밀리면 따라잡지 않고 다시 맞춘다

    # ------------- GUI 훅: 상태를 넘기고 공유 상태를 바꾸는 처리는 ctl_lock 안에서 -------------
    def _ramp(self):
        """GUI 타이머 (RAMP_MS): 상태 동기화 + Soft start + ROM Guard. 위치 램프는 제어 스레드."""
        now = time.monotonic_ns()
        if self.gui_ramp_last is not None:
            self.timing["gui_ramp_period"].add(now - self.gui_ramp_last)
        self.gui_ramp_last = now
        with self.ctl_lock:
            self._sync_ctl()
        super()._ramp()

    def _send_ref(self, sec):
        with self.ctl_lock:
            self._sync_ctl()
            ok = super()._send_ref(sec)
            self._log_command("ref", sec)
        return ok

    def _on_control(self, sec, checked):
        with self.ctl_lock:
            super()._on_control(sec, checked)
            self._sync_ctl()
            self._log_command("control_on" if sec.cb_control.isChecked() else "control_off", sec)

    def _on_mcp_control(self, checked):
        with self.ctl_lock:
            super()._on_mcp_control(checked)
            self._sync_ctl()
            self._log_command("control_on" if self.mcp.cb_control.isChecked() else "control_off", self.mcp)

    def _on_smooth(self, checked):
        with self.ctl_lock:
            super()._on_smooth(checked)
            self._sync_ctl()

    def _on_initialize(self):
        with self.ctl_lock:
            super()._on_initialize()
            self._sync_ctl()
        self._log_event("initialize", "영점 재설정 (이후 각도 기준 변경)")

    def _poll(self):
        start = time.monotonic_ns()
        super()._poll()
        self.timing["gui_poll"].add(time.monotonic_ns() - start)

    # ------------- CAN 수신 탭 -------------
    def _open_bus(self):
        super()._open_bus()
        if self.reader is not None:
            self.reader.recorder = self.can_tap

    # ------------- 데이터 행 만들기 -------------
    def angle_row(self, t0):
        """angle.csv 한 행. 제어 스레드에서 불린다 (GUI 위젯은 읽지 않는다)."""
        now = time.monotonic_ns()
        stale_ns = int(STALE_S * 1e9)
        latest = self.can_tap.latest

        def motor(i, gear):
            sample = latest.get(i)
            if sample is None or now - sample[0] > stale_ns:
                return None, None
            pos, _, cur = motor_status(sample[1])
            return (pos - self.offsets[i]) / gear, cur

        def fmt(v, nd=4):
            return "" if v is None else round(v, nd)

        aa_sec, pip_sec, dip_sec = (self.sections[i] for i in (AA_SECTION, *FINGER_SECTIONS))
        aa, aa_a = motor(aa_sec.motor_ids[0], aa_sec.gear)
        pips = [motor(i, pip_sec.gear) for i in pip_sec.motor_ids]
        pip = None if any(p[0] is None for p in pips) else sum(p[0] for p in pips) / len(pips)
        dip, dip_a = motor(dip_sec.motor_ids[0], dip_sec.gear)

        fe, fe_t = None, ""
        worker = self.dxl_worker
        if worker is not None:
            with worker.lock:
                sample = worker.sample
            if sample is not None and now - sample[0] * 1e9 <= stale_ns:
                fe, fe_t = self._raw_to_deg(sample[1]), int(sample[0] * 1e9) - t0
        return (now - t0, fmt(aa), fmt(fe), fmt(pip), fmt(dip),
                *(fmt(p[0]) for p in pips), fe_t,
                fmt(aa_a), *(fmt(p[1]) for p in pips), fmt(dip_a))

    def _angle_header(self):
        pip_ids = self.sections[FINGER_SECTIONS[0]].motor_ids
        return ("t_ns", "MCP_AA_deg", "MCP_FE_deg", "PIP_deg", "DIP_deg",
                *(f"PIP_M{i}_deg" for i in pip_ids), "MCP_FE_read_t_ns",
                "MCP_AA_A", *(f"PIP_M{i}_A" for i in pip_ids), "DIP_A")

    def _command_header(self):
        cols = ["t_ns", "event", "joint"]
        for name in JOINT_COLUMNS.values():
            cols += [f"{name}_ref_deg", f"{name}_cmd_deg"]
        return cols

    def _log_command(self, event, sec):
        """command.csv 한 행 (ctl_lock 안에서 부른다). 목표/상태는 ctl_state, 명령은 self.cmd."""
        session = self.session
        if session is None:
            return
        t = time.monotonic_ns() - session["t0"]
        row = [t, event, next((n for n, s in self._joint_sections() if s is sec), "")]
        for _, s in self._joint_sections():
            state = self.ctl_state.get(s)
            cmd = self.cmd.get(s)
            row += [round(state["ref"], 3) if state and state["enabled"] else "",
                    "" if cmd is None else round(cmd, 4)]
        session["command"].put(row)

    def _log_event(self, event, detail=""):
        session = self.session
        if session is not None:
            session["events"].put((time.monotonic_ns() - session["t0"], event, detail))

    # ------------- 카메라 -------------
    def _on_camera(self, checked):
        self.btn_camera.setText("Camera ON" if checked else "Camera OFF")
        if checked:
            self.camera.start()
        else:
            if self.session is not None:
                self.btn_record.setChecked(False)
            finishing = self.finishing

            def stop():
                if finishing is not None:
                    finishing.join(timeout=10.0)  # 녹화 파일 마무리 후 카메라를 끈다
                self.camera.stop()
            threading.Thread(target=stop, daemon=True).start()

    def _update_preview(self):
        start = time.monotonic_ns()
        self._flush_logs()
        state, error = self.camera.snapshot()
        if (state, error) != self.cam_state:
            self.cam_state = (state, error)
            if error:
                self._log(f"카메라: {error}")
                self.preview.setText(f"카메라 오류: {error}")
                self.preview.setPixmap(QtGui.QPixmap())
            elif state == "starting":
                self.preview.setText("카메라 연결 중...")
            elif state == "streaming":
                self._log(f"카메라 연결: {self.camera.model} (Android {self.camera.android}), "
                          f"camera {CAMERA_ID}, {CAMERA_SIZE} {CAMERA_FPS} fps")
            elif state == "stopped" and not error:
                self.preview.setPixmap(QtGui.QPixmap())
                self.preview.setText("카메라 OFF")
        for notice in self.camera.take_notices():
            self._log(f"카메라: {notice}")
        if state == "streaming":
            frame = self.camera.latest_preview()
            if frame is not None:
                data, w, h, stride = frame
                image = QtGui.QImage(data, w, h, stride, QtGui.QImage.Format_RGB32)
                self.preview.setPixmap(QtGui.QPixmap.fromImage(image).scaled(
                    self.preview.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.FastTransformation))
            fps = self.camera.measured_fps()
            self.lbl_cam.setText(f"{self.camera.model}  {self.camera.video_size}  "
                                 f"{fps:.1f} fps" if fps else "")
        else:
            self.lbl_cam.setText("")
        self._update_rec_label()
        self._check_finish()
        self.timing["gui_preview"].add(time.monotonic_ns() - start)

    # ------------- 녹화 -------------
    def _on_record(self, checked):
        if checked:
            if not self._start_record():
                self.btn_record.blockSignals(True)
                self.btn_record.setChecked(False)
                self.btn_record.blockSignals(False)
        elif self.session is not None:
            self._stop_record()

    def _start_record(self):
        if self.finishing is not None:
            self._log("Record: 이전 녹화를 저장하는 중입니다. 잠시 후 다시 누르세요.")
            return False
        if self.camera.snapshot()[0] != "streaming":
            self._log("Record: 카메라가 연결되지 않아 시작할 수 없습니다 (Camera ON, 폰 USB 확인).")
            return False
        t0 = time.monotonic_ns()  # 기준점 t = 0 (Record 누른 시점)
        wall = time.time()
        folder = os.path.join(RECORD_ROOT, datetime.fromtimestamp(wall).strftime("%Y-%m-%d_%H%M%S"))
        os.makedirs(folder, exist_ok=False)
        session = {
            "t0": t0, "wall": wall, "folder": folder,
            "timing_before": {name: t.begin() for name, t in self.timing.items()},
            "command": CsvWriter(os.path.join(folder, "command.csv"), self._command_header()),
            "angle": CsvWriter(os.path.join(folder, "angle.csv"), self._angle_header()),
            "events": CsvWriter(os.path.join(folder, "events.csv"), ("t_ns", "event", "detail")),
            "settings": self._settings_snapshot(),
            "screen": None, "camera_ok": None, "errors": [],
            "can_counts_start": dict(self.can_tap.counts),
        }
        with self.ctl_lock:
            self.session = session
            self._log_command("initial", None)  # 시작 시점의 명령 상태 1행
        self._log_event("record_start", folder)
        # 영상 시작(파이프라인 생성, 수 ms~수십 ms)은 GUI를 막지 않게 백그라운드에서 한다.
        rect = self.frameGeometry().intersected(QtWidgets.QApplication.primaryScreen().geometry())
        threading.Thread(target=self._start_media, args=(session, (rect.x(), rect.y(), rect.width(),
                                                                   rect.height())), daemon=True).start()
        self._log(f"Record 시작: {folder}")
        return True

    def _start_media(self, session, region):
        ok = self.camera.start_recording(os.path.join(session["folder"], "video.mp4"), session["t0"])
        session["camera_ok"] = ok
        if not ok:
            session["errors"].append("카메라 녹화 시작 실패")
        try:
            rec = ScreenRecorder(os.path.join(session["folder"], "screen.mp4"), region,
                                 display=os.environ.get("DISPLAY"))
            rec.start()
            session["screen"] = rec
        except Exception as exc:
            session["errors"].append(f"화면 녹화 시작 실패: {exc}")

    def _stop_record(self):
        with self.ctl_lock:
            session, self.session = self.session, None
        session["t_stop"] = time.monotonic_ns() - session["t0"]
        session["events"].put((session["t_stop"], "record_stop", ""))
        session["timing_during"] = {name: t.end() for name, t in self.timing.items()}
        session["settings_end"] = self._settings_snapshot()
        start = session["can_counts_start"]
        session["can_counts"] = {k: v - start.get(k, 0) for k, v in self.can_tap.counts.items()}
        self.finish_result = None
        self.finishing = threading.Thread(target=self._finish_media, args=(session,), daemon=True)
        self.finishing.start()
        self._log(f"Record 정지 ({session['t_stop'] / 1e9:.1f} s), 파일 저장 중...")

    def _finish_media(self, session):
        """백그라운드: 영상 마무리(EOS 대기), CSV 닫기, meta.json 쓰기."""
        folder, t0 = session["folder"], session["t0"]
        deadline = time.monotonic() + 5.0
        while session["camera_ok"] is None and time.monotonic() < deadline:  # 시작 스레드가 아직이면 대기
            time.sleep(0.05)
        cam_info = self.camera.stop_recording(os.path.join(folder, "video_frames.csv"))
        screen_info = None
        if session["screen"] is not None:
            screen_info = session["screen"].stop(os.path.join(folder, "screen_frames.csv"), t0)
        writers = {k: session[k] for k in ("command", "angle", "events")}
        for w in writers.values():
            w.close()
        errors = list(session["errors"])
        for name, w in writers.items():
            if w.error:
                errors.append(f"{name}.csv: {w.error}")
            if w.dropped:
                errors.append(f"{name}.csv: {w.dropped}행 누락 (디스크 지연)")
        for name, info in (("video", cam_info), ("screen", screen_info)):
            if info and info.get("error"):
                errors.append(f"{name}.mp4: {info['error']}")
        meta = self._meta(session, cam_info, screen_info, writers, errors)
        with open(os.path.join(folder, "meta.json"), "w", encoding="utf-8") as f:
            json.dump(meta, f, ensure_ascii=False, indent=2)
        self.finish_result = (folder, meta, errors)

    def _check_finish(self):
        if self.finishing is None or self.finishing.is_alive():
            return
        self.finishing = None
        folder, meta, errors = self.finish_result
        self._log(f"Record 저장 완료: {folder}")
        cam = meta["camera"]
        self._log(f"  영상 {cam.get('frames')}프레임 ({cam.get('fps_measured') or 0:.2f} fps), "
                  f"화면 {(meta['screen'] or {}).get('frames')}프레임, 각도 {meta['angle']['rows']}행, "
                  f"명령 {meta['command']['rows']}행")
        loop = meta["timing"]
        for name, label in (("control_period", "제어 주기"), ("gui_ramp_period", "GUI 타이머 주기")):
            before, during = loop["before_record"].get(name), loop["during_record"].get(name)
            if before and during:
                self._log(f"  {label} [ms] 녹화 전 평균 {before['mean_ms']} / p99 {before['p99_ms']}, "
                          f"녹화 중 평균 {during['mean_ms']} / p99 {during['p99_ms']} / 최대 {during['max_ms']}")
        for err in errors:
            self._log(f"  경고: {err}")

    def _update_rec_label(self):
        session = self.session
        if session is None:
            self.lbl_rec.setText("저장 중..." if self.finishing is not None else "")
            self.lbl_rec.setStyleSheet("")
            return
        sec = (time.monotonic_ns() - session["t0"]) / 1e9
        blink = int(sec * 2) % 2 == 0
        self.lbl_rec.setText(f"{'●' if blink else '○'} REC  {int(sec // 60):02d}:{sec % 60:04.1f}")
        self.lbl_rec.setStyleSheet("color: #ff3b3b; font-weight: bold; font-size: 12pt;")

    # ------------- 동기화 마커 -------------
    def _on_sync_marker(self):
        name = self.combo_sync.currentText()
        sec = dict(self._joint_sections())[name]
        if not sec.cb_control.isChecked():
            self._log(f"Sync Marker: {name} 위치 제어가 OFF입니다.")
            return
        start = self.cmd.get(sec, sec.ref_deg)
        # 범위 가운데 쪽으로 스텝 (끝에 붙어 있어도 움직이게)
        step = SYNC_STEP_DEG if start <= (sec.low + sec.high) / 2 else -SYNC_STEP_DEG
        target = clamp(start + step, sec.low, sec.high)
        with self.ctl_lock:
            sec.set_ref(target)
            self._command(sec, target)  # 목표와 명령을 함께 옮겨 Smoothing 없이 즉시 스텝
        self._log_event("sync_step", f"{name} {start:.2f} -> {target:.2f} deg")
        self._log(f"Sync Marker: {name} {start:.1f} → {target:.1f} deg, {SYNC_HOLD_S:g} s 후 복귀")
        self.btn_sync.setEnabled(False)

        def back():
            self.btn_sync.setEnabled(True)
            if sec.cb_control.isChecked():
                with self.ctl_lock:
                    sec.set_ref(start)
                    self._command(sec, start)
                self._log_event("sync_return", f"{name} {target:.2f} -> {start:.2f} deg")
        QtCore.QTimer.singleShot(int(SYNC_HOLD_S * 1000), back)

    # ------------- meta.json -------------
    def _settings_snapshot(self):
        joints = {}
        for name, sec in self._joint_sections():
            item = {"control": sec.cb_control.isChecked(), "limits_deg": [sec.low, sec.high]}
            if hasattr(sec, "kp"):
                item.update(kp_A_per_deg=sec.kp, kd_A_s_per_deg=sec.kd, gear=sec.gear,
                            motor_ids=list(sec.motor_ids), active_ids=list(sec.active_ids))
            joints[name] = item
        return {"smoothing": self.btn_smooth.isChecked(), "max_slope_deg_s": self.spin_slope.value(),
                "soft_start_s": self.spin_softstart.value(), "cogging_comp": self.btn_cogging.isChecked(),
                "rom_guard": self.btn_kd_guard.isChecked(), "joints": joints,
                "motor_zero_offsets_deg": dict(self.offsets), "dxl_zero_deg": self.dxl_zero}

    def _meta(self, session, cam, screen, writers, errors):
        t_stop = session["t_stop"]
        cam = cam or {}
        can_rates = {f"M{k}": round(v / max(t_stop / 1e9, 1e-9), 1) for k, v in session["can_counts"].items()}
        return {
            "app": "finger_record.py", "app_version": APP_VERSION, "git": git_version(),
            "start_wall_time": datetime.fromtimestamp(session["wall"]).isoformat(timespec="milliseconds"),
            "start_wall_epoch_s": session["wall"],
            "t0_monotonic_ns": session["t0"],
            "duration_s": round(t_stop / 1e9, 3),
            "timebase": "모든 t_ns = time.monotonic_ns() - t0_monotonic_ns (Jetson CLOCK_MONOTONIC), "
                        "t0 = Record 버튼을 누른 시점",
            "joints": [{"name": n, "column": c, "unit": "deg", "axis": "출력축"} for n, c in JOINT_COLUMNS.items()],
            "angle": {
                "file": "angle.csv", "rows": writers["angle"].written,
                "sample_rate_hz": 1000 / RAMP_MS,
                "sampling": "제어 스레드가 명령 주기(RAMP_MS)마다 각 관절의 최신값을 기록",
                "sources": {"MCP A/A, PIP, DIP": "CAN 상태 브로드캐스트 (드라이버당 약 1 kHz)",
                            "MCP F/E": f"다이나믹셀 Present Position {1 / DXL_PERIOD_S:.0f} Hz "
                                       "(실제 읽은 시각 = MCP_FE_read_t_ns)"},
                "can_status_rate_hz_while_recording": can_rates,
                "notes": "PIP = M1, M2 평균. 빈 칸 = 수신 없음/연결 안 됨. 단위 deg (출력축, Initialize 영점 기준)",
            },
            "command": {
                "file": "command.csv", "rows": writers["command"].written,
                "columns": "*_ref_deg = 사용자가 드래그/슬라이더로 정한 목표, *_cmd_deg = 실제 전송한 위치 명령 "
                           "(Smoothing ON이면 기울기 제한 보간값). 빈 칸 = 위치 제어 OFF",
                "events": "cmd = 명령 전송 시점 (CAN TX 큐에 넣은 시각, MCP F/E는 다이나믹셀 스레드 요청 시각, "
                          f"최대 {DXL_PERIOD_S * 1000:.0f} ms 뒤 반영), ref = 목표 변경, control_on/off",
                "ramp_period_ms": RAMP_MS,
            },
            "camera": {
                "file": "video.mp4", "frames_file": "video_frames.csv",
                "connection": f"scrcpy-server v{SERVER_VERSION} camera source, adb USB (Python 직접 클라이언트)",
                "device": self.camera.model, "android": self.camera.android, "camera_id": CAMERA_ID,
                "resolution": CAMERA_SIZE, "fps_requested": CAMERA_FPS, "fps_measured": cam.get("fps"),
                "codec": "H.264 (폰 인코딩, 재인코딩 없이 mp4 저장)", "bit_rate": VIDEO_BIT_RATE,
                "frames": cam.get("frames"), "first_frame_t_ns": cam.get("first_frame_t_ns"),
                "clock_offset_ns": cam.get("clock_offset_ns"),
                "recv_latency_above_min_ms_p50_p99": cam.get("recv_jitter_ms_p50_p99"),
                "timestamp_method": "t_ns = 폰 PTS + min(수신 시각 - PTS) - t0. recv_t_ns = Jetson 수신 시각 그대로",
                "video_offset_ms": 0.0,
                "video_offset_note": "동기화 마커로 측정한 고정 지연. 후처리에서 영상 시각 = t_ns - video_offset_ms",
            },
            "screen": None if screen is None else {
                "file": "screen.mp4", "frames_file": "screen_frames.csv", "fps_requested": SCREEN_FPS,
                "fps_measured": screen.get("fps"), "frames": screen.get("frames"), "size": screen.get("size"),
                "timestamp_method": "GStreamer 파이프라인 시계(CLOCK_MONOTONIC) base_time + ximagesrc PTS",
                "capture_to_probe_ms_p50": screen.get("capture_to_probe_ms_p50"),
            },
            "timing": {
                "description": "control_period = 제어 스레드 명령 주기, gui_ramp_period = GUI 타이머 주기 "
                               "(Soft start/ROM Guard), gui_poll = 상태 표시/3D 뷰 갱신 1회 시간, "
                               "gui_preview = 카메라 미리보기 갱신 1회 시간",
                "target_period_ms": RAMP_MS,
                "before_record": session["timing_before"], "during_record": session["timing_during"],
            },
            "settings_at_start": session["settings"], "settings_at_stop": session["settings_end"],
            "errors": errors,
        }

    # ------------- 종료 -------------
    def closeEvent(self, event):
        if self.session is not None:
            self.btn_record.setChecked(False)
        if self.finishing is not None:
            self.finishing.join(timeout=10.0)
        self.camera.stop()
        super().closeEvent(event)  # 위치 제어 OFF(0 A / 토크 OFF) 후 연결 해제
        self.ctl_stop.set()
        self.ctl_thread.join(timeout=1.0)


# =====================================================================
# 4. 실행
# =====================================================================
def main():
    sys.setswitchinterval(GIL_SWITCH_S)
    app = QtWidgets.QApplication(sys.argv)
    window = FingerRecordApp()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
