# -*- coding: utf-8 -*-
"""LPF ON/OFF 전류 비교 측정 GUI (CAN 프로토콜 v1, mode 0x40~0x45).

실행: python tools/LPF_check.py
순서: Connect → 위치 명령(0x05)으로 위치 유지 → 측정 START → 자동 수신(3076 프레임)
      → token/체크섬/3072 항목 확인 후 그래프·통계 표시 및 CSV 저장.

- 프로토콜/수집 로직(LpfClient, CaptureAssembler)은 Qt에 의존하지 않는다.
- 한 백그라운드 스레드가 CAN RX/TX를 단독으로 소유한다.
- 두 조건은 순차적으로 실제 제어에 적용한 결과다. 같은 입력을 오프라인 필터링한
  비교가 아니므로 위치/부하/게인/feedforward를 측정 동안 일정하게 유지해야 한다.
"""
import csv
import math
import queue
import secrets
import socket
import struct
import sys
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Optional

import can
import numpy as np

from can_io import connection_options
from config import DEFAULT_BITRATE, DEFAULT_CHANNEL, DEFAULT_INTERFACE, MODE_POSITION
from protocol import pack_position_payload, parse_broadcast_frame


# ───────────────────────────── 프로토콜 상수 ─────────────────────────────
PROTOCOL_VERSION = 1
MODE_REQUEST = 0x40
MODE_STATUS = 0x41
MODE_SAMPLE = 0x42
MODE_CONFIG = 0x43
MODE_LPF_INFO = 0x44
MODE_END = 0x45

CMD_QUERY, CMD_START, CMD_CANCEL = 0, 1, 2

RESULT_IDLE = 0
RESULT_ACCEPTED = 1
RESULT_BUSY = 2
RESULT_ABORTED = 6
RESULT_COMPLETE = 7
RESULT_NAMES = {
    0: "Idle", 1: "Accepted", 2: "Busy", 3: "InvalidRequest", 4: "NotPosition",
    5: "NoMemory", 6: "Aborted", 7: "Complete", 8: "Fault", 9: "Unavailable",
}
RESULT_HINTS = {
    2: "기존 세션 처리 중 — 완료 후 다시 시도",
    3: "요청 형식 오류",
    4: "위치 모드(0x05)가 아니거나 코깅 측정 중 — 먼저 위치 명령을 보내세요",
    5: "펌웨어 버퍼(12288 B) 확보 실패",
    8: "드라이버 오류 상태 — 오류 해제 후 다시 시도",
    9: "LPF 비활성 빌드 또는 전류 제어 주파수 ≠ 50 kHz",
}
PHASE_SENDING = 5
PHASE_NAMES = {0: "Idle", 1: "Pending", 2: "LPF ON", 3: "LPF OFF", 4: "Ready",
               5: "Sending", 6: "Finished", 7: "Aborted"}

CONDITION_NAMES = ("LPF ON", "LPF OFF")
CHANNEL_NAMES = ("필터 전 i_q*", "적용 i_q*", "실측 i_q")
CHANNEL_KEYS = ("iq_cmd_raw", "iq_cmd_applied", "iq_meas")
FNV_OFFSET = 2166136261
FNV_PRIME = 16777619


def build_eid(mode, driver_id):
    return ((mode & 0xFF) << 8) | (driver_id & 0xFF)


def pack_request(command, token=0):
    if command not in (CMD_QUERY, CMD_START, CMD_CANCEL):
        raise ValueError(f"Unknown request command {command}")
    # QUERY/CANCEL은 token을 무시하므로 문서 예시대로 0을 보낸다.
    return struct.pack(">BBHI", PROTOCOL_VERSION, command, 0,
                       (token & 0xFFFFFFFF) if command == CMD_START else 0)


def fnv1a(chunks, value=FNV_OFFSET):
    for chunk in chunks:
        for byte in chunk:
            value = ((value ^ byte) * FNV_PRIME) & 0xFFFFFFFF
    return value


@dataclass(frozen=True)
class LpfStatus:
    code: int
    phase: int
    token: int

    @property
    def code_name(self):
        return RESULT_NAMES.get(self.code, f"code {self.code}")

    @property
    def phase_name(self):
        return PHASE_NAMES.get(self.phase, f"phase {self.phase}")

    @classmethod
    def decode(cls, payload):
        if len(payload) != 8:
            raise ValueError("0x41 status must be 8 bytes")
        version, code, phase, reserved, token = struct.unpack(">BBBBI", payload)
        if version != PROTOCOL_VERSION or reserved != 0:
            raise ValueError(f"Unsupported 0x41 status (version {version}, reserved {reserved})")
        return cls(code, phase, token)


@dataclass(frozen=True)
class CaptureConfig:
    channels: int
    samples: int
    sample_hz: int
    wait_ms: int

    @property
    def total_frames(self):
        return len(CONDITION_NAMES) * self.channels * self.samples

    @classmethod
    def decode(cls, payload):
        if len(payload) != 8:
            raise ValueError("0x43 config must be 8 bytes")
        version, channels, samples, sample_hz, wait_ms = struct.unpack(">BBHHH", payload)
        if version != PROTOCOL_VERSION:
            raise ValueError(f"Unsupported 0x43 version {version}")
        if channels != len(CHANNEL_NAMES) or samples == 0 or sample_hz == 0:
            raise ValueError(f"Unsupported capture format: channels={channels}, "
                             f"samples={samples}, fs={sample_hz} Hz")
        return cls(channels, samples, sample_hz, wait_ms)


@dataclass
class LpfCapture:
    driver_id: int
    token: int
    config: CaptureConfig
    cutoff_hz: Optional[float]
    position_cmd_deg: Optional[float]
    checksum: int
    data: np.ndarray          # shape (condition, channel, sample), A
    received_at: datetime = field(default_factory=datetime.now)

    @property
    def time_ms(self):
        return np.arange(self.config.samples) * (1000.0 / self.config.sample_hz)


class CaptureAssembler:
    """Collects one session's 0x42 frames after its 0x43 config."""

    def __init__(self, driver_id, config):
        self.driver_id, self.config = driver_id, config
        self.cutoff_hz = self.position_cmd_deg = None
        self.payloads = {}
        self.invalid = 0
        self.conflicts = 0

    @property
    def received(self):
        return len(self.payloads)

    def set_lpf_info(self, payload):
        if len(payload) != 8:
            raise ValueError("0x44 must be 8 bytes")
        self.cutoff_hz, self.position_cmd_deg = struct.unpack(">ff", payload)

    def add(self, payload):
        if len(payload) != 8:
            self.invalid += 1
            return
        condition, channel, index = struct.unpack(">BBH", payload[:4])
        if (condition >= len(CONDITION_NAMES) or channel >= self.config.channels
                or index >= self.config.samples):
            self.invalid += 1
            return
        key = (condition, index, channel)  # 정상 순서: 조건 → 인덱스 → 채널
        previous = self.payloads.get(key)
        if previous is None:
            self.payloads[key] = bytes(payload)
        elif previous != payload:
            self.conflicts += 1

    def finish(self, token, checksum):
        problems = []
        if self.invalid:
            problems.append(f"범위 밖 프레임 {self.invalid}개")
        if self.conflicts:
            problems.append(f"같은 항목에 다른 값 {self.conflicts}개")
        missing = self.config.total_frames - self.received
        if missing:
            per_condition = [self.config.channels * self.config.samples
                             - sum(1 for key in self.payloads if key[0] == c)
                             for c in range(len(CONDITION_NAMES))]
            detail = ", ".join(f"{name} {count}" for name, count in zip(CONDITION_NAMES, per_condition))
            problems.append(f"누락 {missing}/{self.config.total_frames}개 ({detail})")
        if problems:
            raise ValueError(", ".join(problems))
        cfg = self.config
        ordered = [self.payloads[(c, i, ch)] for c in range(len(CONDITION_NAMES))
                   for i in range(cfg.samples) for ch in range(cfg.channels)]
        computed = fnv1a(ordered)
        if computed != checksum:
            raise ValueError(f"체크섬 불일치: END 0x{checksum:08X}, 계산 0x{computed:08X}")
        data = np.empty((len(CONDITION_NAMES), cfg.channels, cfg.samples), dtype=np.float64)
        for (condition, index, channel), payload in self.payloads.items():
            data[condition, channel, index] = struct.unpack(">f", payload[4:])[0]
        return LpfCapture(self.driver_id, token, cfg, self.cutoff_hz,
                          self.position_cmd_deg, checksum, data)


@dataclass
class _Pending:
    token: int
    sent_at: float
    queries: int = 0


@dataclass
class _Active:
    token: int
    since: float
    last_progress: float
    next_query: float = 0.0
    phase: int = 1
    sending_seen_at: Optional[float] = None
    complete_at: Optional[float] = None
    cancelled: bool = False


class LpfClient:
    """Session state machine for one driver. Not thread-safe: one owner thread.

    send(eid, payload) transmits one frame; emit(kind, value) reports
    kind in {status, state, progress, capture, notice, broadcast}.
    """
    START_REPLY_S = 0.3        # START 응답이 없으면 QUERY로 확인 (START 재전송 금지)
    START_QUERIES = 5
    MEASURE_POLL_S = 0.25      # 측정 단계 phase 표시용 QUERY
    MEASURE_LIMIT_S = 5.0      # 실제 측정은 약 142 ms
    META_GRACE_S = 0.5         # Sending인데 0x43이 없으면 누락으로 판단
    STALL_QUERY_S = 1.0
    STALL_TIMEOUT_S = 12.0     # 펌웨어는 10 s 정체 시 버퍼 반환
    END_GRACE_S = 1.5
    CANCEL_LIMIT_S = 3.0
    BROADCAST_PERIOD_S = 0.1
    PROGRESS_PERIOD_S = 0.05

    def __init__(self, driver_id, send, emit, token_seed=None):
        if not isinstance(driver_id, int) or not 0 <= driver_id <= 0xFF:
            raise ValueError("Driver ID must be 0..255")
        self.driver_id, self._send_frame, self.emit = driver_id, send, emit
        self._token = (secrets.randbits(32) if token_seed is None else token_seed) & 0xFFFFFFFF
        self.pending: Optional[_Pending] = None
        self.active: Optional[_Active] = None
        self.assembler: Optional[CaptureAssembler] = None
        self._broadcast_at = -math.inf
        self._progress_at = -math.inf
        self._state_name = None
        self.bus_errors = 0        # 현재 세션 동안 수신한 CAN 오류 프레임(bus error) 수

    def note_bus_error(self):
        if self.busy:
            self.bus_errors += 1

    def _bus_error_text(self):
        if not self.bus_errors:
            return ""
        return (f" [이 세션 중 CAN bus error {self.bus_errors}회 — 펌웨어 FDCAN이 자동 재전송 OFF면 "
                f"오류 난 프레임은 그대로 손실됩니다. 5M data phase 타이밍/TDC 확인]")

    # ── 상태 ──
    @property
    def state(self):
        if self.pending is not None:
            return "starting"
        if self.active is None:
            return "idle"
        if self.active.cancelled:
            return "cancelling"
        return "measuring" if self.assembler is None else "receiving"

    @property
    def busy(self):
        return self.pending is not None or self.active is not None

    def current_token(self):
        session = self.pending or self.active
        return None if session is None else session.token

    def _publish_state(self):
        state = self.state
        if state != self._state_name:
            self._state_name = state
            self.emit("state", state)

    def _next_token(self):
        self._token = (self._token + 1) & 0xFFFFFFFF or 1
        return self._token

    def _send(self, mode, payload):
        self._send_frame(build_eid(mode, self.driver_id), payload)

    def _query(self):
        try:
            self._send(MODE_REQUEST, pack_request(CMD_QUERY))
        except Exception as exc:
            self.emit("notice", f"QUERY 송신 실패: {exc}")

    def _close(self, message, cancel=False):
        if cancel:
            try:
                self._send(MODE_REQUEST, pack_request(CMD_CANCEL))
            except Exception as exc:
                message += f" (CANCEL 송신 실패: {exc})"
        self.pending = self.active = self.assembler = None
        self.emit("notice", message + self._bus_error_text())
        self._publish_state()

    # ── 사용자 요청 ──
    def start(self, now):
        if self.busy:
            raise RuntimeError("이미 측정 세션이 진행 중입니다")
        token = self._next_token()
        self.bus_errors = 0
        self._send(MODE_REQUEST, pack_request(CMD_START, token))
        self.pending = _Pending(token, now)
        self.emit("notice", f"START 송신: token {token} (0x{token:08X})")
        self._publish_state()

    def query(self):
        self._send(MODE_REQUEST, pack_request(CMD_QUERY))

    def cancel(self, now):
        self._send(MODE_REQUEST, pack_request(CMD_CANCEL))
        session = self.pending or self.active
        if session is None:
            self.emit("notice", "CANCEL 송신 (진행 중인 앱 세션 없음)")
            return
        self.pending, self.assembler = None, None
        self.active = _Active(session.token, now, now, cancelled=True)
        self.emit("notice", "CANCEL 송신 — 부분 데이터 폐기, Aborted 응답 대기")
        self._publish_state()

    def send_position(self, position_deg, i_ff_a=0.0):
        self._send(MODE_POSITION, pack_position_payload(position_deg, i_ff_a))

    # ── 수신 ──
    def handle(self, eid, data, now):
        mode, driver = (eid >> 8) & 0xFFFFFF, eid & 0xFF
        if driver != self.driver_id or len(data) != 8:
            return
        try:
            if mode == 0:
                if now - self._broadcast_at >= self.BROADCAST_PERIOD_S:
                    self._broadcast_at = now
                    self.emit("broadcast", (now, parse_broadcast_frame(data)))
            elif mode == MODE_STATUS:
                self._on_status(LpfStatus.decode(data), now)
            elif mode == MODE_CONFIG:
                self._on_config(data, now)
            elif mode == MODE_LPF_INFO:
                if self.assembler is not None:
                    self.assembler.set_lpf_info(data)
                    self._touch(now)
            elif mode == MODE_SAMPLE:
                self._on_sample(data, now)
            elif mode == MODE_END:
                self._on_end(data, now)
        except ValueError as exc:
            if self.busy and mode in (MODE_STATUS, MODE_CONFIG, MODE_LPF_INFO):
                self._close(f"측정 실패: 0x{mode:02X} 해석 오류 — {exc}", cancel=True)
            else:
                self.emit("notice", f"0x{mode:02X} 무시: {exc}")
        self._publish_state()

    def _touch(self, now):
        if self.active is not None:
            self.active.last_progress = now

    def _receiving(self):
        return self.pending is not None or (self.active is not None and not self.active.cancelled)

    def _on_status(self, status, now):
        self.emit("status", status)
        pending, active = self.pending, self.active
        if pending is not None and status.token == pending.token:
            self.pending = None
            if status.code in (RESULT_ACCEPTED, RESULT_COMPLETE):
                self.active = _Active(pending.token, now, now)
                self.emit("notice", f"Accepted: token {pending.token} — 측정 진행")
                self._on_active_status(status, now)
            else:
                self.assembler = None
                hint = RESULT_HINTS.get(status.code, "")
                self.emit("notice", f"START 거절: {status.code_name}" + (f" — {hint}" if hint else ""))
        elif active is not None and status.token == active.token:
            self._on_active_status(status, now)
        elif pending is not None and pending.queries:
            # QUERY는 기존 세션 token을 반영한다. 우리 token이 아니면 START가 처리되지 않았다.
            if status.code == RESULT_ACCEPTED:
                reason = f"다른 세션(token {status.token}, {status.phase_name}) 진행 중"
            else:
                reason = f"START 미수신 (드라이버 마지막 결과 {status.code_name}, token {status.token})"
            self._close(f"측정 시작 실패: {reason} — 다시 START 하세요")

    def _on_active_status(self, status, now):
        active = self.active
        active.last_progress, active.phase = now, status.phase
        if active.cancelled:
            if status.code != RESULT_ACCEPTED:
                self._close(f"취소 완료 ({status.code_name})")
            return
        if status.code == RESULT_ABORTED:
            done = f" ({self.assembler.received}개 수신 후)" if self.assembler else ""
            self._close(f"펌웨어 측정 중단 (Aborted){done} — 모드/설정 변경·오류·전송 정체. 세션 폐기")
        elif status.code == RESULT_COMPLETE:
            if active.complete_at is None:
                active.complete_at = now
        elif status.code == RESULT_ACCEPTED:
            if status.phase == PHASE_SENDING and active.sending_seen_at is None:
                active.sending_seen_at = now
        else:
            self._close(f"예상하지 못한 상태 {status.code_name}/{status.phase_name} — 세션 폐기")

    def _on_config(self, data, now):
        if not self._receiving():
            return
        config = CaptureConfig.decode(data)
        if self.assembler is not None and self.assembler.received:
            self.emit("notice", "새 0x43 수신 — 이전 부분 데이터 폐기")
        self.assembler = CaptureAssembler(self.driver_id, config)
        self._touch(now)
        self.emit("notice", f"수신 시작: {config.channels}ch × {config.samples} × 2조건, "
                            f"{config.sample_hz} Hz, 대기 {config.wait_ms} ms")
        self.emit("progress", (0, config.total_frames))

    def _on_sample(self, data, now):
        assembler = self.assembler
        if assembler is None:
            if self.active is not None and not self.active.cancelled:
                self._close("측정 실패: 0x43 수집 설정 프레임 누락 — 다시 측정하세요", cancel=True)
            return
        assembler.add(data)
        self._touch(now)
        total = assembler.config.total_frames
        if now - self._progress_at >= self.PROGRESS_PERIOD_S or assembler.received == total:
            self._progress_at = now
            self.emit("progress", (assembler.received, total))

    def _on_end(self, data, now):
        assembler = self.assembler
        if assembler is None:
            return
        token, checksum = struct.unpack(">II", data)
        expected = self.current_token()
        if token != expected:
            self.assembler = None
            self.emit("notice", f"다른 세션의 END(token {token}) — 수신 데이터 폐기")
            return
        self.emit("progress", (assembler.received, assembler.config.total_frames))
        try:
            capture = assembler.finish(token, checksum)
        except ValueError as exc:
            self._close(f"측정 실패 — 불완전한 데이터: {exc}. 새 START로 다시 측정하세요")
            return
        self.pending = self.active = self.assembler = None
        extra = "" if capture.cutoff_hz is not None else " (0x44 누락: 컷오프/위치 정보 없음)"
        if self.bus_errors:
            extra += f" (CAN bus error {self.bus_errors}회 있었으나 데이터 손실 없음)"
        self.emit("notice", f"수신 완료: token {token}, 체크섬 0x{checksum:08X} 일치{extra}")
        self.emit("capture", capture)

    # ── 주기 처리 ──
    def tick(self, now):
        pending = self.pending
        if pending is not None and now - pending.sent_at >= self.START_REPLY_S * (pending.queries + 1):
            if pending.queries >= self.START_QUERIES:
                self._close("START 응답 없음 — CAN 연결/드라이버 ID를 확인하세요")
            else:
                pending.queries += 1
                self._query()
        active = self.active
        if active is None:
            self._publish_state()
            return
        idle = now - active.last_progress
        if active.cancelled:
            if now - active.since > self.CANCEL_LIMIT_S:
                self._close("취소 응답(Aborted) 미수신 — QUERY로 드라이버 상태를 확인하세요")
            elif now >= active.next_query and idle > 0.5:
                active.next_query = now + 0.5
                self._query()
        elif self.assembler is None:
            if active.complete_at is not None and now - active.complete_at > self.END_GRACE_S:
                self._close("측정 실패: Complete 응답을 받았지만 데이터 프레임 미수신 — 다시 측정하세요")
            elif active.sending_seen_at is not None and now - active.sending_seen_at > self.META_GRACE_S:
                self._close("측정 실패: Sending 상태지만 0x43 수집 설정 미수신 — 다시 측정하세요",
                            cancel=True)
            elif now - active.since > self.MEASURE_LIMIT_S:
                self._close(f"측정 단계가 {self.MEASURE_LIMIT_S:g} s 내 끝나지 않음 "
                            f"(마지막 phase {PHASE_NAMES.get(active.phase, active.phase)})", cancel=True)
            elif now >= active.next_query:
                active.next_query = now + self.MEASURE_POLL_S
                self._query()
        else:
            received = self.assembler.received
            total = self.assembler.config.total_frames
            if active.complete_at is not None and now - active.complete_at > self.END_GRACE_S:
                self._close(f"측정 실패: Complete 응답 후 END 미수신 ({received}/{total}) — 다시 측정하세요")
            elif idle > self.STALL_QUERY_S and now >= active.next_query:
                active.next_query = now + self.STALL_QUERY_S
                self._query()
        if self.active is not None and idle > self.STALL_TIMEOUT_S:
            self._close(f"타임아웃: {self.STALL_TIMEOUT_S:g} s 동안 진행 없음 — 세션 폐기")
        self._publish_state()


# ───────────────────────────── 분석 / 저장 ─────────────────────────────
def channel_stats(values):
    values = np.asarray(values, dtype=np.float64)
    finite = values[np.isfinite(values)]
    if finite.size == 0:
        return dict(mean=math.nan, std=math.nan, pp=math.nan, bad=values.size)
    return dict(mean=float(finite.mean()), std=float(finite.std()),
                pp=float(finite.max() - finite.min()), bad=int(values.size - finite.size))


def amplitude_spectrum(values, sample_hz):
    """Single-sided amplitude spectrum [A] of the mean-removed, Hann-windowed signal."""
    values = np.asarray(values, dtype=np.float64)
    values = np.where(np.isfinite(values), values, np.nan)
    values = np.nan_to_num(values - np.nanmean(values))
    window = np.hanning(values.size)
    amplitude = 2.0 * np.abs(np.fft.rfft(values * window)) / window.sum()
    return np.fft.rfftfreq(values.size, 1.0 / sample_hz), amplitude


def write_capture_csv(path, capture):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    cfg = capture.config
    cutoff = "unknown" if capture.cutoff_hz is None else f"{capture.cutoff_hz:.9g}"
    position = "unknown" if capture.position_cmd_deg is None else f"{capture.position_cmd_deg:.9g}"
    columns = ["index", "time_ms"] + [f"{cond}_{key}_A" for cond in ("on", "off") for key in CHANNEL_KEYS]
    with path.open("x", encoding="utf-8", newline="") as stream:  # 기존 파일 덮어쓰지 않음
        for line in (
            "LPF ON/OFF current capture (CAN protocol v1)",
            f"received_at={capture.received_at.isoformat(timespec='seconds')}",
            f"driver_id={capture.driver_id}",
            f"token={capture.token}",
            f"checksum=0x{capture.checksum:08X}",
            f"lpf_cutoff_hz={cutoff}",
            f"position_cmd_deg_at_buffer_ready={position}",
            f"samples_per_condition={cfg.samples}",
            f"sample_hz={cfg.sample_hz}",
            f"wait_ms_per_condition={cfg.wait_ms}",
            "channels: iq_cmd_raw=before LPF, iq_cmd_applied=to PI, iq_meas=direction*i_q (mechanical sign)",
        ):
            stream.write(f"# {line}\n")
        writer = csv.writer(stream)
        writer.writerow(columns)
        times = capture.time_ms
        for index in range(cfg.samples):
            row = [index, f"{times[index]:.4f}"]
            row += [f"{capture.data[cond, ch, index]:.9g}"
                    for cond in range(len(CONDITION_NAMES)) for ch in range(cfg.channels)]
            writer.writerow(row)
    return path


# ───────────────────────────── GUI ─────────────────────────────
try:
    import pyqtgraph as pg
    from PyQt5 import QtCore, QtWidgets
    from serial.tools import list_ports
except ImportError:  # 프로토콜 부분만 import하는 테스트/스크립트용
    QtCore = None

RECORD_DIR = Path(__file__).resolve().parent / "recordings" / "lpf_check"
CHANNEL_COLORS = ("#9aa5b1", "#ffb347", "#4fc3f7")
CONDITION_COLORS = ("#4fc3f7", "#ff7043")


if QtCore is not None:
    class LpfSession(QtCore.QThread):
        """Owns the CAN bus. GUI → enqueue(); thread → event(kind, value)."""
        event = QtCore.pyqtSignal(str, object)
        connected_changed = QtCore.pyqtSignal(bool)

        def __init__(self, interface, channel, driver_id, parent=None):
            super().__init__(parent)
            self.interface, self.channel, self.driver_id = interface, channel, driver_id
            self.commands = queue.Queue(maxsize=16)
            self.stop_event = threading.Event()
            self.error = None

        def enqueue(self, kind, argument=None):
            if self.stop_event.is_set():
                raise RuntimeError("CAN 세션 종료 중")
            try:
                self.commands.put_nowait((kind, argument))
            except queue.Full:
                raise RuntimeError("명령 큐가 가득 찼습니다") from None

        def close_session(self):
            self.stop_event.set()

        def run(self):
            bus = client = None
            try:
                bus = can.Bus(**connection_options(self.interface, self.channel, DEFAULT_BITRATE))

                def send(eid, payload):
                    # 송신은 항상 FD, BRS OFF(1M). 펌웨어 응답(FD+BRS) 수신과는 무관하다.
                    bus.send(can.Message(arbitration_id=eid, is_extended_id=True, is_fd=True,
                                         bitrate_switch=False, data=bytes(payload), check=True),
                             timeout=0.02)

                client = LpfClient(self.driver_id, send, self.event.emit)
                self.connected_changed.emit(True)
                self.event.emit("state", client.state)
                while not self.stop_event.is_set():
                    while True:
                        try:
                            kind, argument = self.commands.get_nowait()
                        except queue.Empty:
                            break
                        self._execute(client, kind, argument)
                    message = bus.recv(timeout=0.005)
                    now = time.monotonic()
                    if message is not None:
                        if message.is_error_frame:
                            if message.arbitration_id & 0x40:  # CAN_ERR_BUSOFF
                                raise can.CanOperationError("CAN BUS-OFF — can0 복구 및 타이밍/ACK/TDC 확인")
                            if message.arbitration_id & 0x88:  # CAN_ERR_PROT | CAN_ERR_BUSERROR
                                client.note_bus_error()
                        elif (message.is_extended_id and not message.is_remote_frame
                              and getattr(message, "is_rx", True)):
                            client.handle(message.arbitration_id, bytes(message.data), now)
                    client.tick(now)
            except Exception as exc:
                self.error = str(exc)
                self.event.emit("notice", f"CAN 연결/통신 실패: {exc}")
            finally:
                if client is not None and client.busy and bus is not None:
                    try:
                        client.cancel(time.monotonic())
                        self.event.emit("notice", "연결 해제 전 CANCEL 송신")
                    except Exception as exc:
                        self.event.emit("notice", f"CANCEL 송신 실패: {exc}")
                if bus is not None:
                    try:
                        bus.shutdown()
                    except Exception as exc:
                        self.error = self.error or str(exc)
                self.connected_changed.emit(False)

        def _execute(self, client, kind, argument):
            try:
                if kind == "start":
                    client.start(time.monotonic())
                elif kind == "query":
                    client.query()
                    self.event.emit("notice", "QUERY 송신")
                elif kind == "cancel":
                    client.cancel(time.monotonic())
                elif kind == "position":
                    position, i_ff = argument
                    client.send_position(position, i_ff)
                    self.event.emit("notice", f"위치 명령 송신 (0x05): {position:.2f} deg, i_ff {i_ff:.3f} A")
            except (can.CanError, OSError, RuntimeError, ValueError) as exc:
                self.event.emit("notice", f"{kind} 실패: {exc}")

    class LpfCheckWindow(QtWidgets.QMainWindow):
        def __init__(self):
            super().__init__()
            self.setWindowTitle("MA732 — LPF ON/OFF Current Check")
            self.resize(1200, 1000)
            self.worker = None
            self.connected = False
            self.session_state = "idle"
            self.capture = None
            self.capture_saved = None
            self._closing = False
            self.settings = QtCore.QSettings("MA732", "LPFCheck")
            self._build_ui()
            self.interface.setCurrentText(DEFAULT_INTERFACE)
            self.interface.currentTextChanged.connect(self._load_channel)
            self._load_channel()
            self.broadcast_timer = QtCore.QTimer(self)
            self.broadcast_timer.timeout.connect(self._age_broadcast)
            self.broadcast_timer.start(500)
            self._last_broadcast = None
            self._sync_controls()

        # ── UI ──
        def _build_ui(self):
            central = QtWidgets.QWidget()
            self.setCentralWidget(central)
            layout = QtWidgets.QVBoxLayout(central)

            config = QtWidgets.QGroupBox("CAN — Extended FD, EID = (mode << 8) | driver_id")
            grid = QtWidgets.QGridLayout(config)
            self.interface = QtWidgets.QComboBox()
            self.interface.addItems(["socketcan", "slcan"])
            self.channel = QtWidgets.QComboBox()
            self.channel.setEditable(True)
            self.refresh = QtWidgets.QPushButton("포트 새로고침")
            self.refresh.clicked.connect(self._refresh_channels)
            self.driver_id = QtWidgets.QSpinBox()
            self.driver_id.setRange(0, 255)
            self.driver_id.setValue(int(self.settings.value("driver_id", 0)))
            self.connect_button = QtWidgets.QPushButton("Connect")
            self.connect_button.clicked.connect(self._connect)
            self.disconnect_button = QtWidgets.QPushButton("Disconnect")
            self.disconnect_button.clicked.connect(self._disconnect)
            for column, (label, widget) in enumerate([("Interface", self.interface), ("Channel", self.channel),
                                                      ("Driver ID", self.driver_id)]):
                grid.addWidget(QtWidgets.QLabel(label), 0, column)
                grid.addWidget(widget, 1, column)
            grid.addWidget(self.refresh, 1, 3)
            grid.addWidget(self.connect_button, 1, 4)
            grid.addWidget(self.disconnect_button, 1, 5)
            self.live_label = QtWidgets.QLabel("상태 방송: -")
            grid.addWidget(self.live_label, 2, 0, 1, 6)
            layout.addWidget(config)

            position = QtWidgets.QGroupBox("1) 위치 모드 유지 (mode 0x05)")
            row = QtWidgets.QHBoxLayout(position)
            self.position_deg = QtWidgets.QDoubleSpinBox()
            self.position_deg.setRange(-3000.0, 3000.0)
            self.position_deg.setDecimals(2)
            self.position_deg.setSuffix(" deg")
            self.position_deg.setValue(float(self.settings.value("position_deg", 0.0)))
            self.i_ff = QtWidgets.QDoubleSpinBox()
            self.i_ff.setRange(-10.0, 10.0)
            self.i_ff.setDecimals(3)
            self.i_ff.setSingleStep(0.01)
            self.i_ff.setSuffix(" A")
            self.position_button = QtWidgets.QPushButton("위치 명령 전송")
            self.position_button.clicked.connect(self._send_position)
            row.addWidget(QtWidgets.QLabel("위치"))
            row.addWidget(self.position_deg)
            row.addWidget(QtWidgets.QLabel("feedforward"))
            row.addWidget(self.i_ff)
            row.addWidget(self.position_button)
            row.addStretch(1)
            layout.addWidget(position)

            measure = QtWidgets.QGroupBox("2) LPF ON → OFF 비교 측정 (각 20 ms 대기 + 512 샘플 @ 10 kHz)")
            mgrid = QtWidgets.QGridLayout(measure)
            self.start_button = QtWidgets.QPushButton("측정 START")
            self.start_button.clicked.connect(lambda: self._enqueue("start"))
            self.query_button = QtWidgets.QPushButton("QUERY")
            self.query_button.clicked.connect(lambda: self._enqueue("query"))
            self.cancel_button = QtWidgets.QPushButton("CANCEL")
            self.cancel_button.clicked.connect(lambda: self._enqueue("cancel"))
            self.auto_save = QtWidgets.QCheckBox(f"완료 시 CSV 자동 저장 ({RECORD_DIR.parent.name}/{RECORD_DIR.name})")
            self.auto_save.setChecked(self.settings.value("auto_save", "true") in (True, "true"))
            self.save_button = QtWidgets.QPushButton("CSV 저장…")
            self.save_button.clicked.connect(self._save_as)
            self.load_button = QtWidgets.QPushButton("CSV 열기…")
            self.load_button.clicked.connect(self._load_csv)
            for column, widget in enumerate((self.start_button, self.query_button, self.cancel_button,
                                             self.auto_save, self.save_button, self.load_button)):
                mgrid.addWidget(widget, 0, column)
            self.session_label = QtWidgets.QLabel("세션: idle")
            self.session_label.setToolTip(measure.toolTip())
            self.firmware_label = QtWidgets.QLabel("펌웨어 응답: -")
            mgrid.addWidget(self.session_label, 1, 0, 1, 3)
            mgrid.addWidget(self.firmware_label, 1, 3, 1, 3)
            self.progress = QtWidgets.QProgressBar()
            self.progress.setRange(0, 3072)
            self.progress.setValue(0)
            self.progress.setFormat("%v / %m 프레임")
            mgrid.addWidget(self.progress, 2, 0, 1, 6)
            layout.addWidget(measure)

            tabs = QtWidgets.QTabWidget()
            tabs.addTab(self._build_time_tab(), "시간 영역")
            tabs.addTab(self._build_spectrum_tab(), "스펙트럼")
            tabs.addTab(self._build_stats_tab(), "통계")
            layout.addWidget(tabs, 1)

            self.summary = QtWidgets.QLabel("측정 결과 없음")
            self.summary.setWordWrap(True)
            self.summary.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            layout.addWidget(self.summary)
            measure.setToolTip(
                "두 조건은 순차적으로 실제 제어에 적용됩니다(같은 입력의 오프라인 비교 아님).\n"
                "측정 중 위치·부하·게인·feedforward를 일정하게 유지하세요.\n"
                "모드/게인/코깅/Flash 설정 변경은 측정을 중단합니다.\n"
                "다운샘플 전 anti-alias 필터가 없으므로 5 kHz 부근 성분은 해석에 주의하세요.")
            self.log = QtWidgets.QPlainTextEdit()
            self.log.setReadOnly(True)
            self.log.setMaximumBlockCount(300)
            self.log.setMaximumHeight(80)
            layout.addWidget(self.log)

        def _build_time_tab(self):
            widget = QtWidgets.QWidget()
            layout = QtWidgets.QVBoxLayout(widget)
            toggles = QtWidgets.QHBoxLayout()
            self.channel_checks = []
            for channel, name in enumerate(CHANNEL_NAMES):
                check = QtWidgets.QCheckBox(f"ch{channel} {name}")
                check.setChecked(True)
                check.toggled.connect(self._apply_channel_visibility)
                self.channel_checks.append(check)
                toggles.addWidget(check)
            self.link_y = QtWidgets.QCheckBox("ON/OFF Y축 연동")
            self.link_y.setChecked(True)
            self.link_y.toggled.connect(self._apply_y_link)
            toggles.addWidget(self.link_y)
            toggles.addStretch(1)
            layout.addLayout(toggles)
            self.time_plots, self.time_curves = [], []
            for condition, name in enumerate(CONDITION_NAMES):
                plot = pg.PlotWidget(background="#17202b", title=name)
                plot.setMinimumHeight(170)
                plot.setLabel("bottom", "Time", units="ms")
                plot.setLabel("left", "i_q", units="A")
                plot.showGrid(x=True, y=True, alpha=0.25)
                plot.addLegend(offset=(-10, 10))
                curves = [plot.plot(name=f"ch{ch} {label}", pen=pg.mkPen(CHANNEL_COLORS[ch], width=1.3),
                                    connect="finite")
                          for ch, label in enumerate(CHANNEL_NAMES)]
                self.time_plots.append(plot)
                self.time_curves.append(curves)
                layout.addWidget(plot, 1)
            self.time_plots[1].setXLink(self.time_plots[0])
            self._apply_y_link(True)
            return widget

        def _build_spectrum_tab(self):
            widget = QtWidgets.QWidget()
            layout = QtWidgets.QVBoxLayout(widget)
            controls = QtWidgets.QHBoxLayout()
            controls.addWidget(QtWidgets.QLabel("채널"))
            self.spectrum_channel = QtWidgets.QComboBox()
            self.spectrum_channel.addItems([f"ch{ch} {name}" for ch, name in enumerate(CHANNEL_NAMES)])
            self.spectrum_channel.setCurrentIndex(2)
            self.spectrum_channel.currentIndexChanged.connect(self._plot_spectrum)
            controls.addWidget(self.spectrum_channel)
            self.spectrum_log_x = QtWidgets.QCheckBox("주파수 로그축")
            self.spectrum_log_x.setChecked(True)
            self.spectrum_log_x.toggled.connect(self._plot_spectrum)
            controls.addWidget(self.spectrum_log_x)
            controls.addWidget(QtWidgets.QLabel("평균 제거 + Hann 창, 단측 진폭 [A]. 해상도 ≈ 19.5 Hz"))
            controls.addStretch(1)
            layout.addLayout(controls)
            self.spectrum_plot = pg.PlotWidget(background="#17202b")
            self.spectrum_plot.setLabel("bottom", "Frequency", units="Hz")
            self.spectrum_plot.setLabel("left", "Amplitude [A]")
            self.spectrum_plot.showGrid(x=True, y=True, alpha=0.25)
            self.spectrum_plot.addLegend(offset=(10, 10))
            self.spectrum_curves = [self.spectrum_plot.plot(name=name, pen=pg.mkPen(color, width=1.3))
                                    for name, color in zip(CONDITION_NAMES, CONDITION_COLORS)]
            self.cutoff_line = pg.InfiniteLine(angle=90, movable=False,
                                               pen=pg.mkPen("#d3f26a", width=1, style=QtCore.Qt.DashLine),
                                               label="fc", labelOpts=dict(color="#d3f26a", position=0.95))
            self.cutoff_line.setVisible(False)
            self.spectrum_plot.addItem(self.cutoff_line)
            layout.addWidget(self.spectrum_plot, 1)
            return widget

        def _build_stats_tab(self):
            self.stats_table = QtWidgets.QTableWidget(6, 5)
            self.stats_table.setHorizontalHeaderLabels(
                ["평균 [A]", "표준편차 [mA]", "p-p [mA]", "비유한값", "std OFF/ON"])
            self.stats_table.setVerticalHeaderLabels(
                [f"{cond} · ch{ch} {name}" for cond in CONDITION_NAMES for ch, name in enumerate(CHANNEL_NAMES)])
            self.stats_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
            self.stats_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
            return self.stats_table

        # ── 연결 ──
        def _load_channel(self):
            interface = self.interface.currentText()
            default = DEFAULT_CHANNEL if interface == DEFAULT_INTERFACE else ""
            self.channel.setEditText(str(self.settings.value(f"{sys.platform}/{interface}/channel", default)))
            self._refresh_channels()

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
                self._log(f"포트 조회 실패: {exc}")

        def _sync_controls(self):
            free = self.worker is None and not self._closing
            for widget in (self.interface, self.channel, self.refresh, self.driver_id):
                widget.setEnabled(free)
            self.connect_button.setEnabled(free)
            ready = self.connected and not self._closing
            idle = ready and self.session_state == "idle"
            self.disconnect_button.setEnabled(ready)
            self.position_button.setEnabled(ready)
            self.start_button.setEnabled(idle)
            self.query_button.setEnabled(ready)
            self.cancel_button.setEnabled(ready and self.session_state != "cancelling")
            self.save_button.setEnabled(self.capture is not None)
            self.load_button.setEnabled(self.session_state in ("idle",) or not self.connected)
            labels = {"idle": "대기", "starting": "START 응답 대기", "measuring": "측정 중 (LPF ON → OFF)",
                      "receiving": "데이터 수신 중", "cancelling": "취소 중"}
            self.session_label.setText(f"세션: {labels.get(self.session_state, self.session_state)}")

        def _connect(self):
            if self.worker is not None:
                return
            channel = self.channel.currentText().strip()
            if not channel:
                self._log("CAN 채널/시리얼 포트를 입력하세요.")
                return
            interface = self.interface.currentText()
            self.settings.setValue(f"{sys.platform}/{interface}/channel", channel)
            self.settings.setValue("driver_id", self.driver_id.value())
            self.settings.remove("tx_brs")  # 이전 버전의 TX BRS 설정 정리
            self.worker = LpfSession(interface, channel, self.driver_id.value(), self)
            self.worker.event.connect(self._on_event)
            self.worker.connected_changed.connect(self._connected_changed)
            self.worker.finished.connect(self._finished)
            self._log(f"연결 중: {interface}/{channel}, driver {self.driver_id.value()}, "
                      "TX FD (BRS off)")
            self._sync_controls()
            self.worker.start()

        def _disconnect(self):
            if self.worker is not None:
                self.worker.close_session()
                self.connected = False
                self._sync_controls()

        def _connected_changed(self, connected):
            self.connected = connected
            if connected:
                self._log("CAN 연결됨 — 상태 방송 수신 대기")
            self._sync_controls()

        def _finished(self):
            worker, self.worker = self.worker, None
            self.connected, self.session_state = False, "idle"
            self._log("CAN 연결 해제" + (f": {worker.error}" if worker.error else ""))
            worker.deleteLater()
            self._sync_controls()
            if self._closing:
                QtCore.QTimer.singleShot(0, self.close)

        def _enqueue(self, kind, argument=None):
            if self.worker is None or not self.connected:
                return
            try:
                self.worker.enqueue(kind, argument)
            except RuntimeError as exc:
                self._log(f"요청 실패: {exc}")

        def _send_position(self):
            self.settings.setValue("position_deg", self.position_deg.value())
            self._enqueue("position", (self.position_deg.value(), self.i_ff.value()))

        # ── 이벤트 ──
        def _on_event(self, kind, value):
            if kind == "notice":
                self._log(value)
            elif kind == "state":
                self.session_state = value
                if value == "starting":
                    self.progress.setValue(0)
                self._sync_controls()
            elif kind == "status":
                self.firmware_label.setText(f"펌웨어 응답: {value.code_name} / phase {value.phase_name} / "
                                            f"token {value.token}")
            elif kind == "progress":
                done, total = value
                self.progress.setRange(0, total)
                self.progress.setValue(done)
            elif kind == "broadcast":
                self._last_broadcast = value
                self._show_broadcast()
            elif kind == "capture":
                self._show_capture(value)
                if self.auto_save.isChecked():
                    self._auto_save(value)

        def _show_broadcast(self, stale=False):
            if self._last_broadcast is None:
                return
            stamp, state = self._last_broadcast
            age = time.monotonic() - stamp
            error = state["error"]
            text = (f"상태 방송: 위치 {state['pos_deg']:.1f} deg · 전류 {state['cur_A']:.3f} A · "
                    f"속도 {state['spd_erpm']:.0f} eRPM · 온도 {state['temp_C']} °C · 오류 0x{error:02X}")
            if error:
                text += "  ⚠ 오류 활성 — START 거절됨"
            if stale or age > 1.0:
                text += f"  (최근 수신 {age:.1f} s 전)"
            self.live_label.setText(text)

        def _age_broadcast(self):
            self._show_broadcast(stale=True)

        # ── 결과 표시 ──
        def _show_capture(self, capture):
            self.capture = capture
            self.capture_saved = None
            times = capture.time_ms
            for condition, curves in enumerate(self.time_curves):
                for channel, curve in enumerate(curves):
                    curve.setData(times, capture.data[condition, channel], connect="finite")
            self._apply_channel_visibility()
            self._plot_spectrum()
            self._fill_stats()
            self.progress.setRange(0, capture.config.total_frames)
            self.progress.setValue(capture.config.total_frames)
            self._sync_controls()

        def _apply_channel_visibility(self, *_):
            for curves in self.time_curves:
                for check, curve in zip(self.channel_checks, curves):
                    curve.setVisible(check.isChecked())
            self._fit_time_plots()

        def _apply_y_link(self, linked):
            self.time_plots[1].setYLink(self.time_plots[0] if linked else None)
            self._fit_time_plots()

        def _fit_time_plots(self):
            """Linked Y: one range covering visible channels of both conditions."""
            for plot in self.time_plots:
                plot.enableAutoRange()
            capture = self.capture
            if capture is None or not self.link_y.isChecked():
                return
            visible = [ch for ch, check in enumerate(self.channel_checks) if check.isChecked()]
            values = capture.data[:, visible, :] if visible else np.empty(0)
            values = values[np.isfinite(values)]
            if values.size:
                self.time_plots[0].setYRange(float(values.min()), float(values.max()), padding=0.05)

        def _plot_spectrum(self, *_):
            capture = self.capture
            log_x = self.spectrum_log_x.isChecked()
            self.spectrum_plot.setLogMode(x=log_x, y=True)
            if capture is None:
                return
            channel = self.spectrum_channel.currentIndex()
            for condition, curve in enumerate(self.spectrum_curves):
                freqs, amplitude = amplitude_spectrum(capture.data[condition, channel], capture.config.sample_hz)
                amplitude = np.maximum(amplitude[1:], 1e-9)  # DC 제외, 로그축용 하한
                curve.setData(freqs[1:], amplitude)
            if capture.cutoff_hz is not None and capture.cutoff_hz > 0:
                self.cutoff_line.setValue(math.log10(capture.cutoff_hz) if log_x else capture.cutoff_hz)
                self.cutoff_line.label.setFormat(f"fc {capture.cutoff_hz:g} Hz")
                self.cutoff_line.setVisible(True)
            else:
                self.cutoff_line.setVisible(False)
            self.spectrum_plot.enableAutoRange()

        def _fill_stats(self):
            capture = self.capture
            stats = [[channel_stats(capture.data[cond, ch]) for ch in range(capture.config.channels)]
                     for cond in range(len(CONDITION_NAMES))]

            def item(text):
                cell = QtWidgets.QTableWidgetItem(text)
                cell.setTextAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
                return cell

            for cond in range(len(CONDITION_NAMES)):
                for ch in range(capture.config.channels):
                    row = cond * capture.config.channels + ch
                    s = stats[cond][ch]
                    ratio = stats[1][ch]["std"] / stats[0][ch]["std"] if stats[0][ch]["std"] > 0 else math.nan
                    values = (f"{s['mean']:.4f}", f"{s['std'] * 1e3:.2f}", f"{s['pp'] * 1e3:.2f}",
                              str(s["bad"]), f"{ratio:.2f}" if cond == 0 else "")
                    for column, text in enumerate(values):
                        self.stats_table.setItem(row, column, item(text))

            on, off = stats
            parts = [f"Driver {capture.driver_id}, token {capture.token}, "
                     f"{capture.received_at:%H:%M:%S}",
                     "컷오프 " + ("?" if capture.cutoff_hz is None else f"{capture.cutoff_hz:g} Hz"),
                     "위치 명령 " + ("?" if capture.position_cmd_deg is None else f"{capture.position_cmd_deg:.2f} deg")]
            lines = [" · ".join(parts)]
            lines.append(f"실측 i_q 리플(std): ON {on[2]['std'] * 1e3:.2f} mA → OFF {off[2]['std'] * 1e3:.2f} mA "
                         f"(OFF/ON {self._ratio(off[2]['std'], on[2]['std'])})  |  "
                         f"적용 i_q* std: ON {on[1]['std'] * 1e3:.2f} mA, OFF {off[1]['std'] * 1e3:.2f} mA")
            lines.append(f"LPF ON 구간 필터 감쇠(적용/필터 전 std): {self._ratio(on[1]['std'], on[0]['std'])}  |  "
                         f"평균 실측 i_q: ON {on[2]['mean']:.4f} A, OFF {off[2]['mean']:.4f} A")
            bad = sum(s["bad"] for row in stats for s in row)
            if bad:
                lines.append(f"⚠ NaN/Inf {bad}개 (통계에서 제외)")
            self.summary.setText("\n".join(lines))

        @staticmethod
        def _ratio(numerator, denominator):
            return f"{numerator / denominator:.2f}×" if denominator > 0 else "-"

        # ── 저장/불러오기 ──
        def _default_name(self, capture):
            return f"lpf_driver{capture.driver_id}_{capture.received_at:%Y%m%d_%H%M%S}_tok{capture.token}.csv"

        def _auto_save(self, capture):
            try:
                path = write_capture_csv(RECORD_DIR / self._default_name(capture), capture)
            except Exception as exc:
                self._log(f"자동 저장 실패: {exc}")
            else:
                self.capture_saved = path
                self._log(f"저장: {path}")

        def _save_as(self):
            if self.capture is None:
                return
            start = RECORD_DIR / self._default_name(self.capture)
            path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "CSV 저장", str(start), "CSV (*.csv)")
            if not path:
                return
            try:
                if Path(path).exists():
                    Path(path).unlink()  # 대화상자에서 덮어쓰기를 이미 확인함
                write_capture_csv(path, self.capture)
            except Exception as exc:
                self._log(f"저장 실패: {exc}")
            else:
                self._log(f"저장: {path}")

        def _load_csv(self):
            path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "CSV 열기", str(RECORD_DIR), "CSV (*.csv)")
            if not path:
                return
            try:
                capture = read_capture_csv(path)
            except Exception as exc:
                self._log(f"불러오기 실패: {exc}")
                return
            self._show_capture(capture)
            self._log(f"불러옴: {path}")

        def _log(self, message):
            self.log.appendPlainText(f"[{datetime.now():%H:%M:%S}] {message}")

        def closeEvent(self, event):
            self.settings.setValue("auto_save", self.auto_save.isChecked())
            if self.worker is not None:
                self._closing = True
                self._disconnect()
                self._sync_controls()
                event.ignore()
            else:
                event.accept()


def read_capture_csv(path):
    """Load a CSV written by write_capture_csv for re-display."""
    meta, rows = {}, []
    with Path(path).open(encoding="utf-8", newline="") as stream:
        lines = []
        for line in stream:
            if line.startswith("#"):
                key, sep, value = line[1:].strip().partition("=")
                if sep:
                    meta[key] = value
            else:
                lines.append(line)
    reader = csv.reader(lines)
    header = next(reader)
    if len(header) != 2 + len(CONDITION_NAMES) * len(CHANNEL_NAMES):
        raise ValueError("LPF_check CSV 형식이 아닙니다")
    for row in reader:
        if row:
            rows.append([float(value) for value in row[2:]])
    values = np.asarray(rows, dtype=np.float64)
    data = values.T.reshape(len(CONDITION_NAMES), len(CHANNEL_NAMES), len(rows))

    def number(key, cast=float):
        value = meta.get(key)
        return None if value in (None, "unknown") else cast(value)

    config = CaptureConfig(len(CHANNEL_NAMES), len(rows), number("sample_hz", int) or 10000,
                           number("wait_ms_per_condition", int) or 0)
    received = meta.get("received_at")
    return LpfCapture(number("driver_id", int) or 0, number("token", int) or 0, config,
                      number("lpf_cutoff_hz"), number("position_cmd_deg_at_buffer_ready"),
                      int(meta.get("checksum", "0x0"), 16), data,
                      datetime.fromisoformat(received) if received else datetime.now())


def main():
    if QtCore is None:
        raise SystemExit("PyQt5/pyqtgraph/pyserial이 필요합니다: python -m pip install -r tools/requirements.txt")
    app = QtWidgets.QApplication(sys.argv)
    window = LpfCheckWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
