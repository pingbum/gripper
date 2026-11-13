# -*- coding: utf-8 -*-
"""
안드로이드 폰 카메라 수신 (scrcpy-server v5.0.1, adb USB) - Qt 의존성 없음.

- 폰에서 scrcpy-server를 카메라 소스로 실행하고, 이 모듈이 직접 클라이언트가 되어 H.264 스트림을 받는다.
  (PC용 scrcpy 프로그램은 쓰지 않는다. 패킷마다 Jetson 시계로 수신 시각을 찍기 위해서다.)
- 프레임 시각: 폰 PTS(캡처 시각, us)를 Jetson monotonic 시계로 옮긴다.
    offset = min(수신 시각 - PTS)   (USB/adb 전송 지연이 가장 짧았던 프레임 기준, 흔들림 제거)
    capture_ns = PTS + offset
  남는 고정 지연(인코딩 등 최소 지연)은 동기화 마커로 측정해 meta.json의 video_offset_ms로 보정한다.
- 녹화: H.264를 다시 인코딩하지 않고 그대로 mp4에 넣는다 (appsrc ! h264parse ! mp4mux).
  Record 시점 직전 키프레임부터 쓰므로 t=0 이전 프레임(음수 t_ns)이 조금 포함될 수 있다.
- 미리보기: Jetson 하드웨어 디코더(nvv4l2decoder)로 작게 디코딩해 최신 프레임 1장만 보관한다.

단독 테스트: python3 phone_camera.py [녹화 초, 기본 5]
"""
import csv
import os
import socket
import struct
import subprocess
import threading
import time

import gi

gi.require_version("Gst", "1.0")
from gi.repository import Gst  # noqa: E402

Gst.init(None)

HERE = os.path.dirname(os.path.abspath(__file__))
SERVER_VERSION = "5.0.1"
SERVER_LOCAL = os.path.join(HERE, "scrcpy", f"scrcpy-server-v{SERVER_VERSION}")
SERVER_REMOTE = "/data/local/tmp/scrcpy-server.jar"
SCID = "2a3b4c5d"                     # 31-bit hex, 소켓 이름 scrcpy_<scid>
LOCAL_PORT = 27183
PHONE_CAMERA_APP = "com.sec.android.app.camera"  # 카메라를 잡고 있으면 서버가 열 수 없다

CAMERA_ID = "0"                       # 0 = 후면, 1 = 전면
CAMERA_SIZE = "1920x1080"
CAMERA_FPS = 30                       # S23+ 일반 모드 최대 30 fps (60 요청 시 30으로 동작 확인)
VIDEO_BIT_RATE = 16_000_000
I_FRAME_INTERVAL_S = 1                # Record 시작 시 앞에 붙는 프리롤 최대 길이
PREVIEW_SIZE = (640, 360)
CONNECT_TIMEOUT_S = 8.0

FLAG_SESSION = 1 << 63
FLAG_CONFIG = 1 << 62
FLAG_KEY = 1 << 61
PTS_MASK = (1 << 61) - 1


def adb(*args, timeout=10, check=True):
    return subprocess.run(["adb", *args], capture_output=True, text=True, timeout=timeout, check=check)


def _recv_exact(sock, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise EOFError("phone stream closed")
        buf += chunk
    return bytes(buf)


class Mp4Writer:
    """H.264 Annex B 패킷 → mp4 (재인코딩 없음). push는 수신 스레드에서만 부른다."""

    def __init__(self, path):
        self.path = path
        self.pipeline = Gst.parse_launch(
            "appsrc name=src is-live=true format=time "
            "caps=video/x-h264,stream-format=byte-stream,alignment=au "
            f"! h264parse ! mp4mux ! filesink location=\"{path}\"")
        self.src = self.pipeline.get_by_name("src")
        self.pipeline.set_state(Gst.State.PLAYING)

    def push(self, data, pts_ns):
        buf = Gst.Buffer.new_wrapped(data)
        buf.pts = buf.dts = pts_ns
        self.src.emit("push-buffer", buf)

    def close(self, timeout=5.0):
        """EOS를 보내 mp4 헤더(moov)를 마무리한다."""
        self.src.emit("end-of-stream")
        msg = self.pipeline.get_bus().timed_pop_filtered(
            int(timeout * Gst.SECOND), Gst.MessageType.EOS | Gst.MessageType.ERROR)
        self.pipeline.set_state(Gst.State.NULL)
        if msg is None:
            return "mp4 마무리 시간 초과"
        if msg.type == Gst.MessageType.ERROR:
            return str(msg.parse_error()[0])
        return None


class PhoneCamera:
    """상태: stopped → starting → streaming (오류 시 error 메시지와 함께 stopped)."""

    def __init__(self, camera_id=CAMERA_ID, size=CAMERA_SIZE, fps=CAMERA_FPS, bit_rate=VIDEO_BIT_RATE):
        self.camera_id, self.size, self.fps, self.bit_rate = camera_id, size, fps, bit_rate
        self.lock = threading.Lock()
        self.state = "stopped"
        self.error = None
        self.notices = []
        self.device_name = None
        self.model = None
        self.android = None
        self.video_size = None
        self.sock = None
        self.server = None
        self.server_log = []
        self.thread = None
        self.stop_event = threading.Event()
        self.preview = None
        self.preview_src = None
        self.preview_sink = None
        self.config = None             # 마지막 SPS/PPS 패킷
        self.gop = []                  # 마지막 키프레임부터의 패킷 (pts_us, recv_ns, key, data)
        self.offset_ns = None          # min(수신 시각 - PTS)
        self.frame_count = 0
        self.rate_window = []          # 최근 수신 시각 (fps 계산)
        self.recording = None          # 녹화 상태 dict
        self.orphan = None             # 내부에서 먼저 닫힌 녹화 (stop_recording(frames_csv)가 가져감)

    # ------------- 연결 -------------
    def start(self):
        """백그라운드에서 연결한다 (adb 명령이 1~2초 걸려 GUI를 막지 않게)."""
        with self.lock:
            if self.state != "stopped":
                return
            self.state, self.error = "starting", None
        self.stop_event.clear()
        self.thread = threading.Thread(target=self._run, name="phone-camera", daemon=True)
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        if self.sock is not None:
            try:
                self.sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        if self.thread is not None:
            self.thread.join(timeout=5.0)
            self.thread = None

    def _notice(self, text):
        with self.lock:
            self.notices.append(text)

    def take_notices(self):
        with self.lock:
            notices, self.notices = self.notices, []
        return notices

    def _connect(self):
        devices = [l.split()[0] for l in adb("devices").stdout.splitlines()[1:] if l.strip().endswith("device")]
        if not devices:
            raise RuntimeError("adb 장치 없음 (USB 연결, USB 디버깅 허용 확인)")
        self.model = adb("shell", "getprop", "ro.product.model").stdout.strip()
        self.android = adb("shell", "getprop", "ro.build.version.release").stdout.strip()
        # 이전 실행에서 남은 서버, 카메라를 잡고 있는 기본 카메라 앱을 정리한다.
        adb("shell", "pkill", "-f", "com.genymobile.scrcpy", check=False)
        if adb("shell", "pidof", PHONE_CAMERA_APP, check=False).stdout.strip():
            adb("shell", "am", "force-stop", PHONE_CAMERA_APP, check=False)
            self._notice("폰 카메라 앱이 카메라를 쓰고 있어 종료했습니다.")
        # 서버는 cleanup 시 jar를 지우므로 매번 올린다 (약 0.7 MB).
        adb("push", SERVER_LOCAL, SERVER_REMOTE, timeout=30)
        adb("forward", f"tcp:{LOCAL_PORT}", f"localabstract:scrcpy_{SCID}")
        args = [f"scid={SCID}", "tunnel_forward=true", "audio=false", "control=false",
                "video_source=camera", f"camera_id={self.camera_id}", f"camera_size={self.size}",
                f"camera_fps={self.fps}", f"video_bit_rate={self.bit_rate}",
                f"video_codec_options=i-frame-interval:int={I_FRAME_INTERVAL_S}", "log_level=info"]
        self.server = subprocess.Popen(
            ["adb", "shell", f"CLASSPATH={SERVER_REMOTE}", "app_process", "/", "com.genymobile.scrcpy.Server",
             SERVER_VERSION, *args], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        threading.Thread(target=self._read_server_log, daemon=True).start()

        # adb forward는 서버가 아직 없어도 연결을 받으므로 dummy byte로 실제 연결을 확인한다.
        deadline = time.monotonic() + CONNECT_TIMEOUT_S
        while True:
            if self.stop_event.is_set():
                raise RuntimeError("취소됨")
            if self.server.poll() is not None:
                raise RuntimeError(self._server_error() or "scrcpy-server 종료")
            try:
                sock = socket.create_connection(("127.0.0.1", LOCAL_PORT), timeout=2.0)
                _recv_exact(sock, 1)
                break
            except (OSError, EOFError):
                if time.monotonic() > deadline:
                    raise RuntimeError(self._server_error() or "scrcpy-server 연결 시간 초과")
                time.sleep(0.1)
        sock.settimeout(5.0)  # 카메라 오류로 프레임이 끊기면 recv가 예외로 끝난다
        self.sock = sock
        self.device_name = _recv_exact(sock, 64).rstrip(b"\0").decode(errors="replace")
        codec, = struct.unpack(">I", _recv_exact(sock, 4))
        if codec != 0x68323634:
            raise RuntimeError(f"H.264가 아닌 코덱 0x{codec:08X}")

    def _read_server_log(self):
        for line in self.server.stdout:
            line = line.rstrip()
            self.server_log.append(line)
            del self.server_log[:-200]

    def _server_error(self):
        text = "\n".join(self.server_log)
        if "number of open cameras" in text or "Camera in use" in text:
            return "카메라가 다른 앱에서 사용 중 (폰의 카메라 앱 종료)"
        errors = [l for l in self.server_log if "ERROR" in l or "Exception" in l]
        return errors[-1] if errors else None

    def _cleanup(self):
        if self.sock is not None:
            self.sock.close()
            self.sock = None
        if self.server is not None:
            self.server.terminate()
            try:
                self.server.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self.server.kill()
            self.server = None
        for cmd in (("shell", "pkill", "-f", "com.genymobile.scrcpy"), ("forward", "--remove", f"tcp:{LOCAL_PORT}")):
            try:
                adb(*cmd, check=False, timeout=5)
            except Exception:
                pass
        if self.preview is not None:
            self.preview.set_state(Gst.State.NULL)
            self.preview = self.preview_src = self.preview_sink = None

    # ------------- 수신 스레드 -------------
    def _run(self):
        try:
            self._connect()
            self._start_preview()
            with self.lock:
                self.state = "streaming"
            self._loop()
        except Exception as exc:
            if not self.stop_event.is_set():
                with self.lock:
                    self.error = self._server_error() or str(exc)
        finally:
            self.stop_recording()
            self._cleanup()
            with self.lock:
                self.state = "stopped"
                self.config, self.gop, self.offset_ns, self.video_size = None, [], None, None

    def _start_preview(self):
        w, h = PREVIEW_SIZE
        # config-interval=-1 + queue가 없으면 라이브 appsrc에서 nvv4l2decoder가 출력을 내지 않는다 (실측).
        self.preview = Gst.parse_launch(
            "appsrc name=src is-live=true do-timestamp=true format=time "
            "caps=video/x-h264,stream-format=byte-stream,alignment=au "
            "! h264parse config-interval=-1 ! video/x-h264,stream-format=byte-stream,alignment=au "
            "! queue ! nvv4l2decoder enable-max-performance=1 ! nvvidconv "
            f"! video/x-raw,format=BGRx,width={w},height={h} "
            "! appsink name=sink max-buffers=1 drop=true sync=false")
        self.preview_src = self.preview.get_by_name("src")
        self.preview_sink = self.preview.get_by_name("sink")
        self.preview.set_state(Gst.State.PLAYING)

    def _loop(self):
        sock = self.sock
        while not self.stop_event.is_set():
            header = _recv_exact(sock, 12)
            recv_ns = time.monotonic_ns()  # 헤더 도착 시각 (패킷 크기에 덜 민감)
            flags_pts, size = struct.unpack(">QI", header)
            if flags_pts & FLAG_SESSION:
                self.video_size = struct.unpack(">II", header[4:])
                continue
            data = _recv_exact(sock, size)
            self.preview_src.emit("push-buffer", Gst.Buffer.new_wrapped(data))
            if flags_pts & FLAG_CONFIG:
                self.config = data
                continue
            pts_us, key = flags_pts & PTS_MASK, bool(flags_pts & FLAG_KEY)
            delta = recv_ns - pts_us * 1000
            if self.offset_ns is None or delta < self.offset_ns:
                self.offset_ns = delta
            self.frame_count += 1
            self.rate_window.append(recv_ns)
            del self.rate_window[:-61]
            packet = (pts_us, recv_ns, key, data)
            if key:
                self.gop = [packet]
            elif self.gop:
                self.gop.append(packet)
            with self.lock:
                rec = self.recording
                if rec is not None and rec["writer"] is not None:
                    self._write_packet(rec, packet)

    # ------------- 미리보기 -------------
    def latest_preview(self):
        """(BGRx bytes, width, height, stride) 또는 None. GUI 스레드에서 부른다."""
        sink = self.preview_sink
        if sink is None:
            return None
        sample = sink.emit("try-pull-sample", 0)
        if sample is None:
            return None
        buf = sample.get_buffer()
        st = sample.get_caps().get_structure(0)
        w, h = st.get_value("width"), st.get_value("height")
        ok, info = buf.map(Gst.MapFlags.READ)
        if not ok:
            return None
        try:
            data = bytes(info.data)
        finally:
            buf.unmap(info)
        return data, w, h, len(data) // h

    def measured_fps(self):
        times = self.rate_window
        if len(times) < 2:
            return None
        return (len(times) - 1) * 1e9 / (times[-1] - times[0])

    # ------------- 녹화 -------------
    def start_recording(self, video_path, t0_ns):
        """직전 키프레임부터 video_path에 쓴다. streaming 상태가 아니면 False."""
        with self.lock:
            if self.state != "streaming" or self.recording is not None or not self.gop or self.config is None:
                return False
            rec = {"writer": Mp4Writer(video_path), "t0_ns": t0_ns, "first_pts_us": self.gop[0][0],
                   "frames": [], "error": None}
            self.recording = rec
            for i, packet in enumerate(list(self.gop)):
                self._write_packet(rec, packet, prefix=self.config if i == 0 else b"")
        return True

    def _write_packet(self, rec, packet, prefix=b""):
        pts_us, recv_ns, key, data = packet
        try:
            rec["writer"].push(prefix + data, (pts_us - rec["first_pts_us"]) * 1000)
            rec["frames"].append((pts_us, recv_ns, key))
        except Exception as exc:
            rec["error"] = str(exc)

    def stop_recording(self, frames_csv=None):
        """녹화를 끝내고 정보를 돌려준다. frames_csv가 있으면 프레임별 시각을 쓴다.

        연결이 끊겨 내부에서 먼저 닫힌 경우(frames_csv 없이 호출)에는 mp4만 마무리하고 보관했다가,
        이후 frames_csv와 함께 호출되면 그 녹화의 정보를 돌려준다.
        """
        with self.lock:
            rec, self.recording = self.recording, None
            if rec is None and frames_csv is not None:
                rec, self.orphan = self.orphan, None
        if rec is None:
            return None
        if "offset_ns" not in rec:
            rec["error"] = rec["writer"].close() or rec["error"]
            rec["offset_ns"] = self.offset_ns
        if frames_csv is None:
            with self.lock:
                self.orphan = rec
            return None
        offset = rec["offset_ns"]
        t0 = rec["t0_ns"]
        if frames_csv is not None:
            with open(frames_csv, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(("frame_idx", "t_ns", "recv_t_ns", "pts_us", "keyframe"))
                for idx, (pts_us, recv_ns, key) in enumerate(rec["frames"]):
                    w.writerow((idx, pts_us * 1000 + offset - t0, recv_ns - t0, pts_us, int(key)))
        frames = rec["frames"]
        lat = sorted(r - (p * 1000 + offset) for p, r, _ in frames)
        return {
            "frames": len(frames),
            "clock_offset_ns": offset,
            "first_frame_t_ns": frames[0][0] * 1000 + offset - t0 if frames else None,
            "fps": (len(frames) - 1) * 1e6 / (frames[-1][0] - frames[0][0]) if len(frames) > 1 else None,
            "recv_jitter_ms_p50_p99": (lat[len(lat) // 2] / 1e6, lat[int(len(lat) * 0.99)] / 1e6) if lat else None,
            "error": rec["error"],
        }

    def snapshot(self):
        with self.lock:
            return self.state, self.error


# =====================================================================
# 단독 테스트: 연결 → N초 녹화 → 통계 출력
# =====================================================================
if __name__ == "__main__":
    import json
    import sys
    import tempfile

    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
    cam = PhoneCamera()
    cam.start()
    while cam.snapshot()[0] == "starting":
        time.sleep(0.1)
    print("state", cam.snapshot(), cam.model, cam.android, cam.take_notices())
    if cam.snapshot()[0] != "streaming":
        sys.exit(1)
    time.sleep(1.5)
    out = tempfile.mkdtemp(prefix="phone_cam_")
    t0 = time.monotonic_ns()
    print("record start", cam.start_recording(os.path.join(out, "video.mp4"), t0))
    end = time.monotonic() + seconds
    previews = 0
    while time.monotonic() < end:
        if cam.latest_preview() is not None:
            previews += 1
        time.sleep(0.01)
    info = cam.stop_recording(os.path.join(out, "video_frames.csv"))
    print(json.dumps(info, indent=1), "previews", previews, "size", cam.video_size, "fps", cam.measured_fps())
    cam.stop()
    print("out", out, os.listdir(out), os.path.getsize(os.path.join(out, "video.mp4")))
