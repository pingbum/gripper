# -*- coding: utf-8 -*-
"""
앱 화면 녹화 (X11 ximagesrc → Jetson 하드웨어 H.264 인코더 → mp4) - Qt 의존성 없음.

- 캡처/인코딩은 GStreamer 스레드에서 돌아 Qt 제어 루프(메인 스레드)를 막지 않는다.
- 화면 영역(x, y, w, h)을 캡처한다. 창 xid 캡처는 OpenGL 위젯(3D 뷰)이 검게 나올 수 있어 쓰지 않는다.
- 프레임 시각: GStreamer 파이프라인 시계는 CLOCK_MONOTONIC(= time.monotonic_ns)이므로
  host_ns = base_time + buffer PTS 로 다른 데이터와 같은 시계가 된다 (ximagesrc는 캡처 시각으로 PTS를 찍는다).
"""
import csv
import threading
import time

import gi

gi.require_version("Gst", "1.0")
from gi.repository import Gst  # noqa: E402

Gst.init(None)

SCREEN_FPS = 30
SCREEN_BIT_RATE = 12_000_000


class ScreenRecorder:
    def __init__(self, path, region, fps=SCREEN_FPS, display=None):
        x, y, w, h = region
        w, h = w - w % 2, h - h % 2  # 인코더는 짝수 크기만 받는다
        self.path, self.region, self.fps = path, (x, y, w, h), fps
        disp = f'display-name="{display}" ' if display else ""
        self.pipeline = Gst.parse_launch(
            f"ximagesrc name=cap {disp}startx={x} starty={y} endx={x + w - 1} endy={y + h - 1} "
            "use-damage=false show-pointer=true "
            f"! video/x-raw,framerate={fps}/1 ! queue max-size-buffers=8 leaky=downstream "
            f"! nvvidconv ! video/x-raw(memory:NVMM),format=NV12,width={w},height={h} "
            f"! nvv4l2h264enc bitrate={SCREEN_BIT_RATE} iframeinterval={fps} insert-sps-pps=true "
            f"! h264parse ! mp4mux ! filesink location=\"{path}\"")
        clock = Gst.SystemClock.obtain()
        clock.set_property("clock-type", Gst.ClockType.MONOTONIC)
        self.pipeline.use_clock(clock)
        self.lock = threading.Lock()
        self.frames = []  # 캡처 시각 [monotonic ns]
        self.check = []   # 프로브 시점 monotonic - 캡처 시각 (시계 일치 확인용)
        pad = self.pipeline.get_by_name("cap").get_static_pad("src")
        pad.add_probe(Gst.PadProbeType.BUFFER, self._on_buffer)

    def _on_buffer(self, pad, info):
        buf = info.get_buffer()
        if buf.pts != Gst.CLOCK_TIME_NONE:
            host = self.pipeline.get_base_time() + buf.pts
            with self.lock:
                self.frames.append(host)
                if len(self.check) < 300:
                    self.check.append(time.monotonic_ns() - host)
        return Gst.PadProbeReturn.OK

    def start(self):
        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError("화면 녹화 파이프라인 시작 실패")

    def stop(self, frames_csv=None, t0_ns=0, timeout=5.0):
        self.pipeline.send_event(Gst.Event.new_eos())
        msg = self.pipeline.get_bus().timed_pop_filtered(
            int(timeout * Gst.SECOND), Gst.MessageType.EOS | Gst.MessageType.ERROR)
        self.pipeline.set_state(Gst.State.NULL)
        error = None
        if msg is None:
            error = "mp4 마무리 시간 초과"
        elif msg.type == Gst.MessageType.ERROR:
            error = str(msg.parse_error()[0])
        with self.lock:
            frames, check = list(self.frames), list(self.check)
        if frames_csv is not None:
            with open(frames_csv, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(("frame_idx", "t_ns"))
                for idx, host in enumerate(frames):
                    w.writerow((idx, host - t0_ns))
        check.sort()
        return {
            "frames": len(frames),
            "size": list(self.region[2:]),
            "fps": (len(frames) - 1) * 1e9 / (frames[-1] - frames[0]) if len(frames) > 1 else None,
            "capture_to_probe_ms_p50": check[len(check) // 2] / 1e6 if check else None,
            "error": error,
        }

    def poll_error(self):
        msg = self.pipeline.get_bus().pop_filtered(Gst.MessageType.ERROR)
        return None if msg is None else str(msg.parse_error()[0])


if __name__ == "__main__":
    import json
    import os
    import sys
    import tempfile

    out = os.path.join(tempfile.mkdtemp(prefix="screen_"), "screen.mp4")
    rec = ScreenRecorder(out, (100, 100, 1750, 940))
    t0 = time.monotonic_ns()
    rec.start()
    time.sleep(float(sys.argv[1]) if len(sys.argv) > 1 else 4.0)
    print(json.dumps(rec.stop(out.replace(".mp4", "_frames.csv"), t0), indent=1), out, os.path.getsize(out))
