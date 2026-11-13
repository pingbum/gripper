# -*- coding: utf-8 -*-
"""
녹화 세션 → 시연용 합성 영상 (composite.mp4).

화면 구성 (1920x1080, 30 fps):
  ┌──────────────────────────────┬────────────┐
  │                              │ MCP A/A    │
  │   폰 카메라 영상 1280x720     │ MCP F/E    │  관절별 명령값 vs 실제 각도
  │                              │ PIP F/E    │  (현재 시점 = 세로선)
  ├───────────────┬──────────────┤ DIP F/E    │
  │ 앱 화면       │ 경과 시간,    │            │
  │ (screen.mp4)  │ 현재 값       │            │
  └───────────────┴──────────────┴────────────┘

시간 맞춤: 출력 프레임 = 카메라 프레임 (t_ns - video_offset_ms).
          그래프 세로선/현재 값/앱 화면 프레임은 그 시각에 맞춰 고른다.
디코딩/인코딩은 Jetson 하드웨어(nvv4l2decoder / nvv4l2h264enc)를 쓴다.

실행: python3 composite.py recordings/2026-11-13_140346 [--offset-ms 0] [--start 0] [--end 30]
                          [--joints "MCP A/A,PIP F/E"] [--show-target] [--no-screen]
"""
import argparse
import csv
import json
import os
import sys
import time

import gi
import matplotlib
import numpy as np
from PIL import Image, ImageDraw, ImageFont

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

gi.require_version("Gst", "1.0")
from gi.repository import Gst  # noqa: E402

Gst.init(None)

W, H = 1920, 1080
CAM_W, CAM_H = 1280, 720
PLOT_W = W - CAM_W               # 640
BOTTOM_H = H - CAM_H             # 360
SCREEN_W, SCREEN_H = 632, 360    # 앱 화면 축소 크기 (원본 비율 약 1.76)
FPS = 30
BIT_RATE = 20_000_000

JOINTS = {"MCP A/A": "MCP_AA", "MCP F/E": "MCP_FE", "PIP F/E": "PIP", "DIP F/E": "DIP"}
BG = "#1e1e1e"
FG = "#d4d4d4"
GRID = "#3a3a3a"
C_CMD = "#f0a030"     # 명령값
C_ACT = "#3fbf5f"     # 실제 각도
C_REF = "#7f7f7f"     # 목표 (선택)
C_CURSOR = (255, 70, 70, 255)
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
FONT_BOLD = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"


# =====================================================================
# 데이터 읽기
# =====================================================================
def read_csv(path):
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def column(rows, name):
    return np.array([float(r[name]) if r[name] != "" else np.nan for r in rows])


def load_session(folder):
    meta = json.load(open(os.path.join(folder, "meta.json"), encoding="utf-8"))
    angle = read_csv(os.path.join(folder, "angle.csv"))
    command = [r for r in read_csv(os.path.join(folder, "command.csv"))
               if r["event"] in ("initial", "cmd", "ref", "control_on", "control_off")]
    data = {
        "meta": meta,
        "angle_t": np.array([int(r["t_ns"]) for r in angle]) / 1e9,
        "cmd_t": np.array([int(r["t_ns"]) for r in command]) / 1e9,
        "video_t": np.array([int(r["t_ns"]) for r in read_csv(os.path.join(folder, "video_frames.csv"))]) / 1e9,
        "screen_t": None,
    }
    for name, col in JOINTS.items():
        data[f"act:{name}"] = column(angle, f"{col}_deg")
        data[f"cmd:{name}"] = column(command, f"{col}_cmd_deg")
        data[f"ref:{name}"] = column(command, f"{col}_ref_deg")
    screen_csv = os.path.join(folder, "screen_frames.csv")
    if os.path.exists(screen_csv) and os.path.exists(os.path.join(folder, "screen.mp4")):
        data["screen_t"] = np.array([int(r["t_ns"]) for r in read_csv(screen_csv)]) / 1e9
    return data


def value_at(t_arr, v_arr, t):
    """t 이전 마지막 값 (없으면 nan)."""
    k = np.searchsorted(t_arr, t, side="right") - 1
    return v_arr[k] if k >= 0 else np.nan


# =====================================================================
# 그래프 배경 (한 번만 그린다)
# =====================================================================
def render_plots(data, joints, t0, t1, show_target):
    """그래프 패널 RGBA 배열과 시각 → x 픽셀 변환, 축별 y 범위(픽셀)를 돌려준다."""
    dpi = 100
    fig, axes = plt.subplots(len(joints), 1, figsize=(PLOT_W / dpi, H / dpi), dpi=dpi, sharex=True)
    axes = np.atleast_1d(axes)
    fig.patch.set_facecolor(BG)
    settings = data["meta"].get("settings_at_start", {}).get("joints", {})
    for ax, name in zip(axes, joints):
        ax.set_facecolor(BG)
        act, cmd, ref = data[f"act:{name}"], data[f"cmd:{name}"], data[f"ref:{name}"]
        # 명령은 다음 명령까지 유지되므로 마지막 값을 표시 구간 끝까지 잇는다.
        cmd_t = np.append(data["cmd_t"], max(t1, data["cmd_t"][-1]))
        cmd, ref = np.append(cmd, cmd[-1]), np.append(ref, ref[-1])
        if show_target:
            ax.step(cmd_t, ref, where="post", color=C_REF, lw=1.0, ls="--", label="Target")
        ax.step(cmd_t, cmd, where="post", color=C_CMD, lw=1.6, label="Command")
        ax.plot(data["angle_t"], act, color=C_ACT, lw=1.6, label="Measured")
        # y 범위 = 관절 제약 ∪ 표시 구간의 데이터
        sel = (data["angle_t"] >= t0) & (data["angle_t"] <= t1)
        vals = [v for v in (act[sel], cmd[cmd_t <= t1]) if np.isfinite(v).any()]
        lo, hi = settings.get(name, {}).get("limits_deg", [np.nan, np.nan])
        if vals:
            lo = np.nanmin([lo, *(np.nanmin(v) for v in vals)])
            hi = np.nanmax([hi, *(np.nanmax(v) for v in vals)])
        if np.isfinite(lo) and np.isfinite(hi):
            pad = max(2.0, 0.08 * (hi - lo))
            ax.set_ylim(lo - pad, hi + pad)
        ax.set_title(name, color=FG, fontsize=12, loc="left", pad=4, fontweight="bold")
        ax.set_ylabel("deg", color=FG, fontsize=9)
        ax.tick_params(colors=FG, labelsize=9)
        ax.grid(True, color=GRID, lw=0.6)
        for spine in ax.spines.values():
            spine.set_color(GRID)
    axes[0].legend(loc="upper right", fontsize=9, facecolor=BG, edgecolor=GRID, labelcolor=FG, ncol=3)
    axes[-1].set_xlim(t0, t1)
    axes[-1].set_xlabel("time [s]", color=FG, fontsize=10)
    fig.tight_layout(pad=0.6, h_pad=0.8)
    fig.canvas.draw()
    panel = np.asarray(fig.canvas.buffer_rgba()).copy()
    # 데이터 좌표 → 픽셀 (matplotlib 디스플레이 좌표는 원점이 아래)
    x0, _ = axes[-1].transData.transform((t0, 0))
    x1, _ = axes[-1].transData.transform((t1, 0))
    spans = []
    for ax in axes:
        bb = ax.get_window_extent()
        spans.append((int(round(H - bb.y1)), int(round(H - bb.y0))))
    plt.close(fig)
    return panel, (x0, x1), spans


# =====================================================================
# GStreamer 디코더 / 인코더
# =====================================================================
class Decoder:
    """mp4 → RGBA 프레임 (지정 크기), 파일 순서대로 하나씩."""

    def __init__(self, path, width, height):
        self.width, self.height = width, height
        self.pipeline = Gst.parse_launch(
            f"filesrc location=\"{path}\" ! qtdemux ! h264parse ! nvv4l2decoder ! nvvidconv "
            f"! video/x-raw,format=RGBA,width={width},height={height} "
            "! appsink name=sink sync=false max-buffers=8")
        self.sink = self.pipeline.get_by_name("sink")
        self.pipeline.set_state(Gst.State.PLAYING)
        self.index = -1

    def next(self):
        sample = self.sink.emit("pull-sample")
        if sample is None:
            return None
        buf = sample.get_buffer()
        ok, info = buf.map(Gst.MapFlags.READ)
        try:
            arr = np.frombuffer(info.data, np.uint8)
            stride = arr.size // self.height
            frame = arr.reshape(self.height, stride)[:, :self.width * 4].reshape(self.height, self.width, 4).copy()
        finally:
            buf.unmap(info)
        self.index += 1
        return frame

    def close(self):
        self.pipeline.set_state(Gst.State.NULL)


class Encoder:
    def __init__(self, path):
        self.pipeline = Gst.parse_launch(
            f"appsrc name=src format=time caps=video/x-raw,format=RGBA,width={W},height={H},framerate={FPS}/1 "
            "! nvvidconv ! video/x-raw(memory:NVMM),format=NV12 "
            f"! nvv4l2h264enc bitrate={BIT_RATE} iframeinterval={FPS} insert-sps-pps=true "
            f"! h264parse ! mp4mux ! filesink location=\"{path}\"")
        self.src = self.pipeline.get_by_name("src")
        self.pipeline.set_state(Gst.State.PLAYING)
        self.count = 0

    def push(self, frame):
        buf = Gst.Buffer.new_wrapped(frame.tobytes())
        buf.pts = self.count * Gst.SECOND // FPS
        buf.duration = Gst.SECOND // FPS
        self.src.emit("push-buffer", buf)
        self.count += 1

    def close(self):
        self.src.emit("end-of-stream")
        msg = self.pipeline.get_bus().timed_pop_filtered(
            30 * Gst.SECOND, Gst.MessageType.EOS | Gst.MessageType.ERROR)
        self.pipeline.set_state(Gst.State.NULL)
        if msg is not None and msg.type == Gst.MessageType.ERROR:
            raise RuntimeError(msg.parse_error()[0])


# =====================================================================
# 합성
# =====================================================================
def main():
    ap = argparse.ArgumentParser(description="녹화 세션 → composite.mp4")
    ap.add_argument("session", help="세션 폴더 (recordings/YYYY-mm-dd_HHMMSS)")
    ap.add_argument("--out", help="출력 파일 (기본: 세션 폴더/composite.mp4)")
    ap.add_argument("--offset-ms", type=float, help="영상 고정 지연 보정 (기본: meta.json video_offset_ms)")
    ap.add_argument("--start", type=float, default=0.0, help="시작 시각 [s] (기본 0 = Record 시점)")
    ap.add_argument("--end", type=float, help="끝 시각 [s] (기본 녹화 끝)")
    ap.add_argument("--joints", help='그래프에 표시할 관절, 쉼표 구분 (기본 전체). 예: "PIP F/E,DIP F/E"')
    ap.add_argument("--show-target", action="store_true", help="드래그 목표(ref)도 점선으로 표시")
    ap.add_argument("--no-screen", action="store_true", help="왼쪽 아래 앱 화면을 넣지 않음")
    ap.add_argument("--title", default="Robot finger: command vs measured joint angle")
    args = ap.parse_args()

    folder = args.session.rstrip("/")
    out = args.out or os.path.join(folder, "composite.mp4")
    data = load_session(folder)
    meta = data["meta"]
    offset = (args.offset_ms if args.offset_ms is not None else meta["camera"].get("video_offset_ms", 0.0)) / 1000
    joints = [j.strip() for j in args.joints.split(",")] if args.joints else list(JOINTS)
    for j in joints:
        if j not in JOINTS:
            sys.exit(f"알 수 없는 관절: {j} (가능: {', '.join(JOINTS)})")
    t0 = args.start
    t1 = args.end if args.end is not None else meta["duration_s"]
    use_screen = not args.no_screen and data["screen_t"] is not None

    # 정적 배경: 그래프 패널 + 아래쪽 정보 패널 틀
    panel, (px0, px1), spans = render_plots(data, joints, t0, t1, args.show_target)
    canvas_bg = np.zeros((H, W, 4), np.uint8)
    canvas_bg[..., :3] = (30, 30, 30)
    canvas_bg[..., 3] = 255
    canvas_bg[:, CAM_W:] = panel
    info_x = SCREEN_W if use_screen else 0
    info_w = CAM_W - info_x
    font_title = ImageFont.truetype(FONT_BOLD, 20)
    font_small = ImageFont.truetype(FONT, 16)
    font_time = ImageFont.truetype(FONT_BOLD, 54)
    font_val = ImageFont.truetype(FONT, 17)
    info_bg = Image.new("RGBA", (info_w, BOTTOM_H), (30, 30, 30, 255))
    d = ImageDraw.Draw(info_bg)
    d.text((24, 16), args.title, font=font_title, fill=FG)
    d.text((24, 46), f"{meta['start_wall_time'][:19].replace('T', ' ')}   "
                     f"camera {meta['camera'].get('device')} {meta['camera'].get('resolution')}", font=font_small,
           fill="#9a9a9a")
    lx = 24
    for label, color in (("Command", C_CMD), ("Measured", C_ACT)):
        d.line((lx, 210, lx + 30, 210), fill=color, width=4)
        d.text((lx + 38, 200), label, font=font_small, fill=FG)
        lx += 170
    d.text((24, 236), "joint", font=font_small, fill="#9a9a9a")
    d.text((190, 236), "command", font=font_small, fill=C_CMD)
    d.text((320, 236), "measured", font=font_small, fill=C_ACT)

    # 출력 프레임 = 표시 구간 안의 카메라 프레임
    video_t = data["video_t"] - offset
    frames = np.nonzero((video_t >= t0) & (video_t <= t1))[0]
    if frames.size == 0:
        sys.exit("표시 구간에 카메라 프레임이 없습니다.")
    cam = Decoder(os.path.join(folder, "video.mp4"), CAM_W, CAM_H)
    scr = Decoder(os.path.join(folder, "screen.mp4"), SCREEN_W, SCREEN_H) if use_screen else None
    scr_frame, scr_next = None, None
    enc = Encoder(out)
    started = time.monotonic()
    print(f"{len(frames)} frames, t = {t0:.2f} .. {t1:.2f} s, offset {offset * 1000:.1f} ms → {out}")

    for n, idx in enumerate(frames):
        frame = None
        while cam.index < idx:  # 카메라를 이 프레임까지 디코딩 (구간 앞 프레임은 버림)
            frame = cam.next()
            if frame is None:
                break
        if frame is None:
            break
        t = video_t[idx]
        canvas = canvas_bg.copy()
        canvas[:CAM_H, :CAM_W] = frame

        if scr is not None:  # t 이전 마지막 앱 화면 프레임
            target = np.searchsorted(data["screen_t"], t, side="right") - 1
            while scr.index < target:
                scr_next = scr.next()
                if scr_next is None:
                    break
                scr_frame = scr_next
            if scr_frame is not None:
                canvas[CAM_H:CAM_H + SCREEN_H, :SCREEN_W] = scr_frame

        # 현재 시점 세로선
        x = CAM_W + int(round(px0 + (px1 - px0) * (t - t0) / (t1 - t0)))
        for top, bottom in spans:
            canvas[top:bottom, x - 1:x + 2] = C_CURSOR

        # 경과 시간과 현재 값
        info = info_bg.copy()
        d = ImageDraw.Draw(info)
        d.text((24, 90), f"t = {t:6.2f} s", font=font_time, fill="#ffffff")
        for k, name in enumerate(joints):
            y = 262 + k * 23
            c = value_at(data["cmd_t"], data[f"cmd:{name}"], t)
            a = value_at(data["angle_t"], data[f"act:{name}"], t)
            d.text((24, y), name, font=font_val, fill=FG)
            d.text((190, y), "  --" if np.isnan(c) else f"{c:6.1f}°", font=font_val, fill=C_CMD)
            d.text((320, y), "  --" if np.isnan(a) else f"{a:6.1f}°", font=font_val, fill=C_ACT)
        canvas[CAM_H:, info_x:CAM_W] = np.asarray(info)

        enc.push(canvas)
        if n % 150 == 0:
            print(f"  {n}/{len(frames)}  ({time.monotonic() - started:.0f} s)", flush=True)

    cam.close()
    if scr is not None:
        scr.close()
    enc.close()
    print(f"완료: {out}  ({enc.count} frames, {time.monotonic() - started:.0f} s)")


if __name__ == "__main__":
    main()
