#!/usr/bin/env python3

import time
import struct
import threading
from collections import defaultdict, deque

import tkinter as tk
from tkinter import ttk

import can

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


# =========================
# User settings
# =========================
CAN_INTERFACE = "can0"

MOTOR_IDS = [1, 2, 3]

MODE_POSITION = 0x05

# command send rate
SEND_PERIOD_MS = 100   # 10 Hz

# plot update rate
PLOT_PERIOD_MS = 100   # 10 Hz

# plot window
WINDOW_SEC = 5.0

# command slider range [deg], output-side/UI angle
# Mechanical/ROM-safe position command range
MIN_ANGLE = -90.0
MAX_ANGLE = 90.0
SLIDER_RESOLUTION = 0.1

# plot y-axis
POS_YLIM = (-100, 100)
CURRENT_YLIM = (-4, 4)


# =========================
# Dark theme colors
# =========================
BG = "#111111"
PANEL_BG = "#1b1b1b"
ENTRY_BG = "#2a2a2a"
FG = "#eeeeee"
MUTED_FG = "#aaaaaa"
GRID = "#444444"
ACCENT = "#4aa3ff"
WARN = "#ffcc66"

LINE_COLORS = {
    1: "#4aa3ff",   # blue
    2: "#ff9f40",   # orange
    3: "#4cd964",   # green
}


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


# =========================
# Shared data for plot
# =========================
data_lock = threading.Lock()
running = True

plot_data = defaultdict(lambda: {
    "t": deque(),
    "pos": deque(),
    "current": deque(),
    "rev": 0,
    "error": 0,
})


# =========================
# CAN command class
# =========================
class CANPositionCommander:
    def __init__(self, interface="can0"):
        self.interface = interface
        self.bus = can.interface.Bus(
            channel=self.interface,
            interface="socketcan"
        )

        self.mode_position = MODE_POSITION
        self.motor_ids = MOTOR_IDS

        # slider target angle [deg]
        self.values_deg = {
            1: 0.0,
            2: 0.0,
            3: 0.0,
        }

        # motor-specific position offset [deg]
        self.offsets_deg = {
            1: 0.0,
            2: 0.0,
            3: 0.0,
        }

    def get_command_deg(self, motor_id):
        """
        Actual command angle sent to motor.

        command_deg = clamp(slider_target_deg + offset_deg, -90, 90)
        """
        command_deg = self.values_deg[motor_id] + self.offsets_deg[motor_id]
        return clamp(command_deg, MIN_ANGLE, MAX_ANGLE)

    def make_position_msg(self, motor_id, position_deg):
        """
        Position command protocol:

        Extended CAN ID = (MODE_POSITION << 8) | motor_id
        MODE_POSITION = 0x05

        payload[0:4] = int32 big-endian
        theta_ref = raw * 0.01 deg

        Example:
            -30 deg -> raw = -3000 -> FF FF F4 48
        """
        can_id = (self.mode_position << 8) | motor_id

        raw = int(round(position_deg * 100.0))
        payload4 = struct.pack(">i", raw)

        msg = can.Message(
            arbitration_id=can_id,
            data=payload4,
            is_extended_id=True
        )

        return msg

    def send_position(self, motor_id, position_deg):
        msg = self.make_position_msg(motor_id, position_deg)
        self.bus.send(msg)

    def send_all_positions(self):
        for motor_id in self.motor_ids:
            command_deg = self.get_command_deg(motor_id)
            self.send_position(motor_id, command_deg)

    def shutdown(self):
        self.bus.shutdown()


# =========================
# CAN monitor / plot decode
# =========================
def decode_motor_status(msg):
    """
    CAN broadcast status decode.

    CAN ID:
        arbitration_id = driver_id

    Payload:
        byte 0~1 : position_deg * 10      int16 big-endian
        byte 2~3 : speed_erpm / 10        int16 big-endian, unused here
        byte 4~5 : current_A * 1000       int16 big-endian
        byte 6   : rev                    int8
        byte 7   : error_code             int8
    """
    if len(msg.data) < 8:
        return None

    pos_scaled, _speed_scaled, current_scaled, rev, error = struct.unpack(
        ">hhhbb",
        bytes(msg.data[:8])
    )

    GEAR_RATIO = 1

    motor_pos_deg_0_360 = pos_scaled / 10.0

    # motor-side accumulated angle
    motor_total_deg = rev * 360.0 + motor_pos_deg_0_360

    # output-side angle for UI/plot
    pos_deg = motor_total_deg / GEAR_RATIO

    return {
        "id": msg.arbitration_id,
        "position_deg": pos_deg,
        "current_A": current_scaled / 1000.0,
        "rev": rev,
        "error": error,
    }


def can_reader_thread():
    global running

    rx_bus = can.interface.Bus(
        channel=CAN_INTERFACE,
        interface="socketcan"
    )

    t0 = time.time()

    try:
        while running:
            msg = rx_bus.recv(timeout=0.1)
            if msg is None:
                continue

            motor_id = msg.arbitration_id

            # status broadcast는 ID 1,2,3만 사용한다고 가정
            if motor_id not in MOTOR_IDS:
                continue

            decoded = decode_motor_status(msg)
            if decoded is None:
                continue

            t = time.time() - t0

            with data_lock:
                d = plot_data[motor_id]

                d["t"].append(t)

                # 여기에는 raw feedback position을 저장한다.
                # 실제 plot에서 offset을 적용해서 그린다.
                d["pos"].append(decoded["position_deg"])

                d["current"].append(decoded["current_A"])
                d["rev"] = decoded["rev"]
                d["error"] = decoded["error"]

                cutoff = t - WINDOW_SEC
                while d["t"] and d["t"][0] < cutoff:
                    d["t"].popleft()
                    d["pos"].popleft()
                    d["current"].popleft()

    finally:
        rx_bus.shutdown()


# =========================
# Main UI
# =========================
class CANControlPlotUI:
    def __init__(self, commander: CANPositionCommander):
        self.commander = commander

        self.root = tk.Tk()
        self.root.title("CAN Motor Control & Monitor")
        self.root.configure(bg=BG)
        self.root.geometry("1250x850")

        self.auto_send = tk.BooleanVar(value=False)

        self.sliders = {}
        self.value_labels = {}
        self.offset_vars = {}
        self.offset_entries = {}
        self.command_labels = {}
        self.feedback_labels = {}

        self.configure_style()
        self.build_layout()

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.periodic_send()
        self.periodic_plot_update()

    def configure_style(self):
        style = ttk.Style()
        style.theme_use("clam")

        style.configure(
            ".",
            background=BG,
            foreground=FG,
            fieldbackground=ENTRY_BG,
            bordercolor=PANEL_BG,
            lightcolor=PANEL_BG,
            darkcolor=PANEL_BG,
            troughcolor=ENTRY_BG,
        )

        style.configure(
            "TFrame",
            background=BG,
        )

        style.configure(
            "Panel.TFrame",
            background=PANEL_BG,
        )

        style.configure(
            "TLabel",
            background=BG,
            foreground=FG,
        )

        style.configure(
            "Panel.TLabel",
            background=PANEL_BG,
            foreground=FG,
        )

        style.configure(
            "Muted.TLabel",
            background=PANEL_BG,
            foreground=MUTED_FG,
        )

        style.configure(
            "Title.TLabel",
            background=BG,
            foreground=FG,
            font=("Arial", 16, "bold"),
        )

        style.configure(
            "TButton",
            background=ENTRY_BG,
            foreground=FG,
            padding=6,
        )

        style.map(
            "TButton",
            background=[("active", "#333333")],
            foreground=[("active", FG)],
        )

        style.configure(
            "TCheckbutton",
            background=PANEL_BG,
            foreground=FG,
        )

        style.map(
            "TCheckbutton",
            background=[("active", PANEL_BG)],
            foreground=[("active", FG)],
        )

        style.configure(
            "TEntry",
            fieldbackground=ENTRY_BG,
            foreground=FG,
            insertcolor=FG,
        )

    def build_layout(self):
        title = ttk.Label(
            self.root,
            text="CAN Motor Control & Monitor",
            style="Title.TLabel"
        )
        title.pack(pady=(12, 8))

        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill="both", expand=True, padx=12, pady=8)

        control_frame = ttk.Frame(main_frame, style="Panel.TFrame")
        control_frame.pack(side="left", fill="y", padx=(0, 10), pady=0)

        plot_frame = ttk.Frame(main_frame, style="Panel.TFrame")
        plot_frame.pack(side="right", fill="both", expand=True)

        self.build_control_panel(control_frame)
        self.build_plot_panel(plot_frame)

    def build_control_panel(self, parent):
        header = ttk.Label(
            parent,
            text="Position Command",
            style="Panel.TLabel",
            font=("Arial", 13, "bold")
        )
        header.pack(anchor="w", padx=12, pady=(12, 8))

        info = ttk.Label(
            parent,
            text="sent angle = clamp(target + offset, -90~90 deg)",
            style="Muted.TLabel"
        )
        info.pack(anchor="w", padx=12, pady=(0, 10))

        for motor_id in self.commander.motor_ids:
            motor_frame = ttk.Frame(parent, style="Panel.TFrame")
            motor_frame.pack(fill="x", padx=12, pady=8)

            label = ttk.Label(
                motor_frame,
                text=f"Motor ID {motor_id}",
                style="Panel.TLabel",
                font=("Arial", 11, "bold")
            )
            label.pack(anchor="w")

            slider = tk.Scale(
                motor_frame,
                from_=MIN_ANGLE,
                to=MAX_ANGLE,
                resolution=SLIDER_RESOLUTION,
                orient="horizontal",
                length=420,
                bg=PANEL_BG,
                fg=FG,
                troughcolor=ENTRY_BG,
                activebackground=ACCENT,
                highlightthickness=0,
                command=lambda value, mid=motor_id: self.on_slider_change(mid, value)
            )
            slider.set(self.commander.values_deg[motor_id])
            slider.pack(fill="x", pady=(5, 3))

            row = ttk.Frame(motor_frame, style="Panel.TFrame")
            row.pack(fill="x", pady=2)

            ttk.Label(row, text="target:", style="Muted.TLabel", width=8).pack(side="left")
            value_label = ttk.Label(
                row,
                text=f"{self.commander.values_deg[motor_id]:.1f} deg",
                style="Panel.TLabel",
                width=11
            )
            value_label.pack(side="left")

            ttk.Label(row, text="offset:", style="Muted.TLabel", width=8).pack(side="left", padx=(8, 0))

            offset_var = tk.StringVar(value="0.0")
            offset_entry = ttk.Entry(
                row,
                textvariable=offset_var,
                width=8
            )
            offset_entry.pack(side="left")
            offset_entry.bind(
                "<KeyRelease>",
                lambda event, mid=motor_id: self.on_offset_change(mid)
            )
            offset_entry.bind(
                "<FocusOut>",
                lambda event, mid=motor_id: self.on_offset_change(mid)
            )
            offset_entry.bind(
                "<Return>",
                lambda event, mid=motor_id: self.on_offset_change(mid)
            )

            ttk.Label(row, text="sent:", style="Muted.TLabel", width=6).pack(side="left", padx=(8, 0))
            command_label = ttk.Label(
                row,
                text=f"{self.commander.get_command_deg(motor_id):.1f} deg",
                style="Panel.TLabel",
                width=12
            )
            command_label.pack(side="left")

            fb_label = ttk.Label(
                motor_frame,
                text="feedback: -- deg, -- A, err --",
                style="Muted.TLabel"
            )
            fb_label.pack(anchor="w", pady=(2, 0))

            self.sliders[motor_id] = slider
            self.value_labels[motor_id] = value_label
            self.offset_vars[motor_id] = offset_var
            self.offset_entries[motor_id] = offset_entry
            self.command_labels[motor_id] = command_label
            self.feedback_labels[motor_id] = fb_label

        button_frame = ttk.Frame(parent, style="Panel.TFrame")
        button_frame.pack(fill="x", padx=12, pady=(16, 8))

        send_once_button = ttk.Button(
            button_frame,
            text="Send once",
            command=self.send_once
        )
        send_once_button.pack(side="left", padx=(0, 6))

        zero_button = ttk.Button(
            button_frame,
            text="Set targets 0 deg",
            command=self.set_all_zero
        )
        zero_button.pack(side="left", padx=6)

        clear_offset_button = ttk.Button(
            button_frame,
            text="Clear offsets",
            command=self.clear_offsets
        )
        clear_offset_button.pack(side="left", padx=6)

        auto_frame = ttk.Frame(parent, style="Panel.TFrame")
        auto_frame.pack(fill="x", padx=12, pady=(8, 8))

        auto_check = ttk.Checkbutton(
            auto_frame,
            text="Send at 10 Hz",
            variable=self.auto_send
        )
        auto_check.pack(side="left")

        self.status_label = ttk.Label(
            parent,
            text="Auto send OFF",
            style="Muted.TLabel",
            wraplength=430,
            justify="left"
        )
        self.status_label.pack(fill="x", padx=12, pady=(8, 12))

    def build_plot_panel(self, parent):
        plot_title = ttk.Label(
            parent,
            text="Live Feedback Plot",
            style="Panel.TLabel",
            font=("Arial", 13, "bold")
        )
        plot_title.pack(anchor="w", padx=12, pady=(12, 4))

        self.fig = Figure(figsize=(8, 6), dpi=100)
        self.fig.patch.set_facecolor(BG)

        self.ax_pos = self.fig.add_subplot(2, 1, 1)
        self.ax_current = self.fig.add_subplot(2, 1, 2, sharex=self.ax_pos)

        self.setup_axis(self.ax_pos, "Position [deg], offset compensated", POS_YLIM)
        self.setup_axis(self.ax_current, "Current [A]", CURRENT_YLIM)
        self.ax_current.set_xlabel("Time [s]", color=FG)

        self.lines = {}

        for motor_id in self.commander.motor_ids:
            color = LINE_COLORS.get(motor_id, None)

            line_pos, = self.ax_pos.plot(
                [],
                [],
                label=f"ID {motor_id}",
                color=color,
                linewidth=1.8
            )

            line_current, = self.ax_current.plot(
                [],
                [],
                label=f"ID {motor_id}",
                color=color,
                linewidth=1.8
            )

            self.lines[motor_id] = {
                "pos": line_pos,
                "current": line_current,
            }

        self.ax_pos.legend(
            loc="upper right",
            facecolor=PANEL_BG,
            edgecolor=GRID,
            labelcolor=FG
        )

        self.ax_current.legend(
            loc="upper right",
            facecolor=PANEL_BG,
            edgecolor=GRID,
            labelcolor=FG
        )

        self.fig.tight_layout(pad=2.0)

        self.canvas = FigureCanvasTkAgg(self.fig, master=parent)
        canvas_widget = self.canvas.get_tk_widget()
        canvas_widget.configure(bg=PANEL_BG, highlightthickness=0)
        canvas_widget.pack(fill="both", expand=True, padx=12, pady=12)

    def setup_axis(self, ax, ylabel, ylim):
        ax.set_facecolor(BG)
        ax.set_ylabel(ylabel, color=FG)
        ax.set_ylim(*ylim)
        ax.grid(True, color=GRID, linewidth=0.7, alpha=0.8)

        ax.tick_params(axis="x", colors=FG)
        ax.tick_params(axis="y", colors=FG)

        for spine in ax.spines.values():
            spine.set_color(GRID)

    def parse_offset(self, motor_id):
        text = self.offset_vars[motor_id].get().strip()

        if text == "":
            return 0.0

        try:
            return float(text)
        except ValueError:
            return self.commander.offsets_deg[motor_id]

    def update_offsets_from_entries(self):
        for motor_id in self.commander.motor_ids:
            self.commander.offsets_deg[motor_id] = self.parse_offset(motor_id)
            self.update_command_label(motor_id)

    def on_offset_change(self, motor_id):
        self.commander.offsets_deg[motor_id] = self.parse_offset(motor_id)
        self.update_command_label(motor_id)

    def on_slider_change(self, motor_id, value):
        position_deg = clamp(float(value), MIN_ANGLE, MAX_ANGLE)
        self.commander.values_deg[motor_id] = position_deg
        self.value_labels[motor_id].config(text=f"{position_deg:.1f} deg")
        self.update_command_label(motor_id)

    def update_command_label(self, motor_id):
        command_deg = self.commander.get_command_deg(motor_id)
        self.command_labels[motor_id].config(text=f"{command_deg:.1f} deg")

    def send_once(self):
        try:
            self.update_offsets_from_entries()
            self.commander.send_all_positions()
            self.update_status("Sent once")
        except can.CanError as e:
            self.update_status(f"CAN send error: {e}")

    def periodic_send(self):
        if self.auto_send.get():
            try:
                self.update_offsets_from_entries()
                self.commander.send_all_positions()
                self.update_status("Sending at 10 Hz")
            except can.CanError as e:
                self.update_status(f"CAN send error: {e}")

        self.root.after(SEND_PERIOD_MS, self.periodic_send)

    def periodic_plot_update(self):
        latest_time = 0.0

        with data_lock:
            for motor_id in self.commander.motor_ids:
                d = plot_data[motor_id]
                if len(d["t"]) > 0:
                    latest_time = max(latest_time, d["t"][-1])

            for motor_id in self.commander.motor_ids:
                d = plot_data[motor_id]

                if len(d["t"]) == 0:
                    continue

                t_list = list(d["t"])

                # CAN에서 받은 실제 feedback position.
                # 이 값은 offset이 반영되지 않은 raw output-side angle이다.
                pos_raw_list = list(d["pos"])

                current_list = list(d["current"])

                # 현재 UI offset.
                offset_deg = self.commander.offsets_deg[motor_id]

                # =========================
                # Important modification
                # =========================
                # Command는 target + offset으로 보낸다.
                # 따라서 graph와 feedback label을 target 기준 좌표계로 보려면
                # feedback에서는 offset을 빼야 한다.
                #
                # command = target + offset
                # graph_feedback = raw_feedback - offset
                pos_list = [
                    p - offset_deg
                    for p in pos_raw_list
                ]

                self.lines[motor_id]["pos"].set_data(t_list, pos_list)
                self.lines[motor_id]["current"].set_data(t_list, current_list)

                self.feedback_labels[motor_id].config(
                    text=(
                        f"feedback: "
                        f"{pos_list[-1]:.1f} deg, "
                        f"{current_list[-1]:.3f} A, "
                        f"rev {d['rev']}, err {d['error']}"
                    )
                )

        if latest_time < WINDOW_SEC:
            xmin = 0.0
            xmax = WINDOW_SEC
        else:
            xmin = latest_time - WINDOW_SEC
            xmax = latest_time

        self.ax_current.set_xlim(xmin, xmax)
        self.canvas.draw_idle()

        self.root.after(PLOT_PERIOD_MS, self.periodic_plot_update)

    def set_all_zero(self):
        for motor_id in self.commander.motor_ids:
            self.sliders[motor_id].set(0.0)
            self.commander.values_deg[motor_id] = 0.0
            self.value_labels[motor_id].config(text="0.0 deg")
            self.update_command_label(motor_id)

        self.send_once()

    def clear_offsets(self):
        for motor_id in self.commander.motor_ids:
            self.offset_vars[motor_id].set("0.0")
            self.commander.offsets_deg[motor_id] = 0.0
            self.update_command_label(motor_id)

        self.send_once()

    def update_status(self, prefix):
        values_text = " | ".join(
            [
                f"ID {mid}: target={self.commander.values_deg[mid]:.1f}, "
                f"offset={self.commander.offsets_deg[mid]:.1f}, "
                f"sent={self.commander.get_command_deg(mid):.1f} deg"
                for mid in self.commander.motor_ids
            ]
        )

        if self.auto_send.get():
            auto_text = "AUTO ON"
        else:
            auto_text = "AUTO OFF"

        self.status_label.config(
            text=f"{prefix} [{auto_text}]\n{values_text}"
        )

    def run(self):
        self.root.mainloop()

    def on_close(self):
        global running
        running = False

        try:
            self.commander.shutdown()
        except Exception:
            pass

        self.root.destroy()


def main():
    global running

    running = True

    rx_thread = threading.Thread(
        target=can_reader_thread,
        daemon=True
    )
    rx_thread.start()

    commander = CANPositionCommander(interface=CAN_INTERFACE)
    ui = CANControlPlotUI(commander)

    try:
        ui.run()
    except KeyboardInterrupt:
        pass
    finally:
        running = False
        try:
            commander.shutdown()
        except Exception:
            pass
        rx_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()