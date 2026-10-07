#!/usr/bin/env python3

import tkinter as tk
from tkinter import ttk
import struct
import can


class CANPositionCommander:
    def __init__(self, interface="can0"):
        self.interface = interface
        self.bus = can.interface.Bus(
            channel=self.interface,
            interface="socketcan"
        )

        self.mode_position = 0x05

        self.motor_ids = [1, 2, 3]

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

        command_deg = slider_target_deg + offset_deg
        """
        return self.values_deg[motor_id] + self.offsets_deg[motor_id]

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


class SliderGUI:
    def __init__(self, commander: CANPositionCommander):
        self.commander = commander

        self.root = tk.Tk()
        self.root.title("CAN Position Slider - 10 Hz")

        self.sliders = {}
        self.value_labels = {}
        self.offset_entries = {}
        self.offset_vars = {}
        self.command_labels = {}

        # degree 범위
        self.min_angle = -180.0
        self.max_angle = 180.0
        self.resolution = 0.1

        # 10 Hz
        self.period_ms = 100

        # 처음에는 안전하게 송신 OFF
        self.auto_send = tk.BooleanVar(value=False)

        title = ttk.Label(
            self.root,
            text="CAN Position Command Slider",
            font=("Arial", 14, "bold")
        )
        title.pack(pady=10)

        header = ttk.Frame(self.root, padding=(10, 0))
        header.pack(fill="x")

        ttk.Label(header, text="Motor", width=15).pack(side="left")
        ttk.Label(header, text="Target angle", width=52).pack(side="left")
        ttk.Label(header, text="Offset [deg]", width=14).pack(side="left")
        ttk.Label(header, text="Sent angle", width=16).pack(side="left")

        for motor_id in self.commander.motor_ids:
            frame = ttk.Frame(self.root, padding=10)
            frame.pack(fill="x")

            label = ttk.Label(frame, text=f"Motor ID {motor_id}", width=15)
            label.pack(side="left")

            slider = tk.Scale(
                frame,
                from_=self.min_angle,
                to=self.max_angle,
                resolution=self.resolution,
                orient="horizontal",
                length=500,
                command=lambda value, mid=motor_id: self.on_slider_change(mid, value)
            )
            slider.set(self.commander.values_deg[motor_id])
            slider.pack(side="left", padx=10)

            value_label = ttk.Label(
                frame,
                text=f"{self.commander.values_deg[motor_id]:.1f} deg",
                width=12
            )
            value_label.pack(side="left")

            offset_var = tk.StringVar(value="0.0")
            offset_entry = ttk.Entry(
                frame,
                textvariable=offset_var,
                width=10
            )
            offset_entry.pack(side="left", padx=5)
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

            command_label = ttk.Label(
                frame,
                text=f"{self.commander.get_command_deg(motor_id):.1f} deg",
                width=14
            )
            command_label.pack(side="left", padx=5)

            self.sliders[motor_id] = slider
            self.value_labels[motor_id] = value_label
            self.offset_vars[motor_id] = offset_var
            self.offset_entries[motor_id] = offset_entry
            self.command_labels[motor_id] = command_label

        button_frame = ttk.Frame(self.root, padding=10)
        button_frame.pack(fill="x")

        send_once_button = ttk.Button(
            button_frame,
            text="Send once",
            command=self.send_once
        )
        send_once_button.pack(side="left", padx=5)

        zero_button = ttk.Button(
            button_frame,
            text="Set all targets 0 deg",
            command=self.set_all_zero
        )
        zero_button.pack(side="left", padx=5)

        clear_offset_button = ttk.Button(
            button_frame,
            text="Clear offsets",
            command=self.clear_offsets
        )
        clear_offset_button.pack(side="left", padx=5)

        auto_check = ttk.Checkbutton(
            button_frame,
            text="Send at 10 Hz",
            variable=self.auto_send
        )
        auto_check.pack(side="left", padx=15)

        self.status_label = ttk.Label(
            self.root,
            text="Auto send OFF",
            padding=10
        )
        self.status_label.pack(fill="x")

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.update_all_command_labels()
        self.periodic_send()

    def parse_offset(self, motor_id):
        text = self.offset_vars[motor_id].get().strip()

        # blank이면 offset 0으로 처리
        if text == "":
            return 0.0

        try:
            return float(text)
        except ValueError:
            # 잘못 입력된 경우 기존 offset 유지
            return self.commander.offsets_deg[motor_id]

    def on_offset_change(self, motor_id):
        offset_deg = self.parse_offset(motor_id)
        self.commander.offsets_deg[motor_id] = offset_deg
        self.update_command_label(motor_id)

    def on_slider_change(self, motor_id, value):
        position_deg = float(value)
        self.commander.values_deg[motor_id] = position_deg
        self.value_labels[motor_id].config(text=f"{position_deg:.1f} deg")
        self.update_command_label(motor_id)

    def update_command_label(self, motor_id):
        command_deg = self.commander.get_command_deg(motor_id)
        self.command_labels[motor_id].config(text=f"{command_deg:.1f} deg")

    def update_all_command_labels(self):
        for motor_id in self.commander.motor_ids:
            self.update_command_label(motor_id)

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

        self.root.after(self.period_ms, self.periodic_send)

    def update_offsets_from_entries(self):
        for motor_id in self.commander.motor_ids:
            self.commander.offsets_deg[motor_id] = self.parse_offset(motor_id)
            self.update_command_label(motor_id)

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
        values_text = ", ".join(
            [
                f"ID {mid}: target={self.commander.values_deg[mid]:.1f}, "
                f"offset={self.commander.offsets_deg[mid]:.1f}, "
                f"sent={self.commander.get_command_deg(mid):.1f} deg"
                for mid in self.commander.motor_ids
            ]
        )
        self.status_label.config(text=f"{prefix} | {values_text}")

    def run(self):
        self.root.mainloop()

    def on_close(self):
        self.commander.shutdown()
        self.root.destroy()


def main():
    commander = CANPositionCommander(interface="can0")
    gui = SliderGUI(commander)
    gui.run()


if __name__ == "__main__":
    main()