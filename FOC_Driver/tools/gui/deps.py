# -*- coding: utf-8 -*-
import os
import sys

if __package__ in (None, ""):
    sys.path.append(os.path.dirname(os.path.dirname(__file__)))

try:
    from signal_gen import sine, square, triangle
    from config import (
        REF_SEND_HZ,
        MODE_CURRENT, MODE_VELOCITY, MODE_POSITION, MODE_CALIBRATION, MODE_COGGING_COMPENSATION, MODE_COGGING_TOGGLE,
        DEFAULT_INTERFACE, DEFAULT_CHANNEL, DEFAULT_BITRATE,
        MAX_POINTS, UPDATE_INTERVAL_MS, PLOT_MAX_HZ, MAX_QUEUE_BACKLOG,
        MAX_DRAIN_PER_TICK, RX_QUEUE_SIZE
    )
    from protocol import (
        build_ext_id, pack_command_payload, parse_broadcast_frame,
        pack_calibration_payload, format_error_code
    )
    from can_io import CANBusManager, CANReaderThread
except ImportError:
    from ..signal_gen import sine, square, triangle
    from ..config import (
        REF_SEND_HZ,
        MODE_CURRENT, MODE_VELOCITY, MODE_POSITION, MODE_CALIBRATION, MODE_COGGING_COMPENSATION, MODE_COGGING_TOGGLE,
        DEFAULT_INTERFACE, DEFAULT_CHANNEL, DEFAULT_BITRATE,
        MAX_POINTS, UPDATE_INTERVAL_MS, PLOT_MAX_HZ, MAX_QUEUE_BACKLOG,
        MAX_DRAIN_PER_TICK, RX_QUEUE_SIZE
    )
    from ..protocol import (
        build_ext_id, pack_command_payload, parse_broadcast_frame,
        pack_calibration_payload, format_error_code
    )
    from ..can_io import CANBusManager, CANReaderThread
