# -*- coding: utf-8 -*-
from PyQt5 import QtWidgets, QtCore
import time
import threading
from collections import deque
import os
import sys

if __package__ in (None, ""):
    sys.path.append(os.path.dirname(os.path.dirname(__file__)))
    from gui.deps import (
        MAX_POINTS, UPDATE_INTERVAL_MS, PLOT_MAX_HZ,
        CANBusManager
    )
    from gui.ui_builder import UIBuilderMixin
    from gui.plot_helpers import PlotHelperMixin
    from gui.can_handlers import CANHandlersMixin
    from gui.utils import UtilsMixin
    from gui.connection_handlers import ConnectionHandlersMixin
else:
    from .deps import (
        MAX_POINTS, UPDATE_INTERVAL_MS, PLOT_MAX_HZ,
        CANBusManager
    )
    from .ui_builder import UIBuilderMixin
    from .plot_helpers import PlotHelperMixin
    from .can_handlers import CANHandlersMixin
    from .utils import UtilsMixin
    from .connection_handlers import ConnectionHandlersMixin


class MainWindow(
    QtWidgets.QMainWindow,
    UIBuilderMixin,
    PlotHelperMixin,
    CANHandlersMixin,
    UtilsMixin,
    ConnectionHandlersMixin,
):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("MA732 CAN Monitor/Control")
        self.resize(1400, 850)
        self.t_ref = deque(maxlen=MAX_POINTS)
        self.y_ref = deque(maxlen=MAX_POINTS)
        self.ref_running = False
        self._last_plot_ts = 0.0
        self._plot_min_dt = 1.0 / max(1.0, float(PLOT_MAX_HZ))
        self._last_sample_ts = 0.0
        self.ref_view = None

        # CAN
        self.manager = CANBusManager()
        self.capture = None
        self._disconnect_thread = None
        self._disconnect_error = None
        self._tx_resume_thread = None
        self._tx_resume_result = None
        self._tx_resume_writer = None
        self._closing = False
        self._last_listen_id = None
        self._last_visibility = None
        self.reader_stop = threading.Event()
        self.reader_thread = None
        self.scan_hits = set()
        self.scan_lock = threading.Lock()
        self.scan_thread = None
        self.scan_stop = threading.Event()
        self._scan_in_progress = False
        self.scan_timer = QtCore.QTimer(self)
        self.scan_timer.timeout.connect(self._update_motor_list)
        self.read_watch = {
            "eid": None,
            "data": None,
            "ts": None,
            "func_id": None,
            "tag": None,
        }
        self.param_read_queue = []
        self.read_retry_left = 0
        self.read_param_current = None
        self.read_lock = threading.Lock()
        self.read_pending = False
        self.read_deadline = 0.0
        self.read_type = "int32"

        # 데이터 버퍼
        self.t_buf = deque(maxlen=MAX_POINTS)
        self.pos_buf = deque(maxlen=MAX_POINTS)
        self.spd_buf = deque(maxlen=MAX_POINTS)
        self.cur_buf = deque(maxlen=MAX_POINTS)
        self.tmp_buf = deque(maxlen=MAX_POINTS)
        self.last_error_code = None
        self._rx_count = 0
        self._last_rate_ts = time.monotonic()

        self._build_ui()
        self._apply_style()
        self._on_interface_changed()

        # 타이머
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._drain_and_update)
        self.timer.start(UPDATE_INTERVAL_MS)

    def closeEvent(self, e):
        self._closing = True
        if self.manager.bus is not None or self._disconnect_thread is not None:
            self._on_disconnect()
            e.ignore()
            return
        if self.capture is not None and not self.capture.done.is_set():
            self.capture.stop()
            e.ignore()
            return
        super().closeEvent(e)
