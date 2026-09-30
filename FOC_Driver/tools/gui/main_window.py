# -*- coding: utf-8 -*-
from PyQt5 import QtWidgets, QtCore
import time
import queue
import threading
from collections import deque
import os
import sys

if __package__ in (None, ""):
    sys.path.append(os.path.dirname(os.path.dirname(__file__)))
    from gui.deps import (
        REF_SEND_HZ,
        MAX_POINTS, UPDATE_INTERVAL_MS, PLOT_MAX_HZ, RX_QUEUE_SIZE,
        CANBusManager
    )
    from gui.ui_builder import UIBuilderMixin
    from gui.plot_helpers import PlotHelperMixin
    from gui.can_handlers import CANHandlersMixin
    from gui.utils import UtilsMixin
else:
    from .deps import (
        REF_SEND_HZ,
        MAX_POINTS, UPDATE_INTERVAL_MS, PLOT_MAX_HZ, RX_QUEUE_SIZE,
        CANBusManager
    )
    from .ui_builder import UIBuilderMixin
    from .plot_helpers import PlotHelperMixin
    from .can_handlers import CANHandlersMixin
    from .utils import UtilsMixin


class MainWindow(
    QtWidgets.QMainWindow,
    UIBuilderMixin,
    PlotHelperMixin,
    CANHandlersMixin,
    UtilsMixin,
):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("FD CAN Slave Monitor/Control (CANable)")
        self.resize(1400, 850)
        self.t_ref = deque(maxlen=MAX_POINTS)
        self.y_ref = deque(maxlen=MAX_POINTS)
        self.ref_running = False
        self.ref_t0 = None
        self._last_ref_plot_ts = 0.0
        self._ref_plot_min_dt = 1.0 / 100.0  # limit ref plot samples to 100 Hz
        self._last_plot_ts = 0.0
        self._plot_min_dt = 1.0 / max(1.0, float(PLOT_MAX_HZ))
        self._last_sample_ts = 0.0
        self.ref_view = None

        # 참조 전송/샘플 타이머
        self.ref_timer = QtCore.QTimer(self)
        self.ref_timer.setTimerType(QtCore.Qt.PreciseTimer)
        self.ref_timer.timeout.connect(self._on_ref_tick)
        self.ref_timer.start(int(1000.0 / max(1.0, float(REF_SEND_HZ))))

        # CAN
        self.manager = CANBusManager()
        self.msg_queue: "queue.Queue" = queue.Queue(maxsize=RX_QUEUE_SIZE)
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
        self._tx_count = 0
        self._last_rate_ts = time.time()

        self._build_ui()
        self._apply_style()

        # 타이머
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._drain_and_update)
        self.timer.start(UPDATE_INTERVAL_MS)

    def closeEvent(self, e):
        self._on_disconnect()
        super().closeEvent(e)
