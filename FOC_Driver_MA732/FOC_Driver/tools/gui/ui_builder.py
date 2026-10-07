# -*- coding: utf-8 -*-
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg

from .deps import (
    MODE_CURRENT, MODE_VELOCITY, MODE_POSITION,
    DEFAULT_INTERFACE, DEFAULT_CHANNEL, DEFAULT_BITRATE
)


class UIBuilderMixin:
    def _apply_style(self):
        """UI에 현대적인 다크 플랫 스타일 시트 및 탭 스타일을 적용합니다."""
        self.setStyleSheet("""
            /* 전체 배경 및 폰트 */
            QMainWindow, QWidget {
                background-color: #1e1e1e;
                color: #d4d4d4;
                font-family: 'Segoe UI', 'Malgun Gothic', sans-serif;
                font-size: 10pt;
            }

            /* 탭 위젯 스타일 (Chrome 스타일) */
            QTabWidget::pane {
                border: 1px solid #333333;
                top: -1px;
                background-color: #1e1e1e;
            }
            QTabBar::tab {
                background: #2d2d2d;
                border: 1px solid #333333;
                border-bottom: none;
                padding: 10px 30px;
                margin-right: 2px;
                border-top-left-radius: 4px;
                border-top-right-radius: 4px;
                color: #888888;
            }
            QTabBar::tab:selected {
                background: #1e1e1e;
                color: #569cd6;
                font-weight: bold;
                border-bottom: 2px solid #007acc;
            }
            QTabBar::tab:hover:!selected {
                background: #333333;
                color: #cccccc;
            }

            /* 그룹 박스 */
            QGroupBox {
                border: 1px solid #333333;
                border-radius: 6px;
                margin-top: 1.2em;
                padding-top: 10px;
                font-weight: bold;
                text-transform: uppercase;
                color: #569cd6;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }

            /* 버튼 디자인 */
            QPushButton {
                background-color: #333333;
                border: 1px solid #444444;
                border-radius: 4px;
                padding: 6px 12px;
                min-width: 80px;
                color: white;
            }
            QPushButton:hover {
                background-color: #444444;
                border: 1px solid #555555;
            }
            QPushButton:pressed {
                background-color: #252525;
            }
            QPushButton#btn_connect, QPushButton#btn_write, QPushButton#btn_raw_send {
                background-color: #007acc;
                border: none;
            }
            QPushButton#btn_connect:hover { background-color: #0098ff; }

            QPushButton#btn_disconnect {
                background-color: #d13438;
                border: none;
            }
            QPushButton#btn_disconnect:hover { background-color: #e81123; }

            /* 입력창 */
            QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox {
                background-color: #252526;
                border: 1px solid #3e3e42;
                border-radius: 3px;
                padding: 4px;
                color: #cccccc;
            }
            QLineEdit:focus, QSpinBox:focus, QComboBox:focus {
                border: 1px solid #007acc;
            }

            /* 로그 창 */
            QPlainTextEdit {
                background-color: #121212;
                border: 1px solid #333333;
                border-radius: 4px;
                font-family: 'Consolas', 'Courier New', monospace;
                font-size: 9pt;
                color: #85e89d;
            }
        """)

        # 스타일 ID 부여
        self.btn_connect.setObjectName("btn_connect")
        self.btn_disconnect.setObjectName("btn_disconnect")
        self.btn_write.setObjectName("btn_write")
        self.btn_raw_send.setObjectName("btn_raw_send")

    def _build_ui(self):
        cw = QtWidgets.QWidget(self)
        self.setCentralWidget(cw)
        main_vbox = QtWidgets.QVBoxLayout(cw)
        main_vbox.setContentsMargins(5, 5, 5, 5)

        # 메인 탭 위젯 생성
        self.tabs = QtWidgets.QTabWidget()
        main_vbox.addWidget(self.tabs)

        # =====================================================================
        # [PAGE 1: Monitor & Control]
        # =====================================================================
        self.tab_monitor = QtWidgets.QWidget()
        self.tabs.addTab(self.tab_monitor, "Monitor & Control")
        monitor_layout = QtWidgets.QHBoxLayout(self.tab_monitor)

        # --- 1-1. 좌측 사이드바 (Connection & Write) ---
        left_panel = QtWidgets.QWidget()
        left_vbox = QtWidgets.QVBoxLayout(left_panel)
        monitor_layout.addWidget(left_panel, 0)

        # Connection Group
        conn = QtWidgets.QGroupBox("Connection")
        gl = QtWidgets.QGridLayout(conn)
        self.iface_combo = QtWidgets.QComboBox()
        self.iface_combo.addItems(["gs_usb", "slcan", "socketcan"])
        self.iface_combo.setCurrentText(DEFAULT_INTERFACE)
        self.channel_combo = QtWidgets.QComboBox()
        self.channel_combo.setEditable(True)
        self.channel_edit = self.channel_combo.lineEdit()
        self.channel_edit.setText(DEFAULT_CHANNEL)
        self.btn_refresh_channels = QtWidgets.QPushButton("Refresh ports")
        self.connection_hint = QtWidgets.QLabel()
        self.connection_hint.setWordWrap(True)
        self.connection_hint.setMaximumWidth(260)
        self.bitrate_edit = QtWidgets.QLineEdit(str(DEFAULT_BITRATE))
        self.btn_connect = QtWidgets.QPushButton("Connect")
        self.btn_disconnect = QtWidgets.QPushButton("Disconnect")
        self.btn_disconnect.setEnabled(False)
        gl.addWidget(QtWidgets.QLabel("Interface"), 0, 0); gl.addWidget(self.iface_combo, 0, 1)
        gl.addWidget(QtWidgets.QLabel("Channel"), 1, 0);   gl.addWidget(self.channel_combo, 1, 1)
        gl.addWidget(QtWidgets.QLabel("Bitrate"), 2, 0);   gl.addWidget(self.bitrate_edit, 2, 1)
        gl.addWidget(self.btn_connect, 3, 0);              gl.addWidget(self.btn_disconnect, 3, 1)
        gl.addWidget(self.btn_refresh_channels, 4, 0, 1, 2)
        gl.addWidget(self.connection_hint, 5, 0, 1, 2)
        left_vbox.addWidget(conn)

        # Write Command Group
        ctrl = QtWidgets.QGroupBox("Write Command")
        gl2 = QtWidgets.QGridLayout(ctrl)
        self.spin_driver = QtWidgets.QSpinBox(); self.spin_driver.setRange(0, 255); self.spin_driver.setValue(1)
        self.combo_mode = QtWidgets.QComboBox()
        self.combo_mode.addItem("Current (A)", MODE_CURRENT)
        self.combo_mode.addItem("Velocity (eRPM)", MODE_VELOCITY)
        self.combo_mode.addItem("Position (deg)", MODE_POSITION)
        self.edit_value = QtWidgets.QLineEdit("0.0")
        self.btn_write = QtWidgets.QPushButton("Write")
        self.btn_calib = QtWidgets.QPushButton("Calibration")
        self.spin_cogging_rpm = QtWidgets.QDoubleSpinBox()
        self.spin_cogging_rpm.setRange(1.0, 20.0)
        self.spin_cogging_rpm.setValue(5.0)
        self.spin_cogging_rpm.setSuffix(" RPM")
        self.btn_cogging = QtWidgets.QPushButton("Start Cogging")
        self.btn_cogging_comp = QtWidgets.QPushButton("Cogging Comp: ON")
        self.btn_cogging_comp.setCheckable(True)
        self.btn_cogging_comp.setChecked(True)
        gl2.addWidget(QtWidgets.QLabel("Driver ID"), 0, 0); gl2.addWidget(self.spin_driver, 0, 1)
        gl2.addWidget(QtWidgets.QLabel("Mode"), 1, 0);      gl2.addWidget(self.combo_mode, 1, 1)
        gl2.addWidget(QtWidgets.QLabel("Value"), 2, 0);     gl2.addWidget(self.edit_value, 2, 1)
        gl2.addWidget(self.btn_write, 3, 0);                gl2.addWidget(self.btn_calib, 3, 1)
        gl2.addWidget(QtWidgets.QLabel("Cogging speed"), 4, 0); gl2.addWidget(self.spin_cogging_rpm, 4, 1)
        gl2.addWidget(self.btn_cogging, 5, 0, 1, 2)
        gl2.addWidget(self.btn_cogging_comp, 6, 0, 1, 2)
        left_vbox.addWidget(ctrl)

        # Control Params (moved to left)
        ctrl_params = QtWidgets.QGroupBox("Control Params")
        gp = QtWidgets.QGridLayout(ctrl_params)
        self.spin_ctrl_bw = QtWidgets.QDoubleSpinBox()
        self.spin_ctrl_bw.setRange(0.1, 10000.0)
        self.spin_ctrl_bw.setDecimals(2)
        self.spin_ctrl_bw.setSingleStep(1.0)
        self.spin_ctrl_bw.setValue(100.0)

        self.spin_ctrl_kp = QtWidgets.QDoubleSpinBox()
        self.spin_ctrl_kp.setRange(0.0, 1e6)
        self.spin_ctrl_kp.setDecimals(6)
        self.spin_ctrl_kp.setSingleStep(0.1)
        self.spin_ctrl_kp.setValue(0.0)

        self.spin_ctrl_ki = QtWidgets.QDoubleSpinBox()
        self.spin_ctrl_ki.setRange(0.0, 1e6)
        self.spin_ctrl_ki.setDecimals(6)
        self.spin_ctrl_ki.setSingleStep(0.1)
        self.spin_ctrl_ki.setValue(0.0)

        self.btn_ctrl_send = QtWidgets.QPushButton("Send Params")
        self.btn_flash_update = QtWidgets.QPushButton("Flash Update")

        gp.addWidget(QtWidgets.QLabel("Cur BW (Hz)"), 0, 0); gp.addWidget(self.spin_ctrl_bw, 0, 1)
        gp.addWidget(QtWidgets.QLabel("Vel Kp"),      1, 0); gp.addWidget(self.spin_ctrl_kp, 1, 1)
        gp.addWidget(QtWidgets.QLabel("Vel Ki"),      2, 0); gp.addWidget(self.spin_ctrl_ki, 2, 1)
        gp.addWidget(self.btn_ctrl_send, 3, 0, 1, 2)
        gp.addWidget(self.btn_flash_update, 4, 0, 1, 2)
        left_vbox.addWidget(ctrl_params)
        left_vbox.addStretch()

        # --- 1-2. 중앙 (Graph & Log) ---
        center_panel = QtWidgets.QWidget()
        center_vbox = QtWidgets.QVBoxLayout(center_panel)
        monitor_layout.addWidget(center_panel, 1)

        self.plot = pg.PlotWidget()
        self.plot.showGrid(x=True, y=True)
        center_vbox.addWidget(self.plot, 4)

        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(500)
        self.log.setMaximumHeight(200)
        center_vbox.addWidget(self.log, 1)

        # --- 1-3. 우측 사이드바 (Filter & Config) ---
        right_panel = QtWidgets.QWidget()
        right_vbox = QtWidgets.QVBoxLayout(right_panel)
        monitor_layout.addWidget(right_panel, 0)

        # Display / Filter
        flt = QtWidgets.QGroupBox("Display / Filter")
        gl3 = QtWidgets.QVBoxLayout(flt)
        self.combo_motor = QtWidgets.QComboBox(); self.combo_motor.addItem("Select Motor")
        self.spin_listen = QtWidgets.QSpinBox(); self.spin_listen.setRange(0, 255); self.spin_listen.setValue(1)
        self.cb_pos = QtWidgets.QCheckBox("Position (deg)"); self.cb_pos.setChecked(True)
        self.cb_spd = QtWidgets.QCheckBox("Speed (eRPM)");  self.cb_spd.setChecked(True)
        self.cb_cur = QtWidgets.QCheckBox("Current (A)");   self.cb_cur.setChecked(True)
        self.cb_tmp = QtWidgets.QCheckBox("Temp (°C)");     self.cb_tmp.setChecked(True)
        self.lbl_error = QtWidgets.QLabel("Error: waiting for selected ID")
        self.lbl_rate = QtWidgets.QLabel("Rate: RX 0.0 Hz / TX 0.0 Hz")
        self.lbl_rate.setToolTip("RX: all valid received IDs. TX: successful backend sends (not motor acknowledgements).")
        self.lbl_tx_health = QtWidgets.QLabel("TX skipped: 0")
        self.btn_record = QtWidgets.QPushButton("Record RX CSV")
        self.btn_record.setEnabled(False)
        self.lbl_record = QtWidgets.QLabel("CSV: off")
        self.btn_scan = QtWidgets.QPushButton("Scan Motors")
        gl3.addWidget(QtWidgets.QLabel("Motor")); gl3.addWidget(self.combo_motor)
        gl3.addWidget(QtWidgets.QLabel("Listen Driver ID")); gl3.addWidget(self.spin_listen)
        gl3.addWidget(self.cb_pos); gl3.addWidget(self.cb_spd)
        gl3.addWidget(self.cb_cur); gl3.addWidget(self.cb_tmp)
        gl3.addWidget(self.lbl_error); gl3.addWidget(self.lbl_rate)
        gl3.addWidget(self.lbl_tx_health)
        gl3.addWidget(self.btn_record); gl3.addWidget(self.lbl_record)
        gl3.addWidget(self.btn_scan)
        right_vbox.addWidget(flt)

        # 2. Reference Generator
        ref = QtWidgets.QGroupBox("Reference Gen")
        gr = QtWidgets.QGridLayout(ref)
        self.cb_ref_show = QtWidgets.QCheckBox("Show Ref"); self.cb_ref_show.setChecked(True)
        self.combo_shape = QtWidgets.QComboBox(); self.combo_shape.addItems(["Sine", "Triangle", "Square"])
        self.spin_amp = QtWidgets.QDoubleSpinBox(); self.spin_amp.setRange(0.0, 1e9); self.spin_amp.setDecimals(2); self.spin_amp.setSingleStep(0.1); self.spin_amp.setValue(1.0)
        self.spin_freq = QtWidgets.QDoubleSpinBox(); self.spin_freq.setRange(0.01, 200.0); self.spin_freq.setValue(1.0)
        self.btn_ref_toggle = QtWidgets.QPushButton("Start Ref")
        from config import REF_SEND_HZ, MAX_REF_SEND_HZ
        self.spin_tx_hz = QtWidgets.QSpinBox()
        self.spin_tx_hz.setRange(1, MAX_REF_SEND_HZ)
        self.spin_tx_hz.setValue(REF_SEND_HZ)

        gr.addWidget(QtWidgets.QLabel("Shape"), 0, 0); gr.addWidget(self.combo_shape, 0, 1)
        gr.addWidget(QtWidgets.QLabel("Amp"), 1, 0);   gr.addWidget(self.spin_amp, 1, 1)
        gr.addWidget(QtWidgets.QLabel("Freq"), 2, 0);  gr.addWidget(self.spin_freq, 2, 1)
        gr.addWidget(QtWidgets.QLabel("TX Hz (target)"), 3, 0); gr.addWidget(self.spin_tx_hz, 3, 1)
        gr.addWidget(self.cb_ref_show, 4, 0);          gr.addWidget(self.btn_ref_toggle, 4, 1)
        right_vbox.addWidget(ref)

        # 3. Axis Controls
        axis_ctrl = QtWidgets.QGroupBox("Axis & Window")
        gl4 = QtWidgets.QGridLayout(axis_ctrl)
        self.slider_timewin = QtWidgets.QSlider(QtCore.Qt.Horizontal); self.slider_timewin.setRange(1, 100); self.slider_timewin.setValue(50)
        self.lbl_timewin = QtWidgets.QLabel("5.0 s")
        self.spin_timewin = QtWidgets.QDoubleSpinBox(); self.spin_timewin.setVisible(False); self.spin_timewin.setValue(5.0)
        self.cb_follow = QtWidgets.QCheckBox("Follow X"); self.cb_follow.setChecked(True)
        self.btn_fit = QtWidgets.QPushButton("Auto Fit")
        self.btn_fit_y = QtWidgets.QPushButton("Auto Y")

        gl4.addWidget(QtWidgets.QLabel("Time Window"), 0, 0, 1, 2)
        gl4.addWidget(self.slider_timewin, 1, 0); gl4.addWidget(self.lbl_timewin, 1, 1)
        gl4.addWidget(self.cb_follow, 2, 0, 1, 2)
        gl4.addWidget(self.btn_fit, 3, 0); gl4.addWidget(self.btn_fit_y, 3, 1)
        right_vbox.addWidget(axis_ctrl)
        right_vbox.addStretch()

        # =====================================================================
        # [PAGE 2: RAW CAN]
        # =====================================================================
        self.tab_raw = QtWidgets.QWidget()
        self.tabs.addTab(self.tab_raw, "RAW CAN")
        raw_main_layout = QtWidgets.QVBoxLayout(self.tab_raw)

        raw_group = QtWidgets.QGroupBox("Raw CAN Communication (Expert Mode)")
        gr2 = QtWidgets.QGridLayout(raw_group)
        
        self.spin_raw_motor = QtWidgets.QSpinBox(); self.spin_raw_motor.setRange(0, 255); self.spin_raw_motor.setValue(1)
        self.spin_raw_func = QtWidgets.QSpinBox();  self.spin_raw_func.setRange(0, 255);  self.spin_raw_func.setValue(0)
        self.edit_raw_value = QtWidgets.QLineEdit("0")
        self.combo_raw_type = QtWidgets.QComboBox(); self.combo_raw_type.addItems(["int32", "uint32", "float32"])
        self.btn_raw_send = QtWidgets.QPushButton("Send (Write)")
        
        self.spin_read_motor = QtWidgets.QSpinBox(); self.spin_read_motor.setRange(0, 255); self.spin_read_motor.setValue(1)
        self.spin_read_func = QtWidgets.QSpinBox();  self.spin_read_func.setRange(0, 255);  self.spin_read_func.setValue(7)
        self.combo_read_type = QtWidgets.QComboBox(); self.combo_read_type.addItems(["int32", "uint32", "float32"])
        self.btn_read = QtWidgets.QPushButton("Read Request")
        self.lbl_read_value = QtWidgets.QLabel("Read Result: -")
        self.lbl_read_value.setStyleSheet("font-size: 14pt; color: #85e89d; font-weight: bold; margin-top: 20px;")

        # Layout for Raw Send
        gr2.addWidget(QtWidgets.QLabel("Motor ID"), 0, 0); gr2.addWidget(self.spin_raw_motor, 0, 1)
        gr2.addWidget(QtWidgets.QLabel("Func ID"), 1, 0);  gr2.addWidget(self.spin_raw_func, 1, 1)
        gr2.addWidget(QtWidgets.QLabel("Value"), 2, 0);    gr2.addWidget(self.edit_raw_value, 2, 1)
        gr2.addWidget(QtWidgets.QLabel("Data Type"), 3, 0); gr2.addWidget(self.combo_raw_type, 3, 1)
        gr2.addWidget(self.btn_raw_send, 4, 0, 1, 2)
        
        # Separator Line
        line = QtWidgets.QFrame(); line.setFrameShape(QtWidgets.QFrame.HLine); line.setFrameShadow(QtWidgets.QFrame.Sunken)
        gr2.addWidget(line, 5, 0, 1, 2)

        # Layout for Raw Read
        gr2.addWidget(QtWidgets.QLabel("Read Motor ID"), 6, 0); gr2.addWidget(self.spin_read_motor, 6, 1)
        gr2.addWidget(QtWidgets.QLabel("Read Func ID"), 7, 0);  gr2.addWidget(self.spin_read_func, 7, 1)
        gr2.addWidget(QtWidgets.QLabel("Read Type"), 8, 0);     gr2.addWidget(self.combo_read_type, 8, 1)
        gr2.addWidget(self.btn_read, 9, 0, 1, 2)
        gr2.addWidget(self.lbl_read_value, 10, 0, 1, 2)
        
        raw_main_layout.addWidget(raw_group)
        raw_main_layout.addStretch()

        # ================== [그래프 초기화 및 시그널 연결] ==================
        self.btn_connect.clicked.connect(self._on_connect)
        self.iface_combo.currentTextChanged.connect(self._on_interface_changed)
        self.btn_refresh_channels.clicked.connect(self._refresh_channels)
        self.btn_record.clicked.connect(self._on_record_toggle)
        self.btn_disconnect.clicked.connect(self._on_disconnect)
        self.btn_write.clicked.connect(self._on_write)
        self.btn_calib.clicked.connect(self._on_calib)
        self.combo_mode.currentIndexChanged.connect(self._on_mode_changed)
        self.btn_cogging.clicked.connect(self._on_cogging)
        self.btn_cogging_comp.toggled.connect(self._on_cogging_toggle)
        self.edit_value.editingFinished.connect(self._on_tx_value_changed)
        self.btn_raw_send.clicked.connect(self._on_raw_send)
        self.btn_read.clicked.connect(self._on_read_request)
        self.btn_ref_toggle.clicked.connect(self._on_ref_toggle)
        self.slider_timewin.valueChanged.connect(self._on_timewin_slider)
        self.spin_timewin.valueChanged.connect(self._on_timewin_spin)
        self.btn_fit.clicked.connect(self._on_fit)
        self.btn_fit_y.clicked.connect(self._on_fit_y)
        self.combo_motor.currentIndexChanged.connect(self._on_motor_select)
        self.btn_scan.clicked.connect(self._on_scan_click)
        self.btn_ctrl_send.clicked.connect(self._on_ctrl_send)
        self.btn_flash_update.clicked.connect(self._on_flash_update)

        # 멀티 축 및 커브 설정
        plot_item = self.plot.getPlotItem()
        self.legend = self.plot.addLegend()
        self.vb_pos = plot_item.getViewBox()
        self.vb_spd = pg.ViewBox(); self.vb_cur = pg.ViewBox(); self.vb_tmp = pg.ViewBox(); self.vb_ref = pg.ViewBox()
        plot_item.scene().addItem(self.vb_spd); plot_item.scene().addItem(self.vb_cur)
        plot_item.scene().addItem(self.vb_tmp); plot_item.scene().addItem(self.vb_ref)

        self.axis_spd = pg.AxisItem("right"); self.axis_cur = pg.AxisItem("right")
        self.axis_tmp = pg.AxisItem("right"); self.axis_ref = pg.AxisItem("right")
        plot_item.layout.addItem(self.axis_spd, 2, 3); plot_item.layout.addItem(self.axis_cur, 2, 4)
        plot_item.layout.addItem(self.axis_tmp, 2, 5); plot_item.layout.addItem(self.axis_ref, 2, 6)

        self.axis_spd.linkToView(self.vb_spd); self.axis_cur.linkToView(self.vb_cur)
        self.axis_tmp.linkToView(self.vb_tmp); self.axis_ref.linkToView(self.vb_ref)
        self.vb_spd.setXLink(self.vb_pos); self.vb_cur.setXLink(self.vb_pos)
        self.vb_tmp.setXLink(self.vb_pos); self.vb_ref.setXLink(self.vb_pos)

        for vb in (self.vb_spd, self.vb_cur, self.vb_tmp, self.vb_ref):
            vb.setMouseEnabled(x=False, y=False)

        plot_item.vb.sigResized.connect(self._sync_views)

        col_pos, col_spd, col_cur, col_tmp, col_ref = '#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd'
        self.curve_pos = pg.PlotCurveItem(pen=pg.mkPen(col_pos, width=2))
        self.curve_spd = pg.PlotCurveItem(pen=pg.mkPen(col_spd, width=2))
        self.curve_cur = pg.PlotCurveItem(pen=pg.mkPen(col_cur, width=2))
        self.curve_tmp = pg.PlotCurveItem(pen=pg.mkPen(col_tmp, width=2))
        self.curve_ref = pg.PlotCurveItem(pen=pg.mkPen(col_ref, width=2, style=QtCore.Qt.DashLine))

        self.vb_pos.addItem(self.curve_pos); self.vb_spd.addItem(self.curve_spd)
        self.vb_cur.addItem(self.curve_cur); self.vb_tmp.addItem(self.curve_tmp)
        self.vb_ref.addItem(self.curve_ref)

        self.legend.addItem(self.curve_pos, "pos(deg)"); self.legend.addItem(self.curve_spd, "spd(eRPM)")
        self.legend.addItem(self.curve_cur, "cur(A)");   self.legend.addItem(self.curve_tmp, "tmp(C)")
        self.legend.addItem(self.curve_ref, "ref")

        self._sync_views()
        self._set_ref_axis_target(self.combo_mode.currentData())
        self.spin_amp.valueChanged.connect(self._on_ref_parameters_changed)
        self.spin_freq.valueChanged.connect(self._on_ref_parameters_changed)
        self.combo_shape.currentTextChanged.connect(self._on_ref_parameters_changed)
        self.combo_mode.currentIndexChanged.connect(self._on_ref_parameters_changed)
        self.spin_driver.valueChanged.connect(self._on_ref_parameters_changed)

    def _update_timewin_label(self, value: float):
        self.lbl_timewin.setText(f"{value:.1f} s")

    def _on_timewin_slider(self, slider_value: int):
        value = slider_value / 10.0
        if abs(self.spin_timewin.value() - value) > 1e-6:
            self.spin_timewin.setValue(value)
        self._update_timewin_label(value)

    def _on_timewin_spin(self, value: float):
        slider_value = int(round(value * 10.0))
        if self.slider_timewin.value() != slider_value:
            self.slider_timewin.setValue(slider_value)
        self._update_timewin_label(value)
