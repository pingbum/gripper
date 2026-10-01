# -*- coding: utf-8 -*-
import sys

# 드라이버/프로토콜 상수
MODE_CURRENT = 1       # 전류 루프 모드
MODE_VELOCITY = 3      # 속도 루프 모드
MODE_POSITION = 5      # 위치 루프 모드
MODE_CALIBRATION = 6   # 튜닝/캘리브레이션 트리거(Write 전용)
MODE_COGGING_COMPENSATION = 0x30
MODE_COGGING_TOGGLE = 0x31

# 기본 GUI/통신 값
DEFAULT_INTERFACE = "socketcan" if sys.platform.startswith("linux") else "slcan"
DEFAULT_CHANNEL = "can0" if DEFAULT_INTERFACE == "socketcan" else ""
DEFAULT_BITRATE = 1000000

# 수신 버퍼 최대 길이(점 개수)
MAX_POINTS = 5000

# 그래프 업데이트 주기(ms)
UPDATE_INTERVAL_MS = 25

# 그래프 렌더링 최대 주파수(Hz)
PLOT_MAX_HZ = 40

# 에러 플래그(펌웨어와 동일 비트)
ERROR_FLAG_CAN = 0x01
ERROR_FLAG_SPI = 0x02
ERROR_FLAG_DRIVER = 0x04

REF_SEND_HZ = 1000  # 목표 송신 주기. 일반 OS에서 정밀 실시간 주기는 보장하지 않음.
MAX_REF_SEND_HZ = 1000
TX_QUEUE_SIZE = 128
TX_TIMEOUT_S = 0.02
RECORD_QUEUE_SIZE = 10000
