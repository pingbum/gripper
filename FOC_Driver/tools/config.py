# -*- coding: utf-8 -*-

# 드라이버/프로토콜 상수
MODE_CURRENT = 1       # 전류 루프 모드
MODE_VELOCITY = 3      # 속도 루프 모드
MODE_POSITION = 5      # 위치 루프 모드
MODE_CALIBRATION = 6   # 튜닝/캘리브레이션 트리거(Write 전용)
MODE_COGGING_COMPENSATION = 0x30
MODE_COGGING_TOGGLE = 0x31

# 기본 GUI/통신 값
DEFAULT_INTERFACE = "socketcan"  # COM 포트 보이면 slcan
DEFAULT_CHANNEL = "can0"    # 예시값
DEFAULT_BITRATE = 1000000

# 수신 버퍼 최대 길이(점 개수)
MAX_POINTS = 5000

# 그래프 업데이트 주기(ms)
UPDATE_INTERVAL_MS = 10

# 그래프 렌더링 최대 주파수(Hz)
PLOT_MAX_HZ = 100

# 에러 플래그(펌웨어와 동일 비트)
ERROR_FLAG_CAN = 0x01
ERROR_FLAG_SPI = 0x02
ERROR_FLAG_DRIVER = 0x04

# ... 기존 상수들 ...
REF_SEND_HZ = 1000  # 참조파 전송/샘플링 주기(Hz). CAN 버스 부하 고려해 50~200 사이 권장
