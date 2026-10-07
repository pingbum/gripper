# FD CAN Slave Monitor/Control (Python + CANable) README

현재 Windows/Linux 설치, 연결 설정, 1000Hz TX worker, 40Hz 그래프 및
CSV 기록 사용법은 [monitor_setup.md](monitor_setup.md)를 참고하세요.

---

## 1) 프로젝트 개요

STM32G474(FDCAN)로 구동되는 **FD can slave 모터 드라이버**와 PC를 연결해:

* 목표 **전류(A)**, **속도(eRPM)** 를 **Write 버튼**으로 전송
* 드라이버가 **브로드캐스트로 보내는 상태 프레임**(position/erpm/current/temp)을 실시간 수신
* 그래프에서 각 항목을 **체크박스로 On/Off**
* **Raw CAN**: Motor ID/Function ID + 32-bit 값 전송(int32/uint32/float32) 및 Read 요청
* **Control Params**: 전류 대역폭/속도 Kp/Ki 설정 및 Flash 저장
* **Motor Scan**: 스캔으로 연결된 모터 ID 표시

프로토콜은 제공하신 펌웨어에 맞춥니다.

* **Tx(PC→모터, 확장 ID)**: `EID = (mode_id << 8) | driver_id`

  * `mode_id=1` Current: **s32 big-endian: mA** (GUI 입력 A → ×1000)
  * `mode_id=3` Velocity: **s32 big-endian: eRPM**
  * DLC=8 (상위 4바이트는 0 패딩)
* **Rx(모터→PC, 확장 ID, 8B)**:
  `pos_deg = int16_BE / 10`, `spd_erpm = int16_BE × 10`, `cur_A = int16_BE / 1000`, `temp_C = int8`, `error = uint8`

> 펌웨어의 상태/응답은 **ISO CAN FD+BRS, 1M/5M**입니다. 앱의 `socketcan` 송신은 기본 **FD, BRS OFF(1M)**이며 `TX BRS (5M)`으로 바꿀 수 있습니다.
> FD 지원 SocketCAN 또는 CANable 2.0 호환 FD SLCAN 펌웨어가 필요합니다.
> Classic-only `gs_usb`/SLCAN 어댑터는 지원하지 않습니다.
> Connect는 수신만 시작하며, 스캔·파라미터 읽기는 버튼을 눌렀을 때 전송합니다.

---

## 2) 폴더 구조

```
tools/
├─ app.py                # 진입점
├─ config.py             # 상수/기본 설정
├─ protocol.py           # ID/스케일/페이로드 pack/unpack
├─ can_io.py             # python-can 연결/송신/수신 스레드
├─ signal_gen.py         # reference 파형
└─ gui/
   ├─ __init__.py
   ├─ deps.py            # 공용 imports/상수
   ├─ ui_builder.py      # UI 레이아웃/스타일
   ├─ plot_helpers.py    # 그래프/축/디케이메이션
   ├─ can_handlers.py    # CAN 송수신/Ref 생성
   ├─ utils.py           # 로그/에러/Raw Send 등 유틸
   └─ main_window.py     # 메인 윈도우 (믹스인)
```

이 저장소(혹은 파일들)를 그대로 `tools/` 폴더에 두고 실행합니다.

---

## 3) 요구 사항

* **Windows 10/11 또는 Linux 데스크톱**
* **Python 3.11+** (OS별 가상환경 권장)

  ```powershell
  python --version
  ```
* 패키지:

  ```powershell
  python -m pip install -r tools/requirements.txt
  ```

  * slcan용 `pyserial`은 requirements에 명시되어 있습니다.

### 3.1 장치 드라이버/펌웨어

장치가 PC에서 **어떤 방식**으로 인식되는지에 따라 인터페이스가 달라집니다.

* **COM 포트(예: COM11)로 보이면 → `slcan`**

  * 장치 관리자 “포트(COM & LPT)”에 **USB 직렬 장치 (COM11)** 등으로 표시됩니다.
* **WinUSB 디바이스(드라이버를 Zadig로 설치) → `gs_usb`**

  * candleLight 펌웨어 기반. 장치 관리자의 “범용 직렬 버스 컨트롤러” 쪽에 나타납니다. COM 포트는 보이지 않습니다.

> 어떤 펌웨어/드라이버를 쓰는지 **본인 장치 상태에 따라 결정**됩니다. 이 README는 두 경우 모두 지원합니다.

---

## 4) 빠른 시작(Quick Start)

1. 저장소를 준비하고 패키지 설치:

   ```powershell
   python -m pip install -r tools/requirements.txt
   ```
2. 실행(둘 중 하나):

   ```powershell
   python tools/app.py
   ```
   또는 (tools 폴더로 이동)
   ```powershell
   cd tools
   python app.py
   ```
3. GUI에서 연결 설정:

   * **Interface**:

     * CANable 2.0 호환 FD 펌웨어가 있는 COM 포트는 `slcan`
     * Linux의 FD 지원 CAN 인터페이스는 `socketcan`
   * **Channel**:

     * `slcan` → `COM11` 같은 실제 포트명
     * `socketcan` → `can0` (먼저 `ip link`로 FD 1M/5M 설정)
   * **Bitrate**: `slcan`은 `1000000`, 데이터 속도는 5M. SocketCAN은 OS에서 설정
   * **Connect** 클릭
   * 상태 프레임이 보이면 그래프가 움직입니다.
4. **Write** 사용:

   * Driver ID: 보드의 `MY_DRIVER_ID`(예: 1)
   * Mode: Current 또는 Velocity
   * Value: A 또는 eRPM 입력
   * **Write** 버튼 → 확장 ID로 s32 BE 8바이트 전송

> 짧은 명령도 **FD**로 송신합니다. `TX BRS (5M)`을 끄면 송신 데이터 구간은 1M이며, FD+BRS 수신은 계속 가능합니다. 설정 변경은 Disconnect 후 적용하세요.

---

## 5) 설정 파일

`config.py`에서 기본값을 바꿀 수 있습니다.

```python
DEFAULT_INTERFACE = "socketcan"   # Linux. Windows 기본값은 "slcan"
DEFAULT_CHANNEL   = "can0"        # Windows에서는 실제 COM 포트 선택
DEFAULT_BITRATE   = 1000000
MAX_POINTS         = 5000
UPDATE_INTERVAL_MS = 25
PLOT_MAX_HZ        = 40
REF_SEND_HZ        = 1000
```

* 수신 상태는 드라이버 ID별로 최신 1개씩 보관합니다. 여러 보드를 연결해도
  다른 ID의 프레임이 선택한 보드의 최신 상태를 덮어쓰지 않습니다.
* **Listen Driver ID**로 표시할 보드를 선택합니다. 새 상태가 도착했을 때만
  그래프에 추가하며, 스캔·파라미터 응답은 상태 그래프에 섞지 않습니다.
* `REF_SEND_HZ`는 참조파 기본 목표 주기입니다. GUI에서 1~1000Hz를 선택하며,
  전용 스레드가 송신합니다. 실제 처리율은 RX/TX 표시와 TX skipped로 확인합니다.

---

## 6) 프로토콜 상세(펌웨어 기준)

### 6.1 PC → 드라이버(명령, 확장 프레임)

* **EID**: `(mode_id << 8) | driver_id`
* **DLC=8**: \[0..3]=**s32 BE**, \[4..7]=0x00
* **모드/스케일**

  | mode\_id | 의미       | GUI 입력 | 전송 s32(BE) |
  | -------- | -------- | -----: | ---------: |
  | 1        | Current  |      A |  mA(×1000) |
  | 3        | Velocity |   eRPM |  eRPM(그대로) |

### 6.2 드라이버 → PC(브로드캐스트, 확장 프레임, **8B**)

펌웨어 예시(`broadcast_motor_status`) 패킹:

* `[0..1]` int16 BE: `pos_scaled = deg × 10` → `deg = /10`
* `[2..3]` int16 BE: `speed_scaled = erpm / 10` → `erpm = ×10`
* `[4..5]` int16 BE: `current_scaled = A × 1000` → `A = /1000`
* `[6]` int8: `rev`
* `[7]` uint8: `error_code`

> 이 브로드캐스트는 펌웨어에서 `FDFormat = FDCAN_FD_CAN`로 설정돼 있습니다.
> **클래식 CAN만 되는 어댑터**는 지원하지 않습니다. FD 지원 장치를 사용하세요.

---

## 7) 사용 예시

### 7.1 COM11(직렬, slcan)로 연결

* Interface: `slcan`
* Channel: `COM11`
* Bitrate: `1000000` (데이터 5M, CANable 2.0 호환 FD 펌웨어 필요)
* Connect

### 7.2 명령 전송

* Driver ID: `1`
* Mode: **Current (A)**, Value: `1.5` → 내부에서 `1500 mA`로 s32 BE 전송
* Mode: **Velocity (eRPM)**, Value: `12000` → s32 BE 전송

### 7.3 그래프 토글

* 상단 “Display / Filter”에서 Position/Speed/Current/Temp 체크박스 On/Off

### 7.4 Reference/축 동작

* **Reference 축**은 모드에 따라 측정 축과 동일하게 붙습니다.
  * Current → Current 축
  * Velocity → Speed 축
* Mode/Value 변경 시 **Reference 진폭 기본값**으로 리셋
  * Current: 0.1 A
  * Velocity: 500 eRPM
* Amplitude 스핀박스 증가 단위
  * Current: 0.1
  * Velocity: 100

### 7.5 Time Window 슬라이더

* 0.1s ~ 10s 범위 슬라이더로 X축 창 길이 조절

### 7.6 Motor Scan / Read

* **Connect 직후 수신만 시작**: 브로드캐스트 상태에서 모터 ID를 자동으로 목록에 추가
* **Scan Motors** 버튼으로 재스캔(드롭다운 목록 초기화 후 갱신)
* Motor 드롭다운 선택 시 **Driver ID / Listen ID / Raw CAN ID**가 함께 변경됨
* **Read Request**는 Func ID에 **MSB 플래그(0x80)** 자동 적용
  * 예) `0x20` 입력 → 요청 `0xA0`, 응답 EID = `(0xA0 << 8) | motor_id`
  * `0x07`은 **1바이트**, `0x11/0x20/0x21`은 **4바이트 BE float32**

### 7.7 Control Params

* Cur BW(Hz) → `0x11` (float32)
* Vel Kp → `0x20` (float32)
* Vel Ki → `0x21` (float32)
* **Flash Update** → `0x10`
* **Read Params**를 누르면 위 파라미터를 읽음(각 2회 재시도)
  * Driver ID 기준으로 읽습니다. 다른 모터 값은 Driver ID 변경 후 Read Params를 누르세요.

---

## 8) 문제 해결(Troubleshooting)

**연결 자체가 안 됨**

* `slcan`인데 포트가 안 보임 → CANable 2.0 호환 FD SLCAN 펌웨어와 실제 COM 포트를 확인.
* Linux `socketcan` → FD 지원 장치인지, `can0`에 `fd on` 및 1M/5M이 설정됐는지 확인.
* “Access denied” / “busy” → 다른 시리얼/캔 툴이 장치를 점유 중인지 확인(시리얼 모니터, 다른 GUI 등).

**프레임이 안 들어옴**

* **비트레이트**가 보드와 다르면 수신/송신 모두 실패합니다.
* **종단저항(120Ω)** 상태 확인(버스 양끝 필요).
* **확장 ID 필터**: GUI의 “Listen Driver ID”가 보드의 `MY_DRIVER_ID`(예: 1)와 일치해야 합니다.
* **CAN FD 이슈**: 브로드캐스트가 FD라면, **클래식 전용 어댑터에선 수신 불가**입니다.

  * 해결: FD 지원 어댑터와 호환 FD 펌웨어 사용.
  * 제 쪽에서 MKS CANable V2.0 Pro의 FD 지원 여부는 보장할 수 없습니다. **모델 스펙으로 확인**해 주세요.

**파이썬 에러**

* `ModuleNotFoundError: can` → `pip install python-can`
* `ModuleNotFoundError: PyQt5` → `pip install PyQt5`
* Qt platform plugin 오류 → `PyQt5` 재설치, 가상환경 재구성 시 해결되는 경우가 많음.

**간이 테스트(연결 확인)**

```python
import can
from can_io import connection_options
# tools 폴더에서 실행. CANable 2.0 호환 FD SLCAN 예시.
with can.Bus(**connection_options('slcan', 'COM11', 1000000)) as bus:
    print(bus.recv(1.0))
```

**Read가 timeout**

* Read 요청은 **확장 ID + MSB 플래그(0x80)**로 전송됩니다.
* 응답 EID는 `(func_id|0x80 << 8) | motor_id` 형식이어야 합니다.
* Motor ID가 펌웨어의 `s_driver_id`와 일치해야 응답합니다.

---

## 9) 확장/유지보수 가이드

* **프로토콜 변경**(스케일/필드/ID 규칙) → `protocol.py`
* **장치 연결/타임아웃/재연결 로직** → `can_io.py`
* **UI 레이아웃/스타일** → `gui/ui_builder.py`
* **그래프/축/디케이메이션** → `gui/plot_helpers.py`
* **CAN/Ref 동작** → `gui/can_handlers.py`
* **유틸/로그/Raw Send** → `gui/utils.py`
* **기본값·주기** → `config.py`

추가하기 쉬운 기능:

* CSV/Parquet 로깅
* 다중 Driver ID 동시 표시(곡선 색상 분리)
* 안전 범위 클램핑(예: 전류 ±6A)
* 에러 코드 바 인디케이터

---

## 10) 안전 주의

* 전류/속도 지령은 **하드웨어 손상/발열 위험**이 있습니다. 합리적 한계 내에서 테스트하십시오.
* 현장 배선에서 **역전압/쇼트/접지 루프**를 피하세요.
* 실제 기계 구동 시 **비상 정지(E-stop)** 확보 필수.

---

## 11) 라이선스/저작권

사내/개인 사용 목적 예제로 가정했습니다. 외부 배포 시 적합한 라이선스를 명시하세요.

---

## 12) 변경 이력(요약)

* v1: 단일 파일 프로토타입
* v2: **모듈 분리**(`gui` 내부 믹스인 구조) 및 UI/성능 옵션 추가

---

## 13) 자주 묻는 질문(FAQ)

**Q. COM11인데 어떤 설정을 고르죠?**
A. **Interface=slcan, Channel=COM11**, Bitrate는 보드와 동일(예: 1000000).

**Q. 그래프가 안 움직여요.**
A. 대부분 **CAN FD/클래식 불일치** 또는 **비트레이트 불일치**입니다. 위 Troubleshooting을 따라 확인해 보세요.

**Q. 속도 명령은 몇 eRPM까지 가능한가요?**
A. PC 쪽에서는 제한을 두지 않습니다. **실제 한계**는 펌웨어/모터/기계적 설계에 따릅니다.

---
