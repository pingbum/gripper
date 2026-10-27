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
├─ open_loop.py          # 독립 실행 오픈루프 전압 제어 GUI (7극쌍 RPM 표시)
├─ cogging_LUT.py        # 독립 실행 cogging 자동 측정 / 6회차+최종 LUT 점검 GUI
├─ cogging_reader.py     # Cogging 0x30/0x31/0x32 프로토콜 / TXT 저장 (Qt 의존성 없음)
├─ LUT.py                # 독립 실행 엔코더 보정 LUT 점검 GUI
├─ lut_reader.py         # LUT 읽기 / 전체 수집 / CSV 저장 (Qt 의존성 없음)
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
  * `0x07`은 **1바이트**, `0x11/0x20/0x21/0x22/0x23`은 **4바이트 BE float32**

### 7.7 Control Params

* Cur BW(Hz) → `0x11` (float32)
* Vel Kp → `0x20` (float32)
* Vel Ki → `0x21` (float32)
* Pos Kp (A/deg) → `0x22` / 읽기 `0xA2` (float32, 배율 없음)
* Pos Kd (A·s/deg) → `0x23` / 읽기 `0xA3` (float32, 배율 없음)
* **Flash Update** → `0x10`
* **Read Params**를 누르면 위 파라미터를 읽음(각 2회 재시도)
  * Driver ID 기준으로 읽습니다. 다른 모터 값은 Driver ID 변경 후 Read Params를 누르세요.
  * 읽는 도중 Driver ID가 바뀌면 읽기를 취소하여 다른 보드의 값이 섞이지 않게 합니다.
* **Send Params**는 기존 전류 BW/속도 Kp/Ki와 위치 Kp/Kd를 RAM에 전송한 뒤
  자동으로 다시 읽습니다. 쓰기 ACK가 없으므로 로그의 `Position Kp/Kd read back`과
  입력란에 표시된 읽기 값을 확인하세요. Flash 저장은 자동으로 수행하지 않습니다.

위치 게인 기본 표시값은 `Kp=0.05 A/deg`, `Kd=0.003 A·s/deg`이며 권장 튜닝값이
아닙니다. 먼저 Read Params로 현재 값을 확인하세요. 음수/NaN/±Inf 및 float32로
표현할 수 없는 위치 게인은 송신하지 않으며 0은 허용합니다. 쓰기는 정확히 4바이트
`struct.pack(">f", gain)`입니다. 요청은 기존 TX BRS 설정을 따르므로 BRS OFF로
송신할 수 있고, 위치 게인 응답은 **4바이트 Extended FD / BRS ON**만 받아들입니다.

위치 PD는 `(rev * 360 + theta_m) / GEAR_RATIO` [deg]의 출력축 위치 기준이며,
속도 PI를 거치지 않고 위치 오차의 P항과 측정 위치의 필터링된 D항으로 q축 전류
지령을 생성합니다. D 필터는 100 Hz이며 위치 게인 변경으로 모드/목표 위치/필터
상태를 변경하지 않습니다. 정지한 상태에서 **Flash Update**를 한 번 수행하면
위치 게인을 포함한 전체 제어 파라미터가 저장됩니다. `0xF1` 브로드캐스트 주파수
저장도 전체 파라미터를 저장하므로 현재 위치 게인이 함께 저장됩니다.
이전 24바이트 Flash 데이터에 위치 게인이 없으면 펌웨어 기본값을 사용합니다.
엔코더 보정 LUT와는 별개입니다.

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


### Open-loop rotation (독립 GUI, Function ID 0x04)

오픈루프 전압 제어는 `app.py`에서 분리한 `open_loop.py`로 실행합니다.
기존 모니터의 GUI와 공용 CAN 송신 코드는 오픈루프 기능 추가 전 상태로 복원했습니다.
오픈루프 전용 프로토콜, 송신 잠금, 정지 처리는 이 독립 프로그램에만 있습니다.

```bash
python tools/open_loop.py
```

Interface / Channel을 지정하고 **Connect**를 누른 뒤 Motor ID, Electrical RPM(-600..600),
Voltage(0..2.24 V)를 입력하고 `Start / Update`를 누릅니다. 입력값 변경만으로는
송신하지 않습니다. 속도 부호가 방향을 결정하고, 기계 RPM은 **eRPM / 7**입니다.
7극쌍 환산은 GUI의 RPM 표시에만 적용되며 펌웨어의 극쌍수 설정을 변경하지 않습니다.
기본 전압은 0.1 V이며 낮은 전압부터 시험하세요.
SLCAN 시리얼 어댑터는 다른 GUI와 동시에 열지 마세요.

Connect는 수신만 시작합니다. 그래프/참조파/스캔 없이 선택한 모터의 상태를
100 ms마다 표시하며, 연결/정지/해제는 백그라운드에서 처리합니다.

명령은 Extended ID `(0x04 << 8) | motor_id`, 정확히 8바이트 `>ii`
(eRPM, mV)로 전송하며 항상 FD / BRS OFF입니다.
`Stop (0 V)`는 대기 명령과 주기 송신을 취소한 뒤 `(0, 0)`을 즉시 한 번
전송합니다. 0 eRPM에서 전압이 남으면 고정 전기각에 전압을 인가합니다.

이 모드는 전류 PI를 사용하지 않으며 통신이 끊겨도 마지막 명령으로 계속
구동됩니다. 연결 해제/창 종료 시 `open_loop.py`에서 오픈루프 명령을 보낸 모든 모터에
정지를 시도합니다. 전송 성공은 펌웨어의 정지 확인 응답이 아니며,
전송 실패는 로그에 표시됩니다. 강제 종료나 통신 장애 시 정지를 보장하지 않습니다.

오프라인 검증 (CAN 장치 불필요):

```bash
QT_QPA_PLATFORM=offscreen python -B -m unittest discover -s tests/can_monitor -p 'test_open_loop.py'
```

### 엔코더 보정 LUT 점검 (독립 GUI)

기존 모니터에 수집/그래프 부하를 추가하지 않고 별도 프로그램으로 실행합니다.
`app.py`와 펌웨어는 이 기능 추가로 변경하지 않습니다. 기존 requirements를 그대로 사용합니다.

```bash
python tools/LUT.py
```

1. **Interface / Channel / Driver ID**를 지정합니다. Driver ID 기본값은 `0`입니다.
   Linux는 `socketcan / can0`, Windows의 FD SLCAN 어댑터는 `slcan / 실제 COM 포트`입니다.
   연결 설정은 위의 모니터와 같습니다. SLCAN 시리얼 어댑터는 `app.py`를 닫고 사용하세요.
   SocketCAN에서는 별도 소켓을 사용합니다. 다른 프로그램에서도 같은 LUT를 동시에 요청하지 마세요.
2. **메타데이터 읽기**로 샘플 수와 direction을 확인하거나 **전체 LUT 수집**을 누릅니다.
   버튼을 누를 때 연결하고, 메타데이터 및 각 인덱스를 순서대로 읽은 뒤 자동 해제합니다.
   CAN 연결/수집/해제는 작업 스레드에서 수행하고 그래프는 완료 시 한 번만 갱신합니다.
3. 그래프의 X축은 기계각, Y축은 **전기각 보정값 [deg]**입니다. 마우스를 올리면
   인덱스와 원래 값을 표시합니다. 통계의 min/max/mean/peak-to-peak는 유한값 기준이며,
   NaN/+Inf/-Inf 개수는 별도로 표시합니다. 그래프에서는 이상값 위치의 선을 끊습니다.
4. 전체 수집 성공 후 **CSV 저장**을 누르면 다음 세 열로 모든 샘플을 저장합니다.
   기존 파일은 덮어쓰지 않으며, 취소/실패한 부분 수집은 저장할 수 없습니다.

   ```csv
   index,mechanical_deg,offset_electrical_deg
   ```

기계각은 `index * 360 / sample_count`입니다. 현재 10000샘플의 인덱스는
`0..9999`이고 간격은 `0.036 deg`입니다. NaN/Inf 및 전체 0인 LUT도 수집/CSV에서
제거하거나 보정하지 않습니다. CSV에는 direction이 포함되지 않으므로 GUI 표시를 함께 확인하세요.

읽기 전용 요청/응답은 모두 **Extended CAN FD / BRS OFF**, big-endian입니다.
응답 ID는 요청 ID와 같은 `(read_mode << 8) | driver_id`입니다.

| Read mode | LUT.py의 요청 | 8바이트 응답 (`>If`) |
| --- | --- | --- |
| `0xC0` | 0바이트 (허용: 0/4/8바이트, 내용 무시) | `uint32 sample_count`, `float32 direction (+1/-1)` |
| `0xC1` | 4바이트 `struct.pack(">I", index)` (허용: 4/8바이트) | `uint32 index`, `float32 offset_electrical_deg` |

0/4바이트 요청을 사용하여 8바이트 응답과 구분합니다. PC TX echo는 `is_rx=False`
검사로 제외하며, 응답 ID/FD/BRS/길이/인덱스가 맞는 응답만 받아들입니다. 특히
`candump`에는 다른 소켓의 요청도 표시되므로 요청을 LUT 응답으로 해석하지 마세요.

한 번에 하나만 요청하고 같은 인덱스의 응답을 받은 후 진행합니다. 기본 타임아웃은
100 ms, **추가 재요청 3회**(최초 요청 포함 최대 4회)입니다. 제한 후에도 응답이 없으면
전체 수집을 실패 처리합니다. CAN 송수신 오류/BUS-OFF도 실패로 표시합니다.
송신 큐 부족으로 펌웨어가 응답을 생략하면 이 재요청 경로로 처리됩니다.
펌웨어의 `can_lut_tx_errors`는 큐잉 실패, `can_lut_tx_queued`는 큐잉 성공 횟수이며
버스 ACK 횟수가 아닙니다. 이 GUI는 해당 카운터를 읽지 않습니다.

반환되는 데이터는 `state.position.theta_e_offset[]`에 적용된 **필터링 후 LUT**입니다.
필터링 전 측정값은 보관되어 있지 않습니다. 샘플 수는 배열 길이이며 보정 성공 플래그가
아닙니다. 보정 로딩 실패 시 FOC 기본값인 전체 0이 반환될 수 있습니다.
읽기는 RAM/Flash 보정값이나 제어 모드를 변경하지 않고, `0x40/0x41` 쓰기 명령도 보내지 않습니다.
기존 상태 브로드캐스트/일반 파라미터 응답의 FD+BRS 설정은 그대로입니다.
프로토콜에 수집 버전/리셋 식별자가 없으므로 수집 중 리셋/재캘리브레이션을 했다면
**수집 취소 후 처음부터 다시 수집**해야 합니다.

오프라인 검증 (CAN 장치 불필요):

```bash
QT_QPA_PLATFORM=offscreen python -B -m unittest discover -s tests/can_monitor -p 'test_lut.py'
```

### Cogging 측정 / 회차별 데이터 점검 (독립 GUI)

```bash
python tools/cogging_LUT.py
```

기존 앱의 그래프/참조파와 독립적으로 실행합니다. 기존 requirements를 사용하며,
CAN 연결, 상태 읽기, 수집, 중단, TXT 저장은 모두 백그라운드에서 처리합니다.
그래프는 수집 완료 시 한 번 갱신하고 수집 진행률은 최대 10 Hz로 표시합니다.

1. **Interface / Channel / Driver ID**를 지정하고 **Connect**를 누릅니다.
   기본 Driver ID는 0이며 연결 후 상태를 약 1초마다 읽습니다. Connect는 운동 명령을
   보내지 않습니다. SocketCAN은 OS에서 FD 1M/5M 설정이 필요하고, FD SLCAN 시리얼
   어댑터는 `app.py` / `LUT.py` / `open_loop.py`와 동시에 열지 마세요.
2. 속도 PI를 먼저 안정적으로 튜닝한 뒤 1~20 mechanical RPM을 설정하고 **측정 시작**을
   누릅니다. CW 안정화 1회전 + 기록 3회전, 정지(≤0.1 RPM), CCW 안정화 1회전 +
   기록 3회전, 전류 지령 0 정지 후 펌웨어가 LUT를 계산하고 Flash에 자동 저장합니다.
   CW/CCW는 각각 양/음의 기계 RPM이며 눈으로 보는 시계 방향과 같다는 뜻은 아닙니다.
   시작 전 작은 외부 부하와 전체 회전이 가능한 상태인지 확인하세요.
3. **측정 중단 / 0 A**는 진행 중인 수집/상태 읽기를 취소한 뒤 `0x30`의 0을 전송합니다.
   자동 측정 타임아웃은 없으므로 회전 정지/진동 시 직접 중단하세요. 이미 진행 중인
   Flash 쓰기는 즉시 취소할 수 없습니다. **Disconnect / 창 종료**도 측정이 진행 중이면
   중단을 시도한 후 연결을 해제합니다. ACK/정지 보장은 아니며 실패는 로그에 표시합니다.
4. **측정 완료 후 자동 수집**은 기본 ON입니다. 상태 2, 원시 데이터 있음, LUT 유효,
   후처리 완료를 확인하면 원시 6회차와 최종 LUT의 총 **10080 bins**를 수집합니다.
   이미 측정한 데이터는 **전체 7개 수집**으로 가져올 수 있습니다. Flash 저장 실패
   (flags `0x10`)도 RAM 데이터가 유효하면 수집할 수 있습니다.
5. 재부팅 후 원시 데이터가 없고 유효한 Flash LUT가 있는 상태 0 / flags `0x0A`에서는
   **최종 LUT만 수집**을 사용합니다. 1440 bins를 수집합니다. **수집 취소**는 부분
   데이터를 저장하지 않으며 측정 중단 명령과는 별도의 기능입니다.
6. 완료된 그래프의 체크박스로 CW/CCW 각 회차와 최종 LUT를 표시합니다. X축은
   `bin * 0.25 deg`, 원시 회차의 Y값은 **기계 방향 기준 iq_ref [A]**입니다.
   측정 `i_dq_sen.q`나 토크 [Nm]가 아닙니다. 원시 데이터에는 DC 제거/보간/필터링을
   추가하지 않고, 누락 bin의 0과 NaN/Inf도 그대로 보존합니다.
7. **TXT 파일 저장**에서 상위 폴더를 고르면 Driver ID/시각이 들어간 새 하위 폴더에
   다음 7개 파일을 저장합니다. 각 파일은 bin 0..1439 순서의 전류 [A]를 한 줄에
   하나씩 기록하며 헤더가 없습니다. 기존 파일을 덮어쓰지 않습니다. 최종 LUT만
   수집했으면 `cogging_lut.txt` 하나를 저장합니다. CAN 연결 해제 후에도 저장할 수 있습니다.

   ```text
   cogging_cw_sweep_1.txt
   cogging_cw_sweep_2.txt
   cogging_cw_sweep_3.txt
   cogging_ccw_sweep_1.txt
   cogging_ccw_sweep_2.txt
   cogging_ccw_sweep_3.txt
   cogging_lut.txt
   ```

모든 요청은 Extended CAN FD / BRS OFF, ID `(mode << 8) | driver_id`입니다.

| Mode | 요청 | 응답/의미 |
| --- | --- | --- |
| `0x30` | 4바이트 `>i`, mechanical RPM × 100 | 양수 1~20 RPM 시작, 0 중단. ACK 없음 |
| `0x31` | 1바이트 `0` / `1` | 보상 OFF / ON, RAM 설정. ACK 없음 |
| `0x32` | 8바이트 전체 0 | 상태 응답: op/state/sweeps/version/bin_count/flags/reserved |
| `0x32` | `struct.pack(">BBHI", 1, selector, bin, 0)` | `struct.unpack(">BBHf", data)`, 마지막 값은 전류 [A] |

읽기에는 **0x32 자체**를 쓰며 `0xB2`를 사용하지 않습니다. 응답 ID는 요청과 같고
8바이트 FD / BRS OFF입니다. selector는 CW `0x00/0x01/0x02`, CCW `0x40/0x41/0x42`,
최종 LUT `0x80`입니다. 한 번에 한 요청만 보내고 ID/FD/BRS/길이/첫 4바이트와 TX echo
여부를 검사합니다. 기본 타임아웃은 350 ms, **추가 재요청 5회**(최대 6회 시도)이며
끝내 응답이 없으면 해당 수집을 실패 처리합니다. 진단 응답 큐 부족은 이 재시도로
처리되며 PC는 펌웨어의 큐잉 카운터를 읽지 않습니다.

상태는 0 idle, 1 측정/정지/후처리 대기, 2 다운로드 가능, 3 실패이며 버전 1,
방향별 3회차, 1440 bins를 사용합니다. flags의 `0x01` 원시 데이터, `0x02` 유효 LUT,
`0x04` 후처리/저장 진행, `0x08` Flash 성공, `0x10` Flash 실패를 각각 표시합니다.
이 상태 프레임에는 보상 ON/OFF 자체의 확인 비트가 없어 토글 버튼은 요청 전송으로
표시합니다. 측정 중 보상 전류를 추가하지 않는 처리는 펌웨어가 수행합니다.

최종 LUT의 방향별 샘플 가중 평균, `(CW + CCW) / 2`, 원형 보간, DC 제거,
±0.1 A 제한, 3점 평활화는 펌웨어에서 이미 적용합니다. PC는 이를 다시 계산하지
않습니다. 원시 6회차는 RAM에만 유지되며 재부팅/새 측정 시 없어집니다. 최종 LUT만
Flash에 저장되고 엔코더 보정 LUT `0xC0/0xC1`은 변경하지 않습니다.

수집 중 상태 변화가 관측되면 실패 처리하지만 프로토콜에 수집 버전 ID는 없습니다.
리셋/재측정하거나 다른 PC가 같은 데이터를 요청했다면 취소 후 처음부터 다시
수집하세요. 측정/후처리/다운로드 중에는 다른 운동 명령, CAN ID 변경, 엔코더 재보정,
리셋 및 수동 Flash 저장을 함께 수행하지 마세요.

PC 오프라인 검증 (가짜 CAN/Qt, 모터/어댑터 불필요):

```bash
QT_QPA_PLATFORM=offscreen python -B -m unittest discover -s tests/can_monitor -p 'test_cogging_lut.py'
QT_QPA_PLATFORM=offscreen python -B -m unittest discover -s tests/can_monitor -p 'test_position_pd.py'
```
