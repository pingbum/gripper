LED : 붉은 색->DRV 오류
LED : 푸른 색 1초 주기 -> 정상 동작
LED : 푸른 색 항상 동작 -> CAN 오류

---

# STM32G4 FOC Motor Driver System

본 프로젝트는 STM32G4 MCU를 기반으로 **DRV8316C** 드라이버 IC와 **MA732** 자기식 엔코더를 사용하여 고성능 BLDC 모터의 FOC(Field Oriented Control) 및 정밀 제어를 수행하는 시스템입니다.

## 1. 하드웨어 구성 (Hardware Requirements)

* **MCU**: STM32G4 시리즈 (고성능 FPU 및 HRTIM 활용).
* **Driver IC**: TI DRV8316C (SPI 제어 방식).
* **Encoder**: MA732 (14-bit 자기식 엔코더, 16-bit SPI 프레임).
* **Communication**: FDCAN1 (실시간 제어 및 데이터 브로드캐스트).

## 2. 소프트웨어 아키텍처 (Software Features)

시스템은 실시간성을 위해 다중 속도 제어 루프를 운용합니다:

* **Current Loop (50kHz)**: ADC 주입 변환 완료 인터럽트와 동기화되어 FOC 연산을 수행합니다.
* **Position Update (10kHz)**: 엔코더 데이터를 읽고 기계적/전기적 위치를 업데이트합니다.
* **Velocity Loop (5kHz)**: 속도 에러를 기반으로 q축 전류 지령을 생성합니다.
* **Status Broadcast (1kHz)**: FDCAN을 통해 모터 상태 데이터를 외부로 전송합니다.
* **System Monitor (1Hz)**: 시스템 온도 측정 및 드라이버 하드웨어 결함을 감시합니다.

## 3. CAN 통신 프로토콜 (FDCAN Protocol)

29비트 확장 ID(Extended ID)를 사용하며, 기본 드라이버 ID는 `0x01`입니다.

### 3.1 수신 명령 (Master → Driver, RX FIFO 1)

데이터는 **Big Endian** 형식으로 처리됩니다.

#### ID 구조
`Bit 0-7: Driver ID`, `Bit 8-15: Mode ID (MSB=Read 플래그)`.
| 0-7 | 8-15 |
|---|---|
|Driver ID|Mode ID (MSB=Read)|

* **Read 플래그**: Mode ID의 MSB(0x80)가 1이면 읽기 요청.  
  예) `0x20`(쓰기) → `0xA0`(읽기)
#### 기본 명령
| Mode ID | 명칭 |R/W| 데이터 타입 | 스케일링 및 설명 |
| --- | --- | --- | --- | ---|
| `0x00` | **MODE_SYSTEM_RESET** |W|` N/A` | 시스템 리셋 |
| `0x01` | **MODE_CURRENT** |W| `int32` | $1 unit = 1mA$. 전류 제어 모드 command 업데이트. |
| `0x03` | **MODE_VELOCITY** |W| `int32` | $1 unit = 1eRPM$. 속도 제어 모드 command 업데이트. |
| `0x06` | **MODE_CALIBRATION** |W| `N/A` | 엔코더 캘리브레이션 플래그 초기화 및 리셋 |
| `0x07` | **MODE_SET_CAN_ID** |R/W| `uint32`| CAN ID 업데이트. |

#### 파라미터 설정
| Mode ID | 명칭 |R/W| 데이터 타입 | 설명 |
| --- | --- | ---| --- | --- |
| `0x10` | **MODE_SAVE_CTRL_PARAMS** | W | `N/A` | 제어파라미터 flash 메모리에 저장. |
| `0x11` | **MODE_SET_CURRENT_BANDWIDTH**| R/W | `float32` | 전류제어 cutoff frequency를 RAM에 저장 |
| `0x20` | **MODE_SET_VELOCITY_KP** | R/W |`float32` | 속도제어 Kp ram에 저장. |
| `0x21` | **MODE_SET_VELOCITY_KI** | R/W |`float32` | 속도제어 Ki ram에 저장 |
| `0xF0` | **MODE_CAN_BROADCAST_TOGGLE** | W | `uint8` | 0=OFF, 그 외=ON. 데이터가 없으면 토글 |
| `0xF1` | **MODE_CAN_BROADCAST_RATE** | W | `uint32` | 브로드캐스트 주파수(Hz). 1~1000. 저장 후 flash write + 검증 |

#### 파라미터 읽기 (Read)
* **Read 가능한 Mode ID**: `0x07`, `0x11`, `0x20`, `0x21`
* **요청 방식**: Mode ID의 MSB(0x80) 세트  
  예) `0x07 → 0x87`, `0x11 → 0x91`, `0x20 → 0xA0`, `0x21 → 0xA1`
* **응답 포맷**: `ID = (MODE_ID << 8) | MOTOR_ID`, 데이터 = 값  
  - `0x07`: 1바이트 (CAN ID)  
  - `0x11/0x20/0x21`: 4바이트 BE float32

### 3.2 상태 Broadcast (Driver → Master, 1kHz base, 1~1000Hz 설정 가능)

* **ID**: `MY_DRIVER_ID` (기본 `0x01`).

#### **패킷 구조**

| Byte | 항목 | 타입 | 스케일링 공식 |
| --- | --- | --- | --- |
| 0-1 | **Position** | `int16_t` | $Value = Deg \times 10$. |
| 2-3 | **Speed** | `int16_t` |  $Value = eRPM \div 10$.|
| 4-5 | **Current** | `int16_t` |$Value = Amps \times 1000$ (mA 단위). |
| 6 | **Temp** | `int8_t` | $1 unit = 1^\circ C$. |
| 7 | **Error** | `uint8_t` | 현재 에러 코드 (기본 `0x00`). |

#### **에러코드**
| ID | 명칭 | 설명 |
| --- | --- | --- | 
|`0x01u` |**ERROR_CAN** | CAN 통신 에러
|`0x02u` |**ERROR_SPI** |SPI 통신 에러
|`0x04u` |**ERROR_DRIVER_FAULT** | 드라이버 하드웨어 결함
|`0x08u` |**ERROR_HAL_RCC** | RCC 설정 에러
|`0x10u` |**ERROR_HAL_ADC** | ADC 에러
|`0x20u` |**ERROR_HAL_TIM** | TIM 에러
|`0x40u` |**ERROR_HAL_HRTIM** | HRTIM 설정 에러
|`0x80u`| **ERROR_HAL_FMAC** | FMAC 에러


## 4. 시스템 상태 변수 (State Variables)

디버깅 시 `state` 구조체(`constants.hpp`)를 통해 실시간 데이터를 모니터링할 수 있습니다.

### 4.1 State 구조체

* **`omega_ref`**: 목표 속도 지령 ($deg/s$).
* **`iq_ref`**: q축 전류 지령 ($A$).
* **`temperature`**: 측정된 시스템 온도 ($^\circ C$).
* **`CONTROL_MODE`**: 현재 동작 모드 (1: 전류, 3: 속도, 6: 튜닝).
* **`error_flags`**: 시스템 결함 상태 (`can_error`, `spi_error`, `driver_fault`).

### 4.2 Position Instance (`position_instance_f32`)

* **`theta_m`**: 기계적 절대 각도 ($0 \sim 360^\circ$).
* **`omega_m`**: LPF가 적용된 기계적 각속도 ($deg/s$).
* **`theta_e`**: FOC 연산용 전기각 ($deg$).
* **`direction`**: 모터 회전 방향 (1.0 또는 -1.0).

## 5. 디버깅 및 제어 파라미터

### 5.1 모터 파라미터 (`constants.hpp`)

* **`MOTOR_POLE_PAIRS`**: 모터 극쌍수 (11).
* **`MOTOR_Rs` / `MOTOR_Ls`**: 모터 저항 및 인덕턴스 (2.85ohm, 0.214mH).
* **`V_DC`**: 입력 전압 (12V).

### 5.2 드라이버 IC 설정 (`main_cpp.cpp`)

* **`setBuckVoltage(4.0f)`**: 내장 벅 컨버터 전압 설정.
* **`setSLEW(SLEW_200V)`**: 게이트 드라이버 슬루 레이트 설정.
* **`setCurrentGain(0x02)`**: CSA 게인 설정 (0.6 V/A).

## 6. 에러 핸들링 및 보호 기능

* **하드웨어 결함**: DRV8316C의 `nFault` 핀 감지 시 `Error_Handler` 호출 및 모터 정지.
* **통신 장애**: CAN 또는 SPI 통신 실패 시 `error_flags`에 기록되고 해당 에러 LED가 점등됩니다.
* **엔코더 캘리브레이션**: Flash 메모리에 유효한 보정 데이터가 없으면 초기 구동 시 자동으로 정/역방향 측정을 통한 캘리브레이션을 수행합니다.

## 7. 디렉토리 구조

* `Application/Control`: FOC, PID 제어 및 위치 추정 로직.
* `Application/Sensor`: 드라이버 및 엔코더 SPI 통신 클래스.
* `Application/Communication`: FDCAN 송수신 처리.
* `Application/Memory`: Flash 기반 데이터 영구 저장.
* `Application/Setup`: 시스템 초기화 및 인터럽트 서비스.

---

**주의**: 모터 구동 전 `constants.hpp`의 하드웨어 파라미터가 실제 모터 사양과 일치하는지 반드시 확인하십시오.
