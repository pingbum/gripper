# MA732 공유 엔코더 Slave 펌웨어

이 폴더는 STM32G474 Slave 보드용이다. PCB 패턴 변경 없이 외부 케이블로
아래 신호를 연결한다. 엔코더 MISO가 Slave PB4로만 연결돼 있다면 이 코드로
수신할 수 없다. 반드시 Slave PB5로 신호가 들어와야 한다.

| Slave 핀 | 기능 | 연결 |
| --- | --- | --- |
| PB3 | SPI3 SCK 입력 | Master PB3 / MA732 SCLK |
| PB5 | SPI3 MOSI 입력 | MA732 MISO / Master PB4 |
| PB4 | Analog | 미사용 |
| PB6 | CS 입력 | Master PB6 / MA732 CS |
| GND | 공통 접지 | Master 및 MA732 GND |

Master PB5는 MA732 MOSI에 연결하며 Slave PB5와 연결하지 않는다.
PA4/PA15 하드웨어 NSS는 사용하지 않는다.

## 설정과 수신 방식

CubeMX: Receive Only Slave, Hardware NSS Disable, 16 bits, MSB first,
CPOL Low, CPHA 1 Edge. PB6은 CubeMX에서 GPIO Input을 유지해도 된다.
`MA732_t::start()`가 실행될 때 PB6을 Pull-up / EXTI Rising으로 설정한다.
EXTI9_5 IRQ는 MA732.cpp에서 구현하며 우선순위는 0이다. CubeMX에서 별도로
EXTI9_5 핸들러를 생성하면 중복 정의가 되므로 현재 GPIO Input 설정을 유지한다.

SPI3_RX DMA2 Channel2는 수신기 전용이다. CubeMX가 설정한 DMAMUX 요청을
사용하며, `MA732_t::start()`에서 DMA IRQ를 끈다. CubeMX의 NVIC 체크가 잠겨
있으면 그대로 둔다. CS 상승 ISR에서 DMA 잔여 개수와 SPI 오류를 확인한다.
SPI3에 HAL 송수신/DMA API를 추가 호출하면 안 된다. SPI1의 DRV8316 통신은
기존 HAL DMA 경로를 사용한다.

CS 상승 때 완성된 16비트 한 워드만 공개하고 SPI3를 리셋한 뒤 다음 수신을
미리 준비한다. 두 워드 크기 Normal DMA로 두 번째 완전한 워드가 들어온
프레임은 거부한다. 부팅 중 걸친 프레임, 누락된 워드, DMA 오류, SPI overrun,
다음 CS Low까지 지연된 ISR은 데이터를 공개하지 않고 재동기화한다.
첫 클럭 직전에 EXTI로 수신을 시작하는 방식이 아니므로 CS 하강 ISR은 없다.
소프트웨어 NSS는 내부 SSI=0으로 미리 선택한다.

## Master가 지켜야 하는 조건

- 운용 중에는 **CS Low 한 번에 정확히 16클럭의 각도 읽기**만 전송한다.
  예상 스트림은 10kHz이다. Master 클럭 설정과 Slave SPI 모드가 같아야 한다.
- **CS High 동안 SCK를 토글하지 않는다.** 전용 엔코더 클럭이어야 한다.
- CS High 구간은 Slave의 최악 IRQ 지연과 재설정 시간을 합친 값보다 길어야
  한다. 초기 시험은 **20us 이상**으로 잡고, 실제 보드에서 최악 부하와
  로직 애널라이저로 확인한다. 20us는 측정으로 보장된 최소값이 아니다.
- 런타임 MA732 레지스터 읽기/쓰기는 금지한다. Slave는 Master MOSI 명령을
  보지 못하므로 레지스터 응답과 각도 데이터를 구분할 수 없다.
- Master 스트림을 먼저 켜고 Slave를 시작한다. 초기 대기 만료나 운용 중
  타임아웃은 오류를 래치하므로 원인을 해결한 뒤 Slave를 리셋한다.

하드웨어 NSS와 달리 PB6 GPIO 방식은 프레임 길이를 비트 단위로 검증하지
못한다. 16클럭 이후 추가된 일부 비트나 전기적 신호 품질 문제까지 검출한다고
가정하면 안 된다. 짧은 CS High, 클럭 공유, 레지스터 접근이 필요한 시스템은
Master 프로토콜 변경 또는 실제 NSS 연결이 필요하다.

## 제어와 오류 처리

TIM6는 최신 수신 워드와 수신 시각(TIM2)을 읽는다. 같은 샘플을 반복 계산하지
않으며, 첫 샘플로 이전 각도를 초기화해 가짜 초기 속도를 방지한다.
기계각→전기각 변환과 Slave 자체 보정 LUT는 기존 로직을 사용한다.

초기 유효 수신 전에는 DRVOFF를 유지하고 PWM 구동을 시작하지 않는다.
드라이버 설정 후 최대 1초 동안 첫 유효 수신을 기다린다.
운용 중 유효 데이터가 5 HAL tick(ms) 동안 없으면 SysTick에서 DRVOFF를
올리고 ERROR_SPI를 래치한다. 기존 오류 처리로 제어 루프도 정지한다.
이는 인터럽트가 정상 실행될 때의 임계값이며, flash 쓰기/인터럽트 차단 중의
정확한 벽시계 5ms 차단을 보장하지 않는다. 수신 복구만으로 모터를 재가동하지 않는다.

캘리브레이션도 공유 각도 캐시를 사용하며, 오류 발생 시 중단하고 LUT를 저장하지
않는다. Slave 캘리브레이션 시 Master는 각도 스트림을 유지하되 모터 구동이
Slave의 측정을 방해하지 않도록 해야 한다. 공유 기계각을 쓸 수 있는 기구 구조와
각 드라이버에 맞는 전기각 보정이 전제다.

디버거에서는 `state.position`과 `state.error_flags.spi_error`를 확인한다.

## CAN이 보이지 않을 때

CAN과 상태 송신용 TIM17은 엔코더/DRV 초기화 전에 시작한다. 엔코더 시작 실패,
초기 수신 타임아웃, 캘리브레이션 오류로 모터가 정지해도 CAN 자체에 오류가
없으면 상태 송신과 수신 명령 처리가 유지된다. 엔코더 오류는 상태 프레임
마지막 바이트의 `0x02` 비트로 확인한다. 초기화/오류 대기 중 각도는 아직
갱신되지 않았을 수 있다.

| 디버거 변수 | 의미 |
| --- | --- |
| `slave_boot_stage` | 1: CAN 시작, 2: DRV 설정, 3: 첫 수신 대기, 4: 위치 보정, 5: 정상 초기화 완료, 255: 초기화 오류 |
| `can_status_tx_queued` | 상태 프레임이 FDCAN 송신 큐에 등록된 횟수. 실제 ACK 확인 횟수는 아님 |
| `can_status_tx_errors` | 상태 송신 큐 등록 실패 횟수 |
| `can_rx_frames` | RX FIFO1에서 꺼낸 프레임 수. 다른 ID의 프레임도 포함 |
| `s_driver_id` | 현재 CAN ID. 저장된 Flash 값이 우선하며 미설정 시 기본값은 `0x00` |
| `state.error_flags.can_error` / `hfdcan1.ErrorCode` | CAN 오류 상태 |

Master와 Slave는 서로 다른 CAN ID를 사용해야 한다. 같은 ID이면 상태 송신이
충돌할 수 있고, 현 프로토콜의 모드 0 수신 처리 때문에 상대 상태 프레임을
리셋 명령으로 해석할 수도 있다. CAN ID를 설정할 때는 대상 보드만 CAN 버스에
연결한다. CANH/CANL은 SPI 선과 별도로 CAN 버스에 연결돼 있어야 한다.
디버거가 중단점이나 HAL_Delay에서 정지돼 있다면 실행을 재개한 뒤 확인한다.

## 검증

```sh
g++ -std=c++20 -Wall -Wextra -Werror \
  -Itests/ma732_slave/mocks -IApplication/Sensor/Inc \
  tests/ma732_slave/receiver_test.cpp -o /tmp/ma732-slave-test
/tmp/ma732-slave-test
python3 tests/startup/run_test.py
cmake --preset Debug
cmake --build --preset Debug -j 8
```

호스트 테스트는 실제 수신기 코드를 모의 레지스터로 실행한다. 정상 수신,
부팅 중 프레임, 불완전/초과 워드, DMA/SPI 오류, 늦은 CS 처리, 타임아웃,
HAL tick wraparound 및 IRQ mask 보존을 검사한다. 실제 DMA 버스 순서나
SPI 전기적 타이밍을 시뮬레이션하지 않는다.

초기화 테스트는 실제 `main_cpp`/`startup_fault` 함수의 코드를 추출하여 모의
주변장치와 실행한다. 정상 부팅, 엔코더 시작 실패, 수신 없음, 캘리브레이션 실패,
CAN 시작 실패, 첫 위치 갱신 실패 경로를 검사한다. 실제 CAN 버스 검증은 별도다.

실기에서는 먼저 토크 지령 0으로 Master/Slave 각도 갱신과 일치를 확인하고,
스트림 중단 시 DRVOFF 및 오류 래치를 확인한 후 제어를 시험한다.

## 2026-09-13 배선 문제 해결

Slave 연결 시 Master가 SPI 완료 대기에 갇히던 원인은 케이블 핀 순서가 다른데
같은 색 선을 같은 핀으로 간주한 오배선이었다. 실제 핀 기준으로 재배선한 뒤
정상 동작을 확인했다. 배선은 색깔 대신 이 문서의 핀 연결표로 확인한다.

Master의 SPI 완료 대기는 타임아웃과 반복 횟수 제한을 유지한다. 실패하면
CS를 해제하고 SPI·DMA를 중단하며, 불완전한 수신값을 사용하지 않는다.
오류 후 자동 재시도는 하지 않으므로 원인을 해결한 뒤 보드를 리셋한다.
임시 SPI 진단 변수와 Slave 프레임 카운터는 제거했다.

## PC 모니터에서 두 CAN ID 확인

이 프로젝트의 `tools/app.py`는 CAN 상태를 드라이버 ID별로 하나씩 보관한다.
기존 공용 1칸 큐에서는 나중에 도착한 다른 ID의 프레임이 앞선 프레임을 지워
두 보드 연결 시 한쪽 그래프가 끊길 수 있었다. 이제 선택한 Listen Driver ID의
새 상태만 화면에 반영하며, 같은 ID의 이전 샘플만 최신 샘플로 교체한다.
스캔 및 파라미터 응답은 그래프 상태와 별도로 처리한다.

앱 변경은 펌웨어 다운로드 없이 앱을 재시작하면 적용된다. 실제 CAN 버스에서
누락된 프레임을 복구하는 기능은 아니므로, 문제가 남으면 ID별 수신률과
펌웨어의 자동 재전송 설정을 확인한다.

CAN 하드웨어 없이 수신 처리와 화면 갱신을 검증하려면 다음을 실행한다.

```sh
QT_QPA_PLATFORM=offscreen .venv/bin/python tests/can_monitor/test_receive.py
```
