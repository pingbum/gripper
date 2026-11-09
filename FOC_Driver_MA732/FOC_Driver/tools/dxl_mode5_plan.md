# DXL 전류 기반 위치 제어(mode 5) 전환 준비

작성: 2026-11-06. 기준 커밋: `a26ea36` (`one_finger_smooth`).
현재 `one_finger.py`의 동작은 mode 4이며, 이 문서는 다음 변경의 설계안이다.

후속 구현은 원본을 수정하지 않고 별도 [dxl_current.py](dxl_current.py)에 작성했다.
실행은 `python3 tools/dxl_current.py`. 전류 상한과 P/I/D를 입력하고 **DXL 설정 적용**을 누르면
현재 값이 적용되며, 처음 위치 제어를 켤 때도 입력값을 적용한다.
초기값은 아래 후보와 같다. 구현 검증은 `tests/can_monitor/test_dxl_current.py`에서 모의 모터로 수행한다.
아래 내용은 전환 준비 시점의 설계이며, 상세 진단 항목과 실기구 튜닝은 후속 작업이다.

## 목표와 변경 위치

MCP F/E의 위치 명령을 유지하면서 출력 전류 상한과 위치 게인을 조절한다.
전류 상한은 위치 유지에 쓰는 힘을 제한하며, 토크 OFF와 같은 자유로운 역구동을 보장하지 않는다.
CAN 관절 제어와 기본 ON인 ROM Guard는 기존 동작을 유지한다.

| 위치 | 현재 | 다음 구현 |
| --- | --- | --- |
| `U2D2Dynamixel` | `r2()`는 있고 `w2()`는 없음 | PID·Goal Current용 2바이트 쓰기 추가 |
| `DxlWorker._apply()` | mode 4, 이동 프로파일과 목표 위치 설정 | mode 5 초기화, 전류·PID 설정 및 읽기 검증 |
| `DxlWorker.run()` / `sample` | 위치만 수신 | 실제 전류와 초기화 성공·실패 상태도 GUI에 전달 |
| `DxlSection` | 위치 슬라이더와 현재 위치 표시 | 전류 상한(mA), 실제 전류(mA), P/I/D 및 적용 모드 표시 |
| `_on_mcp_control()` | 모드 전환 전 수신 위치로 첫 목표 결정 | 모드 전환 후 위치·영점 확인을 마친 뒤 제어 시작 |

## 초기 설정 후보

| 항목 | 후보 | 근거 |
| --- | --- | --- |
| Operating Mode | 5 | 전류 기반 위치 제어 |
| Position P / I / D | 800 / 0 / 500 | MIDAS 공개 API의 raw 게인 값을 비교 출발점으로 사용 |
| Goal Current | 100 mA | 첫 무부하 시험 제안값. 실제 기구의 부하 유지 전류는 미확정 |
| Profile Velocity / Acceleration | 45 / 15 | 사용자가 확인한 현재 이동 프로파일 유지 |

MIDAS의 공개 API 기본 Goal Current는 600 mA다. 100 mA는 이 프로젝트의 시험 제안이며,
MIDAS 측정값이나 검증된 운용값이 아니다. 게인도 CAN 관절의 A/deg, A·s/deg와 직접 비교하지 않는다.
[MIDAS 설정 코드](https://github.com/midas-hand-org/midas_hand_api/blob/6a08263fc607bf219a6b23a9afa5795e3863255a/midas_hand_api/config.py#L153),
[MIDAS 초기화 코드](https://github.com/midas-hand-org/midas_hand_api/blob/6a08263fc607bf219a6b23a9afa5795e3863255a/midas_hand_api/hand.py#L124).

## 레지스터와 통신 단위

| 항목 | 주소 | 바이트 | 처리 |
| --- | --- | --- | --- |
| Operating Mode | 11 | 1 | 토크 OFF에서 5로 변경 |
| Current Limit | 38 | 2 | EEPROM 상한 읽기. Goal Current 입력 검증에 사용 |
| Position D / I / P | 80 / 82 / 84 | 각 2 | 각 raw 값 0~16383 |
| Goal Current | 102 | 2 | XM335는 약 1 mA/count. 상한 용도로 0 이상의 값 사용 |
| Present Current | 126 | 2 | signed int16로 해석하여 mA 표시 |
| Goal / Present Position | 116 / 132 | 각 4 | 기존 signed 위치 변환 유지 |

요청 전류는 읽은 Current Limit 이하인지 검사하고, 범위를 넘으면 오류로 알린다.
Current Limit 자체를 매 제어 시작마다 EEPROM에 쓰지 않는다.
모드 전환 시 PID, 프로파일, 전류/PWM 관련 값이 초기화될 수 있으므로 순서를 명시한다.
[ROBOTIS 제어 테이블 및 Operating Mode 설명](https://emanual.robotis.com/docs/en/dxl/x/xm335-t323/#operating-mode11).

## 제어 시작 순서

모든 모터 접근은 기존처럼 DXL worker 한 곳에서 수행한다.

1. 토크 OFF를 확인하고 모터 모델(기대값 1710), 펌웨어, 모드, 게인, 전류/PWM 상한,
   현재 위치와 영점 관련 값을 기록한다.
2. Drive Mode의 회전 방향 비트는 보존한다. 시간 기반 프로파일(bit 2)과
   Goal 갱신에 의한 자동 토크 ON(bit 3)은 해제한다. 모드 값이 다를 때만 EEPROM을 쓴다.
3. Operating Mode를 5로 변경한 뒤 PID와 전류 상한을 설정한다.
   모드 전환으로 바뀐 PWM 관련 값도 읽어 확인한다.
4. 현재 사용 중인 속도·가속도 프로파일을 다시 적용한다.
5. 전환 후 현재 위치를 새로 읽고 첫 Goal Position을 그 위치에 맞춘다.
   전환 전의 GUI 목표·수신 캐시를 그대로 사용하지 않는다. 위치 좌표가 달라졌다면
   소프트웨어 영점과의 정합을 확인할 때까지 토크 OFF를 유지하고 재초기화를 안내한다.
6. 모드·PID·Goal Current·프로파일·첫 목표를 읽어 검증한다.
   사용자의 위치 제어 요청이 여전히 ON인지 확인한 뒤 토크를 켠다.
7. 초기화 실패나 검증 불일치 시 토크 OFF로 종료하고 GUI 제어 요청을 해제한다.
   기존 오류 재시도 루프가 이전 ON 요청으로 나중에 토크를 켜지 않게 한다.

기존 MCP F/E의 0~90° 명령 제한은 유지한다. mode 5에서는 모터의 Min/Max Position Limit가
적용되지 않으므로 소프트웨어 제한과 영점 확인이 계속 필요하다.
[ROBOTIS Drive Mode](https://emanual.robotis.com/docs/en/dxl/x/xm335-t323/#drive-mode10),
[위치 제한 설명](https://emanual.robotis.com/docs/en/dxl/x/xm335-t323/#minmax-position-limit52-48).

## 구현 후 확인할 동작

- 모의 통신: 쓰기 폭과 순서, 불필요한 EEPROM 쓰기 방지, 음수 전류 표시,
  전류 상한 초과, 설정 실패·읽기 불일치·도중 OFF 요청에서 토크가 켜지지 않는지 확인한다.
- Qt 화면: 전류 상한과 실제 전류를 mA로 구분하고, mode 5 적용 성공 후에만 제어 ON으로 표시한다.
- 실기구: 먼저 토크 OFF에서 수동 역구동을 비교하고, 무부하·작은 목표 이동으로 시작한다.
  실제 전류·추종 오차를 보면서 전류 상한을 조절한 뒤 P/D를 한 항목씩 조정한다.
  I는 우선 0을 유지한다.
- 회귀 확인: 기존 이동 프로파일, Initialize, CAN 관절 및 ROM Guard ON/OFF 동작을 확인한다.
