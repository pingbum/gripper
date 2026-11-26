# 5M data phase bus error로 인한 CAN 프레임 손실

LPF ON/OFF 비교 측정(`0x40`~`0x45`, [LPF_check.py](../tools/LPF_check.py))에서
3072개 데이터 프레임 중 일부가 계속 누락되는 문제의 원인과 펌웨어 수정 방향.
관찰 일자 2026-11-26, Jetson `can0`(mttcan) ↔ 드라이버(STM32G474), 1M/5M FD+BRS.

## 증상

```text
수신 시작: 3ch × 512 × 2조건, 10000 Hz, 대기 20 ms
측정 실패 — 불완전한 데이터: 누락 19/3072개
```

END/체크섬은 정상 도착하지만 중간 데이터 프레임이 빠진다. 재측정해도 매번 실패한다.

## 원인 요약

1. 버스에서 5M data phase 프레임이 약 **0.4%** 확률로 bus error로 깨진다.
2. 펌웨어 FDCAN이 **자동 재전송 OFF**라서 깨진 프레임을 다시 보내지 않는다.
3. 측정 프로토콜에는 재전송 저장소가 없으므로 손실이 그대로 누락이 된다.

앱(수신 측 소프트웨어) 문제는 아니다. 0.4% 손실이면 3076 프레임 전체가 무손실로
도착할 확률은 `(1 − 0.004)^3076 ≈ e^−12`로 사실상 0이므로, 앱에서 재측정을 반복해도
해결되지 않는다.

## 근거 (Jetson에서 수동 측정, TX 없음)

측정 동작 없이 상태 방송만 흐르는 상태에서 5초간:

| 항목 | 5초 증가량 | 의미 |
| --- | --- | --- |
| `rx_packets` | 5009 | 상태 방송 약 1 kHz |
| `rx_errors` | 20 | bus error 약 4회/s (≈0.4%) |
| `rx_dropped` / `rx_over_errors` / `rx_fifo_errors` / `rx_missed_errors` | 0 | 커널·소켓·앱 수신 손실 없음 |

같은 5초 동안 수신한 오류 프레임(`CAN_ERR_PROT | CAN_ERR_BUSERROR`, can_id `0x88`):

| 종류 (`data[2]` / `data[3]`) | 횟수 |
| --- | --- |
| 위치 = CRC delimiter (`data[3] = 0x18`) | 10 |
| stuff error (`data[2] = 0x04`) | 7 |
| form error (`data[2] = 0x02`) | 2 |

CRC delimiter는 BRS가 data → nominal 속도로 돌아오는 지점이다. 이 위치의 오류와
stuff/form 오류는 5M data phase 비트 타이밍 문제의 전형적인 증상이다.
3076 프레임 × 0.4% ≈ 12~19개로 관찰된 누락 19개와 일치한다.
상태 방송(mode 0)도 같은 비율로 손실되고 있다.

## 현재 펌웨어 FDCAN 설정 ([Core/Src/fdcan.c](../Core/Src/fdcan.c))

FDCAN 커널 클럭 = PCLK1 = 170 MHz (HSE 24 MHz, PLLM 6, PLLN 85, PLLR 2).

| 구간 | Prescaler | Seg1 | Seg2 | SJW | tq/bit | Bit rate | Sample point |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Nominal | 2 | 67 | 17 | 8 | 85 | 1 Mbps | 80.0% |
| Data | 2 | 12 | 4 | 4 | 17 | 5 Mbps | 76.5% |

```c
hfdcan1.Init.AutoRetransmission = DISABLE;   // 오류 프레임 재전송 안 함
```

TDC(Transmitter Delay Compensation) 설정 호출이 없다
(`HAL_FDCAN_ConfigTxDelayCompensation` / `HAL_FDCAN_EnableTxDelayCompensation` 미사용).
5M에서는 data 비트가 200 ns이고 transceiver loop delay가 그 상당 부분을 차지하므로,
TDC 없이 송신하면 STM32가 자기 송신 비트를 잘못 판정해 오류를 낼 수 있다.

참고로 Jetson `can0`의 현재 data timing은 sample point 70%, dsjw 1
(`dtq 20 ns`, 10 tq/bit)로 펌웨어(76.5%, SJW 4)와 다르다.

## 펌웨어 수정 제안

### 1. 자동 재전송 켜기 (손실 자체를 없앰, 최우선)

```c
hfdcan1.Init.AutoRetransmission = ENABLE;
```

- 이 값은 CubeMX 생성 코드이므로 **`.ioc`에서도 바꿔야** 재생성 시 되돌아가지 않는다
  (FDCAN1 → Auto Retransmission = Enable).
- 측정 송신 경로는 이미 "FIFO 2칸 이상일 때만 등록 / 실패 시 같은 프레임 재시도 /
  10 s 무진행 시 버퍼 반환"을 하므로 재전송으로 FIFO가 잠시 막혀도 맞물려 동작해야 한다.
- 확인할 점: 원래 DISABLE로 둔 의도가 있는지. 수신 노드가 없어 ACK가 없으면
  프레임이 FIFO에 남아 계속 재시도되므로, 그때 상태 방송/응답 송신 실패 처리와
  CAN 오류 플래그(`ERROR_FLAG_CAN`)가 의도대로 동작하는지 점검한다.

### 2. 5M용 TDC 설정 (오류 발생률 자체를 낮춤)

`USER CODE BEGIN FDCAN1_Init 2` 구역(HAL_FDCAN_Init 직후, `HAL_FDCAN_Start` 이전)에 추가.
두 함수 모두 `State == HAL_FDCAN_STATE_READY`일 때만 동작하므로 Start 이후에 부르면 무시된다.

```c
  /* USER CODE BEGIN FDCAN1_Init 2 */
  /* 5 Mbps data phase: secondary sample point at the data sample point.
     Offset unit = FDCAN kernel clock (mtq). 2 * 12 = 24 mtq (ST example form). */
  if (HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1,
          hfdcan1.Init.DataPrescaler * hfdcan1.Init.DataTimeSeg1, 0) != HAL_OK ||
      HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END FDCAN1_Init 2 */
```

- `TdcOffset` 최대 `0x7F`. 위 값은 24이고, 엄밀한 sample point 위치는 `2 × (1 + 12) = 26` mtq다.
  보드에서 오류율을 보며 조정 가능하다.
- `HAL_FDCAN_Init`을 다시 부르는 경로(재초기화/에러 복구)가 있다면 그 뒤에도 같은 설정을 해야 한다.

### 3. (Jetson 측, 펌웨어 수정 없이 병행 시험) data timing 맞추기

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 sample-point 0.800 sjw 4 \
    dbitrate 5000000 dsample-point 0.800 dsjw 2 fd on berr-reporting on restart-ms 0
sudo ip link set can0 up
```

수정 전후로 아래 검증 절차의 `rx_errors` 증가량을 비교하면 원인이 Jetson 수신 쪽인지,
STM32 송신(TDC) 쪽인지 구분할 수 있다.

## 검증 절차

1. Jetson에서 idle(상태 방송만) 5초 동안 오류 증가량 확인. 목표는 0.
   ```bash
   a=$(cat /sys/class/net/can0/statistics/rx_errors); sleep 5
   b=$(cat /sys/class/net/can0/statistics/rx_errors); echo $((b - a))
   ```
2. `ip -details -statistics link show can0`의 `bus-errors`, `error-pass`, `bus-off`가 늘지 않는지 확인.
3. `python tools/LPF_check.py`로 측정을 여러 번 반복. 모두 "체크섬 일치"로 끝나야 한다.
   - 실패 시 앱 로그에 조건별 누락 수와 그 세션 동안의 CAN bus error 횟수가 함께 표시된다.
     예: `누락 19/3072개 (LPF ON 9, LPF OFF 10) [이 세션 중 CAN bus error 19회 …]`
   - 자동 재전송이 켜졌다면 bus error가 있어도 누락은 0이어야 하며, 로그에
     `CAN bus error N회 있었으나 데이터 손실 없음`이 표시된다.

## 범위

- Jetson 쪽 카운터·오류 프레임 관찰 결과만 근거로 한다. STM32 쪽 TEC/REC(`FDCAN_ECR`)와
  PSR의 LEC/DLEC는 확인하지 않았다. 펌웨어에서 이를 읽어 보면 오류가 STM32 송신 측
  (자기 비트 오류)에서 나는지 직접 확인할 수 있다.
- 위 펌웨어 수정은 이 문서 작성 시점에 적용·검증하지 않았다.
