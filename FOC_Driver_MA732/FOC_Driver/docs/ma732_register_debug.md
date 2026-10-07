# Master MA732 시작 시 레지스터 진단

현재 Master의 `main_cpp()` 첫 부분에서 다음 한 줄이 실행된다.
`Core/Src/main.c`의 `MX_SPI3_Init()`까지 끝난 뒤이며 모터 제어와
10kHz 각도 읽기, CAN 시작 전이다.

```cpp
MA732_DebugRegistersAndHalt(encoder);
```

이 함수는 DRVOFF를 올리고 센서 전원 안정화를 위해 300ms 대기한 뒤,
문서화된 레지스터 11개를 각각 한 번 읽는다. 이후 `HAL_Delay(100)`으로
대기하므로 Live Watch에서 값이 유지된다. 기본 진단은 레지스터를 쓰지 않는다.
센서의 3.3V 전원이 실제 공급돼 있어야 한다.

확인이 끝나면 위 **한 줄만 주석 처리하고 재빌드/플래시**한다.
그러면 기존 `main_cpp()` 초기화와 모터 제어가 실행된다. 현재 진단 상태에서는
각도 업데이트와 모터 제어가 시작되지 않는다. Slave는 분리한 상태로 진단한다.

## Live Watch

`ma732_debug` 전체를 추가하거나 다음 필드를 각각 추가한다.
완료 여부를 먼저 확인하고, 레지스터 값은 16진수 표시로 본다.

| 변수 | 의미 |
| --- | --- |
| `ma732_debug.completed` | 1이면 이번 읽기 시도가 끝남. 성공 여부는 `status`로 확인 |
| `ma732_debug.status` | 0: 전체 성공, 1: 오류, 2: Busy, 3: Timeout |
| `ma732_debug.valid_mask` | 주소 n의 비트가 1이면 `registers[n]`이 이번 읽기로 갱신됨 |
| `ma732_debug.registers[주소]` | 읽은 8비트 값 |
| `ma732_debug.register_status[주소]` | 주소별 HAL 상태. `0xFFFFFFFF`는 읽기 대상 아님 |
| `ma732_debug.response_words[주소]` | 마지막 레지스터 응답 16비트 프레임. 응답을 받지 못하면 0 |
| `ma732_debug.magnetic_flags_valid` | 1이면 자기장 플래그 읽기 성공 |
| `ma732_debug.magnetic_field_low` | R27 bit 6: MGL, 자기장이 설정된 하한보다 낮음 |
| `ma732_debug.magnetic_field_high` | R27 bit 7: MGH, 자기장이 설정된 상한보다 높음 |

| 주소 | 내용 | 공장 초기값 |
| --- | --- | --- |
| `0x00`, `0x01` | Zero setting 하위/상위 | `0x00`, `0x00` |
| `0x02` | BCT | `0x00` |
| `0x03` | ETY/ETX | `0x00` |
| `0x04`, `0x05` | PPT/ILIP | `0xC0`, `0xFF` |
| `0x06` | MGLT/MGHT | `0x1C` |
| `0x09` | 방향 RD | `0x00` |
| `0x0E` | Filter window FW | `0x77` |
| `0x10` | Hysteresis HYS | `0x9C` |
| `0x1B` | MGH/MGL 상태, 읽기 전용 | 자기장 상태에 따라 변함 |

위 초기값은 센서 설정을 수정하지 않은 경우에만 비교 기준이 된다.
SPI에는 센서 ACK나 부품 ID 확인이 없으므로 `HAL_OK`와 모든 값이 0이라는
결과만으로 센서가 정상이라고 판정하지 않는다. 응답의 하위 8비트가 0이 아니면
프레임 형식 오류로 처리하며, 실제 수신된 프레임은 `response_words`에 남는다.

## SPI 함수

```cpp
uint8_t value = 0;
HAL_StatusTypeDef status = encoder.readRegister(0x1B, value);

// 필요할 때만 명시적으로 호출. 기본 시작 진단에는 쓰기 호출이 없다.
uint8_t readback = 0;
// status = encoder.writeRegister(0x0E, desired_fw, readback);
```

`readRegister()`는 주소 0..31을 받아 16비트 요청과 응답 두 프레임으로 읽는다.
명령은 `0x4000 | (address << 8)`, 응답 값은 상위 8비트다.
`writeRegister()`는 쓰기 가능한 주소/비트만 허용하며, 기존 예약 비트를 보존한다.
값이 같으면 재기록하지 않고, 다르면 `0x8000 | (address << 8) | value`를
보내 NVM 완료를 20ms 기다린다. 쓰기 응답과 별도 읽기 결과를 검증한다.
센서 NVM 쓰기 내구성은 25°C에서 1,000회이므로 자동 반복 쓰기에 사용하지 않는다.

두 함수는 인터럽트가 활성화된 foreground에서 호출한다.
레지스터 접근 중 TIM6는 각도 읽기를 건너뛰며, 실패 후 남은 레지스터 응답을
각도로 사용하지 않는다. 런타임 접근 시에도 모터와 공유 SPI Slave 구동은 정지해야 한다.

레지스터 맵, SPI 타이밍, 공장 초기값의 근거:
[MPS MA732 데이터시트 Rev. 1.1, pp. 11, 13–14, 17, 22](https://www.monolithicpower.com/en/documentview/productdocument/index/version/2/document_type/Datasheet/lang/en/sku/MA732GQ-Z/document_id/5009/).

## 검증과 빌드

```sh
python3 -B tests/ma732_registers/run_test.py
cmake --preset Debug
cmake --build --preset Debug -j 8
```

새 `MA732_debug.cpp`를 빌드에 포함하려면 CMake configure를 다시 실행한다.
호스트 테스트는 실제 MA732 구현에 모의 SPI 센서를 연결해 요청/응답 순서,
타이밍 대기, 읽기/쓰기 실패, NVM 재확인, 예약 비트 보존, 시작 시 한 번 읽기와
대기를 검증한다. 실제 보드 SPI 전기적 상태는 검증하지 않는다.
