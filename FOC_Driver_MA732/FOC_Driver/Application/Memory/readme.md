
# Flash 사용 방식 (FOC_Driver/Application 기준)

---

## 1) Flash 주소 맵 (memory_constants.hpp)

파일: `Application/Common/Inc/memory_constants.hpp`

### 1.1 파라미터 저장 영역 (일반 파라미터용)
- `PARAMETER_VARIABLE_ADDR = 0x08040000`

오프셋(32bit 워드 단위):
- `KP_ADDR_OFFSET = 0`
- `KI_ADDR_OFFSET = 1`
- `KD_ADDR_OFFSET = 2`
- `FILTER_CUTOFF_FREQ_ADDR_OFFSET = 10`

주소 계산:
- `ADDR = PARAMETER_VARIABLE_ADDR + (OFFSET * 4)`

예:
- `KP_ADDR = 0x08040000 + 0*4 = 0x08040000`
- `KI_ADDR = 0x08040000 + 1*4 = 0x08040004`
- `KD_ADDR = 0x08040000 + 2*4 = 0x08040008`
- `CUTOFF_ADDR = 0x08040000 + 10*4 = 0x08040028`

> 참고: Application 코드에서는 이 파라미터 영역을 “정의만” 해둔 상태고, 실제 저장/로드는 직접 붙여야 합니다.  
> 아래 예제는 현재 제공된 `flash_memory` API를 그대로 써서 구현하는 방법입니다.

---

### 1.2 엔코더 캘리브레이션 상태 영역
- `ENCODER_STATE_ADDR = 0x0805F800`

오프셋(32bit 워드 단위):
- `ENCODER_DIRECTION_OFFSET = 0`
- `ENCODER_BUFFER_SIZE_OFFSET = 1`
- `ENCODER_FLASH_UPDATE_OFFSET = 3`

플래그 주소:
- `ENCODER_FLASH_UPDATE = ENCODER_STATE_ADDR + (ENCODER_FLASH_UPDATE_OFFSET * 4)`
- 즉 `ENCODER_FLASH_UPDATE = 0x0805F800 + 12 = 0x0805F80C`

---

### 1.3 엔코더 오프셋 테이블 저장 영역 (대용량)
- `ENCODER_BUFFER_BASE_ADDR = 0x08060000`
- `ENCODER_MAX_BUFFER_SIZE = 0x20000` (bytes)
- 실제 범위: `0x08060000 ~ 0x0807FFFF`

---

## 2) Flash 쓰기/읽기 동작 요약 (flash_memory)

파일:
- `Application/Memory/Inc/flash_memory.hpp`
- `Application/Memory/Src/flash_memory.cpp`

### 2.1 read (읽기)
- Flash는 메모리 매핑되어 있으므로, 읽기는 사실상 `memcpy(dst, (void*)addr, nbytes)`입니다.
- 제공 함수:
  - `read_flash_buffer_f32(addr, dst, length, size)`  
    내부적으로 `length*size` 바이트를 memcpy 합니다.

### 2.2 write (쓰기)
- 제공 함수:
  - `write_flash_buffer(addr, src, length, size)`
- 동작:
  1. 대상 주소 범위가 걸치는 Flash 페이지를 **erase(삭제)** 합니다.
  2. `HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, ...)`로 **8바이트(더블워드) 단위**로 씁니다.
  3. 마지막이 8바이트 미만이면 **0xFF로 패딩**합니다.
- Flash 쓰기 중 인터럽트를 막고(`__disable_irq()`), 함수는 RAM에서 실행되도록 배치됩니다(`.ramfunc`).

### 2.3 매우 중요한 주의사항(실무 핵심)
- `write_flash_buffer()`는 **페이지 erase를 포함**합니다.
- 즉 “어떤 값 하나만 바꾸려고 써도”, 그 값이 속한 Flash 페이지 전체가 지워질 수 있습니다.
- 그래서 **권장 패턴은**:
  - (나쁜 패턴) `Kp만 쓰고`, `Ki만 쓰고`, `Kd만 쓰고`… 이런 식으로 **따로따로 저장**
  - (좋은 패턴) `Kp/Ki/Kd/...`를 **한 블록(struct)으로 묶어서 한 번에 저장**

---

## 3) 엔코더 캘리브레이션 Flash 저장 포맷 (현재 코드가 실제로 쓰는 방식)

파일: `Application/Sensor/Src/encoder_calibration.cpp`

### 3.1 오프셋 테이블
- 위치: `ENCODER_BUFFER_BASE_ADDR`
- 포맷: `float32_t offset_table[num_samples]`를 **바이너리 그대로** 연속 저장
- 호출 형태:
  - `write_flash_buffer(ENCODER_BUFFER_BASE_ADDR, (uint8_t*)offset_data_, num_samples_, sizeof(float32_t))`

### 3.2 상태 블록 (16바이트 = 4워드)
- 위치: `ENCODER_STATE_ADDR`
- 포맷 (워드 단위):
  - word0: `float32_t direction`
  - word1: `uint32_t buffer_size`
  - word2: `uint32_t reserved` (0xFFFFFFFF)
  - word3: `uint32_t flash_update` (`ENCODER_CALIBRATION_MARKER`)
- 호출 형태:
  - `write_flash_buffer(ENCODER_STATE_ADDR, (uint8_t*)&state, 4, sizeof(uint32_t))`

---

## 4) FLASH_UPDATE 플래그 흐름 (캘리브레이션 재실행 조건)

파일:
- `Application/Setup/Src/main_cpp.cpp`
- `Application/Control/Src/position.cpp`

### 부팅 시
- `FLASH_UPDATE = *(uint32_t*)ENCODER_FLASH_UPDATE;`

### 초기화 분기
- `FLASH_UPDATE == ENCODER_CALIBRATION_MARKER`이면:
  - Flash에서 로드
- `FLASH_UPDATE == 0`이면:
  - 명시적으로 요청된 MA732 캘리브레이션 수행
  - 성공 시 Flash에 저장하고 `FLASH_UPDATE = ENCODER_CALIBRATION_MARKER`
- 빈 Flash(`0xFFFFFFFF`) 또는 기존 형식의 값이면:
  - 자동 캘리브레이션을 수행하지 않음
  - 오프셋 테이블은 0, 방향은 `+1`로 초기화

---

## 5) (예제) 파라미터를 Flash에 저장/로드하기

이 섹션은 실제로 적용 가능한 형태로 구체적으로 작성했습니다.

### 5.1 추천: “파라미터 블록(struct) 하나로 저장”

아래 예시는 `Kp/Ki/Kd/cutoff`를 한 번에 저장합니다.

#### 포맷(Flash에 저장되는 바이트 레이아웃)
- `ParameterBlock` 구조체의 메모리 덤프 그대로 저장
- 총 16바이트(4워드), 8바이트 단위 프로그래밍에도 잘 맞음

```cpp
// file: Application/Memory 혹은 Application/Common 쪽에 추가하는 것을 권장

#include "flash_memory.hpp"
#include "memory_constants.hpp"
#include <cstdint>
#include <cstring>
#include <cmath>

// Flash에 저장할 블록(16바이트)
// - packed: 컴파일러 패딩 방지
// - aligned(8): 더블워드(8바이트) 쓰기 단위에 맞춤
struct __attribute__((packed, aligned(8))) ParameterBlock
{
    float32_t kp;          // word0
    float32_t ki;          // word1
    float32_t kd;          // word2
    float32_t cutoff_hz;   // word3
};

static_assert(sizeof(ParameterBlock) == 16, "ParameterBlock must be 16 bytes");

// 저장 위치: PARAMETER_VARIABLE_ADDR (0x08040000)
static constexpr uint32_t PARAM_BLOCK_ADDR = PARAMETER_VARIABLE_ADDR;

// 간단한 유효성 검사(Flash가 지워진 상태면 0xFF.. -> float는 NaN/비정상 가능)
static bool is_valid_param_block(const ParameterBlock& p)
{
    // 지나치게 엄격할 필요는 없지만, 최소한 NaN/Inf 방지는 하는 게 좋음
    if (!std::isfinite(p.kp)) return false;
    if (!std::isfinite(p.ki)) return false;
    if (!std::isfinite(p.kd)) return false;
    if (!std::isfinite(p.cutoff_hz)) return false;

    // 프로젝트 상황에 맞게 범위 제한(예: 음수 방지)
    if (p.kp < 0.0f || p.ki < 0.0f || p.kd < 0.0f) return false;
    if (p.cutoff_hz <= 0.0f) return false;

    return true;
}

bool save_parameter_block(float32_t kp, float32_t ki, float32_t kd, float32_t cutoff_hz)
{
    ParameterBlock p { kp, ki, kd, cutoff_hz };

    // length=4, size=4 => 총 16바이트 기록
    uint32_t status = write_flash_buffer(
        PARAM_BLOCK_ADDR,
        reinterpret_cast<uint8_t*>(&p),
        /*length=*/4,
        /*size=*/sizeof(uint32_t));

    return (status == HAL_OK);
}

bool load_parameter_block(ParameterBlock* out)
{
    if (!out) return false;

    // Flash는 메모리 매핑
    std::memcpy(out, reinterpret_cast<const void*>(PARAM_BLOCK_ADDR), sizeof(ParameterBlock));

    // 유효성 확인
    return is_valid_param_block(*out);
}
````

#### 사용 예시(부팅 시 로드 → 없으면 기본값 적용)

```cpp
void apply_params_on_boot()
{
    ParameterBlock p;

    if (load_parameter_block(&p))
    {
        // 여기서 프로젝트의 실제 파라미터 변수에 적용
        // 예: control.kp = p.kp; 등
    }
    else
    {
        // Flash가 비어있거나 깨졌으면 기본값 사용
        // 예: control.kp = DEFAULT_KP; 등
    }
}
```

---

### 5.2 비추천: “값 하나씩 따로 저장”

아래는 “Kp만” 저장하는 예시입니다.
**주의:** 이 방식은 페이지 erase 때문에 같은 페이지의 다른 값들이 함께 지워질 수 있습니다.

```cpp
#include "flash_memory.hpp"
#include "memory_constants.hpp"
#include <cstdint>
#include <cstring>

static inline uint32_t addr_of_param_word(uint32_t offset_word)
{
    return PARAMETER_VARIABLE_ADDR + offset_word * 4u;
}

bool save_kp_only(float32_t kp)
{
    uint32_t status = write_flash_buffer(
        addr_of_param_word(KP_ADDR_OFFSET),
        reinterpret_cast<uint8_t*>(&kp),
        /*length=*/1,
        /*size=*/sizeof(float32_t));

    return (status == HAL_OK);
}

bool load_kp_only(float32_t* out_kp)
{
    if (!out_kp) return false;
    std::memcpy(out_kp, reinterpret_cast<const void*>(addr_of_param_word(KP_ADDR_OFFSET)), sizeof(float32_t));
    return true;
}
```

---

## 6) (예제) “플래그 하나”를 더블워드(8바이트)로 쓰기 (CAN 캘리브레이션 리셋과 동일 패턴)

파일: `Application/Communication/Src/can.cpp`의 방식과 동일한 형태로 작성한 예시입니다.

목표:

* `ENCODER_FLASH_UPDATE(0x0805F80C)` 값을 0으로 만들어서 다음 부팅에서 캘리브레이션을 재실행하게 함

```cpp
#include "flash_memory.hpp"
#include "memory_constants.hpp"
#include <cstdint>

bool clear_encoder_flash_update_flag_like_can()
{
    // flash_update가 들어있는 워드는 0x...F80C
    // 더블워드 쓰기를 위해 8바이트 정렬 주소로 내림: 0x...F808
    const uint32_t dw_addr = (ENCODER_FLASH_UPDATE & ~0x7UL);

    // 8바이트 payload
    // - 앞 4바이트(0x...F808): 0xFFFFFFFF 유지(의미 없음/예약)
    // - 뒤 4바이트(0x...F80C): 0으로 설정(flash_update = 0)
    uint8_t payload[8] = {
        0xFF, 0xFF, 0xFF, 0xFF,
        0x00, 0x00, 0x00, 0x00
    };

    uint32_t status = write_flash_buffer(dw_addr, payload, /*length=*/1, /*size=*/8);
    return (status == HAL_OK);
}
```

> 주의: 이 함수는 대상 페이지를 erase할 수 있으므로, `ENCODER_STATE_ADDR` 페이지 안의 다른 값(direction/buffer_size 등)도 같이 초기화될 수 있습니다.
> 하지만 “재캘리브레이션”이 목적이면 설계적으로 용인됩니다.

---

## 7) 새 값을 Flash에 저장할 때 “포맷 규칙” 요약

* float: `float32_t` (4바이트 IEEE754) 그대로 저장
* int: `uint32_t` (4바이트) 그대로 저장
* 구조체/배열: RAM의 바이트 레이아웃 그대로 저장
* 쓰기는 내부적으로 8바이트 더블워드 단위로 진행되며, 부족한 바이트는 0xFF로 패딩됨
* `write_flash_buffer()`는 페이지 erase를 포함 → **블록으로 묶어 한 번에 저장**하는 설계가 안전

---
