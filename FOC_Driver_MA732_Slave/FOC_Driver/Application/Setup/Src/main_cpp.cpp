/**
 * @file main_cpp.cpp
 * @brief Application entry point and periodic control callbacks.
 *
 * Optimized main loop version:
 * - CMSIS-DSP used to vectorize ADC scaling.
 * - Single-precision arithmetic to leverage the Cortex-M7 FPU.
 * - No functional changes outside the numerical pipeline.
 */

#include "main_cpp.hpp"
#include "MA732.hpp"
#include "DRV8316C_SPI.hpp"
#include "SPI_handler.hpp"
#include "can.hpp"
#include "constants.hpp"
#include "control_params.hpp"
#include "cogging_compensation.hpp"
#include "current_loop.hpp"
#include "error_handler.hpp"
#include "flash_memory.hpp"
#include "low_speed_loop.hpp"
#include "main.h"
#include "memory_constants.hpp"
#include "stm32g4xx_hal_def.h"
#include "stm32g4xx_hal_gpio.h"
#include "stm32g4xx_hal_tim.h"
#include <cstdio>

// ---- SPI and peripheral driver instances ----
static SPIHandler_t spiDrv(&hspi1, DRV8316_NSS_GPIO_Port, DRV8316_NSS_Pin);


static DRV8316C_t drv8316(spiDrv);
static MA732_t encoder;
static volatile bool position_ready = false;

// ---- Application state ----
uint32_t FLASH_UPDATE = 0;
State_t state;

static volatile uint16_t s_adc5_buf[2]; // [0] VREFINT, [1] TEMP

// Debug counters.
uint32_t process_time_pos = 0;
volatile float32_t cogging_lut_debug[CoggingCompensation::BIN_COUNT] = {};

// Debugger: 0=entry, 1=CAN online, 2=DRV setup, 3=encoder wait,
// 4=position/calibration, 5=running, 255=startup fault.
volatile uint32_t slave_boot_stage = 0;

namespace {
[[noreturn]] void startup_fault(uint8_t flags) {
  slave_boot_stage = 255;
  DRVOFF_GPIO_Port->BSRR = DRVOFF_Pin;
  Error_Raise(flags);
  // Leave CAN RX and TIM17 running if their initialization succeeded.
  // Encoder/DRV failures must not hide error reports or reset/ID commands.
  while (1) HAL_Delay(100);
}

CoggingCompensation cogging;
bool cogging_compensation_is_enabled = true;

constexpr uint32_t COGGING_LUT_MAGIC = 0x434F4747UL; // "COGG"
constexpr uint32_t COGGING_LUT_VERSION = 1UL;

struct __attribute__((packed, aligned(8))) CoggingLutHeader {
  uint32_t magic;
  uint32_t version;
  uint32_t bin_count;
  uint32_t checksum;
};
static_assert(sizeof(CoggingLutHeader) == 16U);

uint32_t cogging_checksum(const float32_t *values, uint16_t count) {
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(values);
  const uint32_t byte_count = static_cast<uint32_t>(count) * sizeof(float32_t);
  uint32_t hash = 2166136261UL; // FNV-1a
  for (uint32_t i = 0; i < byte_count; ++i) {
    hash ^= bytes[i];
    hash *= 16777619UL;
  }
  return hash;
}

bool load_cogging_lut_from_flash() {
  const auto *header =
      reinterpret_cast<const CoggingLutHeader *>(COGGING_LUT_HEADER_ADDR);
  const auto *values =
      reinterpret_cast<const float32_t *>(COGGING_LUT_DATA_ADDR);

  if (header->magic != COGGING_LUT_MAGIC ||
      header->version != COGGING_LUT_VERSION ||
      header->bin_count != CoggingCompensation::BIN_COUNT ||
      header->checksum !=
          cogging_checksum(values, CoggingCompensation::BIN_COUNT))
    return false;

  if (!cogging.load_table(values, CoggingCompensation::BIN_COUNT))
    return false;
  for (uint16_t i = 0; i < CoggingCompensation::BIN_COUNT; ++i)
    cogging_lut_debug[i] = values[i];
  return true;
}

bool save_cogging_lut_to_flash() {
  const float32_t *values = cogging.table_data();
  CoggingLutHeader header = {
      .magic = COGGING_LUT_MAGIC,
      .version = COGGING_LUT_VERSION,
      .bin_count = CoggingCompensation::BIN_COUNT,
      .checksum = cogging_checksum(values, CoggingCompensation::BIN_COUNT)};

  // Header and data occupy disjoint flash pages, so each helper erase is safe.
  uint32_t status = write_flash_buffer(
      COGGING_LUT_DATA_ADDR,
      reinterpret_cast<uint8_t *>(const_cast<float32_t *>(values)),
      CoggingCompensation::BIN_COUNT, sizeof(float32_t));
  if (status != HAL_OK)
    return false;

  // Write validity metadata last. A reset during the data write therefore
  // cannot make a partial LUT appear valid on the next boot.
  status = write_flash_buffer(COGGING_LUT_HEADER_ADDR,
                              reinterpret_cast<uint8_t *>(&header), 1U,
                              sizeof(header));
  return status == HAL_OK;
}

enum class CoggingStage : uint8_t {
  Idle,
  SettleForward,
  MeasureForward,
  SettleReverse,
  MeasureReverse
};

CoggingStage cogging_stage = CoggingStage::Idle;
float32_t cogging_rpm = 5.0f;
float32_t cogging_travel_deg = 0.0f;
float32_t cogging_prev_angle = 0.0f;

float32_t signed_angle_delta(float32_t angle, float32_t previous) {
  float32_t delta = angle - previous;
  if (delta > 180.0f)
    delta -= 360.0f;
  else if (delta < -180.0f)
    delta += 360.0f;
  return delta;
}

void update_cogging_measurement(float32_t velocity_iq) {
  if (cogging_stage == CoggingStage::Idle)
    return;

  const float32_t angle = state.position.theta_m;
  const float32_t delta = signed_angle_delta(angle, cogging_prev_angle);
  cogging_prev_angle = angle;

  // Accumulate net travel only in the commanded direction. Using abs(delta)
  // makes encoder noise and low-speed dithering look like real rotation and
  // can finish a multi-revolution stage after only a fraction of a turn.
  const bool reverse_stage =
      cogging_stage == CoggingStage::SettleReverse ||
      cogging_stage == CoggingStage::MeasureReverse;
  cogging_travel_deg += reverse_stage ? -delta : delta;
  if (cogging_travel_deg < 0.0f)
    cogging_travel_deg = 0.0f;

  if (cogging_stage == CoggingStage::MeasureForward)
    cogging.record(angle, velocity_iq, +1);
  else if (cogging_stage == CoggingStage::MeasureReverse)
    cogging.record(angle, velocity_iq, -1);

  constexpr float32_t SETTLE_DEG = 360.0f;
  constexpr float32_t MEASURE_DEG = 3.0f * 360.0f;
  switch (cogging_stage) {
  case CoggingStage::SettleForward:
    if (cogging_travel_deg >= SETTLE_DEG) {
      cogging_stage = CoggingStage::MeasureForward;
      cogging_travel_deg = 0.0f;
    }
    break;
  case CoggingStage::MeasureForward:
    if (cogging_travel_deg >= MEASURE_DEG) {
      cogging_stage = CoggingStage::SettleReverse;
      cogging_travel_deg = 0.0f;
      state.omega_ref = -cogging_rpm;
    }
    break;
  case CoggingStage::SettleReverse:
    if (cogging_travel_deg >= SETTLE_DEG) {
      cogging_stage = CoggingStage::MeasureReverse;
      cogging_travel_deg = 0.0f;
    }
    break;
  case CoggingStage::MeasureReverse:
    if (cogging_travel_deg >= MEASURE_DEG) {
      if (cogging.finalize(0.1f)) {
        for (uint16_t i = 0; i < CoggingCompensation::BIN_COUNT; ++i)
          cogging_lut_debug[i] = cogging.table_value(i);
        save_cogging_lut_to_flash();
      }
      cogging_stage = CoggingStage::Idle;
      state.omega_ref = 0.0f;
      state.iq_ref = 0.0f;
      state.CONTROL_MODE = MODE_CURRENT;
    }
    break;
  case CoggingStage::Idle:
    break;
  }
}
} // namespace

bool cogging_measurement_start(float32_t mechanical_rpm) {
  if (cogging_stage != CoggingStage::Idle || mechanical_rpm < 1.0f ||
      mechanical_rpm > 20.0f)
    return false;
  cogging.reset();
  cogging_rpm = mechanical_rpm;
  cogging_travel_deg = 0.0f;
  cogging_prev_angle = state.position.theta_m;
  for (uint16_t i = 0; i < CoggingCompensation::BIN_COUNT; ++i)
    cogging_lut_debug[i] = 0.0f;
  cogging_stage = CoggingStage::SettleForward;
  state.omega_ref = cogging_rpm;
  state.CONTROL_MODE = MODE_VELOCITY;
  return true;
}

void cogging_measurement_stop() {
  cogging_stage = CoggingStage::Idle;
  state.omega_ref = 0.0f;
  state.iq_ref = 0.0f;
  state.CONTROL_MODE = MODE_CURRENT;
}

bool cogging_measurement_active() { return cogging_stage != CoggingStage::Idle; }
void cogging_compensation_set_enabled(bool enabled) {
  cogging_compensation_is_enabled = enabled;
}
bool cogging_compensation_enabled() {
  return cogging_compensation_is_enabled;
}
float32_t cogging_compensation_current(float32_t angle) {
  return cogging_compensation_is_enabled ? cogging.compensation(angle) : 0.0f;
}
float32_t cogging_table_value(uint16_t bin) { return cogging.table_value(bin); }

/**
 * @brief C++ application entry point called from the startup code.
 */
extern "C" void main_cpp(void) {

  DRVOFF_GPIO_Port->BSRR = DRVOFF_Pin;
  HAL_TIM_Base_Start(&htim2);

  // Initialize values exposed to CAN before enabling its RX/TX interrupts.
  state.position.p = MOTOR_POLE_PAIRS;
  state.position.direction = 1.0f;
  FLASH_UPDATE = *(uint32_t *)ENCODER_FLASH_UPDATE;
  load_control_params_from_flash();
  current_loop_init();
  velocity_loop_init();
  position_loop_init();

  // Status reporting must not depend on encoder/DRV initialization success.
  CAN_Handler::FDCAN1_SetupFiltersAndStart();
  if (state.error_flags.can_error) startup_fault(ERROR_CAN);
  if (HAL_TIM_RegisterCallback(&htim17, HAL_TIM_PERIOD_ELAPSED_CB_ID,
                              TIM17_PeriodElapsedCB) != HAL_OK ||
      HAL_TIM_Base_Start_IT(&htim17) != HAL_OK)
    startup_fault(ERROR_HAL_TIM);
  slave_boot_stage = 1;

  if (encoder.start() != HAL_OK) {
    startup_fault(ERROR_SPI);
  }

  // Set Drv8316
  slave_boot_stage = 2;
  drv8316.clearFAULT();
  HAL_Delay(100);
  drv8316.setBuckVoltage(4.0f);
  HAL_Delay(100);
  drv8316.enableBuck(true);
  HAL_Delay(100);
  drv8316.setSLEW(SLEW_200V);
  HAL_Delay(100);
  drv8316.setCurrentGain(0x02);
  HAL_Delay(100);
  drv8316.enableAAR(false);
  HAL_Delay(100);
  drv8316.enableASR(false);
  HAL_Delay(100);
  drv8316.setDelayCompensation(false, DLY_TARGET_1P2US);
  HAL_Delay(100);
  drv8316.setPWMDUTY(0x01);

  // ADC Calibration
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc5, ADC_SINGLE_ENDED);

  // Do not enable PWM until a fresh complete shared-encoder frame arrives.
  slave_boot_stage = 3;
  const uint32_t wait_start = HAL_GetTick();
  uint16_t first_word = 0;
  while (encoder.readAngle(first_word) != HAL_OK &&
         HAL_GetTick() - wait_start < 1000U) HAL_Delay(1);
  if (encoder.readAngle(first_word) != HAL_OK) {
    startup_fault(ERROR_SPI);
  }
  if (Error_IsActive()) startup_fault(GET_ERROR_FLAGS(&state));

  HAL_GPIO_WritePin(DRVOFF_GPIO_Port, DRVOFF_Pin, GPIO_PIN_RESET);

  // HRTIM
  HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1);
  HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA2);

  HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TC1);
  HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TC2);

  HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TD1);
  HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TD2);

  load_cogging_lut_from_flash();

  // HRTIM Start
  HAL_HRTIM_WaveformCounterStart_IT(&hhrtim1, HRTIM_TIMERID_MASTER);

  if (HAL_HRTIM_WaveformCounterStart(
          &hhrtim1, HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_C |
                        HRTIM_TIMERID_TIMER_D) != HAL_OK) {
    startup_fault(ERROR_HAL_HRTIM);
  }

  // Position
  slave_boot_stage = 4;
  initializePosition(&state.position, MOTOR_POLE_PAIRS, encoder);
  if (Error_IsActive()) {
    startup_fault(GET_ERROR_FLAGS(&state));
  }
  TIM6_PeriodElapsedCB(&htim6); // Seed angle before enabling current control.
  if (Error_IsActive()) {
    startup_fault(GET_ERROR_FLAGS(&state));
  }

  // ADC Initialization
  HAL_ADC_RegisterCallback(&hadc2, HAL_ADC_INJ_CONVERSION_COMPLETE_CB_ID,
                           HAL_ADC_ConversionENDCallback);
  HAL_ADCEx_InjectedStart(&hadc1);
  HAL_ADCEx_InjectedStart_IT(&hadc2);
  __HAL_ADC_ENABLE_IT(&hadc2, (ADC_IT_JEOS));
  __HAL_ADC_DISABLE_IT(&hadc2, (ADC_IT_JEOC));
  HAL_ADCEx_InjectedStart(&hadc3);

  HAL_ADC_Start_DMA(&hadc5, (uint32_t *)s_adc5_buf, 2);

  // Timer Init
  HAL_TIM_RegisterCallback(&htim6, HAL_TIM_PERIOD_ELAPSED_CB_ID,
                           TIM6_PeriodElapsedCB);
  HAL_TIM_RegisterCallback(&htim7, HAL_TIM_PERIOD_ELAPSED_CB_ID,
                           TIM7_PeriodElapsedCB);
  HAL_TIM_RegisterCallback(&htim16, HAL_TIM_PERIOD_ELAPSED_CB_ID,
                           TIM16_PeriodElapsedCB);

  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_TIM_Base_Start_IT(&htim16);

  state.position.rev = 0;
  state.CONTROL_MODE = 0x01;
  slave_boot_stage = 5;
  //state.theta_ref = 30;

  while (1) {
    // drv8316.Read(0x00, result);
    // angle = encoder.readAngleRaw();

    if (Error_IsActive()) _Error_Handler();
    HAL_Delay(1);
  }
}

// 50 kHz loop.
void HAL_ADC_ConversionENDCallback(ADC_HandleTypeDef *hadc) {
  /* Triggered after ADC2 injected conversion complete (JEOS). */
  UNUSED(hadc);
  if (Error_IsActive() || !position_ready) {
    return;
  }
  float32_t iq_command = state.iq_ref;
  if (state.CONTROL_MODE == MODE_CURRENT) {
    const float32_t cogging_current =
        cogging_compensation_current(state.position.theta_m);
    state.debug = cogging_current;
    iq_command += cogging_current;
    if (iq_command > CURRENT_MAX)
      iq_command = CURRENT_MAX;
    else if (iq_command < -CURRENT_MAX)
      iq_command = -CURRENT_MAX;
  }
  current_loop(&state.position, iq_command);
}

// 10 kHz encoder update loop (same update rate used by the previous encoder).
void TIM6_PeriodElapsedCB(TIM_HandleTypeDef *htim) {
  UNUSED(htim);
  static uint32_t previous_cycles = 0;
  static uint32_t previous_sequence = 0;
  const uint32_t begin = TIM2->CNT;
  MA732_t::Sample sample{};
  const auto status = encoder.readSample(sample);
  if (status != HAL_OK) {
    position_ready = false;
    Error_Raise(ERROR_SPI);
    return;
  }
  if (sample.sequence == previous_sequence) return;
  const float32_t angle = (sample.word >> 2U) * (360.0f / 16384.0f);
  if (!position_ready) {
    state.position.theta_m_prev = angle;
    state.position.omega_m = 0.0f;
  }
  const uint32_t elapsed = sample.cycles - previous_cycles;
  const float32_t dt = position_ready && elapsed != 0U ? elapsed * DT : 0.0001f;
  updatePositionMech(&state.position, angle, dt, NUM_SAMPLES);
  previous_cycles = sample.cycles;
  previous_sequence = sample.sequence;
  position_ready = true;
  process_time_pos = TIM2->CNT - begin;
}

// 5 kHz loop.
void TIM7_PeriodElapsedCB(TIM_HandleTypeDef *htim) {
  UNUSED(htim);
  if (Error_IsActive()) {
    return;
  }
  switch (state.CONTROL_MODE) {
  case MODE_CURRENT:
    break;
  case MODE_VELOCITY: {
    state.iq_ref = velocity_loop(&state.position, state.omega_ref);
    update_cogging_measurement(state.iq_ref);
    if (!cogging_measurement_active()) {
      const float32_t cogging_current =
          cogging_compensation_current(state.position.theta_m);
      state.debug = cogging_current;
      state.iq_ref += cogging_current;
      if (state.iq_ref > CURRENT_MAX)
        state.iq_ref = CURRENT_MAX;
      else if (state.iq_ref < -CURRENT_MAX)
        state.iq_ref = -CURRENT_MAX;
    } else {
      state.debug = 0.0f;
    }
    break;
  }
  case MODE_POSITION: {
    state.iq_ref = position_loop(&state.position, state.theta_ref);
    const float32_t cogging_current =
        cogging_compensation_current(state.position.theta_m);
    state.debug = cogging_current;
    state.iq_ref += cogging_current;
    if (state.iq_ref > CURRENT_MAX)
      state.iq_ref = CURRENT_MAX;
    else if (state.iq_ref < -CURRENT_MAX)
      state.iq_ref = -CURRENT_MAX;
    break;
  }
  default:
    state.iq_ref = 0;
    state.debug = 0.0f;
    break;
  }
}

// 1 kHz loop.
void TIM17_PeriodElapsedCB(TIM_HandleTypeDef *htim) {
  UNUSED(htim);
  static uint16_t can_led_ticks = 0;

  if (state.error_flags.driver_fault) {
    can_led_ticks = 0;
    HAL_GPIO_WritePin(DRV_ERROR_GPIO_Port, DRV_ERROR_Pin, GPIO_PIN_SET);
  } else if (state.error_flags.can_error) {
    if (++can_led_ticks >= 250U) {
      can_led_ticks = 0;
      HAL_GPIO_TogglePin(DRV_ERROR_GPIO_Port, DRV_ERROR_Pin);
    }
  } else {
    can_led_ticks = 0;
    HAL_GPIO_WritePin(DRV_ERROR_GPIO_Port, DRV_ERROR_Pin, GPIO_PIN_RESET);
  }

  if (state.error_flags.can_error) {
    return;
  }

  float position_deg = state.position.theta_m;
  float speed_erpm =
      (state.position.p * state.position.omega_m) * DEG_PER_SEC_TO_RPM;
  // Report current with mechanical sign to match speed direction.
  float current_A = state.position.direction * i_dq_sen.q;
  int8_t rev = (int8_t)state.position.rev;
  uint8_t error_code = GET_ERROR_FLAGS(&state);

  CAN_Handler::broadcast_motor_status(position_deg, speed_erpm, current_A,
                                      rev, static_cast<int8_t>(error_code));
}

// 1 Hz loop.
void TIM16_PeriodElapsedCB(TIM_HandleTypeDef *htim) {
  UNUSED(htim);

  // Update temperature reading.
  uint32_t vref_mV =
      __HAL_ADC_CALC_VREFANALOG_VOLTAGE(s_adc5_buf[0], hadc5.Init.Resolution);
  state.temperature = (uint32_t)__HAL_ADC_CALC_TEMPERATURE(
      vref_mV, s_adc5_buf[1], hadc5.Init.Resolution);

  // Update DRV8316 fault LED.
  if (!HAL_GPIO_ReadPin(nFault_GPIO_Port,
                        nFault_Pin)) { // Read DRV8316 fault pin, active low.
    Error_Raise(ERROR_DRIVER_FAULT);
  }
  if (state.error_flags.driver_fault) {
    HAL_GPIO_WritePin(COMM_ERROR_GPIO_Port, COMM_ERROR_Pin, GPIO_PIN_RESET);
  } else {
    HAL_GPIO_TogglePin(COMM_ERROR_GPIO_Port, COMM_ERROR_Pin);
  }
}
