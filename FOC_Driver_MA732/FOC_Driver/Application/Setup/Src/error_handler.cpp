#include "main.h"
#include "main_cpp.hpp"

#include "error_handler.hpp"
#include "fmac.h"

extern State_t state;

namespace {
volatile uint8_t s_error_flags = 0;
volatile uint8_t s_error_active = 0;
bool s_control_disabled = false;

void latch_hal_error_flags() {
  uint8_t flags = 0;

  if (hspi1.ErrorCode != HAL_SPI_ERROR_NONE ||
      hspi3.ErrorCode != HAL_SPI_ERROR_NONE) {
    flags |= ERROR_SPI;
  }

  if (hfdcan1.ErrorCode != HAL_FDCAN_ERROR_NONE) {
    flags |= ERROR_CAN;
  }

  if (hadc1.ErrorCode != HAL_ADC_ERROR_NONE ||
      hadc2.ErrorCode != HAL_ADC_ERROR_NONE ||
      hadc3.ErrorCode != HAL_ADC_ERROR_NONE ||
      hadc5.ErrorCode != HAL_ADC_ERROR_NONE) {
    flags |= ERROR_HAL_ADC;
  }

  if (htim2.State == HAL_TIM_STATE_ERROR ||
      htim2.State == HAL_TIM_STATE_TIMEOUT ||
      htim6.State == HAL_TIM_STATE_ERROR ||
      htim6.State == HAL_TIM_STATE_TIMEOUT ||
      htim7.State == HAL_TIM_STATE_ERROR ||
      htim7.State == HAL_TIM_STATE_TIMEOUT ||
      htim16.State == HAL_TIM_STATE_ERROR ||
      htim16.State == HAL_TIM_STATE_TIMEOUT ||
      htim17.State == HAL_TIM_STATE_ERROR ||
      htim17.State == HAL_TIM_STATE_TIMEOUT) {
    flags |= ERROR_HAL_TIM;
  }

  if (hhrtim1.State == HAL_HRTIM_STATE_ERROR ||
      hhrtim1.State == HAL_HRTIM_STATE_TIMEOUT) {
    flags |= ERROR_HAL_HRTIM;
  }

#ifdef HAL_FMAC_MODULE_ENABLED
  if (hfmac.ErrorCode != HAL_FMAC_ERROR_NONE) {
    flags |= ERROR_HAL_FMAC;
  }
#endif

  if (flags == 0u && s_error_flags == 0u &&
      !state.error_flags.can_error && !state.error_flags.spi_error &&
      !state.error_flags.driver_fault) {
    flags |= ERROR_HAL_RCC;
  }

  if (flags != 0u) {
    Error_SetFlags(flags);
  }
}

void disable_control_loops() {
  if (s_control_disabled) {
    return;
  }
  s_control_disabled = true;

  state.CONTROL_MODE = 0;
  state.iq_ref = 0.0f;

  if (hhrtim1.Instance != nullptr) {
    HAL_HRTIM_WaveformOutputStop(
        &hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TC1 |
                      HRTIM_OUTPUT_TC2 | HRTIM_OUTPUT_TD1 | HRTIM_OUTPUT_TD2);
    HAL_HRTIM_WaveformCounterStop(
        &hhrtim1, HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_A |
                      HRTIM_TIMERID_TIMER_C | HRTIM_TIMERID_TIMER_D);
  }

  // Keep TIM6 running so mechanical position remains observable after a
  // control, CAN, driver, or transient encoder fault. The motor-control loops
  // remain disabled below/through their Error_IsActive() guards.
  if (htim7.Instance != nullptr) {
    HAL_TIM_Base_Stop_IT(&htim7);
  }

  if (hadc1.Instance != nullptr) {
    HAL_ADCEx_InjectedStop_IT(&hadc1);
  }
  if (hadc2.Instance != nullptr) {
    HAL_ADCEx_InjectedStop_IT(&hadc2);
  }
  if (hadc3.Instance != nullptr) {
    HAL_ADCEx_InjectedStop_IT(&hadc3);
  }
}
} // namespace

extern "C" void Error_SetFlags(uint8_t flags) {
  s_error_flags |= flags;
  if ((flags & ERROR_CAN) != 0u) {
    state.error_flags.can_error = true;
  }
  if ((flags & ERROR_SPI) != 0u) {
    state.error_flags.spi_error = true;
  }
  if ((flags & ERROR_DRIVER_FAULT) != 0u) {
    state.error_flags.driver_fault = true;
  }
  s_error_active = 1;
}

extern "C" void Error_Raise(uint8_t flags) {
  Error_SetFlags(flags);
  _Error_Handler();
}

extern "C" uint8_t Error_IsActive(void) { return s_error_active; }

void _Error_Handler() {
  s_error_active = 1;
  latch_hal_error_flags();
  disable_control_loops();
}

uint8_t GET_ERROR_FLAGS(State_t *state) {
  // Compose error flag bitmask.
  uint8_t error_flags = s_error_flags;
  if (state->error_flags.can_error) {
    error_flags |= ERROR_CAN; // Bit 0: CAN error
  }
  if (state->error_flags.spi_error) {
    error_flags |= ERROR_SPI; // Bit 1: SPI error
  }
  if (state->error_flags.driver_fault) {
    error_flags |= ERROR_DRIVER_FAULT; // Bit 2: Driver fault
  }
  return error_flags;
}
