#pragma once

#include "MA732.hpp"
#include <cstdint>

/** Results of the startup-only SPI register snapshot, visible in Live Watch. */
struct MA732DebugState {
  uint32_t completed = 0U;
  uint32_t status = HAL_BUSY;
  uint32_t valid_mask = 0U;
  uint32_t registers[32] = {};
  uint32_t register_status[32] = {};
  uint32_t response_words[32] = {};
  uint32_t magnetic_flags_valid = 0U;
  uint32_t magnetic_field_low = 0U;
  uint32_t magnetic_field_high = 0U;
};

extern volatile MA732DebugState ma732_debug;

/** Read each documented register once. Call after SPI3 init, before timers. */
HAL_StatusTypeDef MA732_ReadRegistersOnce(MA732_t &encoder);

/**
 * Temporary startup diagnostic: disable the motor, allow sensor power-up,
 * capture registers once, then wait forever for Live Watch inspection.
 * Comment out its single main_cpp() call to restore normal startup.
 */
[[noreturn]] void MA732_DebugRegistersAndHalt(MA732_t &encoder);
