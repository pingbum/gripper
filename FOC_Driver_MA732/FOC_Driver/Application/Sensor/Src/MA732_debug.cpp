#include "MA732_debug.hpp"

volatile MA732DebugState ma732_debug;

namespace {
constexpr uint8_t CONFIG_REGISTERS[] = {
    0x00U, 0x01U, 0x02U, 0x03U, 0x04U, 0x05U,
    0x06U, 0x09U, 0x0EU, 0x10U, 0x1BU};

} // namespace

HAL_StatusTypeDef MA732_ReadRegistersOnce(MA732_t &encoder) {
  ma732_debug.completed = 0U;
  ma732_debug.status = HAL_BUSY;
  ma732_debug.valid_mask = 0U;
  ma732_debug.magnetic_flags_valid = 0U;
  ma732_debug.magnetic_field_low = 0U;
  ma732_debug.magnetic_field_high = 0U;
  for (uint32_t address = 0U; address < 32U; ++address) {
    ma732_debug.registers[address] = 0U;
    ma732_debug.register_status[address] = 0xFFFFFFFFU; // Not read.
    ma732_debug.response_words[address] = 0U;
  }
  HAL_StatusTypeDef status = HAL_OK;
  for (uint8_t address : CONFIG_REGISTERS) {
    uint8_t value = 0U;
    const HAL_StatusTypeDef result = encoder.readRegister(address, value);
    ma732_debug.register_status[address] = result;
    ma732_debug.response_words[address] = encoder.lastRegisterResponse();
    if (result == HAL_OK) {
      ma732_debug.registers[address] = value;
      ma732_debug.valid_mask = ma732_debug.valid_mask | (1UL << address);
      if (address == 0x1BU) {
        ma732_debug.magnetic_flags_valid = 1U;
        ma732_debug.magnetic_field_low = (value >> 6U) & 1U;
        ma732_debug.magnetic_field_high = (value >> 7U) & 1U;
      }
    } else if (status == HAL_OK) {
      status = result;
    }
  }
  ma732_debug.status = status;
  __DMB();
  ma732_debug.completed = 1U;
  return status;
}

[[noreturn]] void MA732_DebugRegistersAndHalt(MA732_t &encoder) {
  HAL_GPIO_WritePin(DRVOFF_GPIO_Port, DRVOFF_Pin, GPIO_PIN_SET);
  // MA732 power-up can take 260ms with the longest configured filter window.
  HAL_Delay(300U);
  (void)MA732_ReadRegistersOnce(encoder);

  // SysTick/DMA remain enabled, but control/angle timers and CAN startup have
  // not run. Register values stay stable for debugger inspection.
  while (true)
    HAL_Delay(100U);
}
