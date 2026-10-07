// Execute the production drivers with a pipelined fake sensor, not real SPI.
#include "MA732_debug.hpp"
#include <array>
#include <cassert>
#include <cstring>
#include <functional>
#include <iostream>
#include <vector>

uint32_t test_primask = 0U;
uint32_t test_ipsr = 0U;

namespace {
struct Frame { uint16_t word; uint32_t tick; };
struct HaltObserved {};
std::array<uint8_t, 32> registers{};
std::vector<Frame> frames;
uint32_t tick = 0U;
uint16_t pending_response = 0x2468U;
size_t fail_frame = 0U;
HAL_StatusTypeDef fail_status = HAL_ERROR;
uint16_t response_low_byte = 0U;
uint32_t write_count = 0U;
uint32_t nvm_ready = 0U;
bool nvm_pending = false;
uint8_t nvm_address = 0U;
uint8_t nvm_value = 0U;
bool reject_programming = false;
bool wrong_fresh_read = false;
bool motor_off = false;
bool observe_halt = false;
std::function<void()> delay_hook;

void resetFake() {
  registers.fill(0U);
  registers[4] = 0xC0U;
  registers[5] = 0xFFU;
  registers[6] = 0x1CU;
  registers[14] = 0x77U;
  registers[16] = 0x9CU;
  registers[27] = 0x40U;
  frames.clear();
  tick = 0U;
  pending_response = 0x2468U;
  fail_frame = 0U;
  fail_status = HAL_ERROR;
  response_low_byte = 0U;
  write_count = 0U;
  nvm_ready = 0U;
  nvm_pending = false;
  reject_programming = false;
  wrong_fresh_read = false;
  motor_off = false;
  observe_halt = false;
  test_primask = 0U;
  test_ipsr = 0U;
  delay_hook = nullptr;
}
}

void HAL_Delay(uint32_t milliseconds) {
  assert(test_primask == 0U && test_ipsr == 0U);
  tick += milliseconds;
  if (delay_hook)
    delay_hook();
  if (observe_halt && milliseconds == 100U)
    throw HaltObserved{};
}
void HAL_GPIO_WritePin(GPIO_TypeDef *, uint16_t pin, GPIO_PinState value) {
  assert(pin == DRVOFF_Pin && value == GPIO_PIN_SET);
  motor_off = true;
}
void Error_Raise(uint8_t) { assert(false && "Unexpected angle error"); }

SPIHandler_t::SPIHandler_t(SPI_HandleTypeDef *hspi, GPIO_TypeDef *port,
                           uint16_t pin)
    : m_hspi(hspi), m_csPort(port), m_csPin(pin), m_next(nullptr) {}

HAL_StatusTypeDef SPIHandler_t::transfer(uint8_t *tx, uint8_t *rx, size_t len) {
  assert(len == 1U);
  uint16_t word = 0U;
  std::memcpy(&word, tx, sizeof(word));
  frames.push_back({word, tick});
  if (fail_frame == frames.size())
    return fail_status;

  if (nvm_pending) {
    assert(tick >= nvm_ready); // No premature response/angle polling during NVM.
    if (!reject_programming)
      registers[nvm_address] = nvm_value;
    pending_response = static_cast<uint16_t>(registers[nvm_address]) << 8U;
    nvm_pending = false;
  }
  const uint16_t response = pending_response | response_low_byte;
  std::memcpy(rx, &response, sizeof(response));
  pending_response = 0x2468U;
  const uint8_t address = (word >> 8U) & 0x1FU;
  if ((word & 0xE000U) == 0x4000U) {
    pending_response = static_cast<uint16_t>(registers[address]) << 8U;
    if (wrong_fresh_read && write_count != 0U)
      pending_response ^= 0x0100U;
  } else if ((word & 0xE000U) == 0x8000U) {
    ++write_count;
    nvm_pending = true;
    nvm_ready = tick + 20U;
    nvm_address = address;
    nvm_value = word & 0xFFU;
  } else {
    assert(word == 0U);
  }
  return HAL_OK;
}

int main() {
  SPIHandler_t spi(nullptr, nullptr, 0U);
  {
    resetFake();
    MA732_t encoder(spi);
    uint8_t value = 0xAAU;
    // First returned frame is an angle; only the next frame contains R27.
    assert(encoder.readRegister(0x1BU, value) == HAL_OK && value == 0x40U);
    assert(frames.size() == 2U && frames[0].word == 0x5B00U && frames[1].word == 0U);
    assert(frames[1].tick - frames[0].tick >= 1U);
    assert(encoder.lastRegisterResponse() == 0x4000U);
    assert(!encoder.registerAccessInProgress());
    uint16_t angle = 0U;
    assert(encoder.readAngle(angle) == HAL_OK && angle == 0x2468U);
  }
  {
    resetFake();
    MA732_t encoder(spi);
    uint8_t value = 0xAAU;
    assert(encoder.readRegister(32U, value) == HAL_ERROR);
    test_ipsr = 16U;
    assert(encoder.readRegister(14U, value) == HAL_BUSY);
    test_ipsr = 0U;
    test_primask = 1U;
    assert(encoder.readRegister(14U, value) == HAL_BUSY && test_primask == 1U);
    test_primask = 0U;
    assert(frames.empty() && value == 0xAAU);
    delay_hook = [&] {
      uint16_t angle = 0xAAAAU;
      uint8_t nested = 0xBBU;
      assert(encoder.registerAccessInProgress());
      assert(encoder.readAngle(angle) == HAL_BUSY && angle == 0xAAAAU);
      assert(encoder.readRegister(14U, nested) == HAL_BUSY && nested == 0xBBU);
    };
    assert(encoder.readRegister(14U, value) == HAL_OK && value == 0x77U);
    assert(frames.size() == 2U && !encoder.registerAccessInProgress());
  }
  for (size_t failed : {1U, 2U}) {
    resetFake();
    MA732_t encoder(spi);
    fail_frame = failed;
    fail_status = HAL_TIMEOUT;
    uint8_t value = 0xAAU;
    assert(encoder.readRegister(14U, value) == HAL_TIMEOUT && value == 0xAAU);
    assert(!encoder.registerAccessInProgress());
    uint16_t angle = 0xBBBBU;
    const size_t count = frames.size();
    assert(encoder.readAngle(angle) == HAL_ERROR && angle == 0xBBBBU);
    assert(frames.size() == count); // Never publish a pending register as angle.
    fail_frame = 0U;
    assert(encoder.readRegister(14U, value) == HAL_OK && value == 0x77U);
    assert(encoder.readAngle(angle) == HAL_OK && angle == 0x2468U);
  }
  {
    resetFake();
    MA732_t encoder(spi);
    response_low_byte = 0xFFU;
    uint8_t value = 0xAAU;
    assert(encoder.readRegister(14U, value) == HAL_ERROR && value == 0xAAU);
    assert(encoder.lastRegisterResponse() == 0x77FFU);
  }
  {
    resetFake();
    MA732_t encoder(spi);
    uint8_t readback = 0xAAU;
    assert(encoder.writeRegister(27U, 0U, readback) == HAL_ERROR);
    assert(encoder.writeRegister(7U, 0U, readback) == HAL_ERROR);
    assert(encoder.writeRegister(9U, 1U, readback) == HAL_ERROR);
    assert(frames.empty() && readback == 0xAAU);
    assert(encoder.writeRegister(14U, 0x77U, readback) == HAL_OK);
    assert(write_count == 0U && readback == 0x77U);
    frames.clear();
    readback = 0xAAU;
    assert(encoder.writeRegister(14U, 0x66U, readback) == HAL_OK);
    assert(frames.size() == 6U && frames[2].word == 0x8E66U);
    assert(frames[3].tick - frames[2].tick >= 20U);
    assert(frames[4].word == 0x4E00U); // Independent read verifies NVM result.
    assert(write_count == 1U && readback == 0x66U && registers[14] == 0x66U);
  }
  {
    resetFake();
    MA732_t encoder(spi);
    registers[9] = 0x15U; // Preserve reserved bits while changing RD.
    uint8_t readback = 0U;
    assert(encoder.writeRegister(9U, 0x80U, readback) == HAL_OK);
    assert(frames[2].word == 0x8995U && readback == 0x95U);
  }
  for (bool bad_fresh_read : {false, true}) {
    resetFake();
    MA732_t encoder(spi);
    reject_programming = !bad_fresh_read;
    wrong_fresh_read = bad_fresh_read;
    uint8_t readback = 0xAAU;
    assert(encoder.writeRegister(14U, 0x66U, readback) == HAL_ERROR);
    assert(readback == 0xAAU && !encoder.registerAccessInProgress());
  }
  for (size_t failed : {1U, 2U, 3U, 4U, 5U, 6U}) {
    resetFake();
    MA732_t encoder(spi);
    fail_frame = failed;
    uint8_t readback = 0xAAU;
    assert(encoder.writeRegister(14U, 0x66U, readback) == HAL_ERROR);
    assert(readback == 0xAAU && !encoder.registerAccessInProgress());
    if (failed == 3U)
      assert(tick - frames[2].tick >= 20U);
  }
  {
    resetFake();
    MA732_t encoder(spi);
    assert(MA732_ReadRegistersOnce(encoder) == HAL_OK);
    assert(ma732_debug.completed == 1U && ma732_debug.status == HAL_OK);
    assert(frames.size() == 22U && write_count == 0U);
    assert(ma732_debug.registers[14] == 0x77U);
    assert(ma732_debug.registers[16] == 0x9CU);
    assert(ma732_debug.register_status[7] == 0xFFFFFFFFU);
    assert(ma732_debug.magnetic_flags_valid == 1U);
    assert(ma732_debug.magnetic_field_low == 1U && ma732_debug.magnetic_field_high == 0U);
    const uint32_t valid = (1UL << 0) | (1UL << 1) | (1UL << 2) | (1UL << 3) |
        (1UL << 4) | (1UL << 5) | (1UL << 6) | (1UL << 9) | (1UL << 14) |
        (1UL << 16) | (1UL << 27);
    assert(ma732_debug.valid_mask == valid);
  }
  {
    resetFake();
    MA732_t encoder(spi);
    fail_frame = 2U;
    assert(MA732_ReadRegistersOnce(encoder) == HAL_ERROR);
    assert(ma732_debug.completed == 1U && (ma732_debug.valid_mask & 1U) == 0U);
    assert(ma732_debug.register_status[0] == HAL_ERROR);
    assert(ma732_debug.registers[0] == 0U);
    assert((ma732_debug.valid_mask & (1UL << 14)) != 0U);
  }
  {
    resetFake();
    MA732_t encoder(spi);
    observe_halt = true;
    try {
      MA732_DebugRegistersAndHalt(encoder);
    } catch (const HaltObserved &) {
      assert(motor_off && ma732_debug.completed == 1U);
      assert(frames.size() == 22U && frames[0].tick >= 300U);
      assert(write_count == 0U);
    }
  }
  std::cout << "MA732 register/one-shot startup diagnostic tests passed\n";
}
