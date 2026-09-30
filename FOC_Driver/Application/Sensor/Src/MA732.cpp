/**
 * @file MA732.cpp
 * @brief MA732 SPI angle reads.
 */
#include "MA732.hpp"
#include "error_handler.hpp"

namespace {
constexpr float32_t DEGREES_PER_COUNT = 360.0f / 16384.0f;
}

HAL_StatusTypeDef MA732_t::readAngle(uint16_t &angle) {
  buffer_t tx{};
  buffer_t rx{};

  const HAL_StatusTypeDef status = m_spi.transfer(tx.packet, rx.packet, 1U);
  if (status == HAL_OK)
    angle = rx.data;
  return status;
}

uint16_t MA732_t::readAngleRaw() {
  uint16_t angle = 0U;
  if (readAngle(angle) != HAL_OK) {
    Error_Raise(ERROR_SPI);
    return 0U;
  }

  // MA732 supplies its 14 useful bits in bits 15:2 of the 16-bit frame.
  return angle >> 2U;
}

float MA732_t::readAngleDeg() {
  return static_cast<float32_t>(readAngleRaw()) * DEGREES_PER_COUNT;
}

